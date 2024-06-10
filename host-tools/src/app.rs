use std::{
    cmp::min,
    env,
    fs::OpenOptions,
    io::{self, Read, Seek, Write},
    iter,
    mem::MaybeUninit,
    ops::Deref,
    pin::Pin,
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
    thread::{self, sleep},
    time::Duration,
};

use anyhow::Context;
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use embedded_storage_async::nor_flash::NorFlash;
use ratatui::{
    backend::Backend,
    style::{Color, Style},
    text::{Line, Span, Text},
    widgets::{List, ListState, Paragraph},
    Terminal,
};
use sequential_storage::{cache::NoCache, map, queue};
use serde_json::{Map, Value};
use storage_types::U64Item;
use syntect::{
    easy::HighlightLines, highlighting::ThemeSet, parsing::SyntaxSet, util::LinesWithEndings,
};
use tokio::sync::oneshot;
use tui_menu::{MenuItem, MenuState};

use crate::flash::{FileWrapper, CONFIG_FLASH_RANGE, LOGS_FLASH_RANGE};
use crate::{
    os::{get_devices, Device},
    ui,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Focus {
    Menu,
    Config,
    Logs,
    Popup,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    Read,
    Write,
    Erase,
}

#[derive(Debug)]
pub enum PopupPhase {
    Selection {
        list_state: ListState,
        devices: Vec<Device>,
        instructions: String,
        show_local: bool,
        reason: Reason,
    },
    Progress(String, Arc<AtomicUsize>, Reason),
    Error(bool),
    Closed,
}

pub struct App {
    pub menu: MenuState<&'static str>,
    pub focused: Focus,
    pub config: Config,
    pub logs: Logs,
    pub popup_phase: PopupPhase,
    pub progress_done_channel: Option<oneshot::Receiver<()>>,
    pub error_channel: Option<oneshot::Receiver<(bool, String)>>,
    pub abort_read_channel: Option<oneshot::Sender<()>>,
}

impl App {
    pub fn new() -> Self {
        // let ss = syntect::parsing::SyntaxSet::load_defaults_newlines();
        // let ts = syntect::highlighting::ThemeSet::load_defaults();

        let mut s = Self {
            menu: MenuState::new(vec![
                MenuItem::group(
                    " Sync ",
                    vec![
                        MenuItem::item(" Sync config to device ", "sync.device"),
                        MenuItem::item(" Erase device fully", "sync.erase.logs"),
                    ],
                ),
                MenuItem::group(
                    " Config ",
                    vec![MenuItem::item(" Edit config ", "config.edit")],
                ),
                MenuItem::group(
                    " Logs ",
                    vec![MenuItem::item(" Export logs to CSV ", "logs.csv")],
                ),
            ]),
            focused: Focus::Popup,
            config: Config::default(),
            logs: Logs::default(),
            popup_phase: PopupPhase::Selection {
                list_state: ListState::default(),
                devices: get_devices()
                    .unwrap()
                    .map(|d| d.unwrap())
                    .filter(|d| d.size == LOGS_FLASH_RANGE.end as u64)
                    .collect(),
                instructions: "Do you wish to sync from a device?".to_owned(),
                show_local: true,
                reason: Reason::Read,
            },
            progress_done_channel: None,
            abort_read_channel: None,
            error_channel: None,
        };
        s.focus(Focus::Popup);
        s
    }

    pub fn left(&mut self) {
        match self.focused {
            Focus::Menu => self.menu.left(),
            Focus::Logs => self.focused = Focus::Config,
            _ => {}
        }
    }

    pub fn right(&mut self) {
        match self.focused {
            Focus::Menu => self.menu.right(),
            Focus::Config => self.focused = Focus::Logs,
            _ => {}
        }
    }

    pub fn up(&mut self) {
        match self.focused {
            Focus::Config => {
                if self.config.line == 0 {
                    self.focused = {
                        self.menu.activate();
                        Focus::Menu
                    }
                } else {
                    self.config.line -= 1;
                }
            }
            Focus::Logs => {
                if self.logs.line == 0 {
                    self.focused = {
                        self.menu.activate();
                        Focus::Menu
                    }
                } else {
                    self.logs.line -= 1;
                }
            }
            Focus::Popup => match &mut self.popup_phase {
                PopupPhase::Selection { list_state, .. } => {
                    list_state.select(list_state.selected().map(|s| s.saturating_sub(1)));
                }
                _ => {}
            },
            Focus::Menu => self.menu.up(),
        }
    }

    pub fn down(&mut self) {
        match self.focused {
            Focus::Menu => {
                self.menu.down();
            }
            Focus::Config => {
                self.config.line = min(self.config.line + 1, self.config.own.lines.len() - 2);
            }
            Focus::Logs => {
                self.logs.line = min(self.logs.line + 1, self.logs.text().lines.len() - 2);
            }
            Focus::Popup => match &mut self.popup_phase {
                PopupPhase::Selection {
                    list_state,
                    devices,
                    ..
                } => {
                    list_state.select(list_state.selected().map(|s| min(s + 1, devices.len())));
                }
                _ => {}
            },
        }
    }

    pub fn escape(&mut self) {
        self.menu.reset();
        self.focused = Focus::Config;
    }

    pub fn focus(&mut self, focus: Focus) {
        self.focused = focus;
        match focus {
            Focus::Menu => self.menu.activate(),
            Focus::Popup => match &mut self.popup_phase {
                PopupPhase::Selection { list_state, .. } => list_state.select(0.into()),
                _ => {}
            },
            _ => {}
        }
    }

    pub fn error(&mut self, critical: bool, message: Paragraph) {
        self.popup_phase = PopupPhase::Error(critical);
        self.focus(Focus::Popup);
    }

    pub async fn load_flash(&mut self) {
        let mut opts = std::fs::OpenOptions::new();
        opts.write(true).create(true).read(true);

        #[cfg(target_family = "unix")]
        {
            use std::os::unix::fs::OpenOptionsExt;
            opts.mode(0o777);
        }

        let mut cfg = opts.open("flash.bin").unwrap();

        if cfg.metadata().unwrap().len() == 0 {
            cfg.write_all(&iter::repeat_n(0xFF, LOGS_FLASH_RANGE.end as usize).collect::<Vec<_>>())
                .unwrap();
            cfg.seek(std::io::SeekFrom::Start(0)).unwrap();
        }

        let mut file = FileWrapper::new(cfg).unwrap();
        let id: Option<U64Item> = map::fetch_item(
            &mut file,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 1024],
            "id".try_into().unwrap(),
        )
        .await
        .unwrap();

        let sf: Option<U64Item> = map::fetch_item(
            &mut file,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 1024],
            "sf".try_into().unwrap(),
        )
        .await
        .unwrap();

        let bw: Option<U64Item> = map::fetch_item(
            &mut file,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 1024],
            "bw".try_into().unwrap(),
        )
        .await
        .unwrap();

        let cr: Option<U64Item> = map::fetch_item(
            &mut file,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 1024],
            "cr".try_into().unwrap(),
        )
        .await
        .unwrap();

        let power: Option<U64Item> = map::fetch_item(
            &mut file,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 1024],
            "power".try_into().unwrap(),
        )
        .await
        .unwrap();

        let rf_freq: Option<U64Item> = map::fetch_item(
            &mut file,
            CONFIG_FLASH_RANGE,
            NoCache::new(),
            &mut [0; 1024],
            "rf_freq".try_into().unwrap(),
        )
        .await
        .unwrap();

        let mut logs = queue::peek_many(&mut file, LOGS_FLASH_RANGE, NoCache::new())
            .await
            .unwrap();

        let mut logs_str = String::new();

        while let Ok(Some(log)) = logs.next(&mut [0; 4096]).await {
            logs_str.push_str(&String::from_utf8_lossy(log));
        }

        self.logs = Logs::new(logs_str);

        let config_str = format!(
            r#"{{
    "id": {},
    "sf": {},
    "bw": {},
    "cr": {},
    "power": {},
    "rf_freq": {}
}}"#,
            id.map(|x| x.1 as u64).unwrap_or(0),
            sf.map(|x| x.1 as u64).unwrap_or(0),
            bw.map(|x| x.1 as u64).unwrap_or(0),
            cr.map(|x| x.1 as u64).unwrap_or(0),
            power.map(|x| x.1 as u64).unwrap_or(0),
            rf_freq.map(|x| x.1 as u64).unwrap_or(0),
        );

        let ss = syntect::parsing::SyntaxSet::load_defaults_newlines();
        let ts = syntect::highlighting::ThemeSet::load_defaults();

        let json = serde_json::from_str(&config_str).unwrap();
        self.config = Config::new(&ss, &ts, config_str, json);
    }

    pub async fn select(&mut self) {
        match self.focused {
            Focus::Menu => {
                self.menu.select();
            }
            Focus::Popup => match &self.popup_phase {
                PopupPhase::Selection {
                    list_state,
                    devices,
                    reason,
                    ..
                } => {
                    if let Some(i) = list_state.selected() {
                        match *reason {
                            Reason::Read => {
                                self.read_from_device(i).await.unwrap_or_else(|e| {
                                    self.error(true, Paragraph::new(format!("{:?}", e)));
                                });
                            }
                            Reason::Write => {
                                self.write_to_device(i).await.unwrap_or_else(|e| {
                                    self.error(true, Paragraph::new(format!("{:?}", e)));
                                });
                            }
                            Reason::Erase =>  {
                                self.erase_device(i).await.unwrap_or_else(|e| {
                                    self.error(true, Paragraph::new(format!("{:?}", e)));
                                });
                            }
                        }
                    }
                }
                _ => {}
            },
            _ => {}
        }
    }

    async fn erase_device(&mut self, i: usize) -> anyhow::Result<()> {
        let devices = if let PopupPhase::Selection { devices, .. } = &self.popup_phase {
            devices
        } else {
            unreachable!()
        };
        let selected = devices[i].clone();
        let percent = Arc::new(AtomicUsize::new(0));
        self.popup_phase = PopupPhase::Progress(
            "Erasing device..".to_owned(),
            percent.clone(),
            Reason::Erase,
        );
        self.focus(Focus::Popup);

        let done_channel = oneshot::channel();
        let err_channel = oneshot::channel();
        self.progress_done_channel = Some(done_channel.1);
        self.error_channel = Some(err_channel.1);

        let device_file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(selected.path)?;

        let mut flash_wrapper = FileWrapper::new_no_cache(device_file)?;

        tokio::spawn(async move {
            if let Err(e) = flash_wrapper.erase(CONFIG_FLASH_RANGE.start, LOGS_FLASH_RANGE.end).await {
                err_channel.0.send((true, format!("{:?}", e))).unwrap();
                return;
            }
            done_channel.0.send(()).unwrap();
            println!("erasing done??");
        });

        Ok(())
    }

    async fn read_from_device(&mut self, i: usize) -> anyhow::Result<()> {
        let devices = if let PopupPhase::Selection { devices, .. } = &self.popup_phase {
            devices
        } else {
            unreachable!()
        };
        if i == devices.len() {
            // Open existing flash.bin
            self.popup_phase = PopupPhase::Closed;
            self.load_flash().await;
            self.focus(Focus::Config);
        } else {
            // Sync from device
            let selected = devices[i].clone();
            let percent = Arc::new(AtomicUsize::new(0));
            self.popup_phase = PopupPhase::Progress(
                "Reading from device..".to_owned(),
                percent.clone(),
                Reason::Read,
            );

            let done_channel = oneshot::channel();
            let mut abort_channel = oneshot::channel();
            self.progress_done_channel = Some(done_channel.1);
            self.abort_read_channel = Some(abort_channel.0);

            tokio::spawn(async move {
                let mut device_file = std::fs::OpenOptions::new()
                    .read(true)
                    .open(selected.path)
                    .unwrap();

                let mut opts = std::fs::OpenOptions::new();
                opts.write(true).create(true).truncate(true);

                #[cfg(target_family = "unix")]
                {
                    use std::os::unix::fs::OpenOptionsExt;
                    opts.mode(0o777);
                }

                let mut flash_file = opts.open("flash.bin").unwrap();

                let mut buf = [0u8; 8192];
                let mut written = 0;
                while let Ok(n) = device_file.read(&mut buf) {
                    if n == 0 || abort_channel.1.try_recv().is_ok() {
                        break;
                    }
                    flash_file.write_all(&buf[..n]).unwrap();
                    written += n;
                    percent.store(
                        (written as f64 / selected.size as f64 * 100.0) as usize,
                        Ordering::Relaxed,
                    );
                }

                done_channel.0.send(()).unwrap();
            });
        }
        Ok(())
    }
    pub async fn write_to_device(&mut self, i: usize) -> anyhow::Result<()> {
        let devices = if let PopupPhase::Selection { devices, .. } = &self.popup_phase {
            devices
        } else {
            unreachable!()
        };
        let selected = devices[i].clone();
        let percent = Arc::new(AtomicUsize::new(0));
        self.popup_phase = PopupPhase::Progress(
            "Writing to device..".to_owned(),
            percent.clone(),
            Reason::Write,
        );
        self.focus(Focus::Popup);

        let done_channel = oneshot::channel();
        let err_channel = oneshot::channel();
        self.progress_done_channel = Some(done_channel.1);
        self.error_channel = Some(err_channel.1);

        let json = self.config.json.clone();
        let device_file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(selected.path)?;

        let mut flash_wrapper = FileWrapper::new(device_file).context("Reading file failed")?;

        let total_keys = json.len();

        tokio::spawn(async move {
            for (i, (key, value)) in json.iter().enumerate() {
                if let Value::Number(value) = value {
                    if let Err(e) = map::store_item(
                        &mut flash_wrapper,
                        CONFIG_FLASH_RANGE,
                        NoCache::new(),
                        &mut [0; 2048],
                        &U64Item(
                            (key as &str)
                                .try_into()
                                .expect(&format!("Key {} is too long", key)),
                            value.as_u64().unwrap(),
                        ),
                    )
                    .await
                    {
                        err_channel.0.send((true, format!("{:?}", e))).unwrap();
                        return;
                    }
                }
                percent.store(
                    (i as f64 / total_keys as f64 * 100.0) as usize,
                    Ordering::Relaxed,
                );
            }

            done_channel.0.send(()).unwrap();
        });

        Ok(())
    }
}

#[derive(Default)]
pub struct Config {
    pub own: OwnedString<Text<'static>>,
    pub line: usize,
    pub json: serde_json::Map<String, Value>,
}

#[derive(Default)]
pub struct Logs {
    pub own: OwnedString<Text<'static>>,
    pub line: usize,
}

pub struct OwnedString<T> {
    item: MaybeUninit<T>,
    string: Pin<Box<str>>,
}

impl<T: Default> Default for OwnedString<T> {
    fn default() -> Self {
        Self {
            item: MaybeUninit::new(T::default()),
            string: Box::<str>::into_pin(String::new().into_boxed_str()),
        }
    }
}

impl<T> OwnedString<T> {
    pub fn new<'b, 'a: 'b>(string: String, item_generator: impl FnOnce(&'a str) -> T + 'b) -> Self {
        let mut s = Self {
            item: MaybeUninit::uninit(),
            string: Box::<str>::into_pin(string.into_boxed_str()),
        };

        // SAFETY: Self referential struct, the String is always dropped after
        // the T and cannot be moved or reallocated so the reference is
        // always valid and we can lie by saying it's 'static
        s.item.write(item_generator(unsafe {
            std::mem::transmute::<_, &'static str>(s.string.as_ref())
        }));
        s
    }

    pub fn string(&self) -> &str {
        &self.string
    }
}

impl<T> Deref for OwnedString<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.item.assume_init_ref() }
    }
}

impl Config {
    fn new<'a, 'b>(
        ss: &'a SyntaxSet,
        ts: &'a ThemeSet,
        config_str: String,
        json: Map<String, Value>,
    ) -> Self {
        Config {
            own: OwnedString::new(config_str, move |config_str2: &'static str| {
                let mut text: Text = Text::default();
                for (i, line) in LinesWithEndings::from(config_str2).enumerate() {
                    let mut h = HighlightLines::new(
                        &ss.find_syntax_by_extension("json").unwrap(),
                        &ts.themes["base16-ocean.dark"],
                    );
                    let ranges = h.highlight_line(line, ss).unwrap();
                    let color_convert = |c: syntect::highlighting::Color| Color::Rgb(c.r, c.g, c.b);
                    let spans = ranges.into_iter().map(|(style, text)| {
                        Span::styled(
                            text,
                            Style::new()
                                .fg(color_convert(style.foreground))
                                .bg(color_convert(style.background)),
                        )
                    });
                    let line_number = Span::styled(
                        format!("{: >3} │ ", i + 1),
                        Style::new().fg(Color::from_u32(0x444444)),
                    );

                    let it = [line_number].into_iter().chain(spans);
                    text.lines.push(Line::default().spans(it));
                }
                text
            }),
            line: 0,
            json,
        }
    }
}

impl Logs {
    fn new(logs_str: String) -> Logs {
        Logs {
            own: OwnedString::new(logs_str, move |logs_str| {
                let mut text = Text::default();
                for (i, line) in logs_str.lines().enumerate() {
                    let split = line.split_once(' ').and_then(|(date, rest)| {
                        rest.split_once(',').map(|(src, rest)| (date, src, rest))
                    });

                    let line_number = Span::styled(
                        format!("{: >3} │ ", i + 1),
                        Style::new().fg(Color::from_u32(0x444444)),
                    );

                    if let Some((date, src, rest)) = split {
                        let date = Span::styled(date, Style::new().fg(Color::from_u32(0x444444)));
                        let src = Span::styled(src, Style::new().fg(Color::Blue));
                        let message = Span::styled(rest, Style::new().fg(Color::White));

                        let it = Iterator::intersperse(
                            [line_number, date, src, message].into_iter(),
                            Span::raw(" "),
                        );
                        text.lines.push(Line::default().spans(it));
                    } else {
                        text.lines.push(Line::default().spans([
                            line_number,
                            Span::styled(line, Style::new().fg(Color::White)),
                        ]));
                    }
                }

                text
            }),
            line: 0,
        }
    }

    pub fn string(&self) -> &str {
        self.own.string()
    }

    pub fn text(&self) -> &Text<'static> {
        &self.own
    }
}

pub async fn run_app<B: Backend>(terminal: &mut Terminal<B>, mut app: App) -> io::Result<()> {
    loop {
        if event::poll(std::time::Duration::from_millis(10))? {
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                match key.code {
                    KeyCode::Char('h') | KeyCode::Left => app.left(),
                    KeyCode::Char('l') | KeyCode::Right => app.right(),
                    KeyCode::Char('j') | KeyCode::Down => app.down(),
                    KeyCode::Char('k') | KeyCode::Up => app.up(),
                    KeyCode::Char('m') => app.focus(Focus::Menu),
                    KeyCode::Esc => app.escape(),
                    KeyCode::Enter => app.select().await,
                    KeyCode::Char('c') => {
                        if key.modifiers.contains(KeyModifiers::CONTROL) {
                            app.abort_read_channel.map(|c| c.send(()));
                            break Ok(());
                        }
                    }
                    _ => {}
                }
            }
        }
        for e in app.menu.drain_events() {
            match e {
                tui_menu::MenuEvent::Selected(item) => match item {
                    "config.edit" => {
                        let editor = env::var("EDITOR").unwrap_or(
                            if cfg!(target_os = "windows") {
                                "notepad.exe"
                            } else {
                                "vi"
                            }
                            .to_owned(),
                        );

                        let random_ascii: String = rand::random::<[u8; 12]>()
                            .iter()
                            .map(|b| (b % 26 + 97) as char)
                            .collect();

                        let edit_file_path =
                            std::env::temp_dir().join(format!("config-{}.json", random_ascii));
                        let mut edit_file = OpenOptions::new()
                            .read(true)
                            .write(true)
                            .create(true)
                            .open(&edit_file_path)
                            .unwrap();

                        edit_file
                            .write_all(app.config.own.string.as_ref().as_bytes())
                            .unwrap();

                        let status = std::process::Command::new(editor)
                            .arg(&edit_file_path)
                            .status()
                            .unwrap();

                        if !status.success() {
                            app.error(
                                false,
                                Paragraph::new("Editor failed with non-zero exit code"),
                            );
                            continue;
                        }

                        edit_file.seek(std::io::SeekFrom::Start(0)).unwrap();
                        let mut buf = Vec::new();
                        edit_file.read_to_end(&mut buf).unwrap();

                        std::fs::remove_file(&edit_file_path).unwrap();

                        let new_config_str = String::from_utf8_lossy(&buf);
                        let json =
                            serde_json::from_str::<serde_json::Map<String, Value>>(&new_config_str);

                        if json.is_err() {
                            app.error(false, Paragraph::new("Invalid JSON"));
                            continue;
                        }

                        let json = json.unwrap();

                        for value in json.values() {
                            if value.is_null() || !(value.is_u64()) {
                                app.error(false, Paragraph::new("JSON contains invalid values"));
                                continue;
                            }
                        }

                        app.config = Config::new(
                            &syntect::parsing::SyntaxSet::load_defaults_newlines(),
                            &syntect::highlighting::ThemeSet::load_defaults(),
                            new_config_str.to_string(),
                            json,
                        );

                        terminal.clear().unwrap();
                    }
                    "sync.device" => {
                        app.popup_phase = PopupPhase::Selection {
                            list_state: ListState::default(),
                            devices: get_devices()
                                .unwrap()
                                .map(|d| d.unwrap())
                                .filter(|d| d.size == LOGS_FLASH_RANGE.end as u64)
                                .collect(),
                            instructions: "Choose a device to write to".to_owned(),
                            show_local: false,
                            reason: Reason::Write,
                        };
                        app.focus(Focus::Popup);
                    }
                    "sync.erase.logs" => {
                        app.popup_phase = PopupPhase::Selection {
                            list_state: ListState::default(),
                            devices: get_devices()
                                .unwrap()
                                .map(|d| d.unwrap())
                                .filter(|d| d.size == LOGS_FLASH_RANGE.end as u64)
                                .collect(),
                            instructions: "Choose a device to erase".to_owned(),
                            show_local: false,
                            reason: Reason::Erase,
                        };
                        app.focus(Focus::Popup);
                    }
                    _ => {
                        println!("Selected: {}", item);
                    }
                },
            }
        }

        if let Some(ref mut rx) = app.progress_done_channel {
            if let Ok(()) = rx.try_recv() {
                if matches!(app.popup_phase, PopupPhase::Progress(_, _, Reason::Read | Reason::Erase)) {
                    app.load_flash().await;
                }
                app.progress_done_channel = None;

                app.popup_phase = PopupPhase::Closed;
                app.focus(Focus::Config);
            }
        }

        if let Some(ref mut rx) = app.error_channel {
            if let Ok((critical, message)) = rx.try_recv() {
                app.error(critical, Paragraph::new(message));
            }
        }
        terminal.draw(|f| ui::ui(f, &mut app))?;
    }
}
