use std::{iter, sync::atomic::Ordering};

use ratatui::{
    layout::{Constraint, Layout, Margin, Rect},
    style::{Color, Style, Stylize},
    text::{Line, Span, Text},
    widgets::{Block, Borders, Paragraph, Wrap},
    Frame,
};
use tui_menu::Menu;

use crate::{
    app::{App, Focus, PopupPhase},
    flash::LOGS_FLASH_RANGE,
};

pub fn ui(f: &mut Frame, app: &mut App) {
    let area = f.size();

    let page = Layout::vertical([Constraint::Length(2), Constraint::Min(0)]);
    let [top_bar, body] = page.areas(area);
    let [header, menu_rect] =
        Layout::horizontal([Constraint::Fill(2), Constraint::Min(0)]).areas(top_bar);
    let vertical = Layout::horizontal([Constraint::Percentage(40), Constraint::Percentage(60)]);
    let [config, logs] = vertical.areas(body);

    let menu = Menu::new()
        .default_style(Style::new().fg(Color::White).bg(Color::from_u32(0x222277)))
        .dropdown_style(Style::new().fg(Color::Gray))
        .highlight(
            Style::new()
                .fg(Color::Gray)
                .bg(Color::from_u32(0x4466cc))
                .bold(),
        )
        .dropdown_width(24);

    let config_section = ratatui::widgets::Block::default()
        .title("Config")
        .borders(Borders::ALL)
        .border_style(if app.focused == Focus::Config {
            Color::from_u32(0x4466cc)
        } else {
            Color::Gray
        })
        .bg(Color::from_u32(0x1a1c1e));

    let logs_section = ratatui::widgets::Block::default()
        .title("Logs")
        .borders(Borders::ALL)
        .border_style(if app.focused == Focus::Logs {
            Color::from_u32(0x4466cc)
        } else {
            Color::Gray
        })
        .bg(Color::from_u32(0x1a1c1e));

    let mut config_text = Text::default();
    config_text.extend(
        app.config
            .own
            .lines
            .clone()
            .into_iter()
            .skip(app.config.line),
    );

    let mut logs_text = Text::default();
    logs_text.extend(
        app.logs
            .text()
            .lines
            .clone()
            .into_iter()
            .skip(app.logs.line),
    );

    f.render_widget(
        &Span::styled(
            "Flash Utility for Chronicles and Keystore",
            Style::default()
                .fg(Color::from_u32(0x4466cc))
                .bg(Color::from_u32(0x232323))
                .bold()
                .underlined(),
        ),
        header,
    );
    f.render_widget(&config_text, config_section.inner(config));
    f.render_widget(&config_section, config);
    f.render_widget(&logs_section, logs);
    f.render_widget(&logs_text, logs_section.inner(logs));
    f.render_stateful_widget(menu, menu_rect, &mut app.menu);

    ui_popup(f, app);
}

fn ui_popup(f: &mut Frame<'_>, app: &mut App) {
    let area = f.size();
    match app.popup_phase {
        PopupPhase::Closed => return,
        _ => {}
    }
    let popup_block = Block::default()
        .title("Setup")
        .borders(Borders::ALL)
        .border_style(Color::from_u32(0x4466cc))
        .bg(Color::from_u32(0x1a1c1e));

    let popup_area = Rect {
        x: area.width / 4,
        y: area.height / 4,
        width: area.width / 2,
        height: area.height / 2,
    };

    f.render_widget(
        Paragraph::new(" ".repeat((popup_area.width * popup_area.height) as usize))
            .wrap(Wrap { trim: false })
            .bg(Color::from_u32(0x232323)),
        popup_area,
    );
    f.render_widget(&popup_block, popup_area);

    if let PopupPhase::Selection {
        list_state,
        devices,
        instructions,
        show_local,
        ..
    } = &mut app.popup_phase
    {
        let [instructions_area, popup_menu_area] =
            Layout::vertical([Constraint::Length(2), Constraint::Fill(1)])
                .areas(popup_area.inner(&Margin::new(4, 2)));

        let instructions = Paragraph::new(instructions as &str)
            .style(Style::new().fg(Color::Gray))
            .alignment(ratatui::layout::Alignment::Center);
        let popup_menu = ratatui::widgets::List::new({
            let items: &mut dyn Iterator<Item = _> =
                &mut devices.clone().into_iter().map(|i| {
                    Line::default().spans(
                        [
                            Span::styled("(SYNC) ", Style::new().fg(Color::Blue)),
                            Span::styled(
                                format!(" Device {}", &i.name),
                                Style::new().fg(Color::Gray),
                            ),
                            Span::styled(
                                format!(
                                    "{: >w$.2}K",
                                    i.size / 1024,
                                    w = popup_menu_area.width as usize - i.name.len() - 16
                                ),
                                Style::new().fg(Color::Gray),
                            ),
                        ]
                        .into_iter(),
                    )
                });
            if *show_local {
                items.chain(iter::once(
                    Line::default().spans(
                        [
                            Span::styled("(LOCAL)", Style::new().fg(Color::Blue)),
                            Span::styled(" Open existing flash.bin", Style::new().fg(Color::Gray)),
                            Span::styled(
                                format!(
                                    "{: >w$.2}K",
                                    LOGS_FLASH_RANGE.end / 1024,
                                    w = popup_menu_area.width as usize
                                        - "Open existing flash.bin".len()
                                        - 9
                                ),
                                Style::new().fg(Color::Gray),
                            ),
                        ]
                        .into_iter(),
                    ),
                )).collect::<Vec<Line>>()
            } else {
                items.collect::<Vec<Line>>()
            }
        })
        .style(Style::new().fg(Color::Gray))
        .highlight_style(
            Style::new()
                .fg(Color::Gray)
                .bg(Color::from_u32(0x4466cc))
                .bold(),
        );
        f.render_stateful_widget(&popup_menu, popup_menu_area, list_state);
        f.render_widget(&instructions, instructions_area);
    }
    if let PopupPhase::Progress(message, percent, _) = &app.popup_phase {
        let [text, progress_area] =
            Layout::vertical([Constraint::Length(4), Constraint::Length(3)])
                .areas(popup_area.inner(&Margin::new(4, 4)));
        f.render_widget(
            Paragraph::new(message as &str)
                .style(Style::new().fg(Color::Gray))
                .alignment(ratatui::layout::Alignment::Center),
            text,
        );

        let progress_bar = ratatui::widgets::Gauge::default()
            .block(Block::default().borders(Borders::ALL).title("Progress"))
            .percent(percent.load(Ordering::Relaxed) as u16)
            .gauge_style(Style::new().fg(Color::from_u32(0x4466cc)));
        f.render_widget(&progress_bar, progress_area);
    }
}
