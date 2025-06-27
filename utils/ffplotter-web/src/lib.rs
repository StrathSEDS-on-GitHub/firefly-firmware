use std::{
    cell::RefCell,
    collections::HashMap,
    sync::{
        Mutex,
        atomic::{AtomicI64, Ordering},
    },
};

use color_eyre::eyre;
use ffplotter_lib::{
    Column, ColumnType, RowTokens, match_column,
    ndarray::{Array1, Array2, Axis, s},
};
use once_cell::sync::{Lazy, OnceCell};
use plotters::prelude::*;
use plotters_canvas::CanvasBackend;
use wasm_bindgen::prelude::*;

static DISPLAY_START_TIME: OnceCell<f64> = OnceCell::new();
static LATENCIES: Lazy<Mutex<RefCell<HashMap<String, i32>>>> =
    Lazy::new(|| Mutex::new(RefCell::new(HashMap::new())));

pub fn draw(canvas_id: String) -> eyre::Result<()> {
    let backend = CanvasBackend::new(canvas_id.as_str()).expect("cannot find canvas");
    let root = backend.into_drawing_area();
    root.fill(&RGBColor(31, 41, 55))?;

    let lock = &DATA.lock().unwrap();
    let data = &lock.borrow()[1];

    if DISPLAY_START_TIME.get().is_none() {
        DISPLAY_START_TIME.set(now()).unwrap();
    }

    let elapsed_lock = &ELAPSED_TIMES.lock().unwrap();
    let elapsed_times = &elapsed_lock.borrow()[1];

    if elapsed_times.is_empty() {
        return Ok(());
    }

    let data_start_time = *elapsed_times.first().unwrap();
    let latency = LATENCIES
        .lock()
        .unwrap()
        .borrow()
        .get(&canvas_id)
        .cloned()
        .unwrap_or(0);

    let elapsed_time = now() - DISPLAY_START_TIME.get().unwrap();
    let sample_count = elapsed_times
        .into_iter()
        .take_while(|&t| (*t as i64 - data_start_time as i64) < elapsed_time as i64 + latency as i64)
        .count();

    if sample_count == 0 {
        return Ok(());
    }

    let format_elapsed = |elapsed: usize| {
        let hour = elapsed / 3600_000;
        let minute = (elapsed % 3600_000) / 60_000;
        let second = (elapsed % 60_000) / 1000;
        let millis = elapsed % 1000;
        format!("{:02}:{:02}:{:02}.{:03}", hour, minute, second, millis)
    };

    let hidden_samples = data.dim().0 - sample_count;
    let latency = if hidden_samples > 20 {
        latency + (hidden_samples as i32 / 10)
    } else if hidden_samples < 5 {
        latency - 2
    } else {
        latency
    };

    LATENCIES
        .lock()
        .unwrap()
        .borrow_mut()
        .insert(canvas_id, latency);

    let min = *data
        .slice(s![.., 0])
        .iter()
        .take(sample_count)
        .max_by(|l, r| r.partial_cmp(l).unwrap())
        .unwrap();
    let max = *data
        .slice(s![.., 0])
        .iter()
        .take(sample_count)
        .max_by(|l, r| l.partial_cmp(r).unwrap())
        .unwrap();

    let mut cc = ChartBuilder::on(&root)
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(0..sample_count, min..max)?;

    const NUM_LABELS: usize = 10;
    cc.configure_mesh()
        .x_label_formatter(&|&x| {
            // Only show last label
            let n = sample_count * (NUM_LABELS - 2) / NUM_LABELS;
            if n <= x {
                format_elapsed(elapsed_times[std::cmp::min(x as usize, elapsed_times.len() - 1)] as usize)
            } else {
                "".to_owned()
            }
        })
        .y_label_formatter(&|y: &f64| format!("{:.2}", *y))
        .light_line_style(&RGBColor(35, 78, 82))
        .max_light_lines(1)
        .axis_style(RGBColor(94, 234, 212))
        .label_style(&RGBColor(94, 234, 212))
        .x_labels(NUM_LABELS)
        .y_labels(5)
        .axis_desc_style(("sans-serif", 15))
        .draw()?;

    for &idx in &[0] {
        let series = Array1::from_iter(
            data.slice(s![.., idx])
                .iter()
                .cloned()
                .take(sample_count)
                .enumerate(),
        );

        cc.draw_series(LineSeries::new(series, &RGBColor(94, 234, 212)))?;
    }

    root.present()?;

    Ok(())
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);

    #[wasm_bindgen(js_namespace = Date)]
    fn now() -> f64;
}

#[wasm_bindgen]
pub fn power(canvas: String) -> Result<(), JsValue> {
    let _ = draw(canvas).map_err(|err| err.to_string())?;
    Ok(())
}
static ROW_FORMATS: Lazy<Vec<RowTokens>> = once_cell::sync::Lazy::new(|| {
    vec![
        ffplotter_lib::parse_row_format("[time:t] [src:s prt] pr:f,tmp:f").unwrap(),
        ffplotter_lib::parse_row_format("[time:t] [src:s imu] [accx:f,accy:f,accz:f]").unwrap(),
        ffplotter_lib::parse_row_format("[time:t] [src:s gps] gpx:f, gpy:f, gpz:f").unwrap(),
    ]
});
static COLUMNS_TEMPLATE: Lazy<Vec<Vec<Column>>> =
    once_cell::sync::Lazy::new(|| ROW_FORMATS.iter().map(|fmt| fmt.into()).collect());

static DATA: Lazy<Mutex<RefCell<Vec<Array2<f64>>>>> = once_cell::sync::Lazy::new(|| {
    Mutex::new(RefCell::new(
        COLUMNS_TEMPLATE
            .iter()
            .map(|columns| {
                Array2::from_shape_vec(
                    (
                        0,
                        columns
                            .iter()
                            .filter(|&it| matches!(it.data, ColumnType::Float(_)))
                            .count(),
                    ),
                    vec![],
                )
                .unwrap()
            })
            .collect(),
    ))
});

static ELAPSED_TIMES: Lazy<Mutex<RefCell<Vec<Vec<u64>>>>> =
    once_cell::sync::Lazy::new(|| Mutex::new(RefCell::new(vec![vec![]; COLUMNS_TEMPLATE.len()])));

#[wasm_bindgen]
pub fn parse(line: String) -> Result<(), JsValue> {
    for (((format, mut columns), data), elapsed) in ROW_FORMATS
        .iter()
        .zip(COLUMNS_TEMPLATE.clone())
        .zip(DATA.lock().unwrap().borrow_mut().iter_mut())
        .zip(ELAPSED_TIMES.lock().unwrap().borrow_mut().iter_mut())
    {
        if let Ok(()) =
            ffplotter_lib::parse_row_to_columns(&line, format, &Default::default(), &mut columns)
        {
            if let Some(col) = columns
                .iter()
                .find(|it| matches!(it.data, ColumnType::Time(_)))
            {
                match &col.data {
                    ColumnType::Time(items) => elapsed.extend(items.iter().map(|it| {
                        let (h, m, s, millis) = it.as_hms_milli();
                        millis as u64
                            + (s as u64 * 1000)
                            + (m as u64 * 60 * 1000)
                            + (h as u64 * 3600 * 1000)
                    })),
                    _ => {}
                }
            } else {
                elapsed.extend(std::iter::repeat_n(
                    0,
                    match_column!(&columns[0].data, items => items.len()),
                ));
            };

            let array =
                ffplotter_lib::columns_into_data_matrix(columns).map_err(|err| err.to_string())?;

            data.append(Axis(0), array.view())
                .map_err(|it| it.to_string())?;
        }
    }

    Ok(())
}

#[wasm_bindgen]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}
