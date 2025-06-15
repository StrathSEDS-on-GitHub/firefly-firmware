#![feature(iterator_try_collect)]

use clap::{self, Parser};
use color_eyre::eyre::{self, Context, ContextCompat, eyre};
use ffplotter_lib::ndarray::{Array1, s};
use ffplotter_lib::{
    Column, ColumnType, Filter, RowToken, RowTokens, SampleType, generate_composite_columns,
    match_column, parse_row_format,
};
use piston::EventLoop;
use piston::WindowSettings;
use piston_window::PistonWindow;
use plotters::chart::ChartBuilder;
use plotters::prelude::IntoDrawingArea;
use plotters::prelude::Rectangle;
use plotters::series::LineSeries;
use plotters::style::WHITE;
use plotters::style::{BLACK, Color, Palette, Palette99};
use plotters_piston::draw_piston_window;
use std::{io::BufRead, path::PathBuf};

#[derive(clap::Parser, Debug)]
struct Cli {
    /// The input file to process
    #[arg(value_hint = clap::ValueHint::FilePath)]
    input: PathBuf,

    /// The row format for parsing the input file  
    ///
    /// Alphanumeric column names followed by a colon and sample type
    /// (f for float, s for string, t for timestamp)  
    ///
    /// Other tokens are treated as literals and will be matched exactly.  
    ///
    /// For example: `[time:t] [src:s sensor:s] temperature:f,pressure:f`
    #[arg(value_parser = parse_row_format)]
    row_format: RowTokens,

    /// Filter to apply to the rows
    ///
    /// Example: `src=Local,sensor=imu`
    #[arg(long, value_parser = parse_filter)]
    filter: Option<Filter>,

    /// Expression to evaluate in order to generate composite columns
    #[arg(long, short)]
    expr: Vec<String>,

    /// Columns to plot
    #[arg(long, short)]
    plot: Vec<String>,

    #[arg(long, short, default_value = "1")]
    time_warp: f64,
}

pub fn parse_filter(input: &str) -> eyre::Result<Filter> {
    let mut filter = Filter::new();
    for part in input.split(',') {
        let mut iter = part.splitn(2, '=');
        let key = iter
            .next()
            .ok_or_else(|| eyre!("Missing key in filter"))?
            .to_string();
        let value = iter
            .next()
            .ok_or_else(|| eyre!("Missing value in filter for key '{}'", key))?
            .to_string();
        filter.insert(key, value);
    }
    Ok(filter)
}

fn main() -> eyre::Result<()> {
    color_eyre::install()?;
    env_logger::init();

    let cli = Cli::parse();
    let file = std::fs::File::open(&cli.input)
        .wrap_err_with(|| eyre!("Failed to open input file: {}", cli.input.display()))?;

    let mut columns: Vec<Column> = cli
        .row_format
        .iter()
        .filter_map(|token| match token {
            RowToken::Sample { typ, name } => Some(match typ {
                SampleType::Float => Column {
                    header: name.to_string(),
                    data: ColumnType::Float(Vec::new()),
                },
                SampleType::String => Column {
                    header: name.to_string(),
                    data: ColumnType::String(Vec::new()),
                },
                SampleType::Timestamp => Column {
                    header: name.to_string(),
                    data: ColumnType::Time(Vec::new()),
                },
            }),
            RowToken::Literal(_) => None,
        })
        .collect();

    for (line_number, line) in std::io::BufReader::new(file).lines().enumerate() {
        let line = line.wrap_err("Failed to read line")?;
        let Some(samples) = ffplotter_lib::parse_row(
            &line,
            &cli.row_format,
            cli.filter.as_ref().unwrap_or(&Default::default()),
        )
        .wrap_err_with(|| {
            eyre!(
                "Failed to parse row at {}:{}",
                cli.input.file_name().unwrap().display(),
                line_number + 1
            )
        })?
        else {
            continue;
        };
        // Should be impossible, parse_row should take care of this.
        assert!(
            samples.len() == columns.len(),
            "Row has {} samples but expected {}",
            samples.len(),
            columns.len()
        );

        columns.iter_mut().zip(samples).for_each(|(col, sample)| {
            match_column!(
                &mut col.data,
                items => items.push(sample.try_into().expect("Sample type mismatch")
            ))
        });
    }

    log::info!(
        "Parsed {} rows",
        match_column!(&columns[0].data, items => items.len())
    );

    let mut headers = columns
        .iter_mut()
        .filter(|it| matches!(it.data, ColumnType::Float(_)))
        .map(|col| std::mem::replace(&mut col.header, String::new()))
        .collect::<Vec<_>>();

    let mut elapsed_times = columns
        .iter()
        .find(|it| matches!(it.data, ColumnType::Time(_)))
        .and_then(|col| match &col.data {
            ColumnType::Time(items) => Some(Array1::from_iter(items.iter().map(|it| {
                let (h, m, s, millis) = it.as_hms_milli();
                millis as u64
                    + (s as u64 * 1000)
                    + (m as u64 * 60 * 1000)
                    + (h as u64 * 3600 * 1000)
            }))),
            _ => None,
        })
        .unwrap_or_else(|| {
            log::warn!("No time column found");
            Array1::from_elem(match_column!(&columns[0].data, items => items.len()), 0)
        });
    let data_start_time = elapsed_times[0];
    elapsed_times.map_inplace(|it| {
        *it = it.saturating_sub(data_start_time);
    });

    let mut data = ffplotter_lib::columns_into_data_matrix(columns)
        .wrap_err("Failed to convert columns into data matrix")?;

    log::debug!("Data: {:?}", data);

    log::debug!("Data matrix shape: {:?}", data.shape());

    generate_composite_columns(&mut headers, &mut data, &cli.expr)
        .wrap_err(eyre!("Failed to generate composite columns"))?;

    log::info!("Generated {} composite columns", cli.expr.len());

    let plot_columns = if cli.plot.is_empty() {
        headers.clone()
    } else {
        cli.plot
    };

    let column_indices = plot_columns
        .iter()
        .map(|it| {
            headers
                .iter()
                .position(|h| h == it)
                .wrap_err_with(|| eyre!("Column '{}' not found in headers [{:?}]", it, &headers))
        })
        .try_collect::<Vec<_>>()?;

    let sample_count = data.dim().0;

    let mut window: PistonWindow = WindowSettings::new("Real Time CPU Usage", [800, 300])
        .samples(4)
        .build()
        .map_err(|e| eyre!("Failed to initialise graphics window {:?}", e))?;

    window.set_max_fps(120);

    let display_start_time = std::time::Instant::now();
    let mut max_range = 100.0f64;
    let mut min_range = -100.0f64;

    log::info!("Plotting {} rows", sample_count);

    while let Some(_) = draw_piston_window(&mut window, |b| {
        let root = b.into_drawing_area();
        root.fill(&WHITE)?;

        let mut cc = ChartBuilder::on(&root)
            .margin(10)
            .caption("IMU data", ("sans-serif", 30))
            .x_label_area_size(40)
            .y_label_area_size(50)
            .build_cartesian_2d(0..sample_count, min_range..max_range)?;

        cc.configure_mesh()
            .x_label_formatter(&|x| format!("{}", data_start_time + elapsed_times[*x as usize]))
            .y_label_formatter(&|y: &f64| format!("{:.2}", *y))
            .x_labels(15)
            .y_labels(5)
            .x_desc("Seconds")
            .y_desc("Acceleration m/s^2")
            .axis_desc_style(("sans-serif", 15))
            .draw()?;

        let elapsed_time = display_start_time.elapsed().as_millis() as u64;

        for &idx in &column_indices {
            // debug!("Elapsed time: {:?} ms / {}", elapsed_times, elapsed_time);
            let series = Array1::from_iter(
                data.slice(s![.., idx])
                    .iter()
                    .cloned()
                    .zip(elapsed_times.iter())
                    .take_while(|&(_, t)| (*t as f64) < elapsed_time as f64 * cli.time_warp)
                    .map(|(val, _)| val)
                    .enumerate(),
            );
            max_range = series.iter().fold(
                series.first().map(|it| it.1).unwrap_or_default(),
                |acc, &(_, val)| acc.max(val),
            );
            max_range = max_range + (max_range * 0.1).abs();
            min_range = series.iter().fold(
                series.first().map(|it| it.1).unwrap_or_default(),
                |acc, &(_, val)| acc.min(val),
            );
            min_range = min_range - (min_range * 0.1).abs();

            cc.draw_series(LineSeries::new(series, &Palette99::pick(idx)))?
                .label(format!("Acceleration (m/s^2)"))
                .legend(move |(x, y)| {
                    Rectangle::new([(x - 5, y - 5), (x + 5, y + 5)], &Palette99::pick(idx))
                });
        }

        cc.configure_series_labels()
            .background_style(&WHITE.mix(0.2))
            .border_style(&BLACK)
            .draw()?;
        Ok(())
    }) {}

    Ok(())
}
