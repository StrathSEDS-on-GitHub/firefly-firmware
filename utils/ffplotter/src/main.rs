#![feature(iterator_try_collect)]

use clap::{self, Parser};
use color_eyre::Section;
use color_eyre::eyre::{self, Context, ContextCompat, eyre};
use csv::WriterBuilder;
use ffplotter_lib::ndarray::{Array1, Axis, s};
use ffplotter_lib::time::Duration;
use ffplotter_lib::{
    Column, ColumnType, Filter, RowToken, RowTokens, SampleType, generate_composite_columns,
    match_column, parse_row_format, time_align_data,
};
use ndarray_csv::Array2Writer as _;
use piston::EventLoop;
use piston::WindowSettings;
use piston_window::PistonWindow;
use plotters::chart::ChartBuilder;
use plotters::prelude::{BitMapBackend, IntoDrawingArea};
use plotters::prelude::{DrawingBackend, Rectangle};
use plotters::series::LineSeries;
use plotters::style::WHITE;
use plotters::style::{BLACK, Color, Palette, Palette99};
use plotters_backend::{BackendColor, BackendCoord, BackendStyle, DrawingErrorKind, rasterizer};
use plotters_piston::{PistonBackend, draw_piston_window};
use std::error::Error;
use std::fs::File;
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
    #[arg(long, value_parser = parse_row_format)]
    row: Vec<RowTokens>,

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

    #[arg(long, short)]
    gif: PathBuf,
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

struct MultiBackend<A: DrawingBackend, B: DrawingBackend> {
    a: A,
    b: B,
}

#[derive(Debug, Clone)]
enum EitherErr<A, B>
where
    A: std::error::Error,
    B: std::error::Error,
{
    A(A),
    B(B),
}

impl<A, B> std::fmt::Display for EitherErr<A, B>
where
    A: std::error::Error,
    B: std::error::Error,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EitherErr::A(a) => std::fmt::Display::fmt(&a, f),
            EitherErr::B(b) => std::fmt::Display::fmt(&b, f),
        }
    }
}

impl<A, B> From<A> for EitherErr<A, B>
where
    A: Error,
    B: Error,
{
    fn from(value: A) -> Self {
        Self::A(value)
    }
}

impl<A, B> std::error::Error for EitherErr<A, B>
where
    A: std::error::Error,
    B: std::error::Error,
{
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            EitherErr::A(a) => a.source(),
            EitherErr::B(b) => b.source(),
        }
    }

    fn description(&self) -> &str {
        "description() is deprecated; use Display"
    }

    fn cause(&self) -> Option<&dyn Error> {
        self.source()
    }
}

impl<A, B> DrawingBackend for MultiBackend<A, B>
where
    A: DrawingBackend,
    B: DrawingBackend,
{
    type ErrorType = EitherErr<A::ErrorType, B::ErrorType>;

    fn get_size(&self) -> (u32, u32) {
        let sz1 = self.a.get_size();
        let sz2 = self.b.get_size();
        assert_eq!(sz1, sz2);

        sz1
    }

    fn ensure_prepared(&mut self) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        self.a.ensure_prepared().unwrap();
        self.b.ensure_prepared().unwrap();
        Ok(())
    }

    fn present(&mut self) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        self.a.present().unwrap();
        self.b.present().unwrap();
        Ok(())
    }

    fn draw_pixel(
        &mut self,
        point: BackendCoord,
        color: BackendColor,
    ) -> Result<(), DrawingErrorKind<Self::ErrorType>> {
        self.a.draw_pixel(point, color).unwrap();
        self.b.draw_pixel(point, color).unwrap();
        Ok(())
    }
}

fn main() -> eyre::Result<()> {
    color_eyre::install()?;
    env_logger::init();

    let cli = Cli::parse();
    let file = std::fs::File::open(&cli.input)
        .wrap_err_with(|| eyre!("Failed to open input file: {}", cli.input.display()))?;

    if cli.row.len() == 0 {
        return Err(eyre!(
            "No row formats provided. Use --row to specify at least one format."
        ));
    }

    // Each Vec<Column> represents a set of columns for a single row format.
    let mut multi_columns: Vec<Vec<Column>> = cli.row.iter().map(|it| it.into()).collect();

    for (line_number, line) in std::io::BufReader::new(file).lines().enumerate() {
        let line = line.wrap_err("Failed to read line")?;

        let mut all_failed = true;
        let mut errors = Vec::<eyre::Report>::new();
        for (format, columns) in cli.row.iter().zip(&mut multi_columns) {
            let Err(e) = ffplotter_lib::parse_row_to_columns(
                &line,
                format,
                cli.filter.as_ref().unwrap_or(&Default::default()),
                columns,
            ) else {
                all_failed = false;
                break;
            };

            errors.push(e.wrap_err(format!("Format {format} did not match")));
        }

        if all_failed {
            let error = errors.into_iter().fold(
                eyre!(
                    "No format strings matched the line at {}:{}",
                    cli.input.file_name().unwrap().display(),
                    line_number + 1
                ),
                |acc, e| acc.with_note(|| format!("{:#}", e)),
            );

            // log::warn!("{:?}", error);
            continue;
        }
    }

    log::info!(
        "Parsed {} rows",
        match_column!(&multi_columns[0][0].data, items => items.len())
    );

    let mut columns = time_align_data(
        &multi_columns
            .iter()
            .map(|it| it.as_slice())
            .collect::<Vec<_>>(),
        Duration::milliseconds(5),
    )?;

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

    let mut data = ffplotter_lib::columns_into_data_matrix(columns.clone())
        .wrap_err("Failed to convert columns into data matrix")?;

    generate_composite_columns(&mut headers, &mut data, &cli.expr)
        .wrap_err(eyre!("Failed to generate composite columns"))?;
    {
        let file = File::create("test.csv")?;
        let mut writer = WriterBuilder::new().has_headers(false).from_writer(file);
        writer.serialize_array2(&data)?;
    }


    log::info!("Generated {} composite columns", cli.expr.len());

    let format_elapsed = |elapsed: u64| {
        let total = elapsed + data_start_time;
        let hour = total / 3600_000;
        let minute = (total % 3600_000) / 60_000;
        let second = (total % 60_000) / 1000;
        let millis = total % 1000;
        format!("{:02}:{:02}:{:02}.{:03}", hour, minute, second, millis)
    };

    log::debug!("Data: {:?}", data);
    log::debug!("Data matrix shape: {:?}", data.shape());
    log::debug!("Headers: {:?}", headers);

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

    let mut window: PistonWindow = WindowSettings::new("ffplotter", [800, 300])
        .samples(4)
        .build()
        .map_err(|e| eyre!("Failed to initialise graphics window {:?}", e))?;

    window.set_max_fps(120);

    let display_start_time = std::time::Instant::now();
    let mut max_range = 100.0f64;
    let mut min_range = -100.0f64;

    log::info!("Plotting {} rows", sample_count);

    let gif_backend = BitMapBackend::gif(cli.gif, (1920, 1080), 1)
        .wrap_err_with(|| eyre!("Unable to create gif backend"))?;

    let root2 = gif_backend.into_drawing_area();
    while let Some(_) = draw_piston_window(&mut window, |b| {
        let root1 = b.into_drawing_area();

        root1.fill(&WHITE)?;
        root2.fill(&WHITE)?;

        let mut cc1 = ChartBuilder::on(&root1)
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(50)
            .build_cartesian_2d(0..sample_count, min_range..max_range)?;
        let mut cc2 = ChartBuilder::on(&root2)
            .margin(10)
            .x_label_area_size(40)
            .y_label_area_size(50)
            .build_cartesian_2d(0..sample_count, min_range..max_range)?;

        cc1.configure_mesh()
            .x_label_formatter(&|x| format_elapsed(*x as u64))
            .y_label_formatter(&|y: &f64| format!("{:.2}", *y))
            .x_labels(15)
            .y_labels(5)
            .axis_desc_style(("sans-serif", 15))
            .draw()?;
        cc2.configure_mesh()
            .x_label_formatter(&|x| format_elapsed(*x as u64))
            .y_label_formatter(&|y: &f64| format!("{:.2}", *y))
            .x_labels(15)
            .y_labels(5)
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

            let series_len = series.len();
            cc1.draw_series(LineSeries::new(series.clone(), &Palette99::pick(idx)))?;
            cc2.draw_series(LineSeries::new(series, &Palette99::pick(idx)))?;
            if series_len == elapsed_times.len() {
            }
        }

        cc1.configure_series_labels()
            .background_style(&WHITE.mix(0.2))
            .border_style(&BLACK)
            .draw()?;
        cc2.configure_series_labels()
            .background_style(&WHITE.mix(0.2))
            .border_style(&BLACK)
            .draw()?;
        root2.present().unwrap();

        Ok(())
    }) {}
    Ok(())
}
