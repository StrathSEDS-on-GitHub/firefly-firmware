use std::{
    collections::HashMap,
    fmt::Display,
    ops::{Deref, DerefMut, Index},
};

use color_eyre::eyre::{self, Context, eyre};
pub mod ndarray {
    pub use ndarray::prelude::*;
}

pub mod time {
    pub use time::*;
    pub mod macros {
        pub use time::macros::*;
    }
}

use serde::{Deserialize, Serialize};

use ::time::{Time, macros::format_description};
use evalexpr::{ContextWithMutableFunctions, ContextWithMutableVariables, Function, HashMapContext};
use ndarray::*;

pub fn columns_into_data_matrix(columns: Vec<Column>) -> eyre::Result<Array2<f64>> {
    Ok(Array2::from_shape_vec(
        (
            columns
                .iter()
                .filter(|&it| matches!(it.data, ColumnType::Float(_)))
                .count(),
            match_column!(&columns[0].data, items => items.len()),
        ),
        columns
            .into_iter()
            .filter_map(|it| match it.data {
                ColumnType::Float(items) => Some(items),
                _ => None,
            })
            .flat_map(Vec::into_iter)
            .collect(),
    )
    .wrap_err("Failed to build data matrix")?
    .reversed_axes())
}

pub fn time_align_data(
    multi_columns: &[&[Column]],
    sample_rate: time::Duration,
) -> eyre::Result<Vec<Column>> {
    let multi_times = multi_columns
        .into_iter()
        .map(|it| {
            it.into_iter()
                .filter_map(|col| match &col.data {
                    ColumnType::Time(items) => Some(items.as_slice()),
                    _ => None,
                })
                .next()
                .ok_or(eyre!("No time data found in columns"))
        })
        .collect::<eyre::Result<Vec<_>>>()?;

    let mut current_time = multi_times
        .iter()
        .map(|it| it.first().cloned().ok_or(eyre!("Empty time column")))
        .collect::<eyre::Result<Vec<_>>>()?
        .iter()
        .max()
        .cloned()
        .unwrap();

    let last_time = multi_times
        .iter()
        .map(|it| it.last().cloned().ok_or(eyre!("Empty time column")))
        .collect::<eyre::Result<Vec<_>>>()?
        .iter()
        .min()
        .cloned()
        .unwrap();

    let num_sensors = multi_columns.len();
    let mut last_time_indices: Vec<usize> = std::iter::repeat_n(0, num_sensors).collect();
    let mut aligned_columns: Vec<Column> = Vec::new();
    for column in multi_columns
        .iter()
        .flat_map(|cols| cols.iter())
        .filter(|col| !matches!(col.data, ColumnType::Time(_)))
    {
        aligned_columns.push(Column {
            header: column.header.clone(),
            data: match column.data {
                ColumnType::Float(_) => ColumnType::Float(Vec::new()),
                ColumnType::String(_) => ColumnType::String(Vec::new()),
                _ => unreachable!(),
            },
        });
    }

    let mut aligned_times = Vec::new();

    while current_time < last_time {
        aligned_times.push(current_time);
        let mut column_idx = 0;
        for ((times, lt_idx), columns) in multi_times
            .iter()
            .zip(last_time_indices.iter_mut())
            .zip(multi_columns.iter())
        {
            while *lt_idx < times.len() - 1 && current_time > times[*lt_idx] {
                *lt_idx += 1;
            }

            for column in columns
                .iter()
                .filter(|col| !matches!(col.data, ColumnType::Time(_)))
            {
                match_column!(
                    &mut aligned_columns[column_idx].data,
                    items => items.push(column.data.try_index(*lt_idx)?)
                );
                column_idx += 1;
            }
        }

        current_time += sample_rate;
    }

    aligned_columns.insert(
        0,
        Column {
            header: "Time".to_string(),
            data: ColumnType::Time(aligned_times),
        },
    );

    Ok(aligned_columns)
}

pub fn parse_row_to_columns(
    mut row: &str,
    format: &RowTokens,
    filters: &Filter,
    columns: &mut Vec<Column>,
) -> eyre::Result<()> {
    let mut samples = Vec::new();
    let mut format_iter = format.iter().peekable();
    let mut row_iter = row.char_indices().peekable();

    loop {
        match (format_iter.peek(), row_iter.peek()) {
            (_, Some((_, ' '))) => {
                // Skip spaces in the format
                row_iter.next();
                continue;
            }
            (Some(_), Some(&(i, c))) => match format_iter.next().unwrap() {
                RowToken::Literal(expected) => {
                    if row_iter.next().unwrap().1 != *expected {
                        return Err(eyre!(
                            "Expected literal '{}' but found '{}' at {}",
                            expected,
                            c,
                            i
                        ));
                    }
                }
                RowToken::Sample { name, typ } => match typ {
                    SampleType::Float => {
                        let (rem, f) = nom::number::complete::float::<_, nom::error::Error<&str>>(
                            row.index(i..),
                        )
                        .map_err(|it| it.map_input(ToString::to_string))?;

                        row_iter = rem.char_indices().peekable();
                        row = rem;
                        samples.push(Sample::Float(f as f64));
                    }
                    SampleType::String => {
                        let next_literal = row_iter
                            .clone()
                            .find(|&(_, c)| !c.is_alphanumeric() && c != '_')
                            .map(|(i, _)| i)
                            .unwrap_or(row.len());

                        let value = &row[i..next_literal];
                        while let Some(_) = row_iter.next_if(|(i, _)| *i < next_literal) {}

                        if filters.contains_key(name) && filters[name] != value {
                            return Ok(()); // Filtered out
                        }
                        samples.push(Sample::String(value.to_string()));
                    }
                    SampleType::Timestamp => {
                        let time = ::time::Time::parse(
                            &row[i..i + 12],
                            format_description!("[hour]:[minute]:[second].[subsecond]"),
                        )?;
                        for _ in row_iter.by_ref().take(12) {} // FIXME: ascii assumed
                        samples.push(Sample::Timestamp(time));
                    }
                },
            },
            (Some(token), None) => {
                return Err(eyre!(
                    "Row ended prematurely, expected more characters for token: {:?}",
                    token
                ));
            }
            (None, _) => break,
        }
    }
    columns.iter_mut().zip(samples).for_each(|(col, sample)| {
        match_column!(
            &mut col.data,
            items => items.push(sample.try_into().expect("Sample type mismatch")
        ))
    });
    Ok(())
}

#[derive(Debug, Clone)]
pub struct RowTokens(Vec<RowToken>);
impl Deref for RowTokens {
    type Target = Vec<RowToken>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl DerefMut for RowTokens {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl From<&RowTokens> for Vec<Column> {
    fn from(value: &RowTokens) -> Self {
        value
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
            .collect()
    }
}

impl Display for RowTokens {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for tok in &self.0 {
            match tok {
                RowToken::Literal(c) => write!(f, "{}", c)?,
                RowToken::Sample { name, typ } => {
                    write!(
                        f,
                        "{}:{}",
                        name,
                        match typ {
                            SampleType::Float => 'f',
                            SampleType::String => 's',
                            SampleType::Timestamp => 't',
                        }
                    )?;
                }
            }
        }
        Ok(())
    }
}

pub fn parse_row_format(input: &str) -> eyre::Result<RowTokens> {
    let mut tokens = Vec::new();
    let mut chars = input.chars().peekable();

    while let Some(c) = chars.next() {
        match c {
            ' ' => {
                continue;
            }
            'A'..='Z' | 'a'..='z' => {
                let mut name = String::new();
                name.push(c);
                while let Some(c) = chars.peek()
                    && (c.is_alphanumeric() || *c == '_')
                {
                    name.push(chars.next().unwrap());
                }
                let Some(':') = chars.peek() else {
                    // No sample type specified, treat as literal
                    tokens.extend(name.chars().map(RowToken::Literal));
                    continue;
                };
                let _ = chars.next(); // Consume ':'
                let sample_type = match chars.next() {
                    Some('f') => SampleType::Float,
                    Some('s') => SampleType::String,
                    Some('t') => SampleType::Timestamp,
                    Some(c) => return Err(eyre!("Unexpected character '{}' after sample type", c)),
                    None => return Err(eyre!("Expected sample type after ':'")),
                };
                tokens.push(RowToken::Sample {
                    name,
                    typ: sample_type,
                });
            }
            c => {
                tokens.push(RowToken::Literal(c));
            }
        }
    }

    Ok(RowTokens(tokens))
}

pub fn generate_composite_columns(
    headers: &mut Vec<String>,
    data: &mut Array2<f64>,
    expressions: &[impl AsRef<str>],
) -> eyre::Result<()> {
    let var_map = headers
        .iter()
        .enumerate()
        .map(|(a, b)| (b.to_string(), a as u32))
        .collect::<Vec<_>>();

    let mut new_columns = Array2::zeros((data.shape()[0], expressions.len()));
    use evalexpr::Context;

    for ((i, expr), mut new_col) in expressions
        .iter()
        .enumerate()
        .zip(new_columns.axis_iter_mut(Axis(1)))
    {
        let expr = expr.as_ref().to_string();

        headers.push(format!("expr{}", i));

        println!("new col {:?}", new_col);
        let calculated = data.axis_iter(Axis(0)).map(|it| {
            let mut ctxt = HashMapContext::new();
            ctxt.set_builtin_functions_disabled(false).unwrap();
            ctxt.set_function("powf".to_owned(), Function::new(| x| {
                match x {
                    evalexpr::Value::Tuple(values) => {
                        if values.len() == 2 {
                            Ok(evalexpr::Value::Float(values[0].as_float().unwrap().powf(values[1].as_float().unwrap())))
                        } else {
                            Ok(evalexpr::Value::Float(0.))
                        }
                    },
                    _ => panic!()
                }
            }));
            for (name, idx) in &var_map {
                ctxt.set_value(name.to_string(), evalexpr::Value::Float(it[*idx as usize]))?;
            }
            evalexpr::eval_float_with_context(&expr, &ctxt)
        });
        for (l, r) in new_col.iter_mut().zip(calculated) {
            *l = r.wrap_err_with(|| eyre!("Failed to evaluate expression: {}", expr))?;
        }
    }

    data.append(Axis(1), new_columns.view())?;

    Ok(())
}

#[derive(Debug, Clone)]
pub enum SampleType {
    Float,
    String,
    Timestamp,
}

#[derive(Debug, Clone)]
pub enum RowToken {
    Literal(char),
    Sample { name: String, typ: SampleType },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Sample {
    Float(f64),
    String(String),
    Timestamp(time::Time),
}

impl TryInto<String> for Sample {
    type Error = eyre::Error;

    fn try_into(self) -> Result<String, Self::Error> {
        match self {
            Sample::String(s) => Ok(s),
            _ => Err(eyre!("Not a string sample")),
        }
    }
}

impl TryInto<f64> for Sample {
    type Error = eyre::Error;

    fn try_into(self) -> Result<f64, Self::Error> {
        match self {
            Sample::Float(f) => Ok(f),
            _ => Err(eyre!("Not a float sample")),
        }
    }
}

impl TryInto<Time> for Sample {
    type Error = eyre::Error;

    fn try_into(self) -> Result<Time, Self::Error> {
        match self {
            Sample::Timestamp(t) => Ok(t),
            _ => Err(eyre!("Not a timestamp sample")),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Column {
    pub header: String,
    pub data: ColumnType,
}

#[derive(Debug, Clone)]
pub enum ColumnType {
    Float(Vec<f64>),
    String(Vec<String>),
    Time(Vec<Time>),
}

pub trait TryIndex<T> {
    fn try_index(&self, index: usize) -> Result<T, eyre::Error>;
}

impl TryIndex<f64> for ColumnType {
    fn try_index(&self, index: usize) -> Result<f64, eyre::Error> {
        match self {
            ColumnType::Float(items) => items
                .get(index)
                .cloned()
                .ok_or_else(|| eyre!("Index out of bounds for Float column")),
            _ => Err(eyre!("Column type mismatch, expected Float")),
        }
    }
}
impl TryIndex<String> for ColumnType {
    fn try_index(&self, index: usize) -> Result<String, eyre::Error> {
        match self {
            ColumnType::String(items) => items
                .get(index)
                .cloned()
                .ok_or_else(|| eyre!("Index out of bounds for String column")),
            _ => Err(eyre!("Column type mismatch, expected String")),
        }
    }
}
impl TryIndex<Time> for ColumnType {
    fn try_index(&self, index: usize) -> Result<Time, eyre::Error> {
        match self {
            ColumnType::Time(items) => items
                .get(index)
                .cloned()
                .ok_or_else(|| eyre!("Index out of bounds for Time column")),
            _ => Err(eyre!("Column type mismatch, expected Time")),
        }
    }
}

#[macro_export]
macro_rules! match_column {
    ($disc:expr, $binder:ident => $variant:expr) => {
        match $disc {
            ColumnType::Float($binder) => $variant,
            ColumnType::String($binder) => $variant,
            ColumnType::Time($binder) => $variant,
        }
    };
}

pub type Columns = Vec<Column>;
pub type Filter = HashMap<String, String>;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {}
}
