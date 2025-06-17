use std::{
    collections::HashMap,
    ops::{Deref, DerefMut, Index as _},
};

use color_eyre::eyre::{self, Context, eyre};
use evalexpr_jit::Equation;
pub mod ndarray {
    pub use ndarray::prelude::*;
}

pub mod time {
    pub use time::*;
    pub mod macros {
        pub use time::macros::*;
    }
}

use ndarray::*;
use ::time::{macros::format_description, Time};

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
    .wrap_err("Failed to build data matrix")?.reversed_axes())
}

pub fn parse_row(
    mut row: &str,
    format: &RowTokens,
    filters: &Filter,
) -> eyre::Result<Option<Vec<Sample>>> {
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
                        return Err(eyre!("Expected literal '{}' but found '{}'", expected, c));
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
                            return Ok(None); // Filtered out
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

    Ok(Some(samples))
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
                let Some(':') = chars.next() else {
                    // No sample type specified, treat as literal
                    tokens.extend(name.chars().map(RowToken::Literal));
                    continue;
                };
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
        .collect();

    let mut new_columns = Array2::zeros((data.shape()[0], expressions.len()));

    for (i, expr) in expressions.iter().enumerate() {
        let expr = expr.as_ref().to_string();
        let eq = Equation::from_var_map(expr.clone(), &var_map)
            .wrap_err_with(|| eyre!("Failed to parse expression '{}' ", &expr))?;

        headers.push(format!("expr{}", i));

        for mut new_col in new_columns.axis_iter_mut(Axis(1)) {
            let calculated = data.axis_iter(Axis(0)).map(|it| { 
                eq.eval(&it.into_owned() ) 
            });
            for (l, r) in new_col.iter_mut().zip(calculated) {
                *l = r?;
            }
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

#[derive(Debug, Clone)]
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
