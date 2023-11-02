use heapless::Vec;
use serde::Deserialize;
use serde_json_core;
use heapless::String;

static mut CONFIG: Option<Config> = None;

#[derive(Debug, Deserialize)]

// Instead of staging, we are looking at states of the board. Defining triggers based on sensors triggers a different state. each state requires some sort of sensor
// combination to validate the next state can be performed. heapless vec for a list of structs.

// query language that when completed gui side, converts the query into bytecode (bytecode interpreter) that is then run on the board (?)

pub enum Sensor {
    Pressure,
    Breakwire,
    Acceleration,
    Velocity,
    CurrentStageTimeElapsed,
    // need some stuff in here
}

#[derive(Debug, Deserialize)]

pub struct Action {
    // need some stuff in here
}

#[derive(Debug, Deserialize)]

pub enum Triggers {

    GreaterThan (Sensor, f32), // (value, threshold)
    

    // LessThan (Sensor, f32),
    // EqualTo (Sensor, f32),


    // pressure, f32
    // breakwire_broken: bool,
    // acceleration: f32,
    // velocity: f32,
    // current_stage_time_elapsed: f32,
    // main_deployment_height: f32,

    // need some stuff in here
}

#[derive(Debug, Deserialize)]

pub struct PressureConfig {
    // need some stuff in here
}

#[derive(Debug, Deserialize)]

pub struct TransmissionProfile {
    // need some stuff in here
}

#[derive(Debug, Deserialize)]

pub struct Stage {
    pub stage_name: String<8>,
    pub triggers_needed: f32,
    pub actions: Vec<Action, 10>,
    pub triggers: Vec<Triggers, 10>,

    pub pressure_config: PressureConfig,
    pub transmission_profile: TransmissionProfile,
}

#[derive(Debug, Deserialize)]
pub struct Config {
    // this needs beefing out, here are some example vars to put in...
    pub stage: Vec<Stage, 10>,
    /*
    Each stage should have the following options to configure:

    Transmission profile - TDM settings, etc
    Systems to activate / use
    Polling + transmission rates configurable / system. I.e. lots of GPS when landed, lots of pressure+IMU+GPS when in flight.
    Backup and main stage transition triggers
    The radio settings should be set once, and should not change with stages.

    LORA Settings: Channel, Bandwidth, transmit power etc.
    
    */
}


impl Config {
    pub fn build(file: Vec<u8, 1024>) {
        let (config, _): (Config, _) = serde_json_core::from_slice(&file).unwrap();

        // Safety: Only ran once on startup, so reads will not alias with the write
        unsafe {
            CONFIG = Some(config);
        };
    }

    pub fn get() -> &'static Config {
        unsafe { CONFIG.as_ref().unwrap() }
    }
}