use std::{thread::sleep, time::Duration, rc::Rc};

use std::collections::HashSet;

use esp_idf_hal::{
    gpio::{self, InputPin, Output, OutputPin, PinDriver},
    peripheral::Peripheral,
    uart::{self, Uart, UartDriver},
    units::Hertz,
};

use serde::{Serialize};

use crate::logger::Logger;

const PREAMBLE: u8 = 0xAA;
const PROLOGUE: u8 = 0x55;

const FROM_MASTER: u8 = 0x80;

const TO_MASTER: u8 = 0x80;

const COMMAND_QERRY: u8 = 0xC0;
const COMMAND_SET: u8 = 0xC3;
const COMMAND_LOCK: u8 = 0xCC;
const COMMAND_UNLOCK: u8 = 0xCD;

const OP_MODE_OFF: u8 = 0x00;
const OP_MODE_AUTO: u8 = 0x80;
const OP_MODE_FAN: u8 = 0x81;
const OP_MODE_DRY: u8 = 0x82;
const OP_MODE_HEAT: u8 = 0x84;
const OP_MODE_COOL: u8 = 0x88;

const FAN_MODE_AUTO: u8 = 0x80;
const FAN_MODE_HIGH: u8 = 0x01;
const FAN_MODE_MEDIUM: u8 = 0x02;
const FAN_MODE_LOW: u8 = 0x04;

#[derive(Debug, Default, Serialize)]
enum OpMode {
    OFF = 0,
    AUTO,
    COOL,
    DRY,
    HEAT,
    FAN,
    #[default]
    UNKOWN,
}

#[derive(Debug, Default, Serialize)]
enum FanMode {
    AUTO = 0,
    HIGH,
    MEDIUM,
    LOW,
    #[default]
    UNKNOWN,
}

#[derive(Debug, PartialEq, Eq, Hash, Serialize)]
enum AuxMode {
    TURBO,
    SILENT,
    SWING,
    VENT,
    UNKNOWN(u8),
}



#[derive(Debug, Default, Serialize)]
pub struct State {
    capabilities: u8,
    op_mode: OpMode,
    fan_mode: FanMode,
    target_temp: u8,
    t1_temp: i16,
    t2a_temp: i16,
    t2b_temp: i16,
    t3_temp: i16,
    current: u8,
    timer_start: Duration,
    timer_stop: Duration,
    mode_flags: u8,
    modes: HashSet<AuxMode>,
    op_flags: u8,
    error_flags: u16,
    protect_flags: u16,
    ccm_com_error_flags: u8,
    ac_unavailable: bool,
    unknown1: u8,
    unknown2: u8,
    unknown3: u8,
}

pub trait StateListener{
    fn listen(&self, state: &State);
}

pub struct MideaAC<'a, C: OutputPin> {
    serial: UartDriver<'a>,
    logger: Rc<dyn Logger>,
    com_control: PinDriver<'a, C, Output>,
    state: State,
    state_listener : Box<dyn StateListener>
}

impl<'a, C: OutputPin + 'a> MideaAC<'a, C> {
    pub fn new<T: OutputPin, R: InputPin, U: Uart>(
        uart: impl Peripheral<P = U> + 'a,
        logger: Rc<dyn Logger >,
        tx: T,
        rx: R,
        com_control_pin: C,
        state_listener : Box<dyn StateListener>
    ) -> Self {
        let config_uart = uart::config::Config::default()
            .source_clock(uart::config::SourceClock::APB)
            .baudrate(Hertz(4800));

        let serial = uart::UartDriver::new(
            uart,
            tx,
            rx,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio1>::None,
            &config_uart,
        )
        .unwrap();

        let a = PinDriver::output(com_control_pin).unwrap();
        Self {
            logger,
            serial,
            com_control: a,
            state: State {
                ac_unavailable: true,
                ..Default::default()
            },
            state_listener
        }
    }

    pub fn send_state(&mut self,  new_temp: u8){
        let mut data: [u8; 16] = [0; 16];
        data[0] = PREAMBLE;
        data[1] = COMMAND_SET;
        //SlaveId
        data[2] = 0;
        //MasterId
        data[3] = 0;
        data[4] = FROM_MASTER;
        //MasterId
        data[5] = 0;
        data[6] = match self.state.op_mode {
            OpMode::OFF =>  OP_MODE_OFF,
            OpMode::AUTO =>  OP_MODE_AUTO,
            OpMode::FAN  => OP_MODE_FAN,
            OpMode::DRY =>  OP_MODE_DRY,
            OpMode::HEAT =>  OP_MODE_HEAT,
            OpMode::COOL =>  OP_MODE_COOL,
            _ => OP_MODE_OFF,
        };
        data[7] = match self.state.fan_mode {
            FanMode::AUTO =>  FAN_MODE_AUTO,
            FanMode::HIGH =>   FAN_MODE_HIGH,
            FanMode::MEDIUM =>   FAN_MODE_MEDIUM,
            FanMode::LOW =>   FAN_MODE_LOW,
                _  =>    FAN_MODE_AUTO     
        };
        data[8] = new_temp;
        data[9] = {
            let mut flags = 0;
            if self.state.modes.contains(&AuxMode::TURBO){
                flags |= 1<<1;
            }
            if self.state.modes.contains(&AuxMode::SILENT){
                flags |= 1<<0;
            }
            if self.state.modes.contains(&AuxMode::SWING){
                flags |= 1<<2;
            }
            if self.state.modes.contains(&AuxMode::VENT){
                flags |= 0x88;
            }
            flags
        };
         //set timer start
        data[10] = 0;
        //set timer stop
        data[11] = 0;
        //unknown -> 0
        data[12] = 0;
        data[13] = 0xFF - data[1];
        data[15] = PROLOGUE;
        data[14] = calc_check_sum(&data);
        self.write(&data);

    }

    pub fn lock_ac(&mut self){
        let mut data: [u8; 16] = [0; 16];
        data[0] = PREAMBLE;
        data[1] = COMMAND_LOCK;
        //SlaveId
        data[2] = 0;
        //MasterId
        data[3] = 0;
        data[4] = FROM_MASTER;
        //MasterId
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0xFF - data[1];
        data[15] = PROLOGUE;
        data[14] = calc_check_sum(&data);
        self.write(&data);
    }

    pub fn unlock_ac(&mut self){
        let mut data: [u8; 16] = [0; 16];
        data[0] = PREAMBLE;
        data[1] = COMMAND_UNLOCK;
        //SlaveId
        data[2] = 0;
        //MasterId
        data[3] = 0;
        data[4] = FROM_MASTER;
        //MasterId
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0xFF - data[1];
        data[15] = PROLOGUE;
        data[14] = calc_check_sum(&data);
        self.write(&data);
    }

    pub fn request_status(&mut self) {
        let mut data: [u8; 16] = [0; 16];
        data[0] = PREAMBLE;
        data[1] = COMMAND_QERRY;
        //SlaveId
        data[2] = 0;
        //MasterId
        data[3] = 0;
        data[4] = FROM_MASTER;
        //MasterId
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 0;
        data[11] = 0;
        data[12] = 0;
        data[13] = 0xFF - data[1];
        data[15] = PROLOGUE;
        data[14] = calc_check_sum(&data);
        self.write(&data);
    }

    fn write(&mut self, data: &[u8]) {
        self.com_control.set_high().unwrap();
        self.serial.write(&data).unwrap();

        sleep(Duration::from_millis(40));
        self.com_control.set_low().unwrap();

        self.loop_serial();
    }

    pub fn loop_serial(&mut self) {
        //  if(!_s) return;
        if let Ok(1..) = self.serial.remaining_read() {
            let mut data: Vec<u8> = vec![0; 32];
            let read_status = self.serial.read(&mut data, 100);
            match read_status {
                Ok(size @ 32) => {
                    self.log_and_mttq(format!("Raw Data {:?}", data));
                    let preamble = data[0];
                    if PREAMBLE != preamble {
                        self.log_and_mttq(format!("bad preamble {} ", preamble));
                        return;
                    }

                    let prologue = data[0x1F];
                    if PROLOGUE != prologue {
                        self.log_and_mttq(format!("bad prologue {} ", prologue));
                        return;
                    }

                    let dest = data[0x2];
                    if FROM_MASTER != dest {
                        self.log_and_mttq(format!("not for master. dest: {} ", dest));
                        return;
                    }
                    let checksum = calc_check_sum(&data[0..size - 1]);
                    if checksum != data[size - 1] {
                        self.log_and_mttq(format!(
                            "bad checksum {} != {}",
                            checksum,
                            data[size - 1]
                        ));
                        return;
                    }
                    self.state.capabilities = data[7];

                    self.state.op_mode = match data[8] {
                        OP_MODE_OFF => OpMode::OFF,
                        OP_MODE_AUTO => OpMode::AUTO,
                        OP_MODE_FAN => OpMode::FAN,
                        OP_MODE_DRY => OpMode::DRY,
                        OP_MODE_HEAT => OpMode::HEAT,
                        OP_MODE_COOL => OpMode::COOL,
                        _ => OpMode::UNKOWN,
                    };
                    self.state.fan_mode = match data[9] {
                        FAN_MODE_HIGH => FanMode::HIGH,
                        FAN_MODE_MEDIUM => FanMode::MEDIUM,
                        FAN_MODE_LOW => FanMode::LOW,
                        FAN_MODE_AUTO => FanMode::AUTO,
                        _ => FanMode::UNKNOWN,
                    };

                    self.state.target_temp = data[0x0A];
                    self.state.t1_temp = ((data[0x0B] as i16) - 0x30) / 2;
                    self.state.t2a_temp = ((data[0x0C] as i16) - 0x30) / 2;
                    self.state.t2b_temp = ((data[0x0D] as i16) - 0x30) / 2;
                    self.state.t3_temp = ((data[0x0E] as i16) - 0x30) / 2;
                    self.state.current = data[0x0F];
                    self.state.unknown1 = data[0x10];
                    self.state.timer_start = calc_timer_delay(data[0x11]);
                    self.state.timer_stop = calc_timer_delay(data[0x12]);
                    self.state.modes = {
                        let mut value = data[0x14];
                        let mut modes = HashSet::new();
                        if(value&0x02) == 0x02 {
                            modes.insert(AuxMode::TURBO);
                            value ^= 0x02;
                        }
                        if(value&0x01) == 0x01 {
                            modes.insert(AuxMode::SILENT);
                            value ^= 0x01;
                        }
                        if(value&0x04) == 0x04 {
                            modes.insert(AuxMode::SWING);
                            value ^= 0x04;
                        }
                        if(value&0x88) == 0x88 {
                            modes.insert(AuxMode::VENT);
                            value ^= 0x88;
                        }
                        if value != 0 {
                            modes.insert(AuxMode::UNKNOWN(value));
                        }
                        modes
                    };
                    self.state.mode_flags = data[0x14];
                    self.state.op_flags = data[0x15];
                    self.state.error_flags =
                        ((data[0x16] as u16) << 0) | ((data[0x17] as u16) << 8);
                    self.state.protect_flags =
                        ((data[0x18] as u16) << 0) | ((data[0x19] as u16) << 8);
                    self.state.ccm_com_error_flags = data[0x1A];
                    self.state.unknown2 = data[0x1B];
                    self.state.unknown3 = data[0x1C];
                    self.state.ac_unavailable = false;

                    self.log_and_mttq(format!("State {:?}", self.state));
                    self.state_listener.listen(&self.state);
                }
                Ok(0) => {
                    self.log_and_mttq(format!("timeout"));
                    self.state.ac_unavailable = true;
                }
                Ok(size) => {
                    self.log_and_mttq(format!(
                        "Unexpected size read : {} , flusing the buffer. ",
                        size
                    ));
                    self.serial.flush_read().unwrap();
                    self.state.ac_unavailable = true;
                }
                Err(e) => {
                    self.log_and_mttq(format!("Error reading{:?}", e));
                    self.state.ac_unavailable = true;
                }
            }
        }
    }

    pub fn log_and_mttq(&mut self, text: String) {
        self.logger.log(&text);
    }
}

fn calc_check_sum(data: &[u8]) -> u8 {
    let mut check_sum: u16 = 0;
    for i in 0..data.len() {
        check_sum = check_sum.wrapping_add(data[i].into());
    }
    0xFF - (check_sum as u8 & 0xFF)
}

fn calc_timer_delay(time: u8) -> Duration {
    let mut minutes = 0;
    if 0x40 == (time & 0x40) {
        //16 hours
        minutes += 960;
    }
    if 0x20 == (time & 0x20) {
        //8 hours
        minutes += 480;
    }
    if 0x10 == (time & 0x10) {
        //4 hours
        minutes += 240;
    }
    if 0x08 == (time & 0x08) {
        minutes += 120;
    }
    if 0x04 == (time & 0x04) {
        minutes += 60;
    }
    if 0x02 == (time & 0x02) {
        minutes += 30;
    }
    if 0x01 == (time & 0x01) {
        minutes += 15;
    }
    Duration::from_secs(minutes * 60)
}
