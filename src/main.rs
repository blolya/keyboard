#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_reset as _;

use core::clone;

use peris::peripherals::{
    clock,
    ports::{
        Port,
        PortNum,
        PortMode,
        OutputConfig,
        InputConfig,
        MaxSpeed,
    }
};
use peris::core::{
    rcc::Rcc,
    usb::Usb,
    nvic::Nvic,
    pma::Pma,
    afio::Afio,
    gpio::{
        gpioa::Gpioa,
        gpiob::Gpiob,
        gpioc::Gpioc,
    }
};
use stm32f1::stm32f103::interrupt;


enum DeviceStatus {
    Default,
    GetReportDescriptor,
    SetAddress,
}

static mut DEVICE_STATUS: DeviceStatus = DeviceStatus::Default;
static mut DEVICE_ADDRESS: u8 = 0x00;

struct Led<'a> {
    port: Port<'a>,
    toggle_counter: u32,
}
impl<'a> Led<'a> {
    fn new(port: Port) -> Led {
        Led {
            port,
            toggle_counter: 0,
        }
    }

    fn turn_on(&self) {
        self.port.set_low();   
    }
    fn turn_off(&self) {
        self.port.set_high();   
    }

    fn toggle(&mut self) {
        if self.toggle_counter > 100 {
            self.toggle_counter = 0;
            self.port.toggle();
        } else {
            self.toggle_counter += 1;
        }
    }
}
struct Keyboard<'a> {
    key_matrix: KeyMatrix<'a>,
    keys: [Key; 6],
    leds: [Led<'a>; 3],
    mode: KeyboardMode,
}
impl<'a> Keyboard<'a> {
    fn new( key_matrix: KeyMatrix<'a>, keys: [Key; 6], leds: [Led<'a>; 3] ) -> Keyboard<'a> {

        Keyboard {
            key_matrix,
            keys,
            leds,
            mode: KeyboardMode::Normal,
        }
    }

    fn reset_keys(&mut self) {
        self.keys = [ 
            Key::new(0x01, KeyType::Modifier), 
            Key::new(0x04, KeyType::Modifier), 
            Key::new(0x4c, KeyType::Default), 
            Key::new(0x2a, KeyType::Default), 
            Key::new(0x10, KeyType::Modifier), 
            Key::new(0x40, KeyType::Modifier), 
        ];
    }

    fn scan(&mut self) {

        let safety_cycles_num = 10;

        let mut keys_scan = [0; 6];
        let mut keys_safety_scan = [0; 6]; 

        for _ in 0 .. safety_cycles_num {
            self.key_matrix.scan(&mut keys_scan);
            for (key_index, key_status) in keys_scan.iter().enumerate() {
                if *key_status == 1 {
                    keys_safety_scan[key_index] += 1;
                }
            };
        }

        for (key_index, key_safety_scan) in keys_safety_scan.iter().enumerate() {
            if *key_safety_scan > 7 {
                self.keys[key_index].apply_scan(true);
            } else {
                self.keys[key_index].apply_scan(false);
            }
        }
    }

    fn get_report(&self) -> [u8; 8] {
        let mut report: [u8; 8] = [0; 8];
        let mut default_key_index: usize = 0;
    
    
        for key in self.keys.iter() {
    
            if key.is_pressed {
                match key.key_type {
                    KeyType::Default => {
                        report[default_key_index + 2] = key.key_code;
                        default_key_index += 1;
                    },
                    KeyType::Modifier => {
                        report[0] |= key.key_code;
                    }
                }
            }
    
        }
    
        report
    }
}


enum KeyType {
    Modifier,
    Default,
}
struct Key {
    is_pressed: bool,
    is_released: bool,
    key_code: u8,
    key_type: KeyType,
}
impl Key {
    fn new(key_code: u8, key_type: KeyType) -> Key {
        Key {
            is_pressed: false,
            is_released: false,
            key_code,
            key_type,
        }
    }
    fn apply_scan(&mut self, scan: bool) {
        self.is_released = self.is_pressed & !scan; 
        self.is_pressed = scan;
    }
}


enum KeyboardMode {
    Normal,
    Idle,
    KeySetup(KeySetupMode),
}
enum KeySetupMode {
    SelectKey,
    SelectKeyType(usize),
    ReadKeyCode(usize, usize),
}
impl Clone for KeySetupMode {
    fn clone(&self) -> KeySetupMode {
        match self {
            KeySetupMode::SelectKey => KeySetupMode::SelectKey,
            KeySetupMode::SelectKeyType(a) => KeySetupMode::SelectKeyType(*a),
            KeySetupMode::ReadKeyCode(a, b) => KeySetupMode::ReadKeyCode(*a, *b),
        }
    }
}
impl Copy for KeySetupMode {}
struct KeyMatrix<'a> {
    columns: [Port<'a>; 3],
    rows: [Port<'a>; 2],
}
impl<'a> KeyMatrix<'a> {
    fn new(columns: [Port<'a>; 3], rows: [Port<'a>; 2]) -> KeyMatrix<'a> {
        KeyMatrix {
            columns,
            rows,
        }
    }

    fn scan(&self, keys_scan: &mut [u8] ) {

        let row_len = self.rows.len();

        for (column_index, column) in self.columns.iter().enumerate() {

            column.set_mode( PortMode::Output( OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)) );
            column.set_high();

            for (row_index, row) in self.rows.iter().enumerate() {

                if row.get_input() == 1 {
                    keys_scan[ column_index * row_len + row_index ] = 1;

                } else {
                    keys_scan[ column_index * row_len + row_index ] = 0;
                }
            }

            column.set_low();
            column.set_mode( PortMode::Input( InputConfig::Floating ) );
    
        }
    }
}

#[entry]
fn main() -> ! {
    clock::init();
    init_usb();

    Afio::new().disable_jtag_enable_ports();


    let gpioa = Gpioa::new();
    let matrix_columns = [
        Port::new(PortNum::P5, PortMode::Input( InputConfig::Floating ), &gpioa),
        Port::new(PortNum::P4, PortMode::Input( InputConfig::Floating ), &gpioa),
        Port::new(PortNum::P3, PortMode::Input( InputConfig::Floating ), &gpioa),
    ];
    let matrix_rows = [
        Port::new(PortNum::P6, PortMode::Input( InputConfig::PullDown ), &gpioa),
        Port::new(PortNum::P7, PortMode::Input( InputConfig::PullDown ), &gpioa),
    ];

    let key_matrix = KeyMatrix::new( matrix_columns, matrix_rows );

    let keys =  [ 
        Key::new(0x01, KeyType::Modifier), 
        Key::new(0x04, KeyType::Modifier), 
        Key::new(0x4c, KeyType::Default), 
        Key::new(0x2a, KeyType::Default), 
        Key::new(0x10, KeyType::Modifier), 
        Key::new(0x40, KeyType::Modifier), 
    ];

    let gpiob = Gpiob::new();
    let leds = [
        Led::new( Port::new(PortNum::P15, PortMode::Output(OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)), &gpioa) ),
        Led::new( Port::new(PortNum::P4, PortMode::Output(OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)), &gpiob) ),
        Led::new( Port::new(PortNum::P6, PortMode::Output(OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)), &gpiob) ),
    ];
    leds[0].turn_off();
    leds[1].turn_off();
    leds[2].turn_off();

    let mut keypad = Keyboard::new(key_matrix, keys, leds);

    let gpioc = Gpioc::new();
    let pc13 = Port::new(PortNum::P13, PortMode::Output(OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)), &gpioc);
    pc13.set_high();

    let mut report: [u8; 8] = [
        0x00, // modifier
        0x00, // reserved
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // keys
    ];

    let mut loop_counter: u32 = 0;

    loop {

        let mut new_report: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0];

        keypad.scan();


        match keypad.mode {
            KeyboardMode::Normal => {
                pc13.set_high();

                if keypad.keys[0].is_pressed && keypad.keys[3].is_pressed && keypad.keys[4].is_pressed && !keypad.keys[1].is_pressed && !keypad.keys[2].is_pressed && !keypad.keys[5].is_pressed {
                    keypad.leds[0].turn_on();

                    usb_send_report( [0, 0, 0, 0, 0, 0, 0, 0] );
                    if loop_counter > 500 {
                        keypad.mode = KeyboardMode::Idle;
                        keypad.leds[0].turn_off();
                    } else {
                        loop_counter += 1;
                    }
                } else if keypad.keys[1].is_pressed && keypad.keys[2].is_pressed && keypad.keys[5].is_pressed && !keypad.keys[0].is_pressed && !keypad.keys[3].is_pressed && !keypad.keys[4].is_pressed {
                    keypad.leds[2].turn_on();

                    usb_send_report( [0, 0, 0, 0, 0, 0, 0, 0] );
                    if loop_counter > 500 {
                        loop_counter = 0;
                        keypad.reset_keys();
                        keypad.leds[2].turn_off();
                    } else {
                        loop_counter += 1;
                    }
                }
                else {
                    new_report = keypad.get_report();
                    if new_report != report {
                        report = new_report;
                        usb_send_report(report);
                    }
                }

            },
            KeyboardMode::Idle => {
                loop_counter = 0;
                
                keypad.leds[0].toggle();

                if !keypad.keys[0].is_pressed && !keypad.keys[2].is_pressed && !keypad.keys[4].is_pressed && !keypad.keys[1].is_pressed && !keypad.keys[3].is_pressed && !keypad.keys[5].is_pressed {
                    usb_send_report([0, 0, 0, 0, 0, 0, 0, 0]);
                    keypad.mode = KeyboardMode::KeySetup( KeySetupMode::SelectKey );
                }
            },
            KeyboardMode::KeySetup(key_setup_mode) => {
                match key_setup_mode {
                    KeySetupMode::SelectKey => {

                        keypad.leds[0].turn_off();
                        keypad.leds[1].toggle();

                        if keypad.keys[0].is_released {
                            keypad.mode = KeyboardMode::KeySetup(KeySetupMode::SelectKeyType(0));
                        } else if keypad.keys[1].is_released {
                            keypad.mode = KeyboardMode::KeySetup(KeySetupMode::SelectKeyType(1));
                        } else if keypad.keys[2].is_released {
                            keypad.mode = KeyboardMode::KeySetup(KeySetupMode::SelectKeyType(2));
                        } else if keypad.keys[3].is_released {
                            keypad.mode = KeyboardMode::KeySetup(KeySetupMode::SelectKeyType(3));
                        } else if keypad.keys[4].is_released {
                            keypad.mode = KeyboardMode::KeySetup(KeySetupMode::SelectKeyType(4));
                        } else if keypad.keys[5].is_released {
                            keypad.mode = KeyboardMode::KeySetup(KeySetupMode::SelectKeyType(5));
                        }
                    },
                    KeySetupMode::SelectKeyType(key_num) => {

                        keypad.leds[1].turn_off();
                        keypad.leds[2].toggle();

                        keypad.keys[key_num].key_code = 0x00;

                        if keypad.keys[0].is_released {
                            keypad.keys[key_num].key_type = KeyType::Default;
                            keypad.mode = KeyboardMode::KeySetup( KeySetupMode::ReadKeyCode(key_num, 0) );
                        } else if keypad.keys[1].is_released {
                            keypad.keys[key_num].key_type = KeyType::Modifier;
                            keypad.mode = KeyboardMode::KeySetup( KeySetupMode::ReadKeyCode(key_num, 0) );
                        }
                    },
                    KeySetupMode::ReadKeyCode(key_num, key_shift) => {

                        keypad.leds[2].turn_off();

                        if keypad.keys[0].is_released {
                            keypad.keys[key_num].key_code |= 0x00 << key_shift;
                            keypad.mode = KeyboardMode::KeySetup( KeySetupMode::ReadKeyCode(key_num, key_shift + 2) );
                        } else if keypad.keys[1].is_released {
                            keypad.keys[key_num].key_code |= 0x01 << key_shift;
                            keypad.mode = KeyboardMode::KeySetup( KeySetupMode::ReadKeyCode(key_num, key_shift + 2) );
                        } else if keypad.keys[2].is_released {
                            keypad.keys[key_num].key_code |= 0x02 << key_shift;
                            keypad.mode = KeyboardMode::KeySetup( KeySetupMode::ReadKeyCode(key_num, key_shift + 2) );
                        } else if keypad.keys[3].is_released {
                            keypad.keys[key_num].key_code |= 0x03 << key_shift;
                            keypad.mode = KeyboardMode::KeySetup( KeySetupMode::ReadKeyCode(key_num, key_shift + 2) );
                        }

                        if key_shift == 0 {
                            keypad.leds[0].turn_on();
                            keypad.leds[1].turn_off();
                            keypad.leds[2].turn_off();
                        } else if key_shift == 2 {
                            keypad.leds[0].turn_off();
                            keypad.leds[1].turn_on();
                            keypad.leds[2].turn_off();
                        } else if key_shift == 4 {
                            keypad.leds[0].turn_on();
                            keypad.leds[1].turn_on();
                            keypad.leds[2].turn_off();
                        } else if key_shift == 6 {
                            keypad.leds[0].turn_off();
                            keypad.leds[1].turn_off();
                            keypad.leds[2].turn_on();
                        }

                        if key_shift == 8 {
                            keypad.mode = KeyboardMode::Normal;

                            keypad.leds[0].turn_off();
                            keypad.leds[1].turn_off();
                            keypad.leds[2].turn_off();
                        }
                    }
                }
            },
        }
    }
}


// Usb part

fn usb_send_report(report: [u8; 8]) {
    let usb = Usb::new();
    let mut pma = Pma::new();
    pma.write_u8_buffer(&report, 192);
    pma.write_u8_buffer(&[report.len() as u8], 10);
    usb.ep1r.write(0x0611);
}

fn init_usb() {
    Nvic::new().iser0.write_or(0x0010_0000); // Enable global interrupt for USB low priority
    Rcc::new().enable_usb(); // Enable usb clocking
    let mut pma = Pma::new();

    let usb = Usb::new();
    usb.cntr.reset_bit(1); // Exit power down mode
    for _ in 0..10000 {};
    usb.cntr.reset_bit(0); // Clear usb reset
    usb.istr.write(0);  // Clear all interrupts
    usb.cntr.write(0x8400); // Enable CT and RESET interrupts
    usb.btable.write(0); // Set pma table address offset
    usb.daddr.write(0x80); // Set default device address

    pma.write_u8_buffer(&[0x40, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x84], 0); // Fill btable for EP0

    usb.ep0r.write(0x3200); // Allow EP0 to receive data
}


enum UsbInterrupt {
    CorrectTransfer(UsbTransaction),
    PacketMemoryAreaOverrun,
    Error,
    Wakeup,
    SuspendModeRequest,
    UsbResetRequest,
    StartOfFrame,
    ExpectedStartOfFrame,    
}

pub struct UsbTransaction {
    direction: UsbTransactionDirection,
    endpoint: UsbEndpoint,
}

pub struct UsbEndpoint {
    id: u32,
}

enum UsbTransactionDirection {
    In, 
    Out,
}

fn get_interrupt(usb: &Usb) -> UsbInterrupt {
    let interrupt_type: UsbInterrupt = if usb.istr.get_bit(15) == 1 {
        let ep_id = usb.istr.read() & 0xF;
        let dir = usb.istr.get_bit(4);

        let direction = if dir == 0 {
            UsbTransactionDirection::In
        } else {
            UsbTransactionDirection::Out
        };

        UsbInterrupt::CorrectTransfer(  UsbTransaction { direction, endpoint: UsbEndpoint{ id: ep_id } } )
    }
    else if usb.istr.get_bit(9) == 1 {
        UsbInterrupt::StartOfFrame
    }
    else if usb.istr.get_bit(10) == 1 {
        UsbInterrupt::UsbResetRequest
    }
    else if usb.istr.get_bit(11) == 1 {
        UsbInterrupt::SuspendModeRequest
    }
    else if usb.istr.get_bit(12) == 1 {
        UsbInterrupt::Wakeup
    }
    else if usb.istr.get_bit(13) == 1 {
        UsbInterrupt::Error
    }
    else if usb.istr.get_bit(14) == 1 {
        UsbInterrupt::PacketMemoryAreaOverrun
    }    
    else {
        UsbInterrupt::ExpectedStartOfFrame
    };

    interrupt_type
}


#[interrupt]
fn USB_LP_CAN_RX0() {

    let usb = Usb::new();
    let mut pma = Pma::new();

    let interrupt_type = get_interrupt(&usb);

    match interrupt_type {
        UsbInterrupt::CorrectTransfer(transaction) => {
            let ep_id = transaction.endpoint.id;
            let dir = transaction.direction;
    
            if ep_id == 1 {
                usb.ep1r.write(0x0601);
            }
    
            if ep_id == 0 {

                match dir {
                    UsbTransactionDirection::In => {
                        unsafe {
                            match DEVICE_STATUS {
                                DeviceStatus::SetAddress => {
                                    usb.daddr.write(DEVICE_ADDRESS as u32);
                                    DEVICE_STATUS = DeviceStatus::Default;
                                },
                                DeviceStatus::GetReportDescriptor => {
                                    DEVICE_STATUS = DeviceStatus::Default;
                                },
                                _ => {},
                            }
                        }
        
                        usb.ep0r.write( 0x9200 );
                    },
                    UsbTransactionDirection::Out => {
                        let bytes_received = pma[3] &0xff;
    
                        let mut buffer: [u8; 64] = [0; 64];
                        pma.read_u8_buffer(&mut buffer[..bytes_received as usize], 128);
        
                        if bytes_received == 0 {
                            usb.ep0r.write(0x1200);
                        } else {
                            match (buffer[0] as u16) << 8 | buffer[1] as u16 {
                                0x8006 => {
                                    match buffer[3] {
                
                                        0x01 => {
                                            let device_descriptor: [u8; 18] = [
                                                0x12, 
                                                0x01, 
                                                0x00, 0x02,
                                                0x00, 
                                                0x00, 
                                                0x00, 
                                                0x40,
                                                0xff, 0xff, 
                                                0x01, 0x00,
                                                0x00, 0x02, 
                                                0x01, 
                                                0x02,
                                                0x03, 
                                                0x01,
                                            ];
                        
                                            pma.write_u8_buffer(&device_descriptor, 64);
                                            pma.write_u8_buffer(&[device_descriptor.len() as u8], 2);
                        
                                            usb.ep0r.write(0x0210);
                                        },
                                        0x02 => {
        
                                            let config_descriptor: [u8; 34] = [
                                                0x09, /* bLength: Configuration Descriptor size */
                                                0x02, /* bDescriptorType: Configuration */
                                                0x22, 0x00, /* wTotalLength: Bytes returned */
                                                0x01,         /*bNumInterfaces: 1 interface*/
                                                0x01,         /*bConfigurationValue: Configuration value*/
                                                0x00,         /*iConfiguration: Index of string descriptor describing
                                                the configuration*/
                                                0xb0,         /*bmAttributes: bus powered and Support Remote Wake-up */
                                                0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
                                                
                                                /************** Descriptor of Joystick Mouse interface ****************/
                                                /* 09 */
                                                0x09,         /*bLength: Interface Descriptor size*/
                                                0x04,/*bDescriptorType: Interface descriptor type*/
                                                0x00,         /*bInterfaceNumber: Number of Interface*/
                                                0x00,         /*bAlternateSetting: Alternate setting*/
                                                0x01,         /*bNumEndpoints*/
                                                0x03,         /*bInterfaceClass: HID*/
                                                0x01,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
                                                0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
                                                0x00,            /*iInterface: Index of string descriptor*/
                                                /******************** Descriptor of Joystick Mouse HID ********************/
                                                /* 18 */
                                                0x09,         /*bLength: HID Descriptor size*/
                                                0x21, /*bDescriptorType: HID*/
                                                0x01, 0x01,         /*bcdHID: HID Class Spec release number*/
                                                0x00,         /*bCountryCode: Hardware target country*/
                                                0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
                                                0x22,         /*bDescriptorType*/
                                                0x2d, 0x00,/*wItemLength: Total length of Report descriptor*/
                                                /******************** Descriptor of Mouse endpoint ********************/
                                                /* 27 */
                                                0x07,          /*bLength: Endpoint Descriptor size*/
                                                0x05, /*bDescriptorType:*/
                                                
                                                0x81,     /*bEndpointAddress: Endpoint Address (IN)*/
                                                0x03,          /*bmAttributes: Interrupt endpoint*/
                                                0x08, 0x00, /*wMaxPacketSize: 4 Byte max */ 
                                                0x0a,          /*bInterval: Polling Interval */
                                            ];
                        
                                            let mut config_descriptor_requested_length = if buffer[6] == 0x09 {
                                                0x09
                                            } else {
                                                0x22
                                            };
                                            pma.write_u8_buffer(&config_descriptor, 64);
                                            pma.write_u8_buffer(&[config_descriptor_requested_length as u8], 2);
                        
                                            usb.ep0r.write(0x0210);
        
                                        },
                                        0x03 => {
                        
                                            if buffer[2] == 0x00 {
        
                                                let string_descriptor_1: [u8; 4] = [
                                                    0x04, 0x03, 0x09, 0x00,
                                                ];
        
                                                pma.write_u8_buffer(&string_descriptor_1, 64);
                                                pma.write_u8_buffer(&[string_descriptor_1.len() as u8], 2);
                        
                                            } else {
                                                let string_descriptor_2: [u8; 10] = [
                                                    0x0a, 0x03, 0x41, 0x00,
                                                    0x70, 0x00, 0x77, 0x00,
                                                    0x80, 0x00, 
                                                ];
                                                pma.write_u8_buffer(&string_descriptor_2, 64);
                                                pma.write_u8_buffer(&[string_descriptor_2.len() as u8], 2);
                                            }
                                            
                                            usb.ep0r.write(0x0210);
                                        },
                                        _ => {
                                            pma.write_u8_buffer(&[0 as u8], 2);
                    
                                            usb.ep0r.write(0x0210);
                                        },
                                    }
                                },
                                0x0005 => {        
                                    unsafe {
                                        DEVICE_ADDRESS = buffer[2] | 0x80;
                                        DEVICE_STATUS = DeviceStatus::SetAddress;
                                    }
        
                                    pma.write_u8_buffer(&[0 as u8], 2);
                    
                                    usb.ep0r.write(0x0210);
                                },
                                0x0009 => {        
                                    pma.write_u8_buffer(&[0 as u8], 2);
                    
                                    usb.ep0r.write(0x0210);
                                },
                                0x210a => {
                                    pma.write_u8_buffer(&[0 as u8], 2);
                    
                                    usb.ep0r.write(0x0210);
                                },
                                0x8106 => {
                
                                    match buffer[3] {
                                        0x22 => {
                                            let report_descriptor: [u8; 45] = [
                                                0x05, 0x01,
                                                0x09, 0x06,
                                                0xa1, 0x01,
                                                0x05, 0x07,
                                                0x19, 0xe0,
                                                0x29, 0xe7,
                                                0x15, 0x00,
                                                0x25, 0x01,
                                                0x75, 0x01,
                                                0x95, 0x08,
                                                0x81, 0x02,
                                                0x95, 0x01,
                                                0x75, 0x08,
                                                0x81, 0x01,
                                                0x95, 0x06,
                                                0x75, 0x08,
                                                0x15, 0x00,
                                                0x25, 0x65,
                                                0x05, 0x07,
                                                0x19, 0x00,
                                                0x29, 0x65,
                                                0x81, 0x00,
                                                0xc0,
                                            ];
        
                                            pma.write_u8_buffer(&report_descriptor, 64);
                                            pma.write_u8_buffer(&[report_descriptor.len() as u8], 2);
                                            usb.ep0r.write(0x0210);
                                            unsafe {
                                                DEVICE_STATUS = DeviceStatus::GetReportDescriptor;
                                            }
                        
                                        },
                                        _ => {},
                                    }
                                },
                                _ => {},
                            }
                        }
                    },
                };
            }
        },
        UsbInterrupt::PacketMemoryAreaOverrun => {
            usb.istr.reset_bit(14);
        },
        UsbInterrupt::Error => {
            usb.istr.reset_bit(13);
        },
        UsbInterrupt::Wakeup => {
            usb.istr.reset_bit(12);
        },
        UsbInterrupt::SuspendModeRequest => {
            usb.istr.reset_bit(11);        
        },
        UsbInterrupt::UsbResetRequest => {
            usb.istr.reset_bit(10);

            usb.daddr.write(0x80);
    
            pma.write_u8_buffer(&[0x40, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x84], 0);
            pma.write_u8_buffer(&[0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 8);
    
            usb.ep0r.write(0xb280);
            usb.ep1r.write(0x86a1);
        },
        UsbInterrupt::StartOfFrame => {
            usb.istr.reset_bit(9);
        },
        UsbInterrupt::ExpectedStartOfFrame => {
            usb.istr.reset_bit(8);
        },
    }

}