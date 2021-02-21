#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_reset as _;

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
    gpio::{
        gpioa::Gpioa,
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


enum KeyType {
    Modifier,
    Key,
}
struct Key {
    is_pressed: bool,
    is_pushed: bool,
    is_released: bool,
    key_code: u8,

    safety_counter: u8,
    safety_cycles: u8,
    times_scanned: u8,
}
impl Key {
    fn new(key_code: u8) -> Key {
        Key {
            is_pressed: false,
            is_released: false,
            is_pushed: false,
            key_code,
            safety_counter: 0,
            safety_cycles: 10,
            times_scanned: 0,
        }
    }
    fn apply_status(&mut self, status: bool) {

        self.times_scanned += 1; 

        if status {

            if !self.is_pressed {

                self.is_pressed = true;

            }

        }


        
    }
}

enum KeyT {
    Shift,
    Alt,
    A
}

enum KeyboardMode {
    Normal,
    Idle,
    KeySetup,
}
enum KeySetupMode {
    SelectKey,
    SelectKeyType,
    ReadKeyCode,
}

struct KeyMatrix<'a> {
    columns: &'a [Port<'a>],
    rows: &'a [Port<'a>],
}
impl<'a> KeyMatrix<'a> {
    fn new(columns: &'a [Port<'a>], rows: &'a [Port<'a>]) -> KeyMatrix<'a> {
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


        // let pa3 = &self.columns[0];
        // let pa4 = &self.columns[1];
        // let pa5 = &self.columns[2];
        
        // let pa6 = &self.rows[1];
        // let pa7 = &self.rows[0];

        // pa5.set_mode( PortMode::Output( OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)) );
        // pa5.set_high();

        // if pa6.get_input() == 1 {
        //     keys[0].is_pressed = true;
        //     keys[0].is_released = false;
        // } else {
        //     if keys[0].is_released {
        //         keys[0].is_released = false;
        //     }
        //     if keys[0].is_pressed {
        //         keys[0].is_released = true;
        //     }
        //     keys[0].is_pressed = false;
        // }
        // if pa7.get_input() == 1 {
        //     keys[1].is_pressed = true;
        //     keys[1].is_released = false;
        // } else {
        //     if keys[1].is_released {
        //         keys[1].is_released = false;
        //     }
        //     if keys[1].is_pressed {
        //         keys[1].is_released = true;
        //     }
        //     keys[1].is_pressed = false;
        // }

        // pa5.set_low();
        // pa5.set_mode(PortMode::Input(InputConfig::Floating));

        // pa4.set_mode( PortMode::Output( OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)) );
        // pa4.set_high();

        // if pa6.get_input() == 1 {
        //     keys[2].is_pressed = true;
        //     keys[2].is_released = false;
        // } else {
        //     if keys[2].is_released {
        //         keys[2].is_released = false;
        //     }
        //     if keys[2].is_pressed {
        //         keys[2].is_released = true;
        //     }
        //     keys[2].is_pressed = false;
        // }
        // if pa7.get_input() == 1 {
        //     keys[3].is_pressed = true;
        //     keys[3].is_released = false;
        // } else {
        //     if keys[3].is_released {
        //         keys[3].is_released = false;
        //     }
        //     if keys[3].is_pressed {
        //         keys[3].is_released = true;
        //     }
        //     keys[3].is_pressed = false;
        // }

        // pa4.set_low();
        // pa4.set_mode(PortMode::Input(InputConfig::Floating));

        // pa3.set_mode( PortMode::Output( OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)) );
        // pa3.set_high();

        // if pa6.get_input() == 1 {
        //     keys[4].is_pressed = true;
        //     keys[4].is_released = false;
        // } else {
        //     if keys[4].is_released {
        //         keys[4].is_released = false;
        //     }
        //     if keys[4].is_pressed {
        //         keys[4].is_released = true;
        //     }
        //     keys[4].is_pressed = false;
        // }
        // if pa7.get_input() == 1 {
        //     keys[5].is_pressed = true;
        //     keys[5].is_released = false;
        // } else {
        //     if keys[5].is_released {
        //         keys[5].is_released = false;
        //     }
        //     if keys[5].is_pressed {
        //         keys[5].is_released = true;
        //     }
        //     keys[5].is_pressed = false;
        // }

        // pa3.set_low();
        // pa3.set_mode(PortMode::Input(InputConfig::Floating));
    }
}


#[entry]
fn main() -> ! {
    clock::init();
    init_usb();

    let mut keyboard_mode = KeyboardMode::Normal;
    let mut key_setup_mode = KeySetupMode::SelectKeyType;
    let mut setup_key_num = 0;
    let mut key_setup_shift = 0;

    let gpioc = Gpioc::new();
    let pc13 = Port::new(PortNum::P13, PortMode::Output(OutputConfig::GeneralPurposePushPull(MaxSpeed::S2MHz)), &gpioc);
    pc13.set_high();

    let gpioa = Gpioa::new();

    let matrix_columns = [
        Port::new(PortNum::P3, PortMode::Input( InputConfig::Floating ), &gpioa),
        Port::new(PortNum::P4, PortMode::Input( InputConfig::Floating ), &gpioa),
        Port::new(PortNum::P5, PortMode::Input( InputConfig::Floating ), &gpioa),
    ];
    let matrix_rows = [
        Port::new(PortNum::P6, PortMode::Input( InputConfig::PullDown ), &gpioa),
        Port::new(PortNum::P7, PortMode::Input( InputConfig::PullDown ), &gpioa),
    ];

    let key_matrix = KeyMatrix::new( &matrix_columns, &matrix_rows );

    let mut report: [u8; 8] = [
        0x00, // modifier
        0x00, // reserved
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // keys
    ];

    let mut keys: [Key; 6] = [ Key::new(0x04), Key::new(0x05), Key::new(0x06), Key::new(0x07), Key::new(0x08), Key::new(0x09) ];
    let mut keys_scan = [0; 6];
    let mut keys_safety_scan = [0; 6]; 

    let mut loop_counter: u32 = 0;

    loop {

        let mut new_report: [u8; 8] = [0, 0, 0, 0, 0, 0, 0, 0];


        for _ in 0 .. 10 {

            key_matrix.scan(&mut keys_scan);
    
            for (key_index, key_status) in keys_scan.iter().enumerate() {
    
                if *key_status == 1 {
                    keys_safety_scan[key_index] += 1;
                }
    
            };

        }

        for (key_index, key_safety_scan) in keys_safety_scan.iter().enumerate() {

            if *key_safety_scan > 7 {
                keys[key_index].is_pressed = true;
            } else {
                keys[key_index].is_pressed = false;

            }
        }
        keys_safety_scan = [0; 6];


        match keyboard_mode {
            KeyboardMode::Normal => {
                if keys[0].is_pressed && keys[2].is_pressed && keys[4].is_pressed && !keys[1].is_pressed && !keys[3].is_pressed && !keys[5].is_pressed {
                    if loop_counter > 10000 {
                        keyboard_mode = KeyboardMode::Idle;
                        usb_send_report( [0, 0, 0, 0, 0, 0, 0, 0] );
                    } else {
                        loop_counter += 1;
                    }
                } else {
                    let mut report_index = 0;

                    if keys[0].is_pressed {
                        new_report[report_index + 2] = keys[0].key_code; 
                        report_index += 1
                    }
                    if keys[1].is_pressed {
                        new_report[report_index + 2] = keys[1].key_code; 
                        report_index += 1
                    }
                    if keys[2].is_pressed {
                        new_report[report_index + 2] = keys[2].key_code; 
                        report_index += 1
                    }
                    if keys[3].is_pressed {
                        new_report[report_index + 2] = keys[3].key_code; 
                        report_index += 1
                    }
                    if keys[4].is_pressed {
                        new_report[report_index + 2] = keys[4].key_code; 
                        report_index += 1
                    }
                    if keys[5].is_pressed {
                        new_report[report_index + 2] = keys[5].key_code; 
                    }

                    if new_report != report {
                        report = new_report;
                        usb_send_report(report);
                    }
                }

            },
            KeyboardMode::Idle => {
                loop_counter = 0;
                pc13.set_low();
                if !keys[0].is_pressed && !keys[2].is_pressed && !keys[4].is_pressed && !keys[1].is_pressed && !keys[3].is_pressed && !keys[5].is_pressed {
                    keyboard_mode = KeyboardMode::KeySetup;
                }
            },
            KeyboardMode::KeySetup => {
                match key_setup_mode {
                    KeySetupMode::SelectKey => {
                        if keys[0].is_released {
                            setup_key_num = 0;
                            key_setup_mode = KeySetupMode::SelectKeyType;
                        } else if keys[1].is_released {
                            setup_key_num = 1;
                            key_setup_mode = KeySetupMode::SelectKeyType;
                        } else if keys[2].is_released {
                            setup_key_num = 2;
                            key_setup_mode = KeySetupMode::SelectKeyType;
                        } else if keys[3].is_released {
                            setup_key_num = 3;
                            key_setup_mode = KeySetupMode::SelectKeyType;
                        } else if keys[4].is_released {
                            setup_key_num = 4;
                            key_setup_mode = KeySetupMode::SelectKeyType;
                        } else if keys[5].is_released {
                            setup_key_num = 5;
                            key_setup_mode = KeySetupMode::SelectKeyType;
                        }
                    },
                    KeySetupMode::SelectKeyType => {
                        keys[setup_key_num].key_code = 0x00;
                        key_setup_mode = KeySetupMode::ReadKeyCode;
                    },
                    KeySetupMode::ReadKeyCode => {
                        if keys[0].is_released {
                            keys[setup_key_num].key_code |= 0x00 << key_setup_shift;
                            key_setup_shift += 2;
                        } else if keys[1].is_released {
                            keys[setup_key_num].key_code |= 0x01 << key_setup_shift;
                            key_setup_shift += 2;
                        } else if keys[2].is_released {
                            keys[setup_key_num].key_code |= 0x02 << key_setup_shift;
                            key_setup_shift += 2;
                        } else if keys[3].is_released {
                            keys[setup_key_num].key_code |= 0x03 << key_setup_shift;
                            key_setup_shift += 2;
                        }

                        if key_setup_shift == 8 {
                            key_setup_mode = KeySetupMode::SelectKey;
                            keyboard_mode = KeyboardMode::Normal;
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