#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_reset as _;

use peris::peripherals::clock;
use peris::core::{
    rcc::Rcc,
    usb::Usb,
    nvic::Nvic,
    pma::Pma,
};
use stm32f1::stm32f103::interrupt;


enum DeviceStatus {
    Default,
    GetReportDescriptor,
    SetAddress,
}

static mut DEVICE_STATUS: DeviceStatus = DeviceStatus::Default;
static mut DEVICE_ADDRESS: u8 = 0x00;

#[entry]
fn main() -> ! {
    clock::init();
    init_usb();


    loop {
    }
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

use core::convert::From;
enum UsbInterruptType {
    CorrectTransfer(UsbTransfer),
    PacketMemoryAreaOverrun,
    Error,
    Wakeup,
    SuspendModeRequest,
    UsbResetRequest,
    StartOfFrame,
    ExpectedStartOfFrame,    
}

pub struct UsbTransfer {
    direction: UsbTransferDirection,
    endpoint: UsbEndpoint,
}

pub struct UsbEndpoint {
    id: u32,
}

enum UsbTransferDirection {
    In, 
    Out,
}



#[interrupt]
fn USB_LP_CAN_RX0() {

    let usb = Usb::new();
    let mut pma = Pma::new();

    let interrupt_type: UsbInterruptType = if usb.istr.get_bit(15) == 1 {
        let ep_id = usb.istr.read() & 0xF;
        let dir = usb.istr.get_bit(4);

        let direction = if dir == 0 {
            UsbTransferDirection::In
        } else {
            UsbTransferDirection::Out
        };

        UsbInterruptType::CorrectTransfer(  UsbTransfer { direction, endpoint: UsbEndpoint{ id: ep_id } } )
    }
    else if usb.istr.get_bit(9) == 1 {
        UsbInterruptType::StartOfFrame
    }
    else if usb.istr.get_bit(10) == 1 {
        UsbInterruptType::UsbResetRequest
    }
    else if usb.istr.get_bit(11) == 1 {
        UsbInterruptType::SuspendModeRequest
    }
    else if usb.istr.get_bit(12) == 1 {
        UsbInterruptType::Wakeup
    }
    else if usb.istr.get_bit(13) == 1 {
        UsbInterruptType::Error
    }
    else if usb.istr.get_bit(14) == 1 {
        UsbInterruptType::PacketMemoryAreaOverrun
    }    
    else {
        UsbInterruptType::ExpectedStartOfFrame
    };

    match interrupt_type {
        UsbInterruptType::CorrectTransfer(transfer) => {
            let ep_id = transfer.endpoint.id;
            let dir = match transfer.direction {
                UsbTransferDirection::In => {
                    0
                },
                UsbTransferDirection::Out => {
                    1
                }
            };
    
            if ep_id == 1 {
    
            }
    
            if ep_id == 0 {
                if dir == 0 {
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
                } else {
    
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
                }
            }
        },
        UsbInterruptType::PacketMemoryAreaOverrun => {
            usb.istr.reset_bit(14);
        },
        UsbInterruptType::Error => {
            usb.istr.reset_bit(13);
        },
        UsbInterruptType::Wakeup => {
            usb.istr.reset_bit(12);
        },
        UsbInterruptType::SuspendModeRequest => {
            usb.istr.reset_bit(11);        
        },
        UsbInterruptType::UsbResetRequest => {
            usb.istr.reset_bit(10);

            usb.daddr.write(0x80);
    
            pma.write_u8_buffer(&[0x40, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x84], 0);
            pma.write_u8_buffer(&[0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 8);
    
            usb.ep0r.write(0xb280);
            usb.ep1r.write(0x86a1);
        },
        UsbInterruptType::StartOfFrame => {
            usb.istr.reset_bit(9);
        },
        UsbInterruptType::ExpectedStartOfFrame => {
            usb.istr.reset_bit(8);
        },
    }

}

#[interrupt]
fn EXTI1() {
    let usb = Usb::new();
    let mut pma = Pma::new();

    let report: [u8; 8] = [
        0x00, // modifier
        0x00, // reserved
        0x04, 0x00, 0x00, 0x00, 0x00, 0x00 // keys
    ];
    pma.write_u8_buffer(&report, 192);
    pma.write_u8_buffer(&[report.len() as u8], 10);
    usb.ep1r.write(0x0612);
    
    pma.write_u8_buffer(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 192);
    pma.write_u8_buffer(&[8 as u8], 10);
    usb.ep1r.write(0x0612);
}