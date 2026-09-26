
#![no_std]

pub mod avr {
    pub mod usb {
        use core::result::Result::{self, Ok, Err};
        use core::option::Option::{self, Some, None};

        #[allow(dead_code)]
        pub struct Endpoint {
            info: embassy_usb_driver::EndpointInfo,
        }

        impl Endpoint {
            pub fn new(ep_addr: embassy_usb_driver::EndpointAddress, ep_type: embassy_usb_driver::EndpointType, max_packet_size: u16, interval_ms: u8) -> Self {

                let info = embassy_usb_driver::EndpointInfo {
                    addr: ep_addr,
                    ep_type,
                    max_packet_size,
                    interval_ms,
                };

                Self {
                    info,
                }
            }

            fn packet_size_code(size: u16) -> Option<u8> {
                match size {
                    8 => Some(0),
                    16 => Some(1),
                    32 => Some(2),
                    64 => Some(3),
                    _ => None,
                }
            }
        }

        impl embassy_usb_driver::ControlPipe for Endpoint {
            fn max_packet_size(&self) -> usize {
                self.info.max_packet_size as usize
            }

            async fn accept(&mut self) {
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.uenum().modify(|_, w| w.set(0));
                while device.ueintx().read().txini().bit_is_clear() {}
                device.ueintx().modify(|_, w| w.txini().clear_bit());
            }

            async fn reject(&mut self) {
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.uenum().modify(|_, w| w.set(0));
                device.ueconx().modify(|_, w| w.stallrq().set_bit());
            }

            async fn accept_set_address(&mut self, addr: u8) {
                self.accept().await;
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.udaddr().modify(|_, w| unsafe { w.uadd().bits(addr & 0x7f) });
                device.udaddr().modify(|_, w| w.adden().set_bit());
            }

            async fn setup(&mut self) -> [u8; 8] {
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.uenum().modify(|_, w| w.set(0));
                while device.ueintx().read().rxstpi().bit_is_clear() {}
                let mut setup = [0; 8];
                for byte in &mut setup { *byte = device.uedatx().read().dat().bits(); }
                device.ueintx().modify(|_, w| { w.fifocon().clear_bit(); w.rxstpi().clear_bit() });
                setup
            }

            async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), embassy_usb_driver::EndpointError> {
                let _ = (data, first);
                if last { self.accept().await; }
                Ok(())
            }

            async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, embassy_usb_driver::EndpointError> {
                let _ = (buf, first, last);
                Err(embassy_usb_driver::EndpointError::Disabled)
            }
        }

        impl embassy_usb_driver::EndpointIn for Endpoint {
            async fn write(&mut self, buf: &[u8]) -> Result<(), embassy_usb_driver::EndpointError> {
                if buf.len() > self.info.max_packet_size as usize { return Err(embassy_usb_driver::EndpointError::BufferOverflow); }
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.uenum().modify(|_, w| w.set(self.info.addr.index() as u8));
                if device.ueconx().read().epen().bit_is_clear() { return Err(embassy_usb_driver::EndpointError::Disabled); }
                while device.ueintx().read().txini().bit_is_clear() {}
                for byte in buf { device.uedatx().write(|w| w.dat().set(*byte)); }
                device.ueintx().modify(|_, w| w.txini().clear_bit());
                Ok(())
            }

            async fn write_transfer(&mut self, buf: &[u8], needs_zlp: bool) -> Result<(), embassy_usb_driver::EndpointError> {
                for chunk in buf.chunks(self.info.max_packet_size as usize) { self.write(chunk).await?; }
                if needs_zlp && buf.len() % self.info.max_packet_size as usize == 0 { self.write(&[]).await?; }
                Ok(())
            }
        }

        impl embassy_usb_driver::EndpointOut for Endpoint {
            async fn read(&mut self, buf: &mut [u8]) -> Result<usize, embassy_usb_driver::EndpointError> {
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.uenum().modify(|_, w| w.set(self.info.addr.index() as u8));
                if device.ueconx().read().epen().bit_is_clear() { return Err(embassy_usb_driver::EndpointError::Disabled); }
                while device.ueintx().read().rxouti().bit_is_clear() {}
                let count = device.uebclx().read().bits() as usize | ((device.uebchx().read().bits() as usize) << 8);
                if count > buf.len() { return Err(embassy_usb_driver::EndpointError::BufferOverflow); }
                for byte in &mut buf[..count] { *byte = device.uedatx().read().dat().bits(); }
                device.ueintx().modify(|_, w| { w.fifocon().clear_bit(); w.rxouti().clear_bit() });
                Ok(count)
            }

            async fn read_transfer(&mut self, buf: &mut [u8]) -> Result<usize, embassy_usb_driver::EndpointError> {
                let mut total = 0;
                loop {
                    let count = self.read(&mut buf[total..]).await?;
                    total += count;
                    if count < self.info.max_packet_size as usize { return Ok(total); }
                }
            }
        }

        #[allow(dead_code)]
        pub struct Device {
            usb_device: avr_device::atmega32u4::USB_DEVICE,
            pll_device: avr_device::atmega32u4::PLL
        }

        impl Device {
            pub fn new(usb_device: avr_device::atmega32u4::USB_DEVICE, pll_device: avr_device::atmega32u4::PLL) -> Self {
                Self {
                    usb_device,
                    pll_device,
                }
            }
        }

        impl<'a> embassy_usb_driver::Driver<'a> for Device {
            type Bus = Device;
            type ControlPipe = Endpoint;
            type EndpointIn = Endpoint;
            type EndpointOut = Endpoint;

            fn alloc_endpoint_in(
                &mut self,
                ep_type: embassy_usb_driver::EndpointType,
                ep_addr: Option<embassy_usb_driver::EndpointAddress>,
                max_packet_size: u16,
                interval_ms: u8,
            ) -> Result<Self::EndpointIn, embassy_usb_driver::EndpointAllocError>
            {
                let address = ep_addr.unwrap_or_else(|| embassy_usb_driver::EndpointAddress::from_parts(1, embassy_usb_driver::Direction::In));
                let size = Endpoint::packet_size_code(max_packet_size).ok_or(embassy_usb_driver::EndpointAllocError)?;
                if !address.is_in() || address.index() == 0 || address.index() > 7 {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }
                self.usb_device.uenum().modify(|_, w| w.set(address.index() as u8));
                self.usb_device.ueconx().modify(|_, w| w.epen().set_bit());
                self.usb_device.uecfg1x().modify(|_, w| {
                    w.alloc().clear_bit();
                    w.epsize().set(0x00);
                    w.epbk().set(0x00)
                });
                self.usb_device.uecfg0x().modify(|_, w| {
                    w.eptype().set(ep_type as u8);
                    w.epdir().set_bit()
                });
                self.usb_device.uecfg1x().modify(|_, w| {
                    w.epsize().set(size);
                    w.alloc().set_bit()
                });
                if self.usb_device.uesta0x().read().cfgok().bit_is_clear() {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }
                Ok(Endpoint::new(address, ep_type, max_packet_size, interval_ms))
            }

            fn alloc_endpoint_out(
                &mut self,
                ep_type: embassy_usb_driver::EndpointType,
                ep_addr: Option<embassy_usb_driver::EndpointAddress>,
                max_packet_size: u16,
                interval_ms: u8,
            ) -> Result<Self::EndpointOut, embassy_usb_driver::EndpointAllocError>
            {
                let address = ep_addr.unwrap_or_else(|| embassy_usb_driver::EndpointAddress::from_parts(1, embassy_usb_driver::Direction::Out));
                let size = Endpoint::packet_size_code(max_packet_size).ok_or(embassy_usb_driver::EndpointAllocError)?;
                if !address.is_out() || address.index() == 0 || address.index() > 7 {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }
                self.usb_device.uenum().modify(|_, w| w.set(address.index() as u8));
                self.usb_device.ueconx().modify(|_, w| w.epen().set_bit());
                self.usb_device.uecfg1x().modify(|_, w| {
                    w.alloc().clear_bit();
                    w.epsize().set(0x00);
                    w.epbk().set(0x00)
                });
                self.usb_device.uecfg0x().modify(|_, w| {
                    w.eptype().set(ep_type as u8);
                    w.epdir().clear_bit()
                });
                self.usb_device.uecfg1x().modify(|_, w| {
                    w.epsize().set(size);
                    w.alloc().set_bit()
                });
                if self.usb_device.uesta0x().read().cfgok().bit_is_clear() {
                    return Err(embassy_usb_driver::EndpointAllocError);
                }
                Ok(Endpoint::new(address, ep_type, max_packet_size, interval_ms))
            }

            fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
                self.usb_device.udcon().modify(|_, w| w.detach().clear_bit());
                self.usb_device.uenum().modify(|_, w| w.set(0));
                self.usb_device.ueconx().modify(|_, w| w.epen().set_bit());
                self.usb_device.uecfg1x().modify(|_, w| {
                    w.alloc().clear_bit();
                    w.epsize().set(0x00);
                    w.epbk().set(0x00)
                });
                self.usb_device.uecfg0x().modify(|_, w| {
                    w.eptype().set(embassy_usb_driver::EndpointType::Control as u8);
                    w.epdir().clear_bit()
                });
                if let Some(size) = Endpoint::packet_size_code(control_max_packet_size) {
                    self.usb_device.uecfg1x().modify(|_, w| {
                        w.epsize().set(size);
                        w.alloc().set_bit()
                    });
                }
                let ep_addr = embassy_usb_driver::EndpointAddress::from_parts(0, embassy_usb_driver::Direction::Out);
                (self, Endpoint::new(ep_addr, embassy_usb_driver::EndpointType::Control, control_max_packet_size, 0))

            }
        }

        impl embassy_usb_driver::Bus for Device {
            async fn enable(&mut self) {
                self.usb_device.usbcon().modify(|_, w| w.usbe().set_bit());
            }

            async fn disable(&mut self) {
                self.usb_device.usbcon().modify(|_, w| w.usbe().clear_bit());
            }

            async fn poll(&mut self) -> embassy_usb_driver::Event {
                loop {
                    let reg = self.usb_device.udint().read();
                    let event = if reg.eorsti().bit_is_set() {
                        self.usb_device.udint().modify(|_, w| w.eorsti().clear_bit());
                        Some(embassy_usb_driver::Event::Reset)
                    } else if reg.suspi().bit_is_set() {
                        self.usb_device.udint().modify(|_, w| w.suspi().clear_bit());
                        Some(embassy_usb_driver::Event::Suspend)
                    } else if reg.wakeupi().bit_is_set() {
                        self.usb_device.udint().modify(|_, w| w.wakeupi().clear_bit());
                        Some(embassy_usb_driver::Event::PowerDetected)
                    } else if reg.uprsmi().bit_is_set() {
                        self.usb_device.udint().modify(|_, w| w.uprsmi().clear_bit());
                        Some(embassy_usb_driver::Event::Resume)
                    } else if reg.eorsmi().bit_is_set() {
                        self.usb_device.udint().modify(|_, w| w.eorsmi().clear_bit());
                        Some(embassy_usb_driver::Event::PowerRemoved)
                    } else {
                        None
                    };

                    if let Some(event) = event {
                        return event;
                    }
                }
            }

            async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
                self.usb_device.udcon().modify(|_, w| w.rmwkup().set_bit());
                Ok(())
            }

            fn force_reset(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
                self.usb_device.usbcon().modify(|_, w| w.usbe().clear_bit());
                self.usb_device.usbcon().modify(|_, w| w.usbe().set_bit());
                Ok(())
            }

            fn endpoint_set_enabled(&mut self, ep_addr: embassy_usb_driver::EndpointAddress, enabled: bool) {
                self.usb_device.uenum().modify(|_, w| w.set(ep_addr.index() as u8));
                self.usb_device.ueconx().modify(|_, w| w.epen().bit(enabled));
            }

            fn endpoint_is_stalled(&mut self, ep_addr: embassy_usb_driver::EndpointAddress) -> bool {
                self.usb_device.uenum().modify(|_, w| w.set(ep_addr.index() as u8));
                self.usb_device.ueconx().read().epen().bit_is_clear()
            }

            fn endpoint_set_stalled(&mut self, ep_addr: embassy_usb_driver::EndpointAddress, stalled: bool) {
                self.usb_device.uenum().modify(|_, w| w.set(ep_addr.index() as u8));
                self.usb_device.ueconx().modify(|_, w| w.epen().bit(!stalled));
            }
        }

        impl embassy_usb_driver::Endpoint for Endpoint {
            fn info(&self) -> &embassy_usb_driver::EndpointInfo {
                &self.info
            }
            async fn wait_enabled(&mut self) {
                let device = unsafe { avr_device::atmega32u4::USB_DEVICE::steal() };
                device.uenum().modify(|_, w| w.set(self.info.addr.index() as u8));
                while device.ueconx().read().epen().bit_is_clear() {}
            }
        }
    }
}