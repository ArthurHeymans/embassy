//! USB driver.
use core::future::poll_fn;
use core::marker::PhantomData;
use core::slice;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_hal_internal::PeripheralType;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver as driver;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};

use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::{Peri, RegExt, interrupt, pac, peripherals};

trait SealedInstance {
    fn regs() -> crate::pac::usb::Usb;
    fn dpram() -> crate::pac::usb_dpram::UsbDpram;
}

/// USB peripheral instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

impl crate::usb::SealedInstance for peripherals::USB {
    fn regs() -> pac::usb::Usb {
        pac::USB
    }
    fn dpram() -> crate::pac::usb_dpram::UsbDpram {
        pac::USB_DPRAM
    }
}

impl crate::usb::Instance for peripherals::USB {
    type Interrupt = crate::interrupt::typelevel::USBCTRL_IRQ;
}

const EP_COUNT: usize = 16;
const EP_MEMORY_SIZE: usize = 4096;
const EP_MEMORY: *mut u8 = pac::USB_DPRAM.as_ptr() as *mut u8;

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
static EP_IN_WAKERS: [AtomicWaker; EP_COUNT] = [const { AtomicWaker::new() }; EP_COUNT];
static EP_OUT_WAKERS: [AtomicWaker; EP_COUNT] = [const { AtomicWaker::new() }; EP_COUNT];

/// Buffer index for double buffering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum BufIdx {
    Buf0 = 0,
    Buf1 = 1,
}

impl BufIdx {
    #[inline]
    fn index(self) -> usize {
        self as usize
    }

    /// Get the other buffer index.
    #[inline]
    fn other(self) -> Self {
        match self {
            BufIdx::Buf0 => BufIdx::Buf1,
            BufIdx::Buf1 => BufIdx::Buf0,
        }
    }
}

/// Configuration for a single buffer in double-buffered mode.
#[derive(Clone, Copy)]
struct DoubleBufConfig {
    pid: bool,
    length: u16,
    available: bool,
    full: bool, // Only used for IN endpoints
}

impl DoubleBufConfig {
    /// Create config by selecting values based on buffer index.
    fn select(buf_idx: BufIdx, new_val: Self, current: Self) -> (Self, Self) {
        match buf_idx {
            BufIdx::Buf0 => (new_val, current),
            BufIdx::Buf1 => (current, new_val),
        }
    }
}

/// Write to OUT buffer control register for double-buffered mode.
/// Handles the required write-delay-write pattern per RP2040 datasheet.
fn write_out_buffer_control_double<T: Instance>(
    index: usize,
    buf_idx: BufIdx,
    pid: bool,
    length: u16,
    available: bool,
    current: &pac::usb_dpram::regs::EpBufferControl,
) {
    let new_cfg = DoubleBufConfig {
        pid,
        length,
        available: false, // First write without available
        full: false,
    };
    let cur_cfg = DoubleBufConfig {
        pid: current.pid(buf_idx.other().index()),
        length: current.length(buf_idx.other().index()),
        available: current.available(buf_idx.other().index()),
        full: false,
    };
    let (buf0, buf1) = DoubleBufConfig::select(buf_idx, new_cfg, cur_cfg);

    // First write without available bit
    T::dpram().ep_out_buffer_control(index).write(|w| {
        w.set_pid(0, buf0.pid);
        w.set_length(0, buf0.length);
        w.set_available(0, buf0.available);
        w.set_pid(1, buf1.pid);
        w.set_length(1, buf1.length);
        w.set_available(1, buf1.available);
    });

    cortex_m::asm::delay(12);

    // Second write with available bit set
    let (buf0, buf1) = DoubleBufConfig::select(buf_idx, DoubleBufConfig { available, ..new_cfg }, cur_cfg);
    T::dpram().ep_out_buffer_control(index).write(|w| {
        w.set_pid(0, buf0.pid);
        w.set_length(0, buf0.length);
        w.set_available(0, buf0.available);
        w.set_pid(1, buf1.pid);
        w.set_length(1, buf1.length);
        w.set_available(1, buf1.available);
    });
}

/// Write to IN buffer control register for double-buffered mode.
/// Handles the required write-delay-write pattern per RP2040 datasheet.
fn write_in_buffer_control_double<T: Instance>(
    index: usize,
    buf_idx: BufIdx,
    pid: bool,
    length: u16,
    full: bool,
    available: bool,
    current: &pac::usb_dpram::regs::EpBufferControl,
) {
    let new_cfg = DoubleBufConfig {
        pid,
        length,
        available: false, // First write without available
        full,
    };
    let cur_cfg = DoubleBufConfig {
        pid: current.pid(buf_idx.other().index()),
        length: current.length(buf_idx.other().index()),
        available: current.available(buf_idx.other().index()),
        full: current.full(buf_idx.other().index()),
    };
    let (buf0, buf1) = DoubleBufConfig::select(buf_idx, new_cfg, cur_cfg);

    // First write without available bit
    T::dpram().ep_in_buffer_control(index).write(|w| {
        w.set_pid(0, buf0.pid);
        w.set_length(0, buf0.length);
        w.set_full(0, buf0.full);
        w.set_available(0, buf0.available);
        w.set_pid(1, buf1.pid);
        w.set_length(1, buf1.length);
        w.set_full(1, buf1.full);
        w.set_available(1, buf1.available);
    });

    cortex_m::asm::delay(12);

    // Second write with available bit set
    let (buf0, buf1) = DoubleBufConfig::select(buf_idx, DoubleBufConfig { available, ..new_cfg }, cur_cfg);
    T::dpram().ep_in_buffer_control(index).write(|w| {
        w.set_pid(0, buf0.pid);
        w.set_length(0, buf0.length);
        w.set_full(0, buf0.full);
        w.set_available(0, buf0.available);
        w.set_pid(1, buf1.pid);
        w.set_length(1, buf1.length);
        w.set_full(1, buf1.full);
        w.set_available(1, buf1.available);
    });
}

struct EndpointBuffer<T: Instance> {
    /// Base address of buffer 0 in DPRAM.
    addr: u16,
    /// Length of each buffer (both buffers have the same size).
    len: u16,
    /// Whether this endpoint uses double buffering.
    double_buffered: bool,
    _phantom: PhantomData<T>,
}

impl<T: Instance> EndpointBuffer<T> {
    const fn new(addr: u16, len: u16) -> Self {
        Self {
            addr,
            len,
            double_buffered: false,
            _phantom: PhantomData,
        }
    }

    const fn new_double_buffered(addr: u16, len: u16) -> Self {
        Self {
            addr,
            len,
            double_buffered: true,
            _phantom: PhantomData,
        }
    }

    /// Get the address of the specified buffer.
    #[inline]
    fn buf_addr(&self, idx: BufIdx) -> u16 {
        match idx {
            BufIdx::Buf0 => self.addr,
            // Buffer 1 is immediately after buffer 0
            BufIdx::Buf1 => self.addr + self.len,
        }
    }

    fn read(&mut self, buf: &mut [u8]) {
        self.read_buf(BufIdx::Buf0, buf);
    }

    fn read_buf(&mut self, idx: BufIdx, buf: &mut [u8]) {
        assert!(buf.len() <= self.len as usize);
        compiler_fence(Ordering::SeqCst);
        let addr = self.buf_addr(idx);
        let mem = unsafe { slice::from_raw_parts(EP_MEMORY.add(addr as _), buf.len()) };
        buf.copy_from_slice(mem);
        compiler_fence(Ordering::SeqCst);
    }

    fn write(&mut self, buf: &[u8]) {
        self.write_buf(BufIdx::Buf0, buf);
    }

    fn write_buf(&mut self, idx: BufIdx, buf: &[u8]) {
        assert!(buf.len() <= self.len as usize);
        compiler_fence(Ordering::SeqCst);
        let addr = self.buf_addr(idx);
        let mem = unsafe { slice::from_raw_parts_mut(EP_MEMORY.add(addr as _), buf.len()) };
        mem.copy_from_slice(buf);
        compiler_fence(Ordering::SeqCst);
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct EndpointData {
    ep_type: EndpointType, // only valid if used
    max_packet_size: u16,
    used: bool,
    double_buffered: bool,
}

impl EndpointData {
    const fn new() -> Self {
        Self {
            ep_type: EndpointType::Bulk,
            max_packet_size: 0,
            used: false,
            double_buffered: false,
        }
    }
}

/// RP2040 USB driver handle.
pub struct Driver<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    ep_in: [EndpointData; EP_COUNT],
    ep_out: [EndpointData; EP_COUNT],
    ep_mem_free: u16, // first free address in EP mem, in bytes.
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Create a new USB driver.
    pub fn new(_usb: Peri<'d, T>, _irq: impl Binding<T::Interrupt, InterruptHandler<T>>) -> Self {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        let regs = T::regs();
        unsafe {
            // zero fill regs
            let p = regs.as_ptr() as *mut u32;
            for i in 0..0x9c / 4 {
                p.add(i).write_volatile(0)
            }

            // zero fill epmem
            let p = EP_MEMORY as *mut u32;
            for i in 0..0x100 / 4 {
                p.add(i).write_volatile(0)
            }
        }

        regs.usb_muxing().write(|w| {
            w.set_to_phy(true);
            w.set_softcon(true);
        });
        regs.usb_pwr().write(|w| {
            w.set_vbus_detect(true);
            w.set_vbus_detect_override_en(true);
        });
        regs.main_ctrl().write(|w| {
            w.set_controller_en(true);
        });

        // Initialize the bus so that it signals that power is available
        BUS_WAKER.wake();

        Self {
            phantom: PhantomData,
            ep_in: [EndpointData::new(); EP_COUNT],
            ep_out: [EndpointData::new(); EP_COUNT],
            ep_mem_free: 0x180, // data buffer region
        }
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, T, D>, driver::EndpointAllocError> {
        // Enable double buffering for bulk endpoints to improve throughput.
        // Control endpoints don't benefit from double buffering.
        // Interrupt endpoints typically have small, infrequent transfers.
        // Isochronous could benefit but requires special offset handling.
        let double_buffered = ep_type == EndpointType::Bulk;

        trace!(
            "allocating type={:?} mps={:?} interval_ms={}, dir={:?}, double_buffered={}",
            ep_type,
            max_packet_size,
            interval_ms,
            D::dir(),
            double_buffered
        );

        let alloc = match D::dir() {
            Direction::Out => &mut self.ep_out,
            Direction::In => &mut self.ep_in,
        };

        let index = if let Some(addr) = ep_addr {
            // Use the specified endpoint address
            let requested_index = addr.index();
            if requested_index == 0 || requested_index >= EP_COUNT {
                return Err(EndpointAllocError);
            }
            if alloc[requested_index].used {
                return Err(EndpointAllocError);
            }
            Some((requested_index, &mut alloc[requested_index]))
        } else {
            // Find any available endpoint
            alloc.iter_mut().enumerate().find(|(i, ep)| {
                if *i == 0 {
                    return false; // reserved for control pipe
                }
                !ep.used
            })
        };

        let (index, ep) = index.ok_or(EndpointAllocError)?;
        assert!(!ep.used);

        // as per datasheet, the maximum buffer size is 64, except for isochronous
        // endpoints, which are allowed to be up to 1023 bytes.
        if (ep_type != EndpointType::Isochronous && max_packet_size > 64) || max_packet_size > 1023 {
            warn!("max_packet_size too high: {}", max_packet_size);
            return Err(EndpointAllocError);
        }

        // ep mem addrs must be 64-byte aligned, so there's no point in trying
        // to allocate smaller chunks to save memory.
        let len = (max_packet_size + 63) / 64 * 64;

        // For double buffering, allocate space for two buffers.
        let total_len = if double_buffered { len * 2 } else { len };

        let addr = self.ep_mem_free;
        if addr + total_len > EP_MEMORY_SIZE as u16 {
            warn!("Endpoint memory full");
            return Err(EndpointAllocError);
        }
        self.ep_mem_free += total_len;

        let buf = if double_buffered {
            EndpointBuffer::new_double_buffered(addr, len)
        } else {
            EndpointBuffer::new(addr, len)
        };

        trace!(
            "  index={} addr={} len={} double_buffered={}",
            index, buf.addr, buf.len, double_buffered
        );

        ep.ep_type = ep_type;
        ep.used = true;
        ep.max_packet_size = max_packet_size;
        ep.double_buffered = double_buffered;

        let ep_type_reg = match ep_type {
            EndpointType::Bulk => pac::usb_dpram::vals::EpControlEndpointType::BULK,
            EndpointType::Control => pac::usb_dpram::vals::EpControlEndpointType::CONTROL,
            EndpointType::Interrupt => pac::usb_dpram::vals::EpControlEndpointType::INTERRUPT,
            EndpointType::Isochronous => pac::usb_dpram::vals::EpControlEndpointType::ISOCHRONOUS,
        };

        match D::dir() {
            Direction::Out => T::dpram().ep_out_control(index - 1).write(|w| {
                w.set_enable(false);
                w.set_buffer_address(addr);
                w.set_interrupt_per_buff(true);
                w.set_endpoint_type(ep_type_reg);
                w.set_double_buffered(double_buffered);
            }),
            Direction::In => T::dpram().ep_in_control(index - 1).write(|w| {
                w.set_enable(false);
                w.set_buffer_address(addr);
                w.set_interrupt_per_buff(true);
                w.set_endpoint_type(ep_type_reg);
                w.set_double_buffered(double_buffered);
            }),
        }

        Ok(Endpoint {
            _phantom: PhantomData,
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, D::dir()),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            buf,
        })
    }
}

/// USB interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _uart: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        //let x = regs.istr().read().0;
        //trace!("USB IRQ: {:08x}", x);

        let ints = regs.ints().read();

        if ints.bus_reset() {
            regs.inte().write_clear(|w| w.set_bus_reset(true));
            BUS_WAKER.wake();
        }
        if ints.dev_resume_from_host() {
            regs.inte().write_clear(|w| w.set_dev_resume_from_host(true));
            BUS_WAKER.wake();
        }
        if ints.dev_suspend() {
            regs.inte().write_clear(|w| w.set_dev_suspend(true));
            BUS_WAKER.wake();
        }
        if ints.setup_req() {
            regs.inte().write_clear(|w| w.set_setup_req(true));
            EP_OUT_WAKERS[0].wake();
        }

        if ints.buff_status() {
            let s = regs.buff_status().read();
            regs.buff_status().write_value(s);

            for i in 0..EP_COUNT {
                if s.ep_in(i) {
                    EP_IN_WAKERS[i].wake();
                }
                if s.ep_out(i) {
                    EP_OUT_WAKERS[i].wake();
                }
            }
        }
    }
}

impl<'d, T: Instance> driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, Out>;
    type EndpointIn = Endpoint<'d, T, In>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, driver::EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, driver::EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let regs = T::regs();
        regs.inte().write(|w| {
            w.set_bus_reset(true);
            w.set_buff_status(true);
            w.set_dev_resume_from_host(true);
            w.set_dev_suspend(true);
            w.set_setup_req(true);
        });
        regs.int_ep_ctrl().write(|w| {
            w.set_int_ep_active(0xFFFE); // all EPs
        });
        regs.sie_ctrl().write(|w| {
            w.set_ep0_int_1buf(true);
            w.set_pullup_en(true);
        });

        trace!("enabled");

        (
            Bus {
                phantom: PhantomData,
                inited: false,
                ep_in: self.ep_in,
                ep_out: self.ep_out,
            },
            ControlPipe {
                _phantom: PhantomData,
                max_packet_size: control_max_packet_size,
            },
        )
    }
}

/// Type representing the RP USB bus.
pub struct Bus<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    ep_in: [EndpointData; EP_COUNT],
    ep_out: [EndpointData; EP_COUNT],
    inited: bool,
}

impl<'d, T: Instance> driver::Bus for Bus<'d, T> {
    async fn poll(&mut self) -> Event {
        poll_fn(move |cx| {
            BUS_WAKER.register(cx.waker());

            // TODO: implement VBUS detection.
            if !self.inited {
                self.inited = true;
                return Poll::Ready(Event::PowerDetected);
            }

            let regs = T::regs();
            let siestatus = regs.sie_status().read();
            let intrstatus = regs.intr().read();

            if siestatus.resume() || intrstatus.dev_resume_from_host() {
                regs.sie_status().write(|w| w.set_resume(true));
                return Poll::Ready(Event::Resume);
            }

            if siestatus.bus_reset() {
                regs.sie_status().write(|w| {
                    w.set_bus_reset(true);
                    w.set_setup_rec(true);
                });
                regs.buff_status().write(|w| w.0 = 0xFFFF_FFFF);
                regs.addr_endp().write(|w| w.set_address(0));

                for i in 1..EP_COUNT {
                    T::dpram().ep_in_control(i - 1).modify(|w| w.set_enable(false));
                    T::dpram().ep_out_control(i - 1).modify(|w| w.set_enable(false));
                }

                for w in &EP_IN_WAKERS {
                    w.wake()
                }
                for w in &EP_OUT_WAKERS {
                    w.wake()
                }
                return Poll::Ready(Event::Reset);
            }

            if siestatus.suspended() && intrstatus.dev_suspend() {
                regs.sie_status().write(|w| w.set_suspended(true));
                return Poll::Ready(Event::Suspend);
            }

            // no pending event. Reenable all irqs.
            regs.inte().write_set(|w| {
                w.set_bus_reset(true);
                w.set_dev_resume_from_host(true);
                w.set_dev_suspend(true);
            });
            Poll::Pending
        })
        .await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let n = ep_addr.index();

        if n == 0 {
            T::regs().ep_stall_arm().modify(|w| {
                if ep_addr.is_in() {
                    w.set_ep0_in(stalled);
                } else {
                    w.set_ep0_out(stalled);
                }
            });
        }

        let ctrl = if ep_addr.is_in() {
            T::dpram().ep_in_buffer_control(n)
        } else {
            T::dpram().ep_out_buffer_control(n)
        };

        ctrl.modify(|w| w.set_stall(stalled));

        let wakers = if ep_addr.is_in() { &EP_IN_WAKERS } else { &EP_OUT_WAKERS };
        wakers[n].wake();
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let n = ep_addr.index();

        let ctrl = if ep_addr.is_in() {
            T::dpram().ep_in_buffer_control(n)
        } else {
            T::dpram().ep_out_buffer_control(n)
        };

        ctrl.read().stall()
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        trace!("set_enabled {:?} {}", ep_addr, enabled);
        if ep_addr.index() == 0 {
            return;
        }

        let n = ep_addr.index();
        match ep_addr.direction() {
            Direction::In => {
                let ep_data = &self.ep_in[n];
                T::dpram().ep_in_control(n - 1).modify(|w| w.set_enable(enabled));

                if ep_data.double_buffered {
                    // For double-buffered IN endpoints, initialize both buffers.
                    // PID starts at DATA0, alternating between buffers.
                    T::dpram().ep_in_buffer_control(n).write(|w| {
                        w.set_pid(0, true); // Will be flipped to DATA0 on first write
                        w.set_pid(1, false); // DATA1 for second buffer
                    });
                } else {
                    T::dpram().ep_in_buffer_control(n).write(|w| {
                        w.set_pid(0, true); // first packet is DATA0, but PID is flipped before
                    });
                }
                EP_IN_WAKERS[n].wake();
            }
            Direction::Out => {
                let ep_data = &self.ep_out[n];
                T::dpram().ep_out_control(n - 1).modify(|w| w.set_enable(enabled));

                if ep_data.double_buffered {
                    // For double-buffered OUT endpoints, make both buffers available.
                    // This allows continuous reception without gaps.
                    let mps = ep_data.max_packet_size;
                    T::dpram().ep_out_buffer_control(n).write(|w| {
                        w.set_pid(0, false); // DATA0
                        w.set_length(0, mps);
                        w.set_pid(1, true); // DATA1
                        w.set_length(1, mps);
                    });
                    cortex_m::asm::delay(12);
                    T::dpram().ep_out_buffer_control(n).write(|w| {
                        w.set_pid(0, false);
                        w.set_length(0, mps);
                        w.set_available(0, true);
                        w.set_pid(1, true);
                        w.set_length(1, mps);
                        w.set_available(1, true);
                    });
                } else {
                    T::dpram().ep_out_buffer_control(n).write(|w| {
                        w.set_pid(0, false);
                        w.set_length(0, ep_data.max_packet_size);
                    });
                    cortex_m::asm::delay(12);
                    T::dpram().ep_out_buffer_control(n).write(|w| {
                        w.set_pid(0, false);
                        w.set_length(0, ep_data.max_packet_size);
                        w.set_available(0, true);
                    });
                }
                EP_OUT_WAKERS[n].wake();
            }
        }
    }

    async fn enable(&mut self) {}

    async fn disable(&mut self) {}

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

trait Dir {
    fn dir() -> Direction;
}

/// Type for In direction.
pub enum In {}
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Type for Out direction.
pub enum Out {}
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}

/// Endpoint for RP USB driver.
pub struct Endpoint<'d, T: Instance, D> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
    buf: EndpointBuffer<T>,
}

impl<'d, T: Instance> driver::Endpoint for Endpoint<'d, T, In> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        trace!("wait_enabled IN WAITING");
        let index = self.info.addr.index();
        poll_fn(|cx| {
            EP_IN_WAKERS[index].register(cx.waker());
            let val = T::dpram().ep_in_control(self.info.addr.index() - 1).read();
            if val.enable() { Poll::Ready(()) } else { Poll::Pending }
        })
        .await;
        trace!("wait_enabled IN OK");
    }
}

impl<'d, T: Instance> driver::Endpoint for Endpoint<'d, T, Out> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        trace!("wait_enabled OUT WAITING");
        let index = self.info.addr.index();
        poll_fn(|cx| {
            EP_OUT_WAKERS[index].register(cx.waker());
            let val = T::dpram().ep_out_control(self.info.addr.index() - 1).read();
            if val.enable() { Poll::Ready(()) } else { Poll::Pending }
        })
        .await;
        trace!("wait_enabled OUT OK");
    }
}

impl<'d, T: Instance> driver::EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();

        if self.buf.double_buffered {
            // Double-buffered read: wait for either buffer to have data.
            trace!("READ WAITING (double-buffered), buf.len() = {}", buf.len());

            let (buf_idx, val) = poll_fn(|cx| {
                EP_OUT_WAKERS[index].register(cx.waker());
                let val = T::dpram().ep_out_buffer_control(index).read();

                // Check if either buffer has completed (available cleared by hardware).
                // Use BUFF_CPU_SHOULD_HANDLE to determine which buffer to process.
                let buf0_ready = !val.available(0);
                let buf1_ready = !val.available(1);

                if buf0_ready || buf1_ready {
                    let should_handle = T::regs().buff_cpu_should_handle().read();
                    let preferred = if should_handle.ep_out(index) {
                        BufIdx::Buf1
                    } else {
                        BufIdx::Buf0
                    };

                    // Try the preferred buffer first, fall back to the other if not ready.
                    let buf_idx = if !val.available(preferred.index()) {
                        preferred
                    } else {
                        // Preferred not ready, use the other one (which must be ready).
                        preferred.other()
                    };

                    return Poll::Ready((buf_idx, val));
                }
                Poll::Pending
            })
            .await;

            let rx_len = val.length(buf_idx.index()) as usize;
            if rx_len > buf.len() {
                return Err(EndpointError::BufferOverflow);
            }

            self.buf.read_buf(buf_idx, &mut buf[..rx_len]);
            trace!("READ OK (buf{}), rx_len = {}", buf_idx.index(), rx_len);

            // Toggle PID for next transfer on this buffer and make it available again.
            let pid = !val.pid(buf_idx.index());
            let mps = self.info.max_packet_size;
            let current = T::dpram().ep_out_buffer_control(index).read();

            write_out_buffer_control_double::<T>(index, buf_idx, pid, mps, true, &current);

            Ok(rx_len)
        } else {
            // Single-buffered read (original implementation).
            trace!("READ WAITING, buf.len() = {}", buf.len());

            let val = poll_fn(|cx| {
                EP_OUT_WAKERS[index].register(cx.waker());
                let val = T::dpram().ep_out_buffer_control(index).read();
                if val.available(0) {
                    Poll::Pending
                } else {
                    Poll::Ready(val)
                }
            })
            .await;

            let rx_len = val.length(0) as usize;
            if rx_len > buf.len() {
                return Err(EndpointError::BufferOverflow);
            }
            self.buf.read(&mut buf[..rx_len]);

            trace!("READ OK, rx_len = {}", rx_len);

            let pid = !val.pid(0);
            T::dpram().ep_out_buffer_control(index).write(|w| {
                w.set_pid(0, pid);
                w.set_length(0, self.info.max_packet_size);
            });
            cortex_m::asm::delay(12);
            T::dpram().ep_out_buffer_control(index).write(|w| {
                w.set_pid(0, pid);
                w.set_length(0, self.info.max_packet_size);
                w.set_available(0, true);
            });

            Ok(rx_len)
        }
    }
}

impl<'d, T: Instance> driver::EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if buf.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let index = self.info.addr.index();

        if self.buf.double_buffered {
            // Double-buffered write: wait for either buffer to be free.
            trace!("WRITE WAITING (double-buffered), len = {}", buf.len());

            let (buf_idx, val) = poll_fn(|cx| {
                EP_IN_WAKERS[index].register(cx.waker());
                let val = T::dpram().ep_in_buffer_control(index).read();

                // Check if either buffer is free (available cleared by hardware after send).
                let buf0_free = !val.available(0);
                let buf1_free = !val.available(1);

                if buf0_free || buf1_free {
                    let should_handle = T::regs().buff_cpu_should_handle().read();
                    let preferred = if should_handle.ep_in(index) {
                        BufIdx::Buf1
                    } else {
                        BufIdx::Buf0
                    };

                    // Try the preferred buffer first, fall back to the other if not free.
                    let buf_idx = if !val.available(preferred.index()) {
                        preferred
                    } else {
                        // Preferred not free, use the other one (which must be free).
                        preferred.other()
                    };

                    return Poll::Ready((buf_idx, val));
                }
                Poll::Pending
            })
            .await;

            // Write data to the selected buffer.
            self.buf.write_buf(buf_idx, buf);

            // Toggle PID and mark buffer as full and available.
            let pid = !val.pid(buf_idx.index());
            let len = buf.len() as u16;
            let current = T::dpram().ep_in_buffer_control(index).read();

            write_in_buffer_control_double::<T>(index, buf_idx, pid, len, true, true, &current);

            trace!("WRITE OK (buf{})", buf_idx.index());
            Ok(())
        } else {
            // Single-buffered write (original implementation).
            trace!("WRITE WAITING, len = {}", buf.len());

            let val = poll_fn(|cx| {
                EP_IN_WAKERS[index].register(cx.waker());
                let val = T::dpram().ep_in_buffer_control(index).read();
                if val.available(0) {
                    Poll::Pending
                } else {
                    Poll::Ready(val)
                }
            })
            .await;

            self.buf.write(buf);

            let pid = !val.pid(0);
            T::dpram().ep_in_buffer_control(index).write(|w| {
                w.set_pid(0, pid);
                w.set_length(0, buf.len() as _);
                w.set_full(0, true);
            });
            cortex_m::asm::delay(12);
            T::dpram().ep_in_buffer_control(index).write(|w| {
                w.set_pid(0, pid);
                w.set_length(0, buf.len() as _);
                w.set_full(0, true);
                w.set_available(0, true);
            });

            trace!("WRITE OK");
            Ok(())
        }
    }
}

/// Control pipe for RP USB driver.
pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    max_packet_size: u16,
}

impl<'d, T: Instance> driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        64
    }

    async fn setup(&mut self) -> [u8; 8] {
        loop {
            trace!("SETUP read waiting");
            let regs = T::regs();
            regs.inte().write_set(|w| w.set_setup_req(true));

            poll_fn(|cx| {
                EP_OUT_WAKERS[0].register(cx.waker());
                let regs = T::regs();
                if regs.sie_status().read().setup_rec() {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            let mut buf = [0; 8];
            EndpointBuffer::<T>::new(0, 8).read(&mut buf);

            let regs = T::regs();
            regs.sie_status().write(|w| w.set_setup_rec(true));

            // set PID to 0, so (after toggling) first DATA is PID 1
            T::dpram().ep_in_buffer_control(0).write(|w| w.set_pid(0, false));
            T::dpram().ep_out_buffer_control(0).write(|w| w.set_pid(0, false));

            trace!("SETUP read ok");
            return buf;
        }
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, EndpointError> {
        let bufcontrol = T::dpram().ep_out_buffer_control(0);
        let pid = !bufcontrol.read().pid(0);
        bufcontrol.write(|w| {
            w.set_length(0, self.max_packet_size);
            w.set_pid(0, pid);
        });
        cortex_m::asm::delay(12);
        bufcontrol.write(|w| {
            w.set_length(0, self.max_packet_size);
            w.set_pid(0, pid);
            w.set_available(0, true);
        });

        trace!("control: data_out len={} first={} last={}", buf.len(), first, last);
        let val = poll_fn(|cx| {
            EP_OUT_WAKERS[0].register(cx.waker());
            let val = T::dpram().ep_out_buffer_control(0).read();
            if val.available(0) {
                Poll::Pending
            } else {
                Poll::Ready(val)
            }
        })
        .await;

        let rx_len = val.length(0) as _;
        trace!("control data_out DONE, rx_len = {}", rx_len);

        if rx_len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        EndpointBuffer::<T>::new(0x100, 64).read(&mut buf[..rx_len]);

        Ok(rx_len)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        trace!("control: data_in len={} first={} last={}", data.len(), first, last);

        if data.len() > 64 {
            return Err(EndpointError::BufferOverflow);
        }
        EndpointBuffer::<T>::new(0x100, 64).write(data);

        let bufcontrol = T::dpram().ep_in_buffer_control(0);
        let pid = !bufcontrol.read().pid(0);
        bufcontrol.write(|w| {
            w.set_length(0, data.len() as _);
            w.set_pid(0, pid);
            w.set_full(0, true);
        });
        cortex_m::asm::delay(12);
        bufcontrol.write(|w| {
            w.set_length(0, data.len() as _);
            w.set_pid(0, pid);
            w.set_full(0, true);
            w.set_available(0, true);
        });

        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());
            let bufcontrol = T::dpram().ep_in_buffer_control(0);
            if bufcontrol.read().available(0) {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
        trace!("control: data_in DONE");

        if last {
            // prepare status phase right away.
            let bufcontrol = T::dpram().ep_out_buffer_control(0);
            bufcontrol.write(|w| {
                w.set_length(0, 0);
                w.set_pid(0, true);
            });
            cortex_m::asm::delay(12);
            bufcontrol.write(|w| {
                w.set_length(0, 0);
                w.set_pid(0, true);
                w.set_available(0, true);
            });
        }

        Ok(())
    }

    async fn accept(&mut self) {
        trace!("control: accept");

        let bufcontrol = T::dpram().ep_in_buffer_control(0);
        bufcontrol.write(|w| {
            w.set_length(0, 0);
            w.set_pid(0, true);
            w.set_full(0, true);
        });
        cortex_m::asm::delay(12);
        bufcontrol.write(|w| {
            w.set_length(0, 0);
            w.set_pid(0, true);
            w.set_full(0, true);
            w.set_available(0, true);
        });

        // wait for completion before returning, needed so
        // set_address() doesn't happen early.
        poll_fn(|cx| {
            EP_IN_WAKERS[0].register(cx.waker());
            if bufcontrol.read().available(0) {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }

    async fn reject(&mut self) {
        trace!("control: reject");

        let regs = T::regs();
        regs.ep_stall_arm().write_set(|w| {
            w.set_ep0_in(true);
            w.set_ep0_out(true);
        });
        T::dpram().ep_out_buffer_control(0).write(|w| w.set_stall(true));
        T::dpram().ep_in_buffer_control(0).write(|w| w.set_stall(true));
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;

        let regs = T::regs();
        trace!("setting addr: {}", addr);
        regs.addr_endp().write(|w| w.set_address(addr))
    }
}
