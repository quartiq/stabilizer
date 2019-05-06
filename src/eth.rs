use core::{slice, cmp};
use stm32h7::stm32h7x3 as stm32;
use smoltcp::Result;
use smoltcp::time::Instant;
use smoltcp::wire::EthernetAddress;
use smoltcp::phy;

#[allow(dead_code)]
mod phy_consts {
    pub const PHY_REG_BCR: u8 = 0x00;
    pub const PHY_REG_BSR: u8 = 0x01;
    pub const PHY_REG_ID1: u8 = 0x02;
    pub const PHY_REG_ID2: u8 = 0x03;
    pub const PHY_REG_ANTX: u8 = 0x04;
    pub const PHY_REG_ANRX: u8 = 0x05;
    pub const PHY_REG_ANEXP: u8 = 0x06;
    pub const PHY_REG_ANNPTX: u8 = 0x07;
    pub const PHY_REG_ANNPRX: u8 = 0x08;
    pub const PHY_REG_SSR: u8 = 0x1F;  // Special Status Register
    pub const PHY_REG_CTL: u8 = 0x0D; // Ethernet PHY Register Control
    pub const PHY_REG_ADDAR: u8 = 0x0E; // Ethernet PHY Address or Data

    pub const PHY_REG_WUCSR: u16 = 0x8010;

    pub const PHY_REG_BCR_COLTEST: u16 = 1 << 7;
    pub const PHY_REG_BCR_FD: u16 = 1 << 8;
    pub const PHY_REG_BCR_ANRST: u16 = 1 << 9;
    pub const PHY_REG_BCR_ISOLATE: u16 = 1 << 10;
    pub const PHY_REG_BCR_POWERDN: u16 = 1 << 11;
    pub const PHY_REG_BCR_AN: u16 = 1 << 12;
    pub const PHY_REG_BCR_100M: u16 = 1 << 13;
    pub const PHY_REG_BCR_LOOPBACK: u16 = 1 << 14;
    pub const PHY_REG_BCR_RESET: u16 = 1 << 15;

    pub const PHY_REG_BSR_JABBER: u16 = 1 << 1;
    pub const PHY_REG_BSR_UP: u16 = 1 << 2;
    pub const PHY_REG_BSR_FAULT: u16 = 1 << 4;
    pub const PHY_REG_BSR_ANDONE: u16 = 1 << 5;

    pub const PHY_REG_SSR_ANDONE: u16 = 1 << 12;
    pub const PHY_REG_SSR_SPEED: u16 = 0b111 << 2;
    pub const PHY_REG_SSR_10BASE_HD: u16 = 0b001 << 2;
    pub const PHY_REG_SSR_10BASE_FD: u16 = 0b101 << 2;
    pub const PHY_REG_SSR_100BASE_HD: u16 = 0b010 << 2;
    pub const PHY_REG_SSR_100BASE_FD: u16 = 0b110 << 2;
}
use self::phy_consts::*;

const EMAC_DES3_OWN: u32 = 0x8000_0000;
const EMAC_DES3_CTXT: u32 = 0x4000_0000;
const EMAC_DES3_FD: u32 = 0x2000_0000;
const EMAC_DES3_LD: u32 = 0x1000_0000;
const EMAC_DES3_ES: u32 = 0x0000_8000;
const EMAC_TDES2_IOC: u32 = 0x8000_0000;
const EMAC_RDES3_IOC: u32 = 0x4000_0000;
const EMAC_RDES3_PL: u32 = 0x0000_7FFF;
const EMAC_RDES3_BUF1V: u32 = 0x0100_0000;
const EMAC_TDES2_B1L: u32 = 0x0000_3FFF;
const EMAC_DES0_BUF1AP: u32 = 0xFFFF_FFFF;

// 6 DMAC, 6 SMAC, 4 q tag, 2 ethernet type II, 1500 ip MTU, 4 CRC, 2 padding
const ETH_BUFFER_SIZE: usize = 1524;
const ETH_DESC_U32_SIZE: usize = 4;
const ETH_TX_BUFFER_COUNT: usize = 4;
const ETH_RX_BUFFER_COUNT: usize = 4;

#[allow(dead_code)]
mod cr_consts {
    /* For HCLK 60-100 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
    /* For HCLK 100-150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
    /* For HCLK 20-35 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
    /* For HCLK 35-60 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
    /* For HCLK 150-250 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
    /* For HCLK 250-300 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_124: u8 = 5;
}
use self::cr_consts::*;

// set clock range in MAC MII address register
// 200 MHz AHB clock = eth_hclk
const CLOCK_RANGE: u8 = ETH_MACMIIAR_CR_HCLK_DIV_102;


pub fn setup(rcc: &stm32::RCC, syscfg: &stm32::SYSCFG) {
    rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());
    rcc.ahb1enr.modify(|_, w| {
        w.eth1macen().set_bit()
         .eth1txen().set_bit()
         .eth1rxen().set_bit()
    });
    syscfg.pmcr.modify(|_, w| unsafe { w.epis().bits(0b100) });  // RMII
    //rcc.ahb1rstr.modify(|_, w| w.eth1macrst().set_bit());
    //rcc.ahb1rstr.modify(|_, w| w.eth1macrst().clear_bit());
}

pub fn setup_pins(gpioa: &stm32::GPIOA, gpiob: &stm32::GPIOB,
                  gpioc: &stm32::GPIOC, gpiog: &stm32::GPIOG) {
    // PA1 RMII_REF_CLK
    gpioa.moder.modify(|_, w| w.moder1().alternate());
    gpioa.afrl.modify(|_, w| w.afr1().af11());
    gpioa.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());
    // PA2 RMII_MDIO
    gpioa.moder.modify(|_, w| w.moder2().alternate());
    gpioa.afrl.modify(|_, w| w.afr2().af11());
    gpioa.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());
    // PC1 RMII_MDC
    gpioc.moder.modify(|_, w| w.moder1().alternate());
    gpioc.afrl.modify(|_, w| w.afr1().af11());
    gpioc.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());
    // PA7 RMII_CRS_DV
    gpioa.moder.modify(|_, w| w.moder7().alternate());
    gpioa.afrl.modify(|_, w| w.afr7().af11());
    gpioa.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());
    // PC4 RMII_RXD0
    gpioc.moder.modify(|_, w| w.moder4().alternate());
    gpioc.afrl.modify(|_, w| w.afr4().af11());
    gpioc.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
    // PC5 RMII_RXD1
    gpioc.moder.modify(|_, w| w.moder5().alternate());
    gpioc.afrl.modify(|_, w| w.afr5().af11());
    gpioc.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
    // PB11 RMII_TX_EN
    gpiob.moder.modify(|_, w| w.moder11().alternate());
    gpiob.afrh.modify(|_, w| w.afr11().af11());
    gpiob.ospeedr.modify(|_, w| w.ospeedr11().very_high_speed());
    // PB12 RXII_TXD0
    gpiob.moder.modify(|_, w| w.moder12().alternate());
    gpiob.afrh.modify(|_, w| w.afr12().af11());
    gpiob.ospeedr.modify(|_, w| w.ospeedr12().very_high_speed());
    // PG14 RMII TXD1
    gpiog.moder.modify(|_, w| w.moder14().alternate());
    gpiog.afrh.modify(|_, w| w.afr14().af11());
    gpiog.ospeedr.modify(|_, w| w.ospeedr14().very_high_speed());
}

const PHY_ADDR: u8 = 0;

fn phy_read(reg_addr: u8, mac: &stm32::ETHERNET_MAC) -> u16 {
    while mac.macmdioar.read().mb().bit_is_set() {}
    mac.macmdioar.modify(|_, w| unsafe {
        w
            .pa().bits(PHY_ADDR)
            .rda().bits(reg_addr)
            .goc().bits(0b11)  // read
            .cr().bits(CLOCK_RANGE)
            .mb().set_bit()
    });
    while mac.macmdioar.read().mb().bit_is_set() {}
    mac.macmdiodr.read().md().bits()
}

fn phy_write(reg_addr: u8, reg_data: u16, mac: &stm32::ETHERNET_MAC) {
    while mac.macmdioar.read().mb().bit_is_set() {}
    mac.macmdiodr.write(|w| unsafe { w.md().bits(reg_data) });
    mac.macmdioar.modify(|_, w| unsafe {
        w
            .pa().bits(PHY_ADDR)
            .rda().bits(reg_addr)
            .goc().bits(0b01)  // write
            .cr().bits(CLOCK_RANGE)
            .mb().set_bit()
    });
    while mac.macmdioar.read().mb().bit_is_set() {}
}

// Writes a value to an extended PHY register in MMD address space
fn phy_write_ext(reg_addr: u16, reg_data: u16, mac: &stm32::ETHERNET_MAC) {
    phy_write(PHY_REG_CTL, 0x0003, mac); // set address
    phy_write(PHY_REG_ADDAR, reg_addr, mac);
    phy_write(PHY_REG_CTL, 0x4003, mac); // set data
    phy_write(PHY_REG_ADDAR, reg_data, mac);
}

#[repr(align(4))]
struct RxRing {
    desc_buf: [[u32; ETH_DESC_U32_SIZE]; ETH_RX_BUFFER_COUNT],
    pkt_buf: [[u8; ETH_BUFFER_SIZE]; ETH_RX_BUFFER_COUNT],
    cur_desc: usize,
}

impl RxRing {
    const fn new() -> Self {
        Self {
            desc_buf: [[0; ETH_DESC_U32_SIZE]; ETH_RX_BUFFER_COUNT],
            pkt_buf: [[0; ETH_BUFFER_SIZE]; ETH_RX_BUFFER_COUNT],
            cur_desc: 0,
        }
    }

    unsafe fn init(&mut self, dma: &stm32::ETHERNET_DMA) {
        assert_eq!(self.desc_buf[0].len() % 4, 0);
        assert_eq!(self.pkt_buf[0].len() % 4, 0);

        for i in 0..self.desc_buf.len() {
            for j in 0..self.desc_buf[0].len() {
                self.desc_buf[i][j] = 0;
            }
            for j in 0..self.pkt_buf[0].len() {
                self.pkt_buf[i][j] = 0;
            }
        }

        let addr = &self.desc_buf as *const _ as u32;
        assert_eq!(addr & 0x3, 0);
        dma.dmacrx_dlar.write(|w| w.bits(addr));
        dma.dmacrx_rlr.write(|w| w.rdrl().bits(self.desc_buf.len() as u16 - 1));

        self.cur_desc = 0;
        for _ in 0..self.desc_buf.len() {
            self.buf_release()
        }
    }

    fn next_desc(&self) -> usize {
        (self.cur_desc + 1) % self.desc_buf.len()
    }

    // not owned by DMA
    fn buf_owned(&self) -> bool {
        self.desc_buf[self.cur_desc][3] & EMAC_DES3_OWN == 0
    }

    fn buf_valid(&self) -> bool {
        self.desc_buf[self.cur_desc][3] &
            (EMAC_DES3_FD | EMAC_DES3_LD | EMAC_DES3_ES | EMAC_DES3_CTXT) ==
            (EMAC_DES3_FD | EMAC_DES3_LD)
    }

    unsafe fn buf_as_slice<'a>(&self) -> &'a [u8] {
        let len = (self.desc_buf[self.cur_desc][3] & EMAC_RDES3_PL) as usize;
        let len = cmp::min(len, ETH_BUFFER_SIZE);
        let addr = &self.pkt_buf[self.cur_desc] as *const u8;
        slice::from_raw_parts(addr, len)
    }

    fn buf_release(&mut self) {
        let addr = &self.pkt_buf[self.cur_desc] as *const _;
        self.desc_buf[self.cur_desc][0] = addr as u32 & EMAC_DES0_BUF1AP;
        self.desc_buf[self.cur_desc][3] = EMAC_RDES3_BUF1V | EMAC_RDES3_IOC | EMAC_DES3_OWN;

        let addr = &self.desc_buf[self.cur_desc] as *const _ as u32;
        assert_eq!(addr & 0x3, 0);
        let dma = unsafe { stm32::Peripherals::steal().ETHERNET_DMA };
        dma.dmacrx_dtpr.write(|w| unsafe { w.bits(addr) });

        self.cur_desc = self.next_desc();
    }
}

#[repr(align(4))]
struct TxRing {
    desc_buf: [[u32; ETH_DESC_U32_SIZE]; ETH_TX_BUFFER_COUNT],
    pkt_buf: [[u8; ETH_BUFFER_SIZE]; ETH_TX_BUFFER_COUNT],
    cur_desc: usize,
}

impl TxRing {
    const fn new() -> Self {
        Self {
            desc_buf: [[0; ETH_DESC_U32_SIZE]; ETH_TX_BUFFER_COUNT],
            pkt_buf: [[0; ETH_BUFFER_SIZE]; ETH_TX_BUFFER_COUNT],
            cur_desc: 0,
        }
    }

    unsafe fn init(&mut self, dma: &stm32::ETHERNET_DMA) {
        assert_eq!(self.desc_buf[0].len() % 4, 0);
        assert_eq!(self.pkt_buf[0].len() % 4, 0);

        for i in 0..self.desc_buf.len() {
            for j in 0..self.desc_buf[0].len() {
                self.desc_buf[i][j] = 0;
            }
            for j in 0..self.pkt_buf[0].len() {
                self.pkt_buf[i][j] = 0;
            }
        }
        self.cur_desc = 0;

        let addr = &self.desc_buf as *const _ as u32;
        assert_eq!(addr & 0x3, 0);
        dma.dmactx_dlar.write(|w| w.bits(addr));
        dma.dmactx_rlr.write(|w| w.tdrl().bits(self.desc_buf.len() as u16 - 1));
        let addr = &self.desc_buf[0] as *const _ as u32;
        assert_eq!(addr & 0x3, 0);
        dma.dmactx_dtpr.write(|w| w.bits(addr));
    }

    fn next_desc(&self) -> usize {
        (self.cur_desc + 1) % self.desc_buf.len()
    }

    // not owned by DMA
    fn buf_owned(&self) -> bool {
        self.desc_buf[self.cur_desc][3] & EMAC_DES3_OWN == 0
    }

    unsafe fn buf_as_slice_mut<'a>(&mut self, len: usize) -> &'a mut [u8] {
        let len = cmp::min(len, ETH_BUFFER_SIZE);
        self.desc_buf[self.cur_desc][2] = EMAC_TDES2_IOC | (len as u32 & EMAC_TDES2_B1L);
        let addr = &self.pkt_buf[self.cur_desc] as *const _ as *mut u8;
        self.desc_buf[self.cur_desc][0] = addr as u32 & EMAC_DES0_BUF1AP;
        slice::from_raw_parts_mut(addr, len)
    }

    fn buf_release(&mut self) {
        self.desc_buf[self.cur_desc][3] = EMAC_DES3_OWN | EMAC_DES3_FD | EMAC_DES3_LD;
        self.cur_desc = self.next_desc();

        let addr = &self.desc_buf[self.cur_desc] as *const _ as u32;
        assert_eq!(addr & 0x3, 0);
        let dma = unsafe { stm32::Peripherals::steal().ETHERNET_DMA };
        dma.dmactx_dtpr.write(|w| unsafe { w.bits(addr) });
    }
}

pub struct Device {
    rx: RxRing,
    tx: TxRing,
}

impl Device {
    pub const fn new() -> Self {
        Self{ rx: RxRing::new(), tx: TxRing::new() }
    }

    // After `init` is called, `Device` shall not be moved.
    pub unsafe fn init(&mut self, mac: EthernetAddress,
                       eth_mac: &stm32::ETHERNET_MAC,
                       eth_dma: &stm32::ETHERNET_DMA,
                       eth_mtl: &stm32::ETHERNET_MTL,
                       ) {
        eth_dma.dmamr.modify(|_, w| w.swr().set_bit());
        while eth_dma.dmamr.read().swr().bit_is_set() {}

        // 200 MHz
        eth_mac.mac1ustcr.modify(|_, w| w.tic_1us_cntr().bits(200 - 1));

        // Configuration Register
        eth_mac.maccr.modify(|_, w| {
            w
                .arpen().clear_bit()
                .ipc().set_bit()
                .ipg().bits(0b000)  // 96 bit
                .ecrsfd().clear_bit()
                .dcrs().clear_bit()
                .bl().bits(0b00)  // 19
                .prelen().bits(0b00)  // 7
                // CRC stripping for Type frames
                .cst().set_bit()
                // Fast Ethernet speed
                .fes().set_bit()
                // Duplex mode
                .dm().set_bit()
                // Automatic pad/CRC stripping
                .acs().set_bit()
                // Retry disable in half-duplex mode
                .dr().set_bit()
        });
        eth_mac.macecr.modify(|_, w| {
            w
                .eipgen().clear_bit()
                .usp().clear_bit()
                .spen().clear_bit()
                .dcrcc().clear_bit()
        });
        // Set the MAC address
        eth_mac.maca0lr.write(|w|
            w.addrlo().bits( u32::from(mac.0[0]) |
                            (u32::from(mac.0[1]) <<  8) |
                            (u32::from(mac.0[2]) << 16) |
                            (u32::from(mac.0[3]) << 24))
        );
        eth_mac.maca0hr.write(|w|
            w.addrhi().bits( u16::from(mac.0[4]) |
                            (u16::from(mac.0[5]) << 8))
        );
        // frame filter register
        eth_mac.macpfr.modify(|_, w| {
            w
                .dntu().clear_bit()
                .ipfe().clear_bit()
                .vtfe().clear_bit()
                .hpf().clear_bit()
                .saf().clear_bit()
                .saif().clear_bit()
                .pcf().bits(0b00)
                .dbf().clear_bit()
                .pm().clear_bit()
                .daif().clear_bit()
                .hmc().clear_bit()
                .huc().clear_bit()
                // Receive All
                .ra().clear_bit()
                // Promiscuous mode
                .pr().clear_bit()
        });
        eth_mac.macwtr.write(|w| w.pwe().clear_bit());
        // Flow Control Register
        eth_mac.macqtx_fcr.modify(|_, w| {
            // Pause time
            w.pt().bits(0x100)
        });
        eth_mac.macrx_fcr.modify(|_, w| w);
        eth_mtl.mtlrx_qomr.modify(|_, w|
            w
                // Receive store and forward
                .rsf().set_bit()
                // Dropping of TCP/IP checksum error frames disable
                .dis_tcp_ef().clear_bit()
                // Forward error frames
                .fep().clear_bit()
                // Forward undersized good packets
                .fup().clear_bit()
        );
        eth_mtl.mtltx_qomr.modify(|_, w| {
            w
                // Transmit store and forward
                .tsf().set_bit()
        });

        if  (phy_read(PHY_REG_ID1, eth_mac) != 0x0007) | (phy_read(PHY_REG_ID2, eth_mac) != 0xC131) {
            error!("PHY ID error!");
        }

        phy_write(PHY_REG_BCR, PHY_REG_BCR_RESET, eth_mac);
        while phy_read(PHY_REG_BCR, eth_mac) & PHY_REG_BCR_RESET == PHY_REG_BCR_RESET {};
        phy_write_ext(PHY_REG_WUCSR, 0, eth_mac);
        phy_write(PHY_REG_BCR, PHY_REG_BCR_AN | PHY_REG_BCR_ANRST | PHY_REG_BCR_100M, eth_mac);
        /*
        while phy_read(PHY_REG_BSR) & PHY_REG_BSR_UP == 0 {};
        while phy_read(PHY_REG_BSR) & PHY_REG_BSR_ANDONE == 0 {};
        while phy_read(PHY_REG_SSR) & (PHY_REG_SSR_ANDONE | PHY_REG_SSR_SPEED)
            != PHY_REG_SSR_ANDONE | PHY_REG_SSR_100BASE_FD {};
        */

        // operation mode register
        eth_dma.dmamr.modify(|_, w| {
            w
                .intm().bits(0b00)
                // Rx Tx priority ratio 1:1
                .pr().bits(0b000)
                .txpr().clear_bit()
                .da().clear_bit()
        });
        // bus mode register
        eth_dma.dmasbmr.modify(|_, w| {
            // Address-aligned beats
            w.aal().set_bit()
                // Fixed burst
                .fb().set_bit()
        });
        eth_dma.dmaccr.modify(|_, w| {
            w
                .dsl().bits(0)
                .pblx8().clear_bit()
                .mss().bits(536)
        });
        eth_dma.dmactx_cr.modify(|_, w| {
            w
                // Tx DMA PBL
                .txpbl().bits(32)
                .tse().clear_bit()
                // Operate on second frame
                .osf().clear_bit()
        });

        eth_dma.dmacrx_cr.modify(|_, w| {
            w
                // receive buffer size
                .rbsz().bits(ETH_BUFFER_SIZE as u16)
                // Rx DMA PBL
                .rxpbl().bits(32)
                // Disable flushing of received frames
                .rpf().clear_bit()
        });

        self.rx.init(eth_dma);
        self.tx.init(eth_dma);

        // Manage MAC transmission and reception
        eth_mac.maccr.modify(|_, w| {
            w.re().bit(true) // Receiver Enable
                .te().bit(true) // Transmiter Enable
        });
        eth_mtl.mtltx_qomr.modify(|_, w| w.ftq().set_bit());

        // Manage DMA transmission and reception
        eth_dma.dmactx_cr.modify(|_, w| w.st().set_bit());
        eth_dma.dmacrx_cr.modify(|_, w| w.sr().set_bit());

        eth_dma.dmacsr.modify(|_, w|
            w.tps().set_bit()
                .rps().set_bit()
        );
    }
}

impl<'a, 'b> phy::Device<'a> for &'b mut Device {
    type RxToken = RxToken<'a>;
    type TxToken = TxToken<'a>;

    fn capabilities(&self) -> phy::DeviceCapabilities {
        let mut capabilities = phy::DeviceCapabilities::default();
        // ethernet frame type II (6 smac, 6 dmac, 2 ethertype),
        // sans CRC (4), 1500 IP MTU
        capabilities.max_transmission_unit = 1514;
        capabilities.max_burst_size = Some(self.tx.desc_buf.len());
        capabilities
    }

    fn receive(&mut self) -> Option<(RxToken, TxToken)> {
        // Skip all queued packets with errors.
        while self.rx.buf_owned() && !self.rx.buf_valid() {
            self.rx.buf_release()
        }

        if !(self.rx.buf_owned() && self.tx.buf_owned()) {
            return None
        }

        Some((RxToken(&mut self.rx), TxToken(&mut self.tx)))
    }

    fn transmit(&mut self) -> Option<TxToken> {
        if !self.tx.buf_owned() {
            return None
        }

        Some(TxToken(&mut self.tx))
    }
}

pub struct RxToken<'a>(&'a mut RxRing);

impl<'a> phy::RxToken for RxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R>
            where F: FnOnce(&[u8]) -> Result<R> {
        let result = f(unsafe { self.0.buf_as_slice() });
        self.0.buf_release();
        result
    }
}

pub struct TxToken<'a>(&'a mut TxRing);

impl<'a> phy::TxToken for TxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
            where F: FnOnce(&mut [u8]) -> Result<R> {
        let result = f(unsafe { self.0.buf_as_slice_mut(len) });
        self.0.buf_release();
        result
    }
}

pub unsafe fn interrupt_handler(eth_dma: &stm32::ETHERNET_DMA) {
    eth_dma.dmacsr.write(|w|
        w
            .nis().set_bit()
            .ri().set_bit()
            .ti().set_bit()
    );
}

pub unsafe fn enable_interrupt(eth_dma: &stm32::ETHERNET_DMA) {
    eth_dma.dmacier.modify(|_, w|
        w
            .nie().set_bit()
            .rie().set_bit()
            .tie().set_bit()
    );
}
