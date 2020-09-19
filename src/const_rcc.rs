use crate::{
    pac::{
        flash::acr::LATENCY_A,
        rcc::cfgr::{PLLSRC_A, USBPRE_A},
    },
    rcc::HSI,
    time::Hertz,
};

/// Type to get the clock configuration in a const context.
#[derive(Debug)]
pub struct ClockConfig {
    hse: Option<u32>,
    hclk: Option<u32>,
    pclk1: Option<u32>,
    pclk2: Option<u32>,
    sysclk: Option<u32>,
    adcclk: Option<u32>,
}

/// Calculated configuration to be passed to the RCC.
#[derive(Debug)]
pub struct RccConfig {
    pub(crate) latency: LATENCY_A,
    pub(crate) pllmul_bits: Option<u8>,
    pub(crate) pllsrc: PLLSRC_A,
    pub(crate) adcpre_bits: u8,
    pub(crate) ppre1_bits: u8,
    pub(crate) ppre2_bits: u8,
    pub(crate) hpre_bits: u8,
    pub(crate) usbpre: USBPRE_A,
    pub(crate) sysclk: u32,
    pub(crate) hclk: u32,
    pub(crate) pclk1: u32,
    pub(crate) pclk2: u32,
    pub(crate) adcclk: u32,
    pub(crate) ppre1: u8,
    pub(crate) ppre2: u8,
    pub(crate) usbclk_valid: bool,
}

impl ClockConfig {
    /// Create a new default clock configuration.
    pub const fn new() -> Self {
        Self {
            hse: None,
            hclk: None,
            pclk1: None,
            pclk2: None,
            sysclk: None,
            adcclk: None,
        }
    }

    /// Uses HSE (external oscillator) instead of HSI (internal RC oscillator) as the clock source.
    /// Will result in a hang if an external oscillator is not connected or it fails to start.
    /// The frequency specified must be the frequency of the external oscillator.
    pub const fn use_hse(mut self, freq: Hertz) -> Self {
        self.hse = Some(freq.0);
        self
    }

    /// Sets the desired frequency for the HCLK clock.
    pub const fn hclk(mut self, freq: Hertz) -> Self {
        self.hclk = Some(freq.0);
        self
    }

    /// Sets the desired frequency for the PCKL1 clock.
    pub const fn pclk1(mut self, freq: Hertz) -> Self {
        self.pclk1 = Some(freq.0);
        self
    }

    /// Sets the desired frequency for the PCLK2 clock.
    pub const fn pclk2(mut self, freq: Hertz) -> Self {
        self.pclk2 = Some(freq.0);
        self
    }

    /// Sets the desired frequency for the SYSCLK clock.
    pub const fn sysclk(mut self, freq: Hertz) -> Self {
        self.sysclk = Some(freq.0);
        self
    }

    /// Sets the desired frequency for the ADCCLK clock.
    pub const fn adcclk(mut self, freq: Hertz) -> Self {
        self.adcclk = Some(freq.0);
        self
    }

    /// Calculates the appropriate configuration.
    pub const fn get_config(self) -> RccConfig {
        let pllsrcclk = if let Some(hse) = self.hse {
            hse
        } else {
            HSI / 2
        };
        let sysclk = if let Some(sysclk) = self.sysclk {
            sysclk
        } else {
            pllsrcclk
        };
        let pllmul = sysclk / pllsrcclk;

        let (pllmul_bits, sysclk) = if pllmul == 1 {
            let real_sysclk = if let Some(hse) = self.hse { hse } else { HSI };
            (None, real_sysclk)
        } else {
            #[cfg(not(feature = "connectivity"))]
            let pllmul = u32_min(u32_max(pllmul, 2), 16);

            #[cfg(feature = "connectivity")]
            let pllmul = u32_min(u32_max(pllmul, 4), 9);

            (Some(pllmul as u8 - 2), pllsrcclk * pllmul)
        };

        let hpre_bits = if let Some(hclk) = self.hclk {
            match sysclk / hclk {
                0 | 1 => 0b0111,
                2 => 0b1000,
                3..=5 => 0b1001,
                6..=11 => 0b1010,
                12..=39 => 0b1011,
                40..=95 => 0b1100,
                96..=191 => 0b1101,
                192..=383 => 0b1110,
                _ => 0b1111,
            }
        } else {
            0b0111
        };
        let hclk = if hpre_bits >= 0b1100 {
            sysclk / (1 << (hpre_bits - 0b0110))
        } else {
            sysclk / (1 << (hpre_bits - 0b0111))
        };

        let ppre1_bits = if let Some(pclk1) = self.pclk1 {
            match hclk / pclk1 {
                0 | 1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            }
        } else {
            0b011
        };
        let ppre1 = 1 << (ppre1_bits - 0b011);
        let pclk1 = hclk / ppre1 as u32;

        let ppre2_bits = if let Some(pclk2) = self.pclk2 {
            match hclk / pclk2 {
                0 | 1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            }
        } else {
            0b011
        };
        let ppre2 = 1 << (ppre2_bits - 0b011);
        let pclk2 = hclk / ppre2 as u32;

        #[cfg(any(feature = "stm32f103", feature = "stm32f101", feature = "connectivity"))]
        let latency = if sysclk <= 24_000_000 {
            LATENCY_A::WS0
        } else if sysclk <= 48_000_000 {
            LATENCY_A::WS1
        } else {
            LATENCY_A::WS2
        };

        // the USB clock is only valid if an external crystal is used, the PLL is enabled, and the
        // PLL output frequency is a supported one.
        // usbpre == false: divide clock by 1.5, otherwise no division
        let (usbpre, usbclk_valid) = match (self.hse, pllmul_bits, sysclk) {
            (Some(_), Some(_), 72_000_000) => (USBPRE_A::DIV1_5, true),
            (Some(_), Some(_), 48_000_000) => (USBPRE_A::DIV1, true),
            _ => (USBPRE_A::DIV1, false),
        };

        let adcpre_bits = if let Some(adcclk) = self.adcclk {
            match pclk2 / adcclk {
                0..=2 => 0b00,
                3..=4 => 0b01,
                5..=7 => 0b10,
                _ => 0b11,
            }
        } else {
            0b11
        };
        let apre = (adcpre_bits + 1) << 1;
        let adcclk = pclk2 / apre as u32;

        let pllsrc = if let Some(_) = self.hse {
            PLLSRC_A::HSE_DIV_PREDIV
        } else {
            PLLSRC_A::HSI_DIV2
        };

        RccConfig {
            latency,
            pllmul_bits,
            pllsrc,
            adcpre_bits,
            ppre1_bits,
            ppre2_bits,
            hpre_bits,
            usbpre,
            sysclk,
            hclk,
            pclk1,
            pclk2,
            adcclk,
            ppre1,
            ppre2,
            usbclk_valid,
        }
    }
}

const fn u32_min(a: u32, b: u32) -> u32 {
    if a < b {
        a
    } else {
        b
    }
}

const fn u32_max(a: u32, b: u32) -> u32 {
    if a > b {
        a
    } else {
        b
    }
}
