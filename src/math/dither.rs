#![forbid(unsafe_code)]

use super::color::{
    LinearRgba16, linear_u16_to_srgb_8_8, u16_to_u8_linear, unpremultiply_u16,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum DitherMode {
    #[default]
    BlueNoise,
    InterleavedGradientNoise,
    None,
}

// 16x16 toroidal Ulichney Void-and-Cluster blue-noise threshold matrix (bijection of 0..=255).
pub static BLUE_NOISE_16X16: [u8; 256] = [
    173, 202,  99, 242, 108, 194, 119, 209,  27,  91, 130, 231, 168,  20,  42, 133,
     25,  78, 126,  34,  60,   5,  75, 252, 174,  37, 199,  10,  98, 255, 196,  65,
    245, 154, 208, 232, 140, 162,  48, 226, 148, 107, 240,  67, 123, 146, 182, 222,
      2, 114,  45, 172,  90, 197, 132,  17,  84,  56, 160, 188,  32,  81,  53, 105,
     88, 190,  68,  18, 221,  30, 100, 206, 185,   1, 135, 214, 228,  16, 165, 137,
    212, 238, 147, 121, 250, 180,  62, 235, 117, 248,  93,  47, 112, 200, 241,  39,
     28,  58, 167,   7,  77, 110, 155,  40, 169,  70,  24, 175, 153,  61, 122, 177,
    227, 103, 134, 205,  49, 215, 129,  11, 220, 139, 198,  80, 244,   8,  96,  74,
    156,  14, 186, 243,  95,  26, 193,  86,  54, 104, 233,  36, 131, 216, 143, 195,
     44, 118,  82,  35, 159, 229, 145, 254, 178, 161,   6, 115, 184,  52,  22, 253,
    218,  64, 234, 138, 176,  69,  46, 120,  21, 210,  63, 150, 230,  89, 170, 106,
      0, 164, 203,  19, 113,   4, 102, 204,  76, 237,  97, 192,  15,  71, 207, 128,
     79, 183,  92,  55, 247, 191, 223, 157,  31, 136,  43, 125, 163, 246,  38, 149,
    225,  33, 124, 211, 151,  85, 127,  57, 171, 249,  83, 201,  29, 101,  59, 189,
    109, 251, 144,  72,  23,  41, 239,  13, 187, 111,   3, 224, 179, 141, 236,  12,
    158,  50,   9, 181, 219, 166,  94, 142,  66, 217, 152,  51,  73, 116, 213,  87,
];

#[inline]
pub fn blue_noise_u8(x: u32, y: u32) -> u8 {
    BLUE_NOISE_16X16[(((y & 15) << 4) | (x & 15)) as usize]
}

// Pure integer Q32 Interleaved Gradient Noise (Jimenez formula, zero float drift).
#[inline]
pub const fn ign_u8(x: u32, y: u32) -> u8 {
    let inner = x.wrapping_mul(288_237_609).wrapping_add(y.wrapping_mul(25_070_376));
    let prod = (inner as u128).wrapping_mul(227_559_924_419);
    ((prod as u64) >> 56) as u8
}

#[inline]
pub fn dither_threshold(mode: DitherMode, x: u32, y: u32) -> u8 {
    match mode {
        DitherMode::BlueNoise => blue_noise_u8(x, y),
        DitherMode::InterleavedGradientNoise => ign_u8(x, y),
        DitherMode::None => 128,
    }
}

// Quantizes 8.8 sRGB [0..=65280] using threshold in [0..=255].
// Preserves exact integer values (incl. 0 and 255) with zero speckle and exact spatial mean.
#[inline]
pub const fn quantize_srgb_8_8(s88: u16, threshold: u8) -> u8 {
    let v = ((s88 as u32) + (threshold as u32)) >> 8;
    if v > 255 { 255 } else { v as u8 }
}

#[inline]
pub fn linear_u16_to_srgb_u8_dithered(v: u16, mode: DitherMode, x: u32, y: u32) -> u8 {
    let s88 = linear_u16_to_srgb_8_8(v);
    quantize_srgb_8_8(s88, dither_threshold(mode, x, y))
}

#[inline]
pub fn encode_linear_rgba16_srgb_dithered(
    px: LinearRgba16,
    mode: DitherMode,
    x: u32,
    y: u32,
) -> [u8; 4] {
    if px.a == 0 {
        return [0, 0, 0, 0];
    }
    let t = dither_threshold(mode, x, y);
    if px.a == 65535 {
        return [
            quantize_srgb_8_8(linear_u16_to_srgb_8_8(px.r), t),
            quantize_srgb_8_8(linear_u16_to_srgb_8_8(px.g), t),
            quantize_srgb_8_8(linear_u16_to_srgb_8_8(px.b), t),
            255,
        ];
    }
    let ur = unpremultiply_u16(px.r, px.a);
    let ug = unpremultiply_u16(px.g, px.a);
    let ub = unpremultiply_u16(px.b, px.a);
    [
        quantize_srgb_8_8(linear_u16_to_srgb_8_8(ur), t),
        quantize_srgb_8_8(linear_u16_to_srgb_8_8(ug), t),
        quantize_srgb_8_8(linear_u16_to_srgb_8_8(ub), t),
        u16_to_u8_linear(px.a),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blue_noise_is_exact_bijection() {
        let mut seen = [false; 256];
        for &v in &BLUE_NOISE_16X16 {
            assert!(!seen[v as usize], "duplicate blue-noise value {v}");
            seen[v as usize] = true;
        }
        assert!(seen.iter().all(|&s| s));
    }

    #[test]
    fn endpoints_and_exact_levels_never_speckle() {
        for mode in [
            DitherMode::BlueNoise,
            DitherMode::InterleavedGradientNoise,
            DitherMode::None,
        ] {
            for y in 0..32u32 {
                for x in 0..32u32 {
                    assert_eq!(linear_u16_to_srgb_u8_dithered(0, mode, x, y), 0);
                    assert_eq!(linear_u16_to_srgb_u8_dithered(65535, mode, x, y), 255);
                    let t = dither_threshold(mode, x, y);
                    for level in 0..=255u16 {
                        assert_eq!(quantize_srgb_8_8(level * 256, t), level as u8);
                    }
                }
            }
        }
    }

    #[test]
    fn blue_noise_zero_bias_spatial_mean() {
        for level in [0u16, 1, 7, 128, 254] {
            for frac in [1u16, 37, 64, 128, 192, 255] {
                let s88 = level * 256 + frac;
                let mut sum = 0u32;
                for y in 0..16u32 {
                    for x in 0..16u32 {
                        sum += quantize_srgb_8_8(s88, blue_noise_u8(x, y)) as u32;
                    }
                }
                assert_eq!(sum, (level as u32) * 256 + (frac as u32));
            }
        }
    }

    #[test]
    fn dark_gradient_monotonic_and_deterministic() {
        // 1920px wide dark linear gradient from #000000 to #101010 (~16 sRGB levels).
        let width = 1920u32;
        let height = 16u32;
        let max_lin = crate::math::color::srgb_u8_to_linear_u16(16) as u32;

        for mode in [DitherMode::BlueNoise, DitherMode::InterleavedGradientNoise] {
            let mut band_means = [0u32; 30];
            let mut hash1 = 0xCBF2_9CE4_8422_2325u64;
            for (b, slot) in band_means.iter_mut().enumerate() {
                let x0 = (b as u32) * 64;
                let mut sum = 0u32;
                for y in 0..height {
                    for x in x0..(x0 + 64) {
                        let lin = ((x * max_lin) / (width - 1)) as u16;
                        let v = linear_u16_to_srgb_u8_dithered(lin, mode, x, y);
                        sum += v as u32;
                        hash1 ^= v as u64;
                        hash1 = hash1.wrapping_mul(0x0000_0100_0000_01B3);
                    }
                }
                *slot = sum;
            }
            // Monotonic non-decreasing luminance across all 30 bands
            for w in band_means.windows(2) {
                assert!(w[1] >= w[0], "non-monotonic band mean: {:?}", w);
            }
            assert!(band_means[29] > band_means[0]);

            // Deterministic second run
            let mut hash2 = 0xCBF2_9CE4_8422_2325u64;
            for b in 0..30u32 {
                let x0 = b * 64;
                for y in 0..height {
                    for x in x0..(x0 + 64) {
                        let lin = ((x * max_lin) / (width - 1)) as u16;
                        let v = linear_u16_to_srgb_u8_dithered(lin, mode, x, y);
                        hash2 ^= v as u64;
                        hash2 = hash2.wrapping_mul(0x0000_0100_0000_01B3);
                    }
                }
            }
            assert_eq!(hash1, hash2);
        }
    }
}
