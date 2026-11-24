#![allow(dead_code)]

use std::f64::consts::PI;

pub struct Sha256 {
    state: [u32; 8],
    buf: [u8; 64],
    buf_len: usize,
    total_bits: u64,
}

const SHA256_K: [u32; 64] = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
];

impl Sha256 {
    pub fn new() -> Self {
        Self {
            state: [
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c,
                0x1f83d9ab, 0x5be0cd19,
            ],
            buf: [0; 64],
            buf_len: 0,
            total_bits: 0,
        }
    }

    pub fn update(&mut self, mut data: &[u8]) {
        self.total_bits = self.total_bits.wrapping_add((data.len() as u64) * 8);
        if self.buf_len > 0 {
            let take = (64 - self.buf_len).min(data.len());
            self.buf[self.buf_len..self.buf_len + take].copy_from_slice(&data[..take]);
            self.buf_len += take;
            data = &data[take..];
            if self.buf_len == 64 {
                let block = self.buf;
                self.compress(&block);
                self.buf_len = 0;
            }
        }
        while data.len() >= 64 {
            let mut block = [0u8; 64];
            block.copy_from_slice(&data[..64]);
            self.compress(&block);
            data = &data[64..];
        }
        if !data.is_empty() {
            self.buf[..data.len()].copy_from_slice(data);
            self.buf_len = data.len();
        }
    }

    pub fn finalize(mut self) -> [u8; 32] {
        self.buf[self.buf_len] = 0x80;
        self.buf_len += 1;
        if self.buf_len > 56 {
            self.buf[self.buf_len..64].fill(0);
            let block = self.buf;
            self.compress(&block);
            self.buf_len = 0;
        }
        self.buf[self.buf_len..56].fill(0);
        self.buf[56..64].copy_from_slice(&self.total_bits.to_be_bytes());
        let block = self.buf;
        self.compress(&block);

        let mut out = [0u8; 32];
        for (i, &w) in self.state.iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&w.to_be_bytes());
        }
        out
    }

    fn compress(&mut self, block: &[u8; 64]) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                block[i * 4],
                block[i * 4 + 1],
                block[i * 4 + 2],
                block[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }

        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = self.state;
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let t1 = h
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(SHA256_K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);

            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }

        self.state[0] = self.state[0].wrapping_add(a);
        self.state[1] = self.state[1].wrapping_add(b);
        self.state[2] = self.state[2].wrapping_add(c);
        self.state[3] = self.state[3].wrapping_add(d);
        self.state[4] = self.state[4].wrapping_add(e);
        self.state[5] = self.state[5].wrapping_add(f);
        self.state[6] = self.state[6].wrapping_add(g);
        self.state[7] = self.state[7].wrapping_add(h);
    }
}

pub fn sha256(data: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(data);
    h.finalize()
}

pub fn sha256_hex(data: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let digest = sha256(data);
    let mut s = String::with_capacity(64);
    for b in digest {
        s.push(HEX[(b >> 4) as usize] as char);
        s.push(HEX[(b & 0xF) as usize] as char);
    }
    s
}

pub struct Xoshiro256PlusPlus {
    s: [u64; 4],
}

impl Xoshiro256PlusPlus {
    pub fn seed_from_u64(mut seed: u64) -> Self {
        let mut splitmix = || {
            seed = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = seed;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        };
        Self {
            s: [splitmix(), splitmix(), splitmix(), splitmix()],
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let res = self.s[0]
            .wrapping_add(self.s[3])
            .rotate_left(23)
            .wrapping_add(self.s[0]);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        res
    }

    pub fn next_u32(&mut self) -> u32 {
        (self.next_u64() >> 32) as u32
    }

    pub fn next_u8(&mut self) -> u8 {
        (self.next_u64() >> 56) as u8
    }

    pub fn next_f32_unit(&mut self) -> f32 {
        ((self.next_u64() >> 40) as f32) * (1.0 / 16_777_216.0)
    }

    pub fn next_f32_range(&mut self, min: f32, max: f32) -> f32 {
        min + (max - min) * self.next_f32_unit()
    }
}

fn rgba_to_luma(rgba: &[u8]) -> f64 {
    let a = rgba[3] as f64 / 255.0;
    let r = rgba[0] as f64 * a;
    let g = rgba[1] as f64 * a;
    let b = rgba[2] as f64 * a;
    0.2126 * r + 0.7152 * g + 0.0722 * b
}

// Windowed 8x8 luminance SSIM over tightly packed RGBA8 buffers.
pub fn compute_ssim(img_a: &[u8], img_b: &[u8], width: u32, height: u32) -> f64 {
    let w = width as usize;
    let h = height as usize;
    assert_eq!(img_a.len(), w * h * 4);
    assert_eq!(img_b.len(), w * h * 4);
    if w == 0 || h == 0 {
        return 1.0;
    }

    let mut la = vec![0.0f64; w * h];
    let mut lb = vec![0.0f64; w * h];
    for i in 0..w * h {
        la[i] = rgba_to_luma(&img_a[i * 4..i * 4 + 4]);
        lb[i] = rgba_to_luma(&img_b[i * 4..i * 4 + 4]);
    }

    const C1: f64 = (0.01 * 255.0) * (0.01 * 255.0);
    const C2: f64 = (0.03 * 255.0) * (0.03 * 255.0);

    let win_w = w.min(8);
    let win_h = h.min(8);
    let n = (win_w * win_h) as f64;
    let mut sum_ssim = 0.0;
    let mut count = 0usize;

    for wy in 0..=(h - win_h) {
        for wx in 0..=(w - win_w) {
            let mut mean_a = 0.0;
            let mut mean_b = 0.0;
            for dy in 0..win_h {
                let row = (wy + dy) * w + wx;
                for dx in 0..win_w {
                    mean_a += la[row + dx];
                    mean_b += lb[row + dx];
                }
            }
            mean_a /= n;
            mean_b /= n;

            let mut var_a = 0.0;
            let mut var_b = 0.0;
            let mut cov = 0.0;
            for dy in 0..win_h {
                let row = (wy + dy) * w + wx;
                for dx in 0..win_w {
                    let da = la[row + dx] - mean_a;
                    let db = lb[row + dx] - mean_b;
                    var_a += da * da;
                    var_b += db * db;
                    cov += da * db;
                }
            }
            let denom_n = if n > 1.0 { n - 1.0 } else { 1.0 };
            var_a /= denom_n;
            var_b /= denom_n;
            cov /= denom_n;

            let num = (2.0 * mean_a * mean_b + C1) * (2.0 * cov + C2);
            let den = (mean_a * mean_a + mean_b * mean_b + C1) * (var_a + var_b + C2);
            sum_ssim += num / den;
            count += 1;
        }
    }
    sum_ssim / (count as f64)
}

fn srgb_to_linear_f64(c: f64) -> f64 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_rgb_to_lab(r: f64, g: f64, b: f64) -> (f64, f64, f64) {
    let x = (0.4124564 * r + 0.3575761 * g + 0.1804375 * b) / 0.95047;
    let y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b;
    let z = (0.0193339 * r + 0.1191920 * g + 0.9503041 * b) / 1.08883;

    let f = |t: f64| {
        const EPS: f64 = 216.0 / 24389.0;
        const KAPPA: f64 = 24389.0 / 27.0;
        if t > EPS {
            t.cbrt()
        } else {
            (KAPPA * t + 16.0) / 116.0
        }
    };
    let fx = f(x);
    let fy = f(y);
    let fz = f(z);
    (116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz))
}

// Sharma, Wu, Dalal (2005) CIEDE2000 perceptual color difference.
pub fn ciede2000_lab(lab1: (f64, f64, f64), lab2: (f64, f64, f64)) -> f64 {
    let (l1, a1, b1) = lab1;
    let (l2, a2, b2) = lab2;

    let c1 = (a1 * a1 + b1 * b1).sqrt();
    let c2 = (a2 * a2 + b2 * b2).sqrt();
    let c_bar = 0.5 * (c1 + c2);
    let c_bar7 = c_bar.powi(7);
    let g = 0.5 * (1.0 - (c_bar7 / (c_bar7 + 25.0f64.powi(7))).sqrt());

    let a1p = (1.0 + g) * a1;
    let a2p = (1.0 + g) * a2;
    let c1p = (a1p * a1p + b1 * b1).sqrt();
    let c2p = (a2p * a2p + b2 * b2).sqrt();

    let hp = |ap: f64, b: f64| {
        if ap == 0.0 && b == 0.0 {
            0.0
        } else {
            let deg = b.atan2(ap).to_degrees();
            if deg < 0.0 { deg + 360.0 } else { deg }
        }
    };
    let h1p = hp(a1p, b1);
    let h2p = hp(a2p, b2);

    let dlp = l2 - l1;
    let dcp = c2p - c1p;

    let dhp = if c1p * c2p == 0.0 {
        0.0
    } else if (h2p - h1p).abs() <= 180.0 {
        h2p - h1p
    } else if h2p - h1p > 180.0 {
        h2p - h1p - 360.0
    } else {
        h2p - h1p + 360.0
    };
    let d_hp = 2.0 * (c1p * c2p).sqrt() * (dhp * 0.5 * PI / 180.0).sin();

    let lp_bar = 0.5 * (l1 + l2);
    let cp_bar = 0.5 * (c1p + c2p);
    let hp_bar = if c1p * c2p == 0.0 {
        h1p + h2p
    } else if (h1p - h2p).abs() <= 180.0 {
        0.5 * (h1p + h2p)
    } else if h1p + h2p < 360.0 {
        0.5 * (h1p + h2p + 360.0)
    } else {
        0.5 * (h1p + h2p - 360.0)
    };

    let t = 1.0 - 0.17 * ((hp_bar - 30.0) * PI / 180.0).cos()
        + 0.24 * ((2.0 * hp_bar) * PI / 180.0).cos()
        + 0.32 * ((3.0 * hp_bar + 6.0) * PI / 180.0).cos()
        - 0.20 * ((4.0 * hp_bar - 63.0) * PI / 180.0).cos();

    let d_theta = 30.0 * (-((hp_bar - 275.0) / 25.0).powi(2)).exp();
    let cp_bar7 = cp_bar.powi(7);
    let r_c = 2.0 * (cp_bar7 / (cp_bar7 + 25.0f64.powi(7))).sqrt();
    let lp_m50_sq = (lp_bar - 50.0).powi(2);
    let s_l = 1.0 + (0.015 * lp_m50_sq) / (20.0 + lp_m50_sq).sqrt();
    let s_c = 1.0 + 0.045 * cp_bar;
    let s_h = 1.0 + 0.015 * cp_bar * t;
    let r_t = -(2.0 * d_theta * PI / 180.0).sin() * r_c;

    let dl_term = dlp / s_l;
    let dc_term = dcp / s_c;
    let dh_term = d_hp / s_h;
    (dl_term * dl_term + dc_term * dc_term + dh_term * dh_term + r_t * dc_term * dh_term)
        .max(0.0)
        .sqrt()
}

fn rgba_to_linear_over(rgba: &[u8], bg: f64) -> (f64, f64, f64) {
    let a = rgba[3] as f64 / 255.0;
    let r = srgb_to_linear_f64(rgba[0] as f64 / 255.0) * a + bg * (1.0 - a);
    let g = srgb_to_linear_f64(rgba[1] as f64 / 255.0) * a + bg * (1.0 - a);
    let b = srgb_to_linear_f64(rgba[2] as f64 / 255.0) * a + bg * (1.0 - a);
    (r, g, b)
}

// Returns (max_delta_e, mean_delta_e) over RGBA8 buffers (detecting both RGB and alpha diffs).
pub fn compute_max_and_mean_delta_e2000(
    img_a: &[u8],
    img_b: &[u8],
    width: u32,
    height: u32,
) -> (f64, f64) {
    let n = (width as usize) * (height as usize);
    assert_eq!(img_a.len(), n * 4);
    assert_eq!(img_b.len(), n * 4);
    if n == 0 {
        return (0.0, 0.0);
    }

    let mut max_de = 0.0f64;
    let mut sum_de = 0.0f64;
    for i in 0..n {
        let pa = &img_a[i * 4..i * 4 + 4];
        let pb = &img_b[i * 4..i * 4 + 4];
        if pa == pb {
            continue;
        }
        let de = if pa[3] == 255 && pb[3] == 255 {
            let (ra, ga, ba) = rgba_to_linear_over(pa, 0.0);
            let (rb, gb, bb) = rgba_to_linear_over(pb, 0.0);
            ciede2000_lab(linear_rgb_to_lab(ra, ga, ba), linear_rgb_to_lab(rb, gb, bb))
        } else {
            let (ra0, ga0, ba0) = rgba_to_linear_over(pa, 0.0);
            let (rb0, gb0, bb0) = rgba_to_linear_over(pb, 0.0);
            let de0 = ciede2000_lab(
                linear_rgb_to_lab(ra0, ga0, ba0),
                linear_rgb_to_lab(rb0, gb0, bb0),
            );
            let (ra1, ga1, ba1) = rgba_to_linear_over(pa, 1.0);
            let (rb1, gb1, bb1) = rgba_to_linear_over(pb, 1.0);
            let de1 = ciede2000_lab(
                linear_rgb_to_lab(ra1, ga1, ba1),
                linear_rgb_to_lab(rb1, gb1, bb1),
            );
            de0.max(de1)
        };
        if de > max_de {
            max_de = de;
        }
        sum_de += de;
    }
    (max_de, sum_de / (n as f64))
}

#[macro_export]
macro_rules! install_counting_alloc {
    () => {
        mod __zvg_counting_alloc {
            use std::alloc::{GlobalAlloc, Layout, System};
            use std::cell::Cell;
            use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

            struct CountingAlloc;

            static ACTIVE: AtomicBool = AtomicBool::new(false);
            static ALLOCS: AtomicUsize = AtomicUsize::new(0);
            static DEALLOCS: AtomicUsize = AtomicUsize::new(0);

            thread_local! {
                static TRACK_THIS_THREAD: Cell<bool> = const { Cell::new(false) };
            }

            #[inline]
            fn is_tracked() -> bool {
                ACTIVE.load(Ordering::Relaxed)
                    && TRACK_THIS_THREAD.try_with(|c| c.get()).unwrap_or(false)
            }

            #[global_allocator]
            static GLOBAL: CountingAlloc = CountingAlloc;

            unsafe impl GlobalAlloc for CountingAlloc {
                unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
                    if is_tracked() {
                        ALLOCS.fetch_add(1, Ordering::Relaxed);
                    }
                    unsafe { System.alloc(layout) }
                }

                unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
                    if is_tracked() {
                        DEALLOCS.fetch_add(1, Ordering::Relaxed);
                    }
                    unsafe { System.dealloc(ptr, layout) }
                }

                unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
                    if is_tracked() {
                        ALLOCS.fetch_add(1, Ordering::Relaxed);
                    }
                    unsafe { System.realloc(ptr, layout, new_size) }
                }
            }

            pub fn assert_zero_allocs<F: FnOnce()>(f: F) {
                ALLOCS.store(0, Ordering::SeqCst);
                DEALLOCS.store(0, Ordering::SeqCst);
                TRACK_THIS_THREAD.with(|c| c.set(true));
                ACTIVE.store(true, Ordering::SeqCst);
                f();
                ACTIVE.store(false, Ordering::SeqCst);
                TRACK_THIS_THREAD.with(|c| c.set(false));
                let a = ALLOCS.load(Ordering::SeqCst);
                let d = DEALLOCS.load(Ordering::SeqCst);
                assert_eq!(
                    (a, d),
                    (0, 0),
                    "expected 0 steady-state allocations, observed allocs={a}, deallocs={d}"
                );
            }
        }
        use __zvg_counting_alloc::assert_zero_allocs;
    };
}
