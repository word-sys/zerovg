#![forbid(unsafe_code)]

use alloc::vec;
use alloc::vec::Vec;

use crate::error::{Result, ZvgError};
use crate::math::color::ColorSpace;
use crate::surface::{PixelFormat, Surface};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JpegDecodeLimits {
    pub max_width: u32,
    pub max_height: u32,
    pub max_pixels: usize,
    pub max_scans: usize,
}

impl Default for JpegDecodeLimits {
    fn default() -> Self {
        Self {
            max_width: 16_384,
            max_height: 16_384,
            max_pixels: 64_000_000,
            max_scans: 64,
        }
    }
}

const ZIGZAG: [usize; 64] = [
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
];

#[derive(Clone)]
struct JpegHuff {
    // Fast 8-bit lookup: (len << 8) | sym, or 0 if > 8 bits / invalid
    fast: [u16; 256],
    // Per-length canonical decode tables (1..=16)
    max_code: [i32; 17],
    val_offset: [i32; 17],
    symbols: Vec<u8>,
}

impl JpegHuff {
    fn build(counts: &[u8; 16], symbols: &[u8]) -> Result<Self> {
        let mut fast = [0u16; 256];
        let mut max_code = [-1i32; 17];
        let mut val_offset = [0i32; 17];

        let mut code = 0i32;
        let mut sym_idx = 0usize;
        for bits in 1..=16usize {
            let count = counts[bits - 1] as usize;
            if count > 0 {
                val_offset[bits] = (sym_idx as i32) - code;
                if bits <= 8 {
                    let shift = 8 - bits;
                    for i in 0..count {
                        let c = (code + i as i32) as usize;
                        if c >= (1 << bits) {
                            return Err(ZvgError::ParseError("jpeg huffman code overflow"));
                        }
                        let sym = symbols[sym_idx + i] as u16;
                        let entry = ((bits as u16) << 8) | sym;
                        let base = c << shift;
                        let span = 1usize << shift;
                        fast[base..base + span].fill(entry);
                    }
                }
                code += count as i32;
                if code > (1 << bits) {
                    return Err(ZvgError::ParseError("jpeg oversubscribed huffman table"));
                }
                max_code[bits] = code - 1;
                sym_idx += count;
            }
            code <<= 1;
        }

        Ok(Self {
            fast,
            max_code,
            val_offset,
            symbols: symbols.to_vec(),
        })
    }
}

struct EntropyReader<'a> {
    data: &'a [u8],
    pos: usize,
    bits: u32,
    count: u32,
    pending_marker: Option<u8>,
}

impl<'a> EntropyReader<'a> {
    fn new(data: &'a [u8], pos: usize) -> Self {
        Self {
            data,
            pos,
            bits: 0,
            count: 0,
            pending_marker: None,
        }
    }

    #[inline]
    fn refill(&mut self) {
        while self.count <= 24 {
            if self.pending_marker.is_some() || self.pos >= self.data.len() {
                break;
            }
            let b = self.data[self.pos];
            self.pos += 1;
            if b == 0xFF {
                // Skip fill 0xFF bytes
                while self.pos < self.data.len() && self.data[self.pos] == 0xFF {
                    self.pos += 1;
                }
                if self.pos >= self.data.len() {
                    break;
                }
                let next = self.data[self.pos];
                self.pos += 1;
                if next == 0x00 {
                    // Byte-stuffed literal 0xFF
                    self.bits = (self.bits << 8) | 0xFF;
                    self.count += 8;
                } else {
                    self.pending_marker = Some(next);
                    break;
                }
            } else {
                self.bits = (self.bits << 8) | (b as u32);
                self.count += 8;
            }
        }
    }

    #[inline]
    fn read_bits(&mut self, n: u32) -> Result<u32> {
        if n == 0 {
            return Ok(0);
        }
        if self.count < n {
            self.refill();
            if self.count < n {
                return Err(ZvgError::ParseError("jpeg unexpected eof in entropy scan"));
            }
        }
        self.count -= n;
        let val = (self.bits >> self.count) & ((1u32 << n) - 1);
        Ok(val)
    }

    #[inline]
    fn read_bit(&mut self) -> Result<u32> {
        self.read_bits(1)
    }

    #[inline]
    fn decode_huff(&mut self, h: &JpegHuff) -> Result<u8> {
        if self.count < 8 {
            self.refill();
        }
        if self.count >= 8 {
            let peek = ((self.bits >> (self.count - 8)) & 0xFF) as usize;
            let entry = h.fast[peek];
            if entry != 0 {
                let len = (entry >> 8) as u32;
                self.count -= len;
                return Ok((entry & 0xFF) as u8);
            }
        }
        let mut code = 0i32;
        for bits in 1..=16usize {
            code = (code << 1) | (self.read_bit()? as i32);
            if code <= h.max_code[bits] {
                let idx = (code + h.val_offset[bits]) as usize;
                return h
                    .symbols
                    .get(idx)
                    .copied()
                    .ok_or(ZvgError::ParseError("jpeg huffman symbol out of bounds"));
            }
        }
        Err(ZvgError::ParseError("jpeg invalid huffman code"))
    }

    fn consume_restart_marker(&mut self, expected_rst: u8) -> Result<()> {
        self.bits = 0;
        self.count = 0;
        if let Some(m) = self.pending_marker.take() {
            if m == expected_rst {
                return Ok(());
            }
            return Err(ZvgError::ParseError("jpeg unexpected restart marker"));
        }
        while self.pos < self.data.len() && self.data[self.pos] == 0xFF {
            self.pos += 1;
        }
        if self.pos < self.data.len() && self.data[self.pos] == expected_rst {
            self.pos += 1;
            return Ok(());
        }
        Err(ZvgError::ParseError("jpeg missing restart marker"))
    }

    fn finish_position(&self) -> usize {
        if self.pending_marker.is_some() {
            // Rewind 2 bytes so the main marker loop sees 0xFF <marker>.
            self.pos.saturating_sub(2)
        } else {
            self.pos
        }
    }
}

#[inline]
fn extend_sign(v: u32, bits: u32) -> i32 {
    if bits == 0 {
        return 0;
    }
    let half = 1u32 << (bits - 1);
    if v < half {
        (v as i32) - (((1u32 << bits) - 1) as i32)
    } else {
        v as i32
    }
}

struct Component {
    id: u8,
    h_samp: usize,
    v_samp: usize,
    qt_id: usize,
    blocks_x: usize,
    blocks_y: usize,
    blocks: Vec<[i16; 64]>,
}

struct ScanComp {
    comp_idx: usize,
    dc_id: usize,
    ac_id: usize,
}

fn decode_baseline_block(
    er: &mut EntropyReader<'_>,
    dc_huff: &JpegHuff,
    ac_huff: &JpegHuff,
    dc_pred: &mut i32,
    block: &mut [i16; 64],
) -> Result<()> {
    let t = er.decode_huff(dc_huff)? as u32;
    if t > 11 {
        return Err(ZvgError::ParseError("jpeg invalid dc magnitude"));
    }
    let diff = extend_sign(er.read_bits(t)?, t);
    *dc_pred = dc_pred.wrapping_add(diff);
    block[0] = (*dc_pred).clamp(i16::MIN as i32, i16::MAX as i32) as i16;

    let mut k = 1usize;
    while k < 64 {
        let rs = er.decode_huff(ac_huff)?;
        let r = (rs >> 4) as usize;
        let s = (rs & 0x0F) as u32;
        if s == 0 {
            if r == 15 {
                k += 16;
                continue;
            }
            break;
        }
        k += r;
        if k >= 64 {
            return Err(ZvgError::ParseError("jpeg ac coefficient index overflow"));
        }
        let val = extend_sign(er.read_bits(s)?, s);
        block[ZIGZAG[k]] = val.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        k += 1;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn decode_ac_first_block(
    er: &mut EntropyReader<'_>,
    ac_huff: &JpegHuff,
    ss: usize,
    se: usize,
    al: u32,
    eob_run: &mut u32,
    block: &mut [i16; 64],
) -> Result<()> {
    if *eob_run > 0 {
        *eob_run -= 1;
        return Ok(());
    }
    let mut k = ss;
    while k <= se {
        let rs = er.decode_huff(ac_huff)?;
        let r = (rs >> 4) as usize;
        let s = (rs & 0x0F) as u32;
        if s == 0 {
            if r == 15 {
                k += 16;
            } else {
                let extra = er.read_bits(r as u32)?;
                *eob_run = ((1u32 << r) + extra).saturating_sub(1);
                break;
            }
        } else {
            k += r;
            if k > se {
                return Err(ZvgError::ParseError("jpeg progressive ac overflow"));
            }
            let val = extend_sign(er.read_bits(s)?, s) << al;
            block[ZIGZAG[k]] = val.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            k += 1;
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn decode_ac_refine_block(
    er: &mut EntropyReader<'_>,
    ac_huff: &JpegHuff,
    ss: usize,
    se: usize,
    al: u32,
    eob_run: &mut u32,
    block: &mut [i16; 64],
) -> Result<()> {
    let p1 = 1i16 << al;
    let m1 = -p1;
    let mut k = ss;

    if *eob_run == 0 {
        while k <= se {
            let rs = er.decode_huff(ac_huff)?;
            let mut r = (rs >> 4) as usize;
            let s = (rs & 0x0F) as u32;
            let new_val = if s == 0 {
                if r != 15 {
                    let extra = er.read_bits(r as u32)?;
                    *eob_run = (1u32 << r) + extra;
                    break;
                }
                0i16
            } else if s == 1 {
                if er.read_bit()? != 0 { p1 } else { m1 }
            } else {
                return Err(ZvgError::ParseError("jpeg progressive ac refine invalid size"));
            };

            loop {
                if k > se {
                    return Err(ZvgError::ParseError("jpeg progressive ac refine overflow"));
                }
                let zi = ZIGZAG[k];
                let cur = block[zi];
                if cur != 0 {
                    if er.read_bit()? != 0 && (cur & p1) == 0 {
                        block[zi] = if cur > 0 {
                            cur.saturating_add(p1)
                        } else {
                            cur.saturating_add(m1)
                        };
                    }
                } else {
                    if r == 0 {
                        if new_val != 0 {
                            block[zi] = new_val;
                        }
                        k += 1;
                        break;
                    }
                    r -= 1;
                }
                k += 1;
            }
        }
    }

    if *eob_run > 0 {
        while k <= se {
            let zi = ZIGZAG[k];
            let cur = block[zi];
            if cur != 0 && er.read_bit()? != 0 && (cur & p1) == 0 {
                block[zi] = if cur > 0 {
                    cur.saturating_add(p1)
                } else {
                    cur.saturating_add(m1)
                };
            }
            k += 1;
        }
        *eob_run -= 1;
    }
    Ok(())
}

// IJG 12-bit fixed-point separable Loeffler 8x8 integer IDCT (deterministic, zero float).
fn idct_8x8(coeff: &[i16; 64], quant: &[u16; 64], out: &mut [u8; 64]) {
    const C1: i32 = 2217;
    const C2: i32 = -7567;
    const C3: i32 = 3135;
    const C4: i32 = -3685;
    const C5: i32 = -10497;
    const C6: i32 = -8034;
    const C7: i32 = -1597;
    const C8: i32 = 1223;
    const C9: i32 = 8410;
    const C10: i32 = 12586;
    const C11: i32 = 6149;

    let mut tmp = [0i32; 64];

    // Pass 1: columns
    for col in 0..8 {
        let d0 = coeff[col] as i32 * quant[col] as i32;
        let d1 = coeff[col + 8] as i32 * quant[col + 8] as i32;
        let d2 = coeff[col + 16] as i32 * quant[col + 16] as i32;
        let d3 = coeff[col + 24] as i32 * quant[col + 24] as i32;
        let d4 = coeff[col + 32] as i32 * quant[col + 32] as i32;
        let d5 = coeff[col + 40] as i32 * quant[col + 40] as i32;
        let d6 = coeff[col + 48] as i32 * quant[col + 48] as i32;
        let d7 = coeff[col + 56] as i32 * quant[col + 56] as i32;

        if d1 == 0 && d2 == 0 && d3 == 0 && d4 == 0 && d5 == 0 && d6 == 0 && d7 == 0 {
            let dc = d0 << 2;
            for r in 0..8 {
                tmp[r * 8 + col] = dc;
            }
            continue;
        }

        let p1 = (d2 + d6) * C1;
        let t2 = p1 + d6 * C2;
        let t3 = p1 + d2 * C3;
        let t0 = (d0 + d4) << 12;
        let t1 = (d0 - d4) << 12;
        let x0 = t0 + t3 + 512;
        let x3 = t0 - t3 + 512;
        let x1 = t1 + t2 + 512;
        let x2 = t1 - t2 + 512;

        let p3 = d7 + d3;
        let p4 = d5 + d1;
        let p1o = d7 + d1;
        let p2o = d5 + d3;
        let p5 = (p3 + p4) * 4816; // f2f(1.175875602)

        let o0 = d7 * C8 + p1o * C4 + p3 * C6 + p5;
        let o1 = d5 * C9 + p2o * C5 + p4 * C7 + p5;
        let o2 = d3 * C10 + p2o * C5 + p3 * C6 + p5;
        let o3 = d1 * C11 + p1o * C4 + p4 * C7 + p5;

        tmp[col] = (x0 + o3) >> 10;
        tmp[col + 56] = (x0 - o3) >> 10;
        tmp[col + 8] = (x1 + o2) >> 10;
        tmp[col + 48] = (x1 - o2) >> 10;
        tmp[col + 16] = (x2 + o1) >> 10;
        tmp[col + 40] = (x2 - o1) >> 10;
        tmp[col + 24] = (x3 + o0) >> 10;
        tmp[col + 32] = (x3 - o0) >> 10;
    }

    // Pass 2: rows + level shift (+128) + clamp [0, 255]
    for row in 0..8 {
        let base = row * 8;
        let d0 = tmp[base];
        let d1 = tmp[base + 1];
        let d2 = tmp[base + 2];
        let d3 = tmp[base + 3];
        let d4 = tmp[base + 4];
        let d5 = tmp[base + 5];
        let d6 = tmp[base + 6];
        let d7 = tmp[base + 7];

        let t0 = (d0 + d4) << 12;
        let t1 = (d0 - d4) << 12;
        let p1 = (d2 + d6) * C1;
        let t2 = p1 + d6 * C2;
        let t3 = p1 + d2 * C3;
        let bias = (128 << 17) + (1 << 16);
        let x0 = t0 + t3 + bias;
        let x3 = t0 - t3 + bias;
        let x1 = t1 + t2 + bias;
        let x2 = t1 - t2 + bias;

        let p3 = d7 + d3;
        let p4 = d5 + d1;
        let p1o = d7 + d1;
        let p2o = d5 + d3;
        let p5 = (p3 + p4) * 4816;

        let o0 = d7 * C8 + p1o * C4 + p3 * C6 + p5;
        let o1 = d5 * C9 + p2o * C5 + p4 * C7 + p5;
        let o2 = d3 * C10 + p2o * C5 + p3 * C6 + p5;
        let o3 = d1 * C11 + p1o * C4 + p4 * C7 + p5;

        out[base] = ((x0 + o3) >> 17).clamp(0, 255) as u8;
        out[base + 7] = ((x0 - o3) >> 17).clamp(0, 255) as u8;
        out[base + 1] = ((x1 + o2) >> 17).clamp(0, 255) as u8;
        out[base + 6] = ((x1 - o2) >> 17).clamp(0, 255) as u8;
        out[base + 2] = ((x2 + o1) >> 17).clamp(0, 255) as u8;
        out[base + 5] = ((x2 - o1) >> 17).clamp(0, 255) as u8;
        out[base + 3] = ((x3 + o0) >> 17).clamp(0, 255) as u8;
        out[base + 4] = ((x3 - o0) >> 17).clamp(0, 255) as u8;
    }
}

#[inline]
fn sample_plane_bilinear(plane: &[u8], pw: usize, ph: usize, x: usize, y: usize, sx: usize, sy: usize) -> u8 {
    if sx == 1 && sy == 1 {
        return plane[y * pw + x];
    }
    if sx == 2 && sy == 2 {
        // Centered 3:1 triangle/bilinear chroma upsampling (matching libjpeg smooth upsample)
        let cx = x / 2;
        let cy = y / 2;
        let nx = if (x & 1) == 0 {
            cx.saturating_sub(1)
        } else {
            (cx + 1).min(pw - 1)
        };
        let ny = if (y & 1) == 0 {
            cy.saturating_sub(1)
        } else {
            (cy + 1).min(ph - 1)
        };
        let p00 = plane[cy * pw + cx] as u32;
        let p10 = plane[cy * pw + nx] as u32;
        let p01 = plane[ny * pw + cx] as u32;
        let p11 = plane[ny * pw + nx] as u32;
        return ((9 * p00 + 3 * p10 + 3 * p01 + p11 + 8) >> 4) as u8;
    }
    if sx == 2 && sy == 1 {
        let cx = x / 2;
        let nx = if (x & 1) == 0 {
            cx.saturating_sub(1)
        } else {
            (cx + 1).min(pw - 1)
        };
        let p0 = plane[y * pw + cx] as u32;
        let p1 = plane[y * pw + nx] as u32;
        return ((3 * p0 + p1 + 2) >> 2) as u8;
    }
    let px = (x / sx).min(pw - 1);
    let py = (y / sy).min(ph - 1);
    plane[py * pw + px]
}

#[inline]
fn ycbcr_to_rgb(y: u8, cb: u8, cr: u8) -> [u8; 3] {
    let y = y as i32;
    let cb = cb as i32 - 128;
    let cr = cr as i32 - 128;
    let r = y + ((91_881 * cr + 32_768) >> 16);
    let g = y - ((22_554 * cb + 46_802 * cr - 32_768) >> 16);
    let b = y + ((116_130 * cb + 32_768) >> 16);
    [r.clamp(0, 255) as u8, g.clamp(0, 255) as u8, b.clamp(0, 255) as u8]
}

pub fn decode_jpeg_with_limits(data: &[u8], limits: JpegDecodeLimits) -> Result<Surface> {
    if data.len() < 4 || data[0] != 0xFF || data[1] != 0xD8 {
        return Err(ZvgError::ParseError("invalid jpeg SOI marker"));
    }

    let mut pos = 2usize;
    let mut quants: [Option<[u16; 64]>; 4] = [None, None, None, None];
    let mut dc_huffs: [Option<JpegHuff>; 4] = [None, None, None, None];
    let mut ac_huffs: [Option<JpegHuff>; 4] = [None, None, None, None];
    let mut restart_interval = 0u16;
    let mut adobe_transform: Option<u8> = None;

    let mut width = 0u32;
    let mut height = 0u32;
    let mut progressive = false;
    let mut max_h = 1usize;
    let mut max_v = 1usize;
    let mut mcu_x = 0usize;
    let mut mcu_y = 0usize;
    let mut components: Vec<Component> = Vec::new();
    let mut scans_decoded = 0usize;

    while pos < data.len() {
        // Find next 0xFF marker byte
        while pos < data.len() && data[pos] != 0xFF {
            pos += 1;
        }
        while pos < data.len() && data[pos] == 0xFF {
            pos += 1;
        }
        if pos >= data.len() {
            break;
        }
        let marker = data[pos];
        pos += 1;

        match marker {
            0x00 => continue,
            0xD8 => continue,
            0xD9 => break, // EOI
            0xD0..=0xD7 => continue,
            0xDB => {
                // DQT
                if pos + 2 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg DQT"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len < 2 || pos + seg_len > data.len() {
                    return Err(ZvgError::ParseError("invalid jpeg DQT length"));
                }
                let end = pos + seg_len;
                pos += 2;
                while pos < end {
                    let info = data[pos];
                    pos += 1;
                    let pq = info >> 4;
                    let tq = (info & 0x0F) as usize;
                    if tq >= 4 || pq > 1 {
                        return Err(ZvgError::ParseError("invalid jpeg DQT header"));
                    }
                    let mut qt = [0u16; 64];
                    if pq == 0 {
                        if pos + 64 > end {
                            return Err(ZvgError::ParseError("truncated jpeg 8-bit DQT"));
                        }
                        for k in 0..64 {
                            qt[ZIGZAG[k]] = data[pos + k] as u16;
                        }
                        pos += 64;
                    } else {
                        if pos + 128 > end {
                            return Err(ZvgError::ParseError("truncated jpeg 16-bit DQT"));
                        }
                        for k in 0..64 {
                            qt[ZIGZAG[k]] = u16::from_be_bytes([data[pos + k * 2], data[pos + k * 2 + 1]]);
                        }
                        pos += 128;
                    }
                    quants[tq] = Some(qt);
                }
            }
            0xC4 => {
                // DHT
                if pos + 2 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg DHT"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len < 2 || pos + seg_len > data.len() {
                    return Err(ZvgError::ParseError("invalid jpeg DHT length"));
                }
                let end = pos + seg_len;
                pos += 2;
                while pos < end {
                    if pos + 17 > end {
                        return Err(ZvgError::ParseError("truncated jpeg DHT counts"));
                    }
                    let tc_th = data[pos];
                    pos += 1;
                    let tc = tc_th >> 4;
                    let th = (tc_th & 0x0F) as usize;
                    if tc > 1 || th >= 4 {
                        return Err(ZvgError::ParseError("invalid jpeg DHT class/id"));
                    }
                    let mut counts = [0u8; 16];
                    counts.copy_from_slice(&data[pos..pos + 16]);
                    pos += 16;
                    let total_syms: usize = counts.iter().map(|&c| c as usize).sum();
                    if total_syms > 256 || pos + total_syms > end {
                        return Err(ZvgError::ParseError("truncated jpeg DHT symbols"));
                    }
                    let huff = JpegHuff::build(&counts, &data[pos..pos + total_syms])?;
                    pos += total_syms;
                    if tc == 0 {
                        dc_huffs[th] = Some(huff);
                    } else {
                        ac_huffs[th] = Some(huff);
                    }
                }
            }
            0xDD => {
                // DRI
                if pos + 4 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg DRI"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len != 4 {
                    return Err(ZvgError::ParseError("invalid jpeg DRI length"));
                }
                restart_interval = u16::from_be_bytes([data[pos + 2], data[pos + 3]]);
                pos += 4;
            }
            0xC0..=0xC2 => {
                // SOF0 (Baseline), SOF1 (Extended Sequential), SOF2 (Progressive)
                if !components.is_empty() {
                    return Err(ZvgError::ParseError("duplicate jpeg SOF"));
                }
                progressive = marker == 0xC2;
                if pos + 2 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg SOF"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len < 8 || pos + seg_len > data.len() {
                    return Err(ZvgError::ParseError("invalid jpeg SOF length"));
                }
                let precision = data[pos + 2];
                if precision != 8 {
                    return Err(ZvgError::UnsupportedFeature("jpeg non-8-bit precision"));
                }
                height = u16::from_be_bytes([data[pos + 3], data[pos + 4]]) as u32;
                width = u16::from_be_bytes([data[pos + 5], data[pos + 6]]) as u32;
                let num_comp = data[pos + 7] as usize;
                if width == 0 || height == 0 {
                    return Err(ZvgError::ParseError("jpeg zero dimensions"));
                }
                if width > limits.max_width || height > limits.max_height {
                    return Err(ZvgError::AllocationLimit);
                }
                let pixels = (width as usize)
                    .checked_mul(height as usize)
                    .ok_or(ZvgError::AllocationLimit)?;
                if pixels > limits.max_pixels {
                    return Err(ZvgError::AllocationLimit);
                }
                if !(num_comp == 1 || num_comp == 3) {
                    return Err(ZvgError::UnsupportedFeature("jpeg component count"));
                }
                if seg_len != 8 + num_comp * 3 {
                    return Err(ZvgError::ParseError("invalid jpeg SOF component length"));
                }
                let mut raw_comps = Vec::with_capacity(num_comp);
                max_h = 1;
                max_v = 1;
                for i in 0..num_comp {
                    let o = pos + 8 + i * 3;
                    let id = data[o];
                    let hv = data[o + 1];
                    let h = (hv >> 4) as usize;
                    let v = (hv & 0x0F) as usize;
                    let qt_id = data[o + 2] as usize;
                    if h == 0 || v == 0 || h > 4 || v > 4 || qt_id >= 4 {
                        return Err(ZvgError::ParseError("invalid jpeg sampling factor"));
                    }
                    max_h = max_h.max(h);
                    max_v = max_v.max(v);
                    raw_comps.push((id, h, v, qt_id));
                }

                let mcu_w = max_h * 8;
                let mcu_h = max_v * 8;
                mcu_x = (width as usize).div_ceil(mcu_w);
                mcu_y = (height as usize).div_ceil(mcu_h);

                for (id, h, v, qt_id) in raw_comps {
                    let bx = mcu_x * h;
                    let by = mcu_y * v;
                    let total_blocks = bx
                        .checked_mul(by)
                        .ok_or(ZvgError::AllocationLimit)?;
                    components.push(Component {
                        id,
                        h_samp: h,
                        v_samp: v,
                        qt_id,
                        blocks_x: bx,
                        blocks_y: by,
                        blocks: vec![[0i16; 64]; total_blocks],
                    });
                }
                pos += seg_len;
            }
            0xDA => {
                // SOS
                if components.is_empty() {
                    return Err(ZvgError::ParseError("jpeg SOS before SOF"));
                }
                scans_decoded += 1;
                if scans_decoded > limits.max_scans {
                    return Err(ZvgError::AllocationLimit);
                }
                if pos + 2 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg SOS"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len < 6 || pos + seg_len > data.len() {
                    return Err(ZvgError::ParseError("invalid jpeg SOS length"));
                }
                let ns = data[pos + 2] as usize;
                if ns == 0 || ns > components.len() || seg_len != 6 + ns * 2 {
                    return Err(ZvgError::ParseError("invalid jpeg SOS component count"));
                }
                let mut scan_comps = Vec::with_capacity(ns);
                for i in 0..ns {
                    let cid = data[pos + 3 + i * 2];
                    let td_ta = data[pos + 4 + i * 2];
                    let dc_id = (td_ta >> 4) as usize;
                    let ac_id = (td_ta & 0x0F) as usize;
                    if dc_id >= 4 || ac_id >= 4 {
                        return Err(ZvgError::ParseError("invalid jpeg SOS huffman selector"));
                    }
                    let comp_idx = components
                        .iter()
                        .position(|c| c.id == cid)
                        .ok_or(ZvgError::ParseError("unknown jpeg scan component"))?;
                    scan_comps.push(ScanComp {
                        comp_idx,
                        dc_id,
                        ac_id,
                    });
                }
                let ss = data[pos + 3 + ns * 2] as usize;
                let se = data[pos + 4 + ns * 2] as usize;
                let ah_al = data[pos + 5 + ns * 2];
                let ah = (ah_al >> 4) as u32;
                let al = (ah_al & 0x0F) as u32;
                if ss > se || se >= 64 || al > 13 {
                    return Err(ZvgError::ParseError("invalid jpeg spectral selection"));
                }
                pos += seg_len;

                let mut er = EntropyReader::new(data, pos);
                let mut dc_pred = [0i32; 4];
                let mut eob_run = 0u32;
                let mut mcu_until_rst = restart_interval as usize;
                let mut next_rst = 0u8;

                let handle_restart = |er: &mut EntropyReader<'_>,
                                      dc_pred: &mut [i32; 4],
                                      eob_run: &mut u32,
                                      mcu_until_rst: &mut usize,
                                      next_rst: &mut u8|
                 -> Result<()> {
                    if restart_interval > 0 {
                        *mcu_until_rst -= 1;
                        if *mcu_until_rst == 0 {
                            er.consume_restart_marker(0xD0 + (*next_rst & 7))?;
                            *next_rst = next_rst.wrapping_add(1);
                            dc_pred.fill(0);
                            *eob_run = 0;
                            *mcu_until_rst = restart_interval as usize;
                        }
                    }
                    Ok(())
                };

                if !progressive {
                    if ns == 1 {
                        let sc = &scan_comps[0];
                        let dc_h = dc_huffs[sc.dc_id]
                            .as_ref()
                            .ok_or(ZvgError::ParseError("missing jpeg DC huffman table"))?;
                        let ac_h = ac_huffs[sc.ac_id]
                            .as_ref()
                            .ok_or(ZvgError::ParseError("missing jpeg AC huffman table"))?;
                        let comp = &mut components[sc.comp_idx];
                        let eff_bx = ((width as usize) * comp.h_samp).div_ceil(max_h * 8);
                        let eff_by = ((height as usize) * comp.v_samp).div_ceil(max_v * 8);
                        let total_mcus = eff_bx * eff_by;
                        let mut done = 0usize;
                        for by in 0..eff_by {
                            for bx in 0..eff_bx {
                                let idx = by * comp.blocks_x + bx;
                                decode_baseline_block(
                                    &mut er,
                                    dc_h,
                                    ac_h,
                                    &mut dc_pred[0],
                                    &mut comp.blocks[idx],
                                )?;
                                done += 1;
                                if done < total_mcus {
                                    handle_restart(
                                        &mut er,
                                        &mut dc_pred,
                                        &mut eob_run,
                                        &mut mcu_until_rst,
                                        &mut next_rst,
                                    )?;
                                }
                            }
                        }
                    } else {
                        let total_mcus = mcu_x * mcu_y;
                        let mut done = 0usize;
                        for my in 0..mcu_y {
                            for mx in 0..mcu_x {
                                for (si, sc) in scan_comps.iter().enumerate() {
                                    let dc_h = dc_huffs[sc.dc_id]
                                        .as_ref()
                                        .ok_or(ZvgError::ParseError("missing jpeg DC huffman"))?;
                                    let ac_h = ac_huffs[sc.ac_id]
                                        .as_ref()
                                        .ok_or(ZvgError::ParseError("missing jpeg AC huffman"))?;
                                    let comp = &mut components[sc.comp_idx];
                                    for v in 0..comp.v_samp {
                                        for h in 0..comp.h_samp {
                                            let bx = mx * comp.h_samp + h;
                                            let by = my * comp.v_samp + v;
                                            let idx = by * comp.blocks_x + bx;
                                            decode_baseline_block(
                                                &mut er,
                                                dc_h,
                                                ac_h,
                                                &mut dc_pred[si],
                                                &mut comp.blocks[idx],
                                            )?;
                                        }
                                    }
                                }
                                done += 1;
                                if done < total_mcus {
                                    handle_restart(
                                        &mut er,
                                        &mut dc_pred,
                                        &mut eob_run,
                                        &mut mcu_until_rst,
                                        &mut next_rst,
                                    )?;
                                }
                            }
                        }
                    }
                } else if ss == 0 {
                    // Progressive DC scan (interleaved or non-interleaved)
                    if se != 0 {
                        return Err(ZvgError::ParseError("progressive DC scan must have se=0"));
                    }
                    let total_mcus = if ns == 1 {
                        let comp = &components[scan_comps[0].comp_idx];
                        let eff_bx = ((width as usize) * comp.h_samp).div_ceil(max_h * 8);
                        let eff_by = ((height as usize) * comp.v_samp).div_ceil(max_v * 8);
                        eff_bx * eff_by
                    } else {
                        mcu_x * mcu_y
                    };
                    let mut done = 0usize;

                    if ns == 1 {
                        let sc = &scan_comps[0];
                        let comp = &mut components[sc.comp_idx];
                        let eff_bx = ((width as usize) * comp.h_samp).div_ceil(max_h * 8);
                        let eff_by = ((height as usize) * comp.v_samp).div_ceil(max_v * 8);
                        for by in 0..eff_by {
                            for bx in 0..eff_bx {
                                let idx = by * comp.blocks_x + bx;
                                if ah == 0 {
                                    let dc_h = dc_huffs[sc.dc_id]
                                        .as_ref()
                                        .ok_or(ZvgError::ParseError("missing jpeg DC huffman"))?;
                                    let t = er.decode_huff(dc_h)? as u32;
                                    if t > 11 {
                                        return Err(ZvgError::ParseError("invalid dc magnitude"));
                                    }
                                    let diff = extend_sign(er.read_bits(t)?, t);
                                    dc_pred[0] = dc_pred[0].wrapping_add(diff);
                                    comp.blocks[idx][0] = ((dc_pred[0] << al)
                                        .clamp(i16::MIN as i32, i16::MAX as i32))
                                        as i16;
                                } else if er.read_bit()? != 0 {
                                    comp.blocks[idx][0] |= 1i16 << al;
                                }
                                done += 1;
                                if done < total_mcus {
                                    handle_restart(
                                        &mut er,
                                        &mut dc_pred,
                                        &mut eob_run,
                                        &mut mcu_until_rst,
                                        &mut next_rst,
                                    )?;
                                }
                            }
                        }
                    } else {
                        for my in 0..mcu_y {
                            for mx in 0..mcu_x {
                                for (si, sc) in scan_comps.iter().enumerate() {
                                    let comp = &mut components[sc.comp_idx];
                                    for v in 0..comp.v_samp {
                                        for h in 0..comp.h_samp {
                                            let bx = mx * comp.h_samp + h;
                                            let by = my * comp.v_samp + v;
                                            let idx = by * comp.blocks_x + bx;
                                            if ah == 0 {
                                                let dc_h = dc_huffs[sc.dc_id].as_ref().ok_or(
                                                    ZvgError::ParseError("missing jpeg DC huffman"),
                                                )?;
                                                let t = er.decode_huff(dc_h)? as u32;
                                                if t > 11 {
                                                    return Err(ZvgError::ParseError(
                                                        "invalid dc magnitude",
                                                    ));
                                                }
                                                let diff = extend_sign(er.read_bits(t)?, t);
                                                dc_pred[si] = dc_pred[si].wrapping_add(diff);
                                                comp.blocks[idx][0] = ((dc_pred[si] << al)
                                                    .clamp(i16::MIN as i32, i16::MAX as i32))
                                                    as i16;
                                            } else if er.read_bit()? != 0 {
                                                comp.blocks[idx][0] |= 1i16 << al;
                                            }
                                        }
                                    }
                                }
                                done += 1;
                                if done < total_mcus {
                                    handle_restart(
                                        &mut er,
                                        &mut dc_pred,
                                        &mut eob_run,
                                        &mut mcu_until_rst,
                                        &mut next_rst,
                                    )?;
                                }
                            }
                        }
                    }
                } else {
                    // Progressive AC scan (always single component per ITU-T T.81)
                    if ns != 1 {
                        return Err(ZvgError::ParseError("progressive AC scan must have 1 component"));
                    }
                    let sc = &scan_comps[0];
                    let ac_h = ac_huffs[sc.ac_id]
                        .as_ref()
                        .ok_or(ZvgError::ParseError("missing jpeg AC huffman"))?;
                    let comp = &mut components[sc.comp_idx];
                    let eff_bx = ((width as usize) * comp.h_samp).div_ceil(max_h * 8);
                    let eff_by = ((height as usize) * comp.v_samp).div_ceil(max_v * 8);
                    let total_mcus = eff_bx * eff_by;
                    let mut done = 0usize;

                    for by in 0..eff_by {
                        for bx in 0..eff_bx {
                            let idx = by * comp.blocks_x + bx;
                            if ah == 0 {
                                decode_ac_first_block(
                                    &mut er,
                                    ac_h,
                                    ss,
                                    se,
                                    al,
                                    &mut eob_run,
                                    &mut comp.blocks[idx],
                                )?;
                            } else {
                                decode_ac_refine_block(
                                    &mut er,
                                    ac_h,
                                    ss,
                                    se,
                                    al,
                                    &mut eob_run,
                                    &mut comp.blocks[idx],
                                )?;
                            }
                            done += 1;
                            if done < total_mcus {
                                handle_restart(
                                    &mut er,
                                    &mut dc_pred,
                                    &mut eob_run,
                                    &mut mcu_until_rst,
                                    &mut next_rst,
                                )?;
                            }
                        }
                    }
                }

                pos = er.finish_position();
            }
            0xEE => {
                // APP14 Adobe marker (detects direct RGB vs YCbCr)
                if pos + 2 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg APP14"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len < 2 || pos + seg_len > data.len() {
                    return Err(ZvgError::ParseError("invalid jpeg APP14 length"));
                }
                if seg_len >= 14 && &data[pos + 2..pos + 7] == b"Adobe" {
                    adobe_transform = Some(data[pos + 13]);
                }
                pos += seg_len;
            }
            0xC3 | 0xC5..=0xC7 | 0xC9..=0xCB | 0xCD..=0xCF => {
                return Err(ZvgError::UnsupportedFeature("arithmetic or lossless jpeg"));
            }
            _ => {
                // Skip unknown APPn / COM / ancillary segments cleanly
                if pos + 2 > data.len() {
                    return Err(ZvgError::ParseError("truncated jpeg segment"));
                }
                let seg_len = u16::from_be_bytes([data[pos], data[pos + 1]]) as usize;
                if seg_len < 2 || pos + seg_len > data.len() {
                    return Err(ZvgError::ParseError("invalid jpeg segment length"));
                }
                pos += seg_len;
            }
        }
    }

    if components.is_empty() || scans_decoded == 0 {
        return Err(ZvgError::ParseError("jpeg missing image data"));
    }

    // Dequantize + IDCT all component planes
    let mut planes: Vec<(Vec<u8>, usize, usize, usize, usize)> = Vec::with_capacity(components.len());
    for comp in &components {
        let qt = quants[comp.qt_id]
            .as_ref()
            .ok_or(ZvgError::ParseError("missing jpeg quantization table"))?;
        let pw = comp.blocks_x * 8;
        let ph = comp.blocks_y * 8;
        let mut plane = vec![0u8; pw * ph];
        let mut block_out = [0u8; 64];
        for by in 0..comp.blocks_y {
            for bx in 0..comp.blocks_x {
                let b_idx = by * comp.blocks_x + bx;
                idct_8x8(&comp.blocks[b_idx], qt, &mut block_out);
                for r in 0..8 {
                    let dst_off = (by * 8 + r) * pw + bx * 8;
                    plane[dst_off..dst_off + 8].copy_from_slice(&block_out[r * 8..r * 8 + 8]);
                }
            }
        }
        let sx = max_h / comp.h_samp;
        let sy = max_v / comp.v_samp;
        planes.push((plane, pw, ph, sx, sy));
    }

    let mut surface = Surface::try_new(width, height, PixelFormat::Rgba8Srgb, ColorSpace::Srgb)?;
    let stride = surface.stride();
    let dst = surface.data_mut();
    let w = width as usize;
    let h = height as usize;

    if planes.len() == 1 {
        let (ref p0, pw0, ph0, sx0, sy0) = planes[0];
        for y in 0..h {
            let row = &mut dst[y * stride..y * stride + w * 4];
            for x in 0..w {
                let g = sample_plane_bilinear(p0, pw0, ph0, x, y, sx0, sy0);
                row[x * 4..x * 4 + 4].copy_from_slice(&[g, g, g, 255]);
            }
        }
    } else {
        let is_direct_rgb = adobe_transform == Some(0)
            || (components[0].id == b'R' && components[1].id == b'G' && components[2].id == b'B');
        let (ref p0, pw0, ph0, sx0, sy0) = planes[0];
        let (ref p1, pw1, ph1, sx1, sy1) = planes[1];
        let (ref p2, pw2, ph2, sx2, sy2) = planes[2];

        for y in 0..h {
            let row = &mut dst[y * stride..y * stride + w * 4];
            for x in 0..w {
                let c0 = sample_plane_bilinear(p0, pw0, ph0, x, y, sx0, sy0);
                let c1 = sample_plane_bilinear(p1, pw1, ph1, x, y, sx1, sy1);
                let c2 = sample_plane_bilinear(p2, pw2, ph2, x, y, sx2, sy2);
                let rgb = if is_direct_rgb {
                    [c0, c1, c2]
                } else {
                    ycbcr_to_rgb(c0, c1, c2)
                };
                row[x * 4] = rgb[0];
                row[x * 4 + 1] = rgb[1];
                row[x * 4 + 2] = rgb[2];
                row[x * 4 + 3] = 255;
            }
        }
    }

    Ok(surface)
}

pub fn decode_jpeg(data: &[u8]) -> Result<Surface> {
    decode_jpeg_with_limits(data, JpegDecodeLimits::default())
}

impl Surface {
    pub fn from_jpeg_bytes(data: &[u8]) -> Result<Self> {
        decode_jpeg(data)
    }

    #[cfg(feature = "std")]
    pub fn load_jpeg<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let bytes = std::fs::read(path)?;
        Self::from_jpeg_bytes(&bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Authentic libjpeg (Pillow 9.0.1) 16x16 Baseline 4:4:4 fixture
    const BASELINE_444: &[u8] = &[
        255, 216, 255, 224, 0, 16, 74, 70, 73, 70, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 255, 219, 0, 67,
        0, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 2, 4, 3, 2, 2, 2, 2, 5, 4, 4, 3, 4, 6, 5, 6,
        6, 6, 5, 6, 6, 6, 7, 9, 8, 6, 7, 9, 7, 6, 6, 8, 11, 8, 9, 10, 10, 10, 10, 10, 6, 8, 11, 12,
        11, 10, 12, 9, 10, 10, 10, 255, 219, 0, 67, 1, 2, 2, 2, 2, 2, 2, 5, 3, 3, 5, 10, 7, 6, 7,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 10, 10, 10, 255, 192, 0, 17, 8, 0, 16, 0, 16, 3, 1, 17, 0, 2, 17, 1, 3, 17, 1, 255,
        196, 0, 31, 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
        9, 10, 11, 255, 196, 0, 181, 16, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125, 1, 2, 3,
        0, 4, 17, 5, 18, 33, 49, 65, 6, 19, 81, 97, 7, 34, 113, 20, 50, 129, 145, 161, 8, 35, 66,
        177, 193, 21, 82, 209, 240, 36, 51, 98, 114, 130, 9, 10, 22, 23, 24, 25, 26, 37, 38, 39,
        40, 41, 42, 52, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70, 71, 72, 73, 74, 83, 84, 85, 86, 87,
        88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119, 120, 121, 122,
        131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162,
        163, 164, 165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194,
        195, 196, 197, 198, 199, 200, 201, 202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 225,
        226, 227, 228, 229, 230, 231, 232, 233, 234, 241, 242, 243, 244, 245, 246, 247, 248, 249,
        250, 255, 196, 0, 31, 1, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11, 255, 196, 0, 181, 17, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119,
        0, 1, 2, 3, 17, 4, 5, 33, 49, 6, 18, 65, 81, 7, 97, 113, 19, 34, 50, 129, 8, 20, 66, 145,
        161, 177, 193, 9, 35, 51, 82, 240, 21, 98, 114, 209, 10, 22, 36, 52, 225, 37, 241, 23, 24,
        25, 26, 38, 39, 40, 41, 42, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70, 71, 72, 73, 74, 83, 84,
        85, 86, 87, 88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119,
        120, 121, 122, 130, 131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151,
        152, 153, 154, 162, 163, 164, 165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183,
        184, 185, 186, 194, 195, 196, 197, 198, 199, 200, 201, 202, 210, 211, 212, 213, 214, 215,
        216, 217, 218, 226, 227, 228, 229, 230, 231, 232, 233, 234, 242, 243, 244, 245, 246, 247,
        248, 249, 250, 255, 218, 0, 12, 3, 1, 0, 2, 17, 3, 17, 0, 63, 0, 252, 237, 248, 73, 251,
        36, 255, 0, 97, 121, 95, 241, 44, 221, 187, 31, 193, 143, 47, 31, 158, 115, 159, 195, 249,
        25, 215, 25, 255, 0, 168, 87, 247, 185, 249, 255, 0, 237, 223, 99, 111, 252, 11, 159, 159,
        155, 203, 146, 221, 45, 238, 30, 24, 248, 131, 245, 174, 79, 127, 254, 15, 249, 91, 250,
        243, 251, 131, 225, 31, 236, 145, 253, 134, 98, 31, 217, 155, 183, 99, 248, 49, 229, 227,
        249, 231, 63, 135, 242, 254, 180, 206, 184, 207, 253, 66, 191, 189, 207, 207, 255, 0, 110,
        251, 27, 127, 224, 92, 252, 252, 222, 92, 150, 233, 111, 115, 251, 147, 195, 31, 16, 126,
        181, 201, 239, 255, 0, 193, 255, 0, 43, 127, 94, 127, 78, 252, 34, 253, 146, 6, 134, 98,
        206, 153, 187, 118, 63, 131, 30, 94, 63, 158, 115, 248, 127, 47, 201, 243, 174, 51, 255, 0,
        80, 175, 239, 115, 243, 252, 189, 141, 191, 240, 46, 126, 126, 111, 46, 75, 116, 183, 185,
        254, 5, 248, 99, 226, 15, 214, 185, 61, 255, 0, 248, 63, 229, 111, 235, 207, 217, 126, 16,
        254, 201, 95, 216, 126, 87, 252, 75, 55, 110, 199, 240, 99, 203, 199, 243, 206, 127, 15,
        229, 254, 73, 231, 92, 103, 254, 161, 95, 222, 231, 231, 255, 0, 183, 125, 141, 191, 240,
        46, 126, 126, 111, 46, 75, 116, 183, 185, 254, 158, 120, 99, 226, 15, 214, 185, 61, 255, 0,
        248, 63, 229, 111, 235, 207, 255, 217,
    ];

    // Authentic libjpeg (Pillow 9.0.1) 16x16 Grayscale fixture
    const GRAYSCALE_16X16: &[u8] = &[
        255, 216, 255, 224, 0, 16, 74, 70, 73, 70, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 255, 219, 0, 67,
        0, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 2, 4, 3, 2, 2, 2, 2, 5, 4, 4, 3, 4, 6, 5, 6,
        6, 6, 5, 6, 6, 6, 7, 9, 8, 6, 7, 9, 7, 6, 6, 8, 11, 8, 9, 10, 10, 10, 10, 10, 6, 8, 11, 12,
        11, 10, 12, 9, 10, 10, 10, 255, 192, 0, 11, 8, 0, 16, 0, 16, 1, 1, 17, 0, 255, 196, 0, 31,
        0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
        255, 196, 0, 181, 16, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125, 1, 2, 3, 0, 4, 17,
        5, 18, 33, 49, 65, 6, 19, 81, 97, 7, 34, 113, 20, 50, 129, 145, 161, 8, 35, 66, 177, 193,
        21, 82, 209, 240, 36, 51, 98, 114, 130, 9, 10, 22, 23, 24, 25, 26, 37, 38, 39, 40, 41, 42,
        52, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70, 71, 72, 73, 74, 83, 84, 85, 86, 87, 88, 89, 90,
        99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119, 120, 121, 122, 131, 132,
        133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162, 163, 164,
        165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194, 195, 196,
        197, 198, 199, 200, 201, 202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 225, 226, 227,
        228, 229, 230, 231, 232, 233, 234, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 255,
        218, 0, 8, 1, 1, 0, 0, 63, 0, 252, 237, 248, 73, 251, 36, 255, 0, 97, 121, 95, 241, 44,
        221, 187, 31, 193, 143, 47, 31, 158, 115, 159, 195, 249, 125, 193, 240, 143, 246, 72, 254,
        195, 49, 15, 236, 205, 219, 177, 252, 24, 242, 241, 252, 243, 159, 195, 249, 125, 59, 240,
        139, 246, 72, 26, 25, 139, 58, 102, 237, 216, 254, 12, 121, 120, 254, 121, 207, 225, 252,
        189, 151, 225, 15, 236, 149, 253, 135, 229, 127, 196, 179, 118, 236, 127, 6, 60, 188, 127,
        60, 231, 240, 254, 95, 255, 217,
    ];

    // Authentic libjpeg (Pillow 9.0.1) 16x16 Progressive (SOF2) fixture (all 4 scan types)
    const PROGRESSIVE_444: &[u8] = &[
        255, 216, 255, 224, 0, 16, 74, 70, 73, 70, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 255, 219, 0, 67,
        0, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 2, 4, 3, 2, 2, 2, 2, 5, 4, 4, 3, 4, 6, 5, 6,
        6, 6, 5, 6, 6, 6, 7, 9, 8, 6, 7, 9, 7, 6, 6, 8, 11, 8, 9, 10, 10, 10, 10, 10, 6, 8, 11, 12,
        11, 10, 12, 9, 10, 10, 10, 255, 219, 0, 67, 1, 2, 2, 2, 2, 2, 2, 5, 3, 3, 5, 10, 7, 6, 7,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 10, 10, 10, 255, 194, 0, 17, 8, 0, 16, 0, 16, 3, 1, 17, 0, 2, 17, 1, 3, 17, 1, 255,
        196, 0, 22, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 8, 255, 196, 0, 23, 1,
        0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 8, 6, 255, 218, 0, 12, 3, 1, 0, 2,
        16, 3, 16, 0, 0, 1, 206, 192, 224, 218, 121, 39, 100, 224, 91, 57, 38, 158, 255, 196, 0,
        23, 16, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 6, 19, 255, 218, 0, 8, 1, 1,
        0, 1, 5, 2, 83, 37, 128, 162, 75, 1, 68, 150, 2, 137, 44, 15, 255, 196, 0, 24, 17, 0, 2, 3,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 17, 19, 33, 255, 218, 0, 8, 1, 3, 1, 1, 63, 1,
        88, 96, 182, 52, 88, 96, 182, 52, 88, 96, 182, 52, 88, 96, 182, 52, 255, 196, 0, 28, 17, 0,
        1, 4, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 4, 5, 129, 177, 18, 21, 65, 162, 255, 218,
        0, 8, 1, 2, 1, 1, 63, 1, 90, 243, 161, 233, 49, 133, 146, 98, 150, 188, 232, 122, 76, 97,
        100, 152, 165, 175, 58, 30, 147, 226, 201, 49, 75, 94, 116, 61, 38, 48, 178, 76, 87, 255,
        196, 0, 23, 16, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 35, 225, 255, 218, 0,
        8, 1, 1, 0, 6, 63, 2, 83, 193, 79, 5, 60, 20, 240, 255, 196, 0, 23, 16, 0, 3, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 81, 97, 255, 218, 0, 8, 1, 1, 0, 1, 63, 33, 185, 120,
        42, 94, 10, 151, 130, 197, 224, 255, 218, 0, 12, 3, 1, 0, 2, 0, 3, 0, 0, 0, 16, 2, 15, 255,
        196, 0, 24, 17, 0, 2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 17, 240, 33, 97, 209, 255,
        218, 0, 8, 1, 3, 1, 1, 63, 16, 87, 56, 27, 87, 56, 27, 87, 56, 27, 87, 56, 27, 255, 196, 0,
        26, 17, 1, 0, 2, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 33, 0, 17, 65, 225, 240, 193, 255,
        218, 0, 8, 1, 2, 1, 1, 63, 16, 94, 153, 204, 28, 84, 175, 76, 230, 14, 42, 87, 227, 28,
        193, 197, 74, 244, 206, 96, 226, 167, 255, 196, 0, 29, 16, 0, 1, 4, 2, 3, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 17, 33, 81, 129, 177, 1, 113, 49, 65, 145, 255, 218, 0, 8, 1, 1, 0, 1, 63,
        16, 213, 198, 163, 210, 76, 81, 88, 245, 26, 139, 38, 43, 30, 71, 196, 106, 44, 152, 173,
        44, 106, 44, 152, 175, 255, 217,
    ];

    #[test]
    fn jpeg_baseline_and_progressive_match_libjpeg() {
        let base = decode_jpeg(BASELINE_444).unwrap();
        let prog = decode_jpeg(PROGRESSIVE_444).unwrap();
        assert_eq!(base.width(), 16);
        assert_eq!(base.height(), 16);
        // Baseline and Progressive encodings of the exact same image must decode identically!
        assert_eq!(base.data(), prog.data());

        // Compare first 8 and last 4 pixels against libjpeg (Pillow) reference within <= 2 levels
        let ref_first: [u8; 32] = [
            0, 0, 0, 255, 18, 0, 16, 255, 35, 1, 36, 255, 46, 0, 47, 255, 72, 1, 69, 255, 83, 0,
            82, 255, 99, 1, 100, 255, 119, 0, 120, 255,
        ];
        let ref_last: [u8; 16] = [
            200, 255, 52, 255, 221, 255, 37, 255, 243, 253, 17, 255, 254, 255, 1, 255,
        ];
        for (i, (&got, &exp)) in base.data()[..32].iter().zip(ref_first.iter()).enumerate() {
            assert!((got as i16 - exp as i16).abs() <= 2, "first[{i}] got={got} exp={exp}");
        }
        let tail = &base.data()[base.data().len() - 16..];
        for (i, (&got, &exp)) in tail.iter().zip(ref_last.iter()).enumerate() {
            assert!((got as i16 - exp as i16).abs() <= 2, "last[{i}] got={got} exp={exp}");
        }
    }

    #[test]
    fn jpeg_grayscale_matches_libjpeg() {
        let gray = decode_jpeg(GRAYSCALE_16X16).unwrap();
        assert_eq!(gray.width(), 16);
        assert_eq!(gray.height(), 16);
        let ref_first: [u8; 32] = [
            0, 0, 0, 255, 7, 7, 7, 255, 15, 15, 15, 255, 19, 19, 19, 255, 30, 30, 30, 255, 34, 34,
            34, 255, 42, 42, 42, 255, 49, 49, 49, 255,
        ];
        for (i, (&got, &exp)) in gray.data()[..32].iter().zip(ref_first.iter()).enumerate() {
            assert!((got as i16 - exp as i16).abs() <= 1, "gray[{i}] got={got} exp={exp}");
        }
    }

    // Authentic libjpeg (Pillow 9.0.1) 17x13 Baseline 4:2:0 odd-dimension fixture
    const BASELINE_420_ODD: &[u8] = &[
        255, 216, 255, 224, 0, 16, 74, 70, 73, 70, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 255, 219, 0, 67,
        0, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 2, 2, 2, 2, 2, 4, 3, 2, 2, 2, 2, 5, 4, 4, 3, 4, 6, 5, 6,
        6, 6, 5, 6, 6, 6, 7, 9, 8, 6, 7, 9, 7, 6, 6, 8, 11, 8, 9, 10, 10, 10, 10, 10, 6, 8, 11, 12,
        11, 10, 12, 9, 10, 10, 10, 255, 219, 0, 67, 1, 2, 2, 2, 2, 2, 2, 5, 3, 3, 5, 10, 7, 6, 7,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
        10, 10, 10, 10, 255, 192, 0, 17, 8, 0, 13, 0, 17, 3, 1, 34, 0, 2, 17, 1, 3, 17, 1, 255,
        196, 0, 31, 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
        9, 10, 11, 255, 196, 0, 181, 16, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125, 1, 2, 3,
        0, 4, 17, 5, 18, 33, 49, 65, 6, 19, 81, 97, 7, 34, 113, 20, 50, 129, 145, 161, 8, 35, 66,
        177, 193, 21, 82, 209, 240, 36, 51, 98, 114, 130, 9, 10, 22, 23, 24, 25, 26, 37, 38, 39,
        40, 41, 42, 52, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70, 71, 72, 73, 74, 83, 84, 85, 86, 87,
        88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119, 120, 121, 122,
        131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151, 152, 153, 154, 162,
        163, 164, 165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183, 184, 185, 186, 194,
        195, 196, 197, 198, 199, 200, 201, 202, 210, 211, 212, 213, 214, 215, 216, 217, 218, 225,
        226, 227, 228, 229, 230, 231, 232, 233, 234, 241, 242, 243, 244, 245, 246, 247, 248, 249,
        250, 255, 196, 0, 31, 1, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11, 255, 196, 0, 181, 17, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 119,
        0, 1, 2, 3, 17, 4, 5, 33, 49, 6, 18, 65, 81, 7, 97, 113, 19, 34, 50, 129, 8, 20, 66, 145,
        161, 177, 193, 9, 35, 51, 82, 240, 21, 98, 114, 209, 10, 22, 36, 52, 225, 37, 241, 23, 24,
        25, 26, 38, 39, 40, 41, 42, 53, 54, 55, 56, 57, 58, 67, 68, 69, 70, 71, 72, 73, 74, 83, 84,
        85, 86, 87, 88, 89, 90, 99, 100, 101, 102, 103, 104, 105, 106, 115, 116, 117, 118, 119,
        120, 121, 122, 130, 131, 132, 133, 134, 135, 136, 137, 138, 146, 147, 148, 149, 150, 151,
        152, 153, 154, 162, 163, 164, 165, 166, 167, 168, 169, 170, 178, 179, 180, 181, 182, 183,
        184, 185, 186, 194, 195, 196, 197, 198, 199, 200, 201, 202, 210, 211, 212, 213, 214, 215,
        216, 217, 218, 226, 227, 228, 229, 230, 231, 232, 233, 234, 242, 243, 244, 245, 246, 247,
        248, 249, 250, 255, 218, 0, 12, 3, 1, 0, 2, 17, 3, 17, 0, 63, 0, 248, 43, 225, 71, 236, 57,
        253, 135, 229, 127, 196, 163, 118, 236, 127, 203, 60, 121, 120, 254, 121, 207, 225, 252,
        190, 215, 248, 81, 251, 14, 127, 97, 249, 95, 241, 40, 221, 187, 31, 242, 207, 30, 94, 63,
        158, 115, 248, 127, 47, 180, 126, 18, 254, 206, 222, 8, 209, 252, 141, 137, 191, 206, 251,
        159, 186, 3, 102, 49, 158, 252, 231, 63, 134, 43, 214, 254, 18, 126, 206, 222, 8, 210, 5,
        185, 68, 223, 231, 125, 207, 221, 1, 179, 24, 207, 126, 115, 154, 241, 120, 191, 199, 44,
        103, 128, 247, 179, 117, 189, 183, 253, 185, 245, 78, 91, 91, 254, 126, 123, 127, 111, 237,
        163, 111, 135, 217, 123, 187, 105, 236, 126, 31, 232, 221, 244, 144, 204, 51, 79, 99, 239,
        73, 90, 221, 253, 239, 202, 214, 183, 207, 243, 252, 243, 255, 0, 135, 124, 15, 250, 1,
        159, 251, 245, 69, 126, 209, 127, 195, 48, 252, 63, 255, 0, 158, 99, 254, 252, 143, 241,
        162, 191, 54, 254, 200, 129, 254, 134, 255, 0, 196, 211, 230, 95, 205, 47, 196, 255, 217,
    ];

    #[test]
    fn jpeg_420_odd_dimensions_matches_libjpeg() {
        let surf = decode_jpeg(BASELINE_420_ODD).unwrap();
        assert_eq!(surf.width(), 17);
        assert_eq!(surf.height(), 13);
        let ref_first: [u8; 16] = [0, 1, 0, 255, 10, 5, 12, 255, 28, 4, 28, 255, 40, 2, 43, 255];
        let ref_last: [u8; 16] = [
            213, 250, 48, 255, 228, 249, 56, 255, 239, 248, 97, 255, 255, 255, 186, 255,
        ];
        for (i, (&got, &exp)) in surf.data()[..16].iter().zip(ref_first.iter()).enumerate() {
            assert!((got as i16 - exp as i16).abs() <= 2, "420 first[{i}] got={got} exp={exp}");
        }
        let tail = &surf.data()[surf.data().len() - 16..];
        for (i, (&got, &exp)) in tail.iter().zip(ref_last.iter()).enumerate() {
            assert!((got as i16 - exp as i16).abs() <= 2, "420 last[{i}] got={got} exp={exp}");
        }
    }

    #[test]
    fn jpeg_malformed_and_limits_rejected() {
        assert!(decode_jpeg(b"").is_err());
        assert!(decode_jpeg(b"not a jpeg").is_err());

        // Every truncated prefix of baseline and progressive JPEGs must never panic
        for cut in 0..BASELINE_444.len() - 4 {
            let _ = decode_jpeg(&BASELINE_444[..cut]);
        }
        for cut in 0..PROGRESSIVE_444.len() - 4 {
            let _ = decode_jpeg(&PROGRESSIVE_444[..cut]);
        }

        // Strict dimension limits
        let tight = JpegDecodeLimits {
            max_width: 8,
            max_height: 8,
            ..JpegDecodeLimits::default()
        };
        assert_eq!(
            decode_jpeg_with_limits(BASELINE_444, tight).unwrap_err(),
            ZvgError::AllocationLimit
        );
    }
}
