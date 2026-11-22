#![forbid(unsafe_code)]

use alloc::vec;
use alloc::vec::Vec;

use crate::error::{Result, ZvgError};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompressionLevel {
    None,
    Fast,
    #[default]
    Balanced,
}

const fn build_crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0u32;
    while i < 256 {
        let mut c = i;
        let mut k = 0;
        while k < 8 {
            if (c & 1) != 0 {
                c = 0xEDB8_8320 ^ (c >> 1);
            } else {
                c >>= 1;
            }
            k += 1;
        }
        table[i as usize] = c;
        i += 1;
    }
    table
}

static CRC32_TABLE: [u32; 256] = build_crc32_table();

#[derive(Debug, Clone, Copy)]
pub struct Crc32 {
    crc: u32,
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc32 {
    #[inline]
    pub const fn new() -> Self {
        Self { crc: 0xFFFF_FFFF }
    }

    #[inline]
    pub fn update(&mut self, data: &[u8]) {
        let mut c = self.crc;
        for &b in data {
            c = CRC32_TABLE[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
        }
        self.crc = c;
    }

    #[inline]
    pub const fn finalize(self) -> u32 {
        !self.crc
    }
}

#[inline]
pub fn crc32(data: &[u8]) -> u32 {
    let mut h = Crc32::new();
    h.update(data);
    h.finalize()
}

#[derive(Debug, Clone, Copy)]
pub struct Adler32 {
    s1: u32,
    s2: u32,
}

impl Default for Adler32 {
    fn default() -> Self {
        Self::new()
    }
}

impl Adler32 {
    const MOD: u32 = 65521;
    const NMAX: usize = 5552;

    #[inline]
    pub const fn new() -> Self {
        Self { s1: 1, s2: 0 }
    }

    pub fn update(&mut self, data: &[u8]) {
        let mut s1 = self.s1;
        let mut s2 = self.s2;
        for chunk in data.chunks(Self::NMAX) {
            for &b in chunk {
                s1 += b as u32;
                s2 += s1;
            }
            s1 %= Self::MOD;
            s2 %= Self::MOD;
        }
        self.s1 = s1;
        self.s2 = s2;
    }

    #[inline]
    pub const fn finalize(self) -> u32 {
        (self.s2 << 16) | self.s1
    }
}

#[inline]
pub fn adler32(data: &[u8]) -> u32 {
    let mut h = Adler32::new();
    h.update(data);
    h.finalize()
}

// RFC 1951 static length & distance tables.
const LEN_BASE: [u16; 29] = [
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131,
    163, 195, 227, 258,
];
const LEN_EXTRA: [u8; 29] = [
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0,
];
const DIST_BASE: [u16; 30] = [
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537,
    2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577,
];
const DIST_EXTRA: [u8; 30] = [
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13,
    13,
];
const CL_ORDER: [usize; 19] = [
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15,
];

#[inline]
fn reverse_bits(code: u32, len: u8) -> u32 {
    if len == 0 {
        0
    } else {
        code.reverse_bits() >> (32 - len)
    }
}

// ---------- INFLATE (RFC 1951 Decoder) ----------

struct BitReader<'a> {
    data: &'a [u8],
    pos: usize,
    bits: u64,
    count: u32,
}

impl<'a> BitReader<'a> {
    #[inline]
    fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            bits: 0,
            count: 0,
        }
    }

    #[inline]
    fn refill(&mut self) {
        while self.count <= 56 && self.pos < self.data.len() {
            self.bits |= (self.data[self.pos] as u64) << self.count;
            self.pos += 1;
            self.count += 8;
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
                return Err(ZvgError::ParseError("deflate unexpected eof"));
            }
        }
        let val = (self.bits & ((1u64 << n) - 1)) as u32;
        self.bits >>= n;
        self.count -= n;
        Ok(val)
    }

    #[inline]
    fn peek_bits(&mut self, n: u32) -> u32 {
        if self.count < n {
            self.refill();
        }
        (self.bits & ((1u64 << n) - 1)) as u32
    }

    #[inline]
    fn consume_bits(&mut self, n: u32) -> Result<()> {
        if self.count < n {
            return Err(ZvgError::ParseError("deflate unexpected eof"));
        }
        self.bits >>= n;
        self.count -= n;
        Ok(())
    }

    #[inline]
    fn align_byte(&mut self) {
        let drop = self.count & 7;
        self.bits >>= drop;
        self.count -= drop;
    }

    #[inline]
    fn byte_pos(&self) -> usize {
        self.pos - (self.count / 8) as usize
    }

    fn read_aligned_slice(&mut self, len: usize) -> Result<&'a [u8]> {
        self.align_byte();
        let start = self.byte_pos();
        let end = start
            .checked_add(len)
            .ok_or(ZvgError::ParseError("deflate slice overflow"))?;
        if end > self.data.len() {
            return Err(ZvgError::ParseError("deflate stored block eof"));
        }
        self.pos = end;
        self.bits = 0;
        self.count = 0;
        Ok(&self.data[start..end])
    }
}

// 2-level 9-bit primary + secondary canonical Huffman lookup table.
struct HuffTable {
    primary: [u32; 512],
    secondary: Vec<u32>,
    max_len: u8,
}

const PRIMARY_BITS: u32 = 9;
const PRIMARY_MASK: usize = (1 << PRIMARY_BITS) - 1;
const SECONDARY_FLAG: u32 = 1 << 31;

impl HuffTable {
    fn build(lengths: &[u8]) -> Result<Self> {
        let mut bl_count = [0u32; 16];
        let mut max_len = 0u8;
        let mut num_syms = 0u32;
        for &l in lengths {
            if l > 15 {
                return Err(ZvgError::ParseError("invalid huffman code length"));
            }
            if l > 0 {
                bl_count[l as usize] += 1;
                if l > max_len {
                    max_len = l;
                }
                num_syms += 1;
            }
        }

        if num_syms == 0 {
            return Ok(Self {
                primary: [0; 512],
                secondary: Vec::new(),
                max_len: 0,
            });
        }

        // Strict Kraft inequality check (RFC 1951 permits incomplete tree only for 1 symbol of len 1).
        let mut left = 1i32;
        for &count in &bl_count[1..] {
            left <<= 1;
            left -= count as i32;
            if left < 0 {
                return Err(ZvgError::ParseError("oversubscribed huffman tree"));
            }
        }
        if left > 0 && !(num_syms == 1 && max_len == 1) {
            return Err(ZvgError::ParseError("incomplete huffman tree"));
        }

        let mut next_code = [0u32; 16];
        let mut code = 0u32;
        for bits in 1..=15 {
            code = (code + bl_count[bits - 1]) << 1;
            next_code[bits] = code;
        }

        let mut primary = [0u32; 512];
        let mut secondary = Vec::new();

        // First pass: assign codes <= 9 bits directly into primary table and track secondary needs.
        let mut sym_codes = vec![(0u32, 0u8); lengths.len()];
        let mut sec_max_bits = [0u8; 512];

        for (sym, &len) in lengths.iter().enumerate() {
            if len == 0 {
                continue;
            }
            let c = next_code[len as usize];
            next_code[len as usize] += 1;
            let rev = reverse_bits(c, len);
            sym_codes[sym] = (rev, len);

            if (len as u32) <= PRIMARY_BITS {
                let entry = ((sym as u32) << 8) | (len as u32);
                let step = 1usize << len;
                let mut idx = rev as usize;
                while idx < 512 {
                    primary[idx] = entry;
                    idx += step;
                }
            } else {
                let prefix = (rev as usize) & PRIMARY_MASK;
                let extra = len - (PRIMARY_BITS as u8);
                if extra > sec_max_bits[prefix] {
                    sec_max_bits[prefix] = extra;
                }
            }
        }

        // Allocate secondary subtables for prefixes needing > 9 bits.
        for prefix in 0..512 {
            let sub_bits = sec_max_bits[prefix];
            if sub_bits > 0 {
                let off = secondary.len() as u32;
                secondary.resize(secondary.len() + (1usize << sub_bits), 0);
                primary[prefix] = SECONDARY_FLAG | (off << 8) | (sub_bits as u32);
            }
        }

        // Populate secondary subtables.
        for (sym, &(rev, len)) in sym_codes.iter().enumerate() {
            if (len as u32) <= PRIMARY_BITS {
                continue;
            }
            let prefix = (rev as usize) & PRIMARY_MASK;
            let p = primary[prefix];
            let sub_bits = (p & 0xFF) as u8;
            let off = ((p & !SECONDARY_FLAG) >> 8) as usize;
            let sub_len = len - (PRIMARY_BITS as u8);
            let entry = ((sym as u32) << 8) | (sub_len as u32);
            let step = 1usize << sub_len;
            let mut idx = (rev >> PRIMARY_BITS) as usize;
            let sub_size = 1usize << sub_bits;
            while idx < sub_size {
                secondary[off + idx] = entry;
                idx += step;
            }
        }

        Ok(Self {
            primary,
            secondary,
            max_len,
        })
    }

    #[inline]
    fn decode(&self, br: &mut BitReader<'_>) -> Result<u16> {
        if self.max_len == 0 {
            return Err(ZvgError::ParseError("empty huffman table"));
        }
        let peek = br.peek_bits(PRIMARY_BITS) as usize;
        let entry = self.primary[peek];
        if entry == 0 {
            return Err(ZvgError::ParseError("invalid huffman code"));
        }
        if (entry & SECONDARY_FLAG) == 0 {
            let len = entry & 0xFF;
            br.consume_bits(len)?;
            Ok((entry >> 8) as u16)
        } else {
            br.consume_bits(PRIMARY_BITS)?;
            let sub_bits = entry & 0xFF;
            let off = ((entry & !SECONDARY_FLAG) >> 8) as usize;
            let sub_peek = br.peek_bits(sub_bits) as usize;
            let sub_entry = self.secondary[off + sub_peek];
            if sub_entry == 0 {
                return Err(ZvgError::ParseError("invalid secondary huffman code"));
            }
            let sub_len = sub_entry & 0xFF;
            br.consume_bits(sub_len)?;
            Ok((sub_entry >> 8) as u16)
        }
    }
}

fn fixed_lit_table() -> HuffTable {
    let mut lens = [0u8; 288];
    lens[..144].fill(8);
    lens[144..256].fill(9);
    lens[256..280].fill(7);
    lens[280..288].fill(8);
    HuffTable::build(&lens).unwrap()
}

fn fixed_dist_table() -> HuffTable {
    let lens = [5u8; 32];
    HuffTable::build(&lens).unwrap()
}

fn decode_huffman_block(
    br: &mut BitReader<'_>,
    lit_tbl: &HuffTable,
    dist_tbl: &HuffTable,
    out: &mut Vec<u8>,
    max_output_bytes: usize,
) -> Result<()> {
    loop {
        let sym = lit_tbl.decode(br)?;
        if sym < 256 {
            if out.len() >= max_output_bytes {
                return Err(ZvgError::AllocationLimit);
            }
            out.push(sym as u8);
        } else if sym == 256 {
            return Ok(());
        } else if sym <= 285 {
            let li = (sym - 257) as usize;
            let extra_l = LEN_EXTRA[li] as u32;
            let length = LEN_BASE[li] as usize + br.read_bits(extra_l)? as usize;

            let dsym = dist_tbl.decode(br)? as usize;
            if dsym >= 30 {
                return Err(ZvgError::ParseError("invalid deflate distance symbol"));
            }
            let extra_d = DIST_EXTRA[dsym] as u32;
            let dist = DIST_BASE[dsym] as usize + br.read_bits(extra_d)? as usize;
            if dist == 0 || dist > out.len() {
                return Err(ZvgError::ParseError("deflate distance out of range"));
            }
            if out.len().saturating_add(length) > max_output_bytes {
                return Err(ZvgError::AllocationLimit);
            }
            let start = out.len() - dist;
            if dist >= length {
                out.extend_from_within(start..start + length);
            } else {
                // Overlapping RLE match copy
                out.reserve(length);
                for i in 0..length {
                    let b = out[start + i];
                    out.push(b);
                }
            }
        } else {
            return Err(ZvgError::ParseError("invalid deflate literal/length symbol"));
        }
    }
}

pub fn inflate_raw_with_consumed(data: &[u8], max_output_bytes: usize) -> Result<(Vec<u8>, usize)> {
    let mut br = BitReader::new(data);
    let mut out = Vec::new();

    loop {
        let bfinal = br.read_bits(1)?;
        let btype = br.read_bits(2)?;
        match btype {
            0 => {
                let hdr = br.read_aligned_slice(4)?;
                let len = u16::from_le_bytes([hdr[0], hdr[1]]);
                let nlen = u16::from_le_bytes([hdr[2], hdr[3]]);
                if len != !nlen {
                    return Err(ZvgError::ParseError("stored block length mismatch"));
                }
                let len = len as usize;
                if out.len().saturating_add(len) > max_output_bytes {
                    return Err(ZvgError::AllocationLimit);
                }
                let raw = br.read_aligned_slice(len)?;
                out.extend_from_slice(raw);
            }
            1 => {
                let lit_tbl = fixed_lit_table();
                let dist_tbl = fixed_dist_table();
                decode_huffman_block(&mut br, &lit_tbl, &dist_tbl, &mut out, max_output_bytes)?;
            }
            2 => {
                let hlit = br.read_bits(5)? as usize + 257;
                let hdist = br.read_bits(5)? as usize + 1;
                let hclen = br.read_bits(4)? as usize + 4;
                if hlit > 286 || hdist > 32 {
                    return Err(ZvgError::ParseError("invalid dynamic huffman header"));
                }

                let mut cl_lens = [0u8; 19];
                for &idx in &CL_ORDER[..hclen] {
                    cl_lens[idx] = br.read_bits(3)? as u8;
                }
                let cl_tbl = HuffTable::build(&cl_lens)?;

                let total = hlit + hdist;
                let mut lens = vec![0u8; total];
                let mut i = 0usize;
                while i < total {
                    let sym = cl_tbl.decode(&mut br)?;
                    match sym {
                        0..=15 => {
                            lens[i] = sym as u8;
                            i += 1;
                        }
                        16 => {
                            if i == 0 {
                                return Err(ZvgError::ParseError("repeat code at start"));
                            }
                            let rep = br.read_bits(2)? as usize + 3;
                            if i + rep > total {
                                return Err(ZvgError::ParseError("code length repeat overflow"));
                            }
                            let prev = lens[i - 1];
                            lens[i..i + rep].fill(prev);
                            i += rep;
                        }
                        17 => {
                            let rep = br.read_bits(3)? as usize + 3;
                            if i + rep > total {
                                return Err(ZvgError::ParseError("zero repeat overflow"));
                            }
                            i += rep;
                        }
                        18 => {
                            let rep = br.read_bits(7)? as usize + 11;
                            if i + rep > total {
                                return Err(ZvgError::ParseError("long zero repeat overflow"));
                            }
                            i += rep;
                        }
                        _ => return Err(ZvgError::ParseError("invalid code length symbol")),
                    }
                }

                if lens[256] == 0 {
                    return Err(ZvgError::ParseError("missing end-of-block symbol"));
                }
                let lit_tbl = HuffTable::build(&lens[..hlit])?;
                let dist_tbl = HuffTable::build(&lens[hlit..])?;
                decode_huffman_block(&mut br, &lit_tbl, &dist_tbl, &mut out, max_output_bytes)?;
            }
            _ => return Err(ZvgError::ParseError("invalid deflate block type")),
        }
        if bfinal == 1 {
            break;
        }
    }

    br.align_byte();
    Ok((out, br.byte_pos()))
}

pub fn inflate_raw(data: &[u8], max_output_bytes: usize) -> Result<Vec<u8>> {
    inflate_raw_with_consumed(data, max_output_bytes).map(|(out, _)| out)
}

pub fn zlib_decompress(data: &[u8], max_output_bytes: usize) -> Result<Vec<u8>> {
    if data.len() < 6 {
        return Err(ZvgError::ParseError("zlib stream too short"));
    }
    let cmf = data[0];
    let flg = data[1];
    if (((cmf as u16) << 8) | (flg as u16)) % 31 != 0 {
        return Err(ZvgError::ParseError("invalid zlib header checksum"));
    }
    if (cmf & 0x0F) != 8 {
        return Err(ZvgError::ParseError("unsupported zlib compression method"));
    }
    if (cmf >> 4) > 7 {
        return Err(ZvgError::ParseError("invalid zlib window size"));
    }
    if (flg & 0x20) != 0 {
        return Err(ZvgError::UnsupportedFeature("zlib preset dictionary"));
    }

    let payload = &data[2..];
    let (out, consumed) = inflate_raw_with_consumed(payload, max_output_bytes)?;
    if consumed + 4 > payload.len() {
        return Err(ZvgError::ParseError("missing zlib adler32 trailer"));
    }
    let expected = u32::from_be_bytes([
        payload[consumed],
        payload[consumed + 1],
        payload[consumed + 2],
        payload[consumed + 3],
    ]);
    if adler32(&out) != expected {
        return Err(ZvgError::ParseError("zlib adler32 mismatch"));
    }
    Ok(out)
}

// ---------- DEFLATE (RFC 1951 Encoder) ----------

struct BitWriter {
    out: Vec<u8>,
    bits: u64,
    count: u32,
}

impl BitWriter {
    fn new() -> Self {
        Self {
            out: Vec::new(),
            bits: 0,
            count: 0,
        }
    }

    #[inline]
    fn write_bits(&mut self, val: u32, n: u32) {
        self.bits |= (val as u64) << self.count;
        self.count += n;
        while self.count >= 8 {
            self.out.push((self.bits & 0xFF) as u8);
            self.bits >>= 8;
            self.count -= 8;
        }
    }

    fn flush_byte(&mut self) {
        if self.count > 0 {
            self.out.push((self.bits & 0xFF) as u8);
            self.bits = 0;
            self.count = 0;
        }
    }

    fn write_aligned_bytes(&mut self, bytes: &[u8]) {
        self.flush_byte();
        self.out.extend_from_slice(bytes);
    }

    fn finish(mut self) -> Vec<u8> {
        self.flush_byte();
        self.out
    }
}

#[derive(Clone, Copy)]
enum Token {
    Literal(u8),
    Match { len: u16, dist: u16 },
}

#[inline]
fn length_code(len: u16) -> (u16, u8, u16) {
    if len == 258 {
        return (285, 0, 0);
    }
    let mut i = 0usize;
    while i + 1 < 28 && len >= LEN_BASE[i + 1] {
        i += 1;
    }
    (257 + i as u16, LEN_EXTRA[i], len - LEN_BASE[i])
}

#[inline]
fn dist_code(dist: u16) -> (u16, u8, u16) {
    let mut i = 0usize;
    while i + 1 < 30 && dist >= DIST_BASE[i + 1] {
        i += 1;
    }
    (i as u16, DIST_EXTRA[i], dist - DIST_BASE[i])
}

// Package-Merge length-limited optimal canonical Huffman code-length builder.
fn build_limited_code_lengths(freqs: &[u32], max_bits: usize) -> Vec<u8> {
    let mut lens = vec![0u8; freqs.len()];
    let mut active: Vec<(u64, usize)> = freqs
        .iter()
        .enumerate()
        .filter(|&(_, &f)| f > 0)
        .map(|(i, &f)| (f as u64, i))
        .collect();

    if active.is_empty() {
        return lens;
    }
    if active.len() == 1 {
        lens[active[0].1] = 1;
        return lens;
    }

    active.sort_unstable_by(|a, b| a.0.cmp(&b.0).then(a.1.cmp(&b.1)));
    let n = active.len();

    // Node in package-merge chain: weight + list of leaf symbol indices inside package.
    // Max active symbols <= 286 and max_bits <= 15, so cloning small index lists is fast & simple.
    let base: Vec<(u64, Vec<u16>)> = active.iter().map(|&(w, s)| (w, vec![s as u16])).collect();
    let mut level: Vec<(u64, Vec<u16>)> = base.clone();

    for _ in 1..max_bits {
        // Pair adjacent items from previous level into packages
        let mut packages = Vec::with_capacity(level.len() / 2);
        let mut i = 0;
        while i + 1 < level.len() {
            let w = level[i].0 + level[i + 1].0;
            let mut syms = Vec::with_capacity(level[i].1.len() + level[i + 1].1.len());
            syms.extend_from_slice(&level[i].1);
            syms.extend_from_slice(&level[i + 1].1);
            packages.push((w, syms));
            i += 2;
        }

        // Merge sorted base leaves and sorted packages
        let mut next = Vec::with_capacity(base.len() + packages.len());
        let (mut bi, mut pi) = (0usize, 0usize);
        while bi < base.len() && pi < packages.len() {
            if base[bi].0 <= packages[pi].0 {
                next.push(base[bi].clone());
                bi += 1;
            } else {
                next.push(packages[pi].clone());
                pi += 1;
            }
        }
        while bi < base.len() {
            next.push(base[bi].clone());
            bi += 1;
        }
        while pi < packages.len() {
            next.push(packages[pi].clone());
            pi += 1;
        }
        level = next;
    }

    let take = (2 * n - 2).min(level.len());
    for (_, syms) in &level[..take] {
        for &s in syms {
            lens[s as usize] += 1;
        }
    }
    lens
}

fn canonical_codes_reversed(lens: &[u8]) -> Vec<u32> {
    let mut bl_count = [0u32; 16];
    for &l in lens {
        if l > 0 {
            bl_count[l as usize] += 1;
        }
    }
    let mut next_code = [0u32; 16];
    let mut code = 0u32;
    for bits in 1..=15 {
        code = (code + bl_count[bits - 1]) << 1;
        next_code[bits] = code;
    }
    let mut codes = vec![0u32; lens.len()];
    for (sym, &l) in lens.iter().enumerate() {
        if l > 0 {
            let c = next_code[l as usize];
            next_code[l as usize] += 1;
            codes[sym] = reverse_bits(c, l);
        }
    }
    codes
}

fn write_stored_blocks(bw: &mut BitWriter, raw: &[u8], is_final: bool) {
    if raw.is_empty() {
        bw.write_bits(if is_final { 1 } else { 0 }, 1);
        bw.write_bits(0, 2);
        bw.write_aligned_bytes(&[0, 0, 0xFF, 0xFF]);
        return;
    }
    let mut off = 0usize;
    while off < raw.len() {
        let take = (raw.len() - off).min(65535);
        let last = is_final && (off + take == raw.len());
        bw.write_bits(if last { 1 } else { 0 }, 1);
        bw.write_bits(0, 2);
        let len = take as u16;
        let nlen = !len;
        let mut hdr = [0u8; 4];
        hdr[0..2].copy_from_slice(&len.to_le_bytes());
        hdr[2..4].copy_from_slice(&nlen.to_le_bytes());
        bw.write_aligned_bytes(&hdr);
        bw.out.extend_from_slice(&raw[off..off + take]);
        off += take;
    }
}

fn write_tokens(
    bw: &mut BitWriter,
    tokens: &[Token],
    lit_codes: &[u32],
    lit_lens: &[u8],
    dist_codes: &[u32],
    dist_lens: &[u8],
) {
    for &tok in tokens {
        match tok {
            Token::Literal(b) => {
                let s = b as usize;
                bw.write_bits(lit_codes[s], lit_lens[s] as u32);
            }
            Token::Match { len, dist } => {
                let (l_sym, l_ex_bits, l_ex_val) = length_code(len);
                bw.write_bits(lit_codes[l_sym as usize], lit_lens[l_sym as usize] as u32);
                if l_ex_bits > 0 {
                    bw.write_bits(l_ex_val as u32, l_ex_bits as u32);
                }
                let (d_sym, d_ex_bits, d_ex_val) = dist_code(dist);
                bw.write_bits(dist_codes[d_sym as usize], dist_lens[d_sym as usize] as u32);
                if d_ex_bits > 0 {
                    bw.write_bits(d_ex_val as u32, d_ex_bits as u32);
                }
            }
        }
    }
    bw.write_bits(lit_codes[256], lit_lens[256] as u32);
}

fn rle_code_lengths(lens: &[u8]) -> Vec<(u8, u8, u8)> {
    // Each entry: (symbol 0..=18, extra_bits, extra_val)
    let mut out = Vec::with_capacity(lens.len());
    let mut i = 0usize;
    while i < lens.len() {
        let v = lens[i];
        let mut run = 1usize;
        while i + run < lens.len() && lens[i + run] == v {
            run += 1;
        }
        let step = run;
        if v == 0 {
            while run >= 11 {
                let c = run.min(138);
                out.push((18, 7, (c - 11) as u8));
                run -= c;
            }
            if run >= 3 {
                out.push((17, 3, (run - 3) as u8));
                run = 0;
            }
            while run > 0 {
                out.push((0, 0, 0));
                run -= 1;
            }
        } else {
            out.push((v, 0, 0));
            run -= 1;
            while run >= 3 {
                let c = run.min(6);
                out.push((16, 2, (c - 3) as u8));
                run -= c;
            }
            while run > 0 {
                out.push((v, 0, 0));
                run -= 1;
            }
        }
        i += step;
    }
    out
}

fn emit_block(bw: &mut BitWriter, raw: &[u8], tokens: &[Token], level: CompressionLevel, is_final: bool) {
    if matches!(level, CompressionLevel::None) || raw.is_empty() {
        write_stored_blocks(bw, raw, is_final);
        return;
    }

    // Compute fixed Huffman bit cost
    let mut fixed_lit_lens = [0u8; 288];
    fixed_lit_lens[..144].fill(8);
    fixed_lit_lens[144..256].fill(9);
    fixed_lit_lens[256..280].fill(7);
    fixed_lit_lens[280..288].fill(8);
    let fixed_dist_lens = [5u8; 30];

    let mut lit_freqs = [0u32; 286];
    let mut dist_freqs = [0u32; 30];
    lit_freqs[256] = 1;
    let mut fixed_bits = 3usize + 7; // header + EOB
    for &tok in tokens {
        match tok {
            Token::Literal(b) => {
                lit_freqs[b as usize] += 1;
                fixed_bits += fixed_lit_lens[b as usize] as usize;
            }
            Token::Match { len, dist } => {
                let (ls, le, _) = length_code(len);
                let (ds, de, _) = dist_code(dist);
                lit_freqs[ls as usize] += 1;
                dist_freqs[ds as usize] += 1;
                fixed_bits += fixed_lit_lens[ls as usize] as usize + le as usize + 5 + de as usize;
            }
        }
    }

    let stored_bits = 3 + 7 + ((raw.len() / 65535 + 1) * 32) + raw.len() * 8;

    if matches!(level, CompressionLevel::Fast) {
        if stored_bits < fixed_bits {
            write_stored_blocks(bw, raw, is_final);
        } else {
            let flc = canonical_codes_reversed(&fixed_lit_lens);
            let fdc = canonical_codes_reversed(&fixed_dist_lens);
            bw.write_bits(if is_final { 1 } else { 0 }, 1);
            bw.write_bits(1, 2);
            write_tokens(bw, tokens, &flc, &fixed_lit_lens, &fdc, &fixed_dist_lens);
        }
        return;
    }

    // Dynamic Huffman tree construction (Balanced)
    let lit_lens = build_limited_code_lengths(&lit_freqs, 15);
    let mut dist_lens = build_limited_code_lengths(&dist_freqs, 15);
    if dist_lens.iter().all(|&l| l == 0) {
        dist_lens[0] = 1;
    }

    let mut hlit = 286usize;
    while hlit > 257 && lit_lens[hlit - 1] == 0 {
        hlit -= 1;
    }
    let mut hdist = 30usize;
    while hdist > 1 && dist_lens[hdist - 1] == 0 {
        hdist -= 1;
    }

    let mut combined = Vec::with_capacity(hlit + hdist);
    combined.extend_from_slice(&lit_lens[..hlit]);
    combined.extend_from_slice(&dist_lens[..hdist]);
    let rle = rle_code_lengths(&combined);

    let mut cl_freqs = [0u32; 19];
    for &(sym, _, _) in &rle {
        cl_freqs[sym as usize] += 1;
    }
    let cl_lens = build_limited_code_lengths(&cl_freqs, 7);
    let mut hclen = 19usize;
    while hclen > 4 && cl_lens[CL_ORDER[hclen - 1]] == 0 {
        hclen -= 1;
    }

    let mut dyn_bits = 3 + 14 + hclen * 3;
    for &(sym, ex_bits, _) in &rle {
        dyn_bits += cl_lens[sym as usize] as usize + ex_bits as usize;
    }
    for (s, &f) in lit_freqs.iter().enumerate() {
        dyn_bits += (f as usize) * (lit_lens[s] as usize);
    }
    for (s, &f) in dist_freqs.iter().enumerate() {
        dyn_bits += (f as usize) * (dist_lens[s] as usize);
    }
    for &tok in tokens {
        if let Token::Match { len, dist } = tok {
            dyn_bits += length_code(len).1 as usize + dist_code(dist).1 as usize;
        }
    }

    if stored_bits <= fixed_bits && stored_bits <= dyn_bits {
        write_stored_blocks(bw, raw, is_final);
    } else if fixed_bits <= dyn_bits {
        let flc = canonical_codes_reversed(&fixed_lit_lens);
        let fdc = canonical_codes_reversed(&fixed_dist_lens);
        bw.write_bits(if is_final { 1 } else { 0 }, 1);
        bw.write_bits(1, 2);
        write_tokens(bw, tokens, &flc, &fixed_lit_lens, &fdc, &fixed_dist_lens);
    } else {
        let cl_codes = canonical_codes_reversed(&cl_lens);
        let lit_codes = canonical_codes_reversed(&lit_lens);
        let dist_codes = canonical_codes_reversed(&dist_lens);

        bw.write_bits(if is_final { 1 } else { 0 }, 1);
        bw.write_bits(2, 2);
        bw.write_bits((hlit - 257) as u32, 5);
        bw.write_bits((hdist - 1) as u32, 5);
        bw.write_bits((hclen - 4) as u32, 4);
        for &idx in &CL_ORDER[..hclen] {
            bw.write_bits(cl_lens[idx] as u32, 3);
        }
        for &(sym, ex_bits, ex_val) in &rle {
            bw.write_bits(cl_codes[sym as usize], cl_lens[sym as usize] as u32);
            if ex_bits > 0 {
                bw.write_bits(ex_val as u32, ex_bits as u32);
            }
        }
        write_tokens(bw, tokens, &lit_codes, &lit_lens, &dist_codes, &dist_lens);
    }
}

#[inline]
fn hash3(data: &[u8], pos: usize) -> usize {
    let v = ((data[pos] as u32) << 16) | ((data[pos + 1] as u32) << 8) | (data[pos + 2] as u32);
    ((v.wrapping_mul(0x1E35_A7BD)) >> 18) as usize
}

fn find_best_match(
    data: &[u8],
    pos: usize,
    head: &[u32],
    prev: &[u32],
    max_chain: usize,
) -> (usize, usize) {
    const WINDOW_MASK: usize = 32767;
    const MAX_DIST: usize = 32768;
    const MAX_LEN: usize = 258;
    const NIL: u32 = u32::MAX;

    if pos + 3 > data.len() {
        return (0, 0);
    }
    let max_len = (data.len() - pos).min(MAX_LEN);
    let min_pos = pos.saturating_sub(MAX_DIST);

    let mut cur = head[hash3(data, pos)];
    let mut best_len = 2usize;
    let mut best_dist = 0usize;
    let mut chain = max_chain;

    while cur != NIL && (cur as usize) >= min_pos && chain > 0 {
        let p = cur as usize;
        if p < pos
            && data[p + best_len] == data[pos + best_len]
            && data[p] == data[pos]
            && data[p + 1] == data[pos + 1]
            && data[p + 2] == data[pos + 2]
        {
            let mut l = 3usize;
            while l < max_len && data[p + l] == data[pos + l] {
                l += 1;
            }
            if l > best_len {
                best_len = l;
                best_dist = pos - p;
                if best_len == max_len {
                    break;
                }
            }
        }
        let next = prev[p & WINDOW_MASK];
        if next >= cur {
            break;
        }
        cur = next;
        chain -= 1;
    }

    if best_len >= 3 {
        (best_len, best_dist)
    } else {
        (0, 0)
    }
}

pub fn deflate_raw(data: &[u8], level: CompressionLevel) -> Vec<u8> {
    let mut bw = BitWriter::new();
    if matches!(level, CompressionLevel::None) || data.is_empty() {
        write_stored_blocks(&mut bw, data, true);
        return bw.finish();
    }

    const HASH_SIZE: usize = 1 << 14;
    const WINDOW_SIZE: usize = 32768;
    const WINDOW_MASK: usize = WINDOW_SIZE - 1;
    const BLOCK_TOKENS: usize = 32768;
    const NIL: u32 = u32::MAX;

    let max_chain = match level {
        CompressionLevel::None => 0,
        CompressionLevel::Fast => 12,
        CompressionLevel::Balanced => 48,
    };
    let use_lazy = matches!(level, CompressionLevel::Balanced);

    let mut head = vec![NIL; HASH_SIZE];
    let mut prev = vec![NIL; WINDOW_SIZE];
    let mut tokens = Vec::with_capacity(BLOCK_TOKENS);

    let insert = |pos: usize, head: &mut [u32], prev: &mut [u32]| {
        if pos + 3 <= data.len() {
            let h = hash3(data, pos);
            prev[pos & WINDOW_MASK] = head[h];
            head[h] = pos as u32;
        }
    };

    let mut pos = 0usize;
    let mut block_start = 0usize;
    while pos < data.len() {
        let (len, dist) = find_best_match(data, pos, &head, &prev, max_chain);
        insert(pos, &mut head, &mut prev);

        if len >= 3 {
            // Optional lazy match check at pos + 1
            if use_lazy && len < 32 && pos + 1 < data.len() {
                let (next_len, _) = find_best_match(data, pos + 1, &head, &prev, max_chain / 2);
                if next_len > len + 1 {
                    tokens.push(Token::Literal(data[pos]));
                    pos += 1;
                    if tokens.len() >= BLOCK_TOKENS && pos < data.len() {
                        emit_block(&mut bw, &data[block_start..pos], &tokens, level, false);
                        tokens.clear();
                        block_start = pos;
                    }
                    continue;
                }
            }
            tokens.push(Token::Match {
                len: len as u16,
                dist: dist as u16,
            });
            for k in 1..len {
                insert(pos + k, &mut head, &mut prev);
            }
            pos += len;
        } else {
            tokens.push(Token::Literal(data[pos]));
            pos += 1;
        }

        if tokens.len() >= BLOCK_TOKENS && pos < data.len() {
            emit_block(&mut bw, &data[block_start..pos], &tokens, level, false);
            tokens.clear();
            block_start = pos;
        }
    }

    emit_block(&mut bw, &data[block_start..], &tokens, level, true);
    bw.finish()
}

pub fn zlib_compress(data: &[u8], level: CompressionLevel) -> Vec<u8> {
    let flg: u8 = match level {
        CompressionLevel::None => 0x01,
        CompressionLevel::Fast => 0x5E,
        CompressionLevel::Balanced => 0x9C,
    };
    let deflated = deflate_raw(data, level);
    let mut out = Vec::with_capacity(2 + deflated.len() + 4);
    out.push(0x78);
    out.push(flg);
    out.extend_from_slice(&deflated);
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_and_adler32_reference_vectors() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
        assert_eq!(adler32(b"Wikipedia"), 0x11E6_0398);
        assert_eq!(adler32(b""), 1);

        // Streaming matches one-shot across NMAX boundary
        let big = vec![0x5Au8; 15_000];
        let mut a = Adler32::new();
        for chunk in big.chunks(777) {
            a.update(chunk);
        }
        assert_eq!(a.finalize(), adler32(&big));
    }

    #[test]
    fn deflate_zlib_roundtrip_all_levels_and_payloads() {
        // 1. Empty, 1 byte, small text
        let smalls: [&[u8]; 4] = [
            b"",
            b"X",
            b"Hello ZeroVG pure-Rust RFC 1951 DEFLATE!",
            b"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        ];
        for &payload in &smalls {
            for level in [
                CompressionLevel::None,
                CompressionLevel::Fast,
                CompressionLevel::Balanced,
            ] {
                let z = zlib_compress(payload, level);
                let back = zlib_decompress(&z, 1024 * 1024).unwrap();
                assert_eq!(back, payload);
            }
        }

        // 2. Overlapping LZ77 (dist=1, dist=3) highly repetitive payload > 100 KB
        let mut rle = Vec::with_capacity(120_000);
        for i in 0..40_000 {
            rle.push(b'A');
            rle.push(b'B');
            rle.push(if i % 1000 == 0 { b'Z' } else { b'C' });
        }
        for level in [CompressionLevel::Fast, CompressionLevel::Balanced] {
            let z = zlib_compress(&rle, level);
            assert!(z.len() < rle.len() / 20, "expected strong compression, got {}", z.len());
            assert_eq!(zlib_decompress(&z, 256 * 1024).unwrap(), rle);
        }

        // 3. Incompressible pseudo-random > 200 KB (multi-block stored fallback)
        let mut rng = 0x1234_5678_9ABC_DEF1u64;
        let mut rand_buf = vec![0u8; 200_000];
        for b in &mut rand_buf {
            rng = rng.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            *b = (rng >> 56) as u8;
        }
        for level in [
            CompressionLevel::None,
            CompressionLevel::Fast,
            CompressionLevel::Balanced,
        ] {
            let z = zlib_compress(&rand_buf, level);
            // Must not bloat incompressible data significantly (< 0.5% overhead)
            assert!(z.len() < rand_buf.len() + 1024, "bloated: {}", z.len());
            assert_eq!(zlib_decompress(&z, 300_000).unwrap(), rand_buf);
        }
    }

    #[test]
    fn package_merge_respects_15_bit_huffman_limit() {
        // Fibonacci frequencies force naive Huffman trees > 15 bits (24+ bits!).
        let mut freqs = [0u32; 30];
        let mut a = 1u32;
        let mut b = 1u32;
        for f in &mut freqs {
            *f = a;
            let next = a.saturating_add(b);
            a = b;
            b = next;
        }
        let lens = build_limited_code_lengths(&freqs, 15);
        assert!(lens.iter().all(|&l| (1..=15).contains(&l)));
        // Must build a valid RFC 1951 Huffman table (Kraft exact)
        assert!(HuffTable::build(&lens).is_ok());
    }

    #[test]
    fn malformed_streams_and_zip_bomb_rejected() {
        // Too short / bad checksum / bad method / preset dict
        assert!(zlib_decompress(b"", 1024).is_err());
        assert!(zlib_decompress(&[0x78, 0x00, 0, 0, 0, 0], 1024).is_err());
        assert!(zlib_decompress(&[0x79, 0x00, 0, 0, 0, 0], 1024).is_err());
        assert!(zlib_decompress(&[0x78, 0xBB, 0, 0, 0, 0], 1024).is_err());

        // Bad stored block NLEN
        assert!(inflate_raw(&[0x01, 0x05, 0x00, 0x00, 0x00], 1024).is_err());
        // Invalid BTYPE = 3
        assert!(inflate_raw(&[0x07], 1024).is_err());
        // Truncated stream
        let valid = zlib_compress(b"Hello world repeated Hello world repeated!", CompressionLevel::Balanced);
        for cut in 0..valid.len() {
            assert!(zlib_decompress(&valid[..cut], 1024).is_err(), "cut={cut}");
        }
        // Corrupted Adler-32 trailer
        let mut bad_crc = valid.clone();
        *bad_crc.last_mut().unwrap() ^= 0x01;
        assert!(zlib_decompress(&bad_crc, 1024).is_err());

        // Zip bomb defense: 500 KB of zeros compressed to tiny bytes, cap output at 1024 bytes
        let zeros = vec![0u8; 500_000];
        let bomb = zlib_compress(&zeros, CompressionLevel::Balanced);
        assert!(bomb.len() < 1000);
        assert_eq!(
            zlib_decompress(&bomb, 1024).unwrap_err(),
            ZvgError::AllocationLimit
        );

        // Oversubscribed & incomplete Huffman tables rejected
        assert!(HuffTable::build(&[1, 1, 1]).is_err());
        assert!(HuffTable::build(&[2, 2]).is_err());
    }

    #[test]
    fn external_zlib_interop_vector() {
        // Real C zlib (level 9) dynamic-Huffman output for b"ZeroVG RFC 1951 interop test! "*8 + (0..64)
        const ZLIB_STREAM: &[u8] = &[
            120, 218, 139, 74, 45, 202, 15, 115, 87, 8, 114, 115, 86, 48, 180, 52, 53, 84, 200,
            204, 43, 1, 138, 20, 40, 148, 164, 22, 151, 40, 42, 68, 13, 59, 89, 6, 70, 38, 102,
            22, 86, 54, 118, 14, 78, 46, 110, 30, 94, 62, 126, 1, 65, 33, 97, 17, 81, 49, 113, 9,
            73, 41, 105, 25, 89, 57, 121, 5, 69, 37, 101, 21, 85, 53, 117, 13, 77, 45, 109, 29,
            93, 61, 125, 3, 67, 35, 99, 19, 83, 51, 115, 11, 75, 43, 107, 27, 91, 59, 123, 0,
            233, 210, 83, 49,
        ];
        let mut expected = Vec::new();
        for _ in 0..8 {
            expected.extend_from_slice(b"ZeroVG RFC 1951 interop test! ");
        }
        expected.extend(0u8..64);
        let decoded = zlib_decompress(ZLIB_STREAM, 4096).unwrap();
        assert_eq!(decoded, expected);
    }
}
