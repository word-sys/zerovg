#![forbid(unsafe_code)]

use alloc::vec::Vec;
use core::mem::size_of;

use super::affine::Point;

pub const TILE_SIZE: usize = 32;
pub const TILE_AREA: usize = TILE_SIZE * TILE_SIZE;
pub const CHUNK_CAP: usize = 30;
pub const NO_CHUNK: u32 = u32::MAX;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EdgeSeg {
    pub x0: i32,
    pub y0: i32,
    pub x1: i32,
    pub y1: i32,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TileSeg {
    pub x0: i16,
    pub y0: i16,
    pub x1: i16,
    pub y1: i16,
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TileSegmentChunk {
    pub segs: [TileSeg; CHUNK_CAP],
    pub next: u32,
    pub len: u16,
    _pad: [u8; 10],
}

const _: () = assert!(size_of::<EdgeSeg>() == 16);
const _: () = assert!(size_of::<TileSeg>() == 8);
const _: () = assert!(size_of::<TileSegmentChunk>() == 256);

impl TileSegmentChunk {
    #[inline]
    pub const fn new(next: u32) -> Self {
        Self {
            segs: [TileSeg {
                x0: 0,
                y0: 0,
                x1: 0,
                y1: 0,
            }; CHUNK_CAP],
            next,
            len: 0,
            _pad: [0; 10],
        }
    }

    #[inline]
    pub fn slice(&self) -> &[TileSeg] {
        &self.segs[..self.len as usize]
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TileCmdKind {
    PathFill = 0,
    SolidTile = 1,
    GlyphRun = 2,
    PushClip = 3,
    PopClip = 4,
    PushLayer = 5,
    PopLayer = 6,
    Clear = 7,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileCmd {
    pub kind: TileCmdKind,
    pub flags: u8,
    pub aux: u16,
    pub cmd_index: u32,
    pub head_chunk: u32,
    pub backdrop: i32,
}

const _: () = assert!(size_of::<TileCmd>() == 16);

#[derive(Debug, Default)]
pub struct FrameArena {
    pub edges: Vec<EdgeSeg>,
    pub tile_chunks: Vec<TileSegmentChunk>,
    pub tile_cmds: Vec<TileCmd>,
    pub clip_masks: Vec<[u8; TILE_AREA]>,
    pub scratch_points: Vec<Point>,
    pub backdrop_rows: Vec<i32>,
    pub scratch_u32: Vec<u32>,
    pub scratch_bytes: Vec<u8>,
}

impl FrameArena {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn reset(&mut self) {
        self.edges.clear();
        self.tile_chunks.clear();
        self.tile_cmds.clear();
        self.clip_masks.clear();
        self.scratch_points.clear();
        self.backdrop_rows.clear();
        self.scratch_u32.clear();
        self.scratch_bytes.clear();
    }

    #[inline]
    pub fn alloc_chunk(&mut self, next: u32) -> u32 {
        let idx = self.tile_chunks.len() as u32;
        self.tile_chunks.push(TileSegmentChunk::new(next));
        idx
    }

    #[inline]
    pub fn push_tile_seg(&mut self, head: &mut u32, seg: TileSeg) {
        if *head != NO_CHUNK {
            let chunk = &mut self.tile_chunks[*head as usize];
            if (chunk.len as usize) < CHUNK_CAP {
                chunk.segs[chunk.len as usize] = seg;
                chunk.len += 1;
                return;
            }
        }
        let new_idx = self.alloc_chunk(*head);
        let chunk = &mut self.tile_chunks[new_idx as usize];
        chunk.segs[0] = seg;
        chunk.len = 1;
        *head = new_idx;
    }

    #[inline]
    pub fn alloc_clip_mask(&mut self, fill: u8) -> u32 {
        let idx = self.clip_masks.len() as u32;
        self.clip_masks.push([fill; TILE_AREA]);
        idx
    }

    pub fn bytes_allocated(&self) -> usize {
        self.edges.capacity() * size_of::<EdgeSeg>()
            + self.tile_chunks.capacity() * size_of::<TileSegmentChunk>()
            + self.tile_cmds.capacity() * size_of::<TileCmd>()
            + self.clip_masks.capacity() * TILE_AREA
            + self.scratch_points.capacity() * size_of::<Point>()
            + self.backdrop_rows.capacity() * size_of::<i32>()
            + self.scratch_u32.capacity() * size_of::<u32>()
            + self.scratch_bytes.capacity()
    }

    pub fn bytes_used(&self) -> usize {
        self.edges.len() * size_of::<EdgeSeg>()
            + self.tile_chunks.len() * size_of::<TileSegmentChunk>()
            + self.tile_cmds.len() * size_of::<TileCmd>()
            + self.clip_masks.len() * TILE_AREA
            + self.scratch_points.len() * size_of::<Point>()
            + self.backdrop_rows.len() * size_of::<i32>()
            + self.scratch_u32.len() * size_of::<u32>()
            + self.scratch_bytes.len()
    }

    // Reclaims excess slab capacity after an unusually large spike frame.
    pub fn shrink_to_limit(&mut self, max_bytes: usize) {
        if self.bytes_allocated() <= max_bytes {
            return;
        }
        let per_slab = max_bytes / 8;
        shrink_vec(&mut self.edges, per_slab / size_of::<EdgeSeg>());
        shrink_vec(&mut self.tile_chunks, per_slab / size_of::<TileSegmentChunk>());
        shrink_vec(&mut self.tile_cmds, per_slab / size_of::<TileCmd>());
        shrink_vec(&mut self.clip_masks, per_slab / TILE_AREA);
        shrink_vec(&mut self.scratch_points, per_slab / size_of::<Point>());
        shrink_vec(&mut self.backdrop_rows, per_slab / size_of::<i32>());
        shrink_vec(&mut self.scratch_u32, per_slab / size_of::<u32>());
        shrink_vec(&mut self.scratch_bytes, per_slab);
    }
}

fn shrink_vec<T>(v: &mut Vec<T>, max_cap: usize) {
    let target = max_cap.max(v.len());
    if v.capacity() > target {
        v.shrink_to(target);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunk_chain_and_reset() {
        let mut arena = FrameArena::new();
        let mut head = NO_CHUNK;
        for i in 0..75i16 {
            arena.push_tile_seg(
                &mut head,
                TileSeg {
                    x0: i,
                    y0: i + 1,
                    x1: i + 2,
                    y1: i + 3,
                },
            );
        }
        // 75 segments across 30-cap chunks = 3 chunks (15 + 30 + 30)
        assert_eq!(arena.tile_chunks.len(), 3);

        let mut total = 0usize;
        let mut cur = head;
        while cur != NO_CHUNK {
            let chunk = &arena.tile_chunks[cur as usize];
            total += chunk.slice().len();
            cur = chunk.next;
        }
        assert_eq!(total, 75);

        let cap_before = arena.bytes_allocated();
        assert!(arena.bytes_used() > 0);
        arena.reset();
        assert_eq!(arena.bytes_used(), 0);
        assert_eq!(arena.bytes_allocated(), cap_before);
    }

    #[test]
    fn shrink_to_limit_caps_memory() {
        let mut arena = FrameArena::new();
        arena.edges.resize(10_000, EdgeSeg::default());
        arena.clip_masks.resize(128, [0; TILE_AREA]);
        assert!(arena.bytes_allocated() > 200_000);

        arena.reset();
        arena.shrink_to_limit(64 * 1024);
        assert!(arena.bytes_allocated() <= 64 * 1024 + 4096);
    }
}
