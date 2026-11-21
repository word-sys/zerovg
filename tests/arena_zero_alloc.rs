use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use zerovg::math::{EdgeSeg, FrameArena, NO_CHUNK, Point, TileCmd, TileCmdKind, TileSeg};

struct CountingAlloc;

static COUNTING: AtomicBool = AtomicBool::new(false);
static ALLOC_COUNT: AtomicUsize = AtomicUsize::new(0);
static DEALLOC_COUNT: AtomicUsize = AtomicUsize::new(0);

#[global_allocator]
static GLOBAL: CountingAlloc = CountingAlloc;

unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if COUNTING.load(Ordering::Relaxed) {
            DEALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if COUNTING.load(Ordering::Relaxed) {
            ALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

fn simulate_frame(arena: &mut FrameArena) {
    arena.reset();
    for i in 0..512i32 {
        arena.edges.push(EdgeSeg {
            x0: i * 16,
            y0: i * 8,
            x1: i * 16 + 128,
            y1: i * 8 + 256,
        });
        arena.scratch_points.push(Point::new(i as f32, (i * 2) as f32));
        arena.backdrop_rows.push(i & 3);
        arena.scratch_u32.push(i as u32);
    }
    let mut head = NO_CHUNK;
    for i in 0..120i16 {
        arena.push_tile_seg(
            &mut head,
            TileSeg {
                x0: i,
                y0: i,
                x1: i + 10,
                y1: i + 20,
            },
        );
    }
    arena.tile_cmds.push(TileCmd {
        kind: TileCmdKind::PathFill,
        flags: 0,
        aux: 0,
        cmd_index: 0,
        head_chunk: head,
        backdrop: 1,
    });
    let _mask = arena.alloc_clip_mask(255);
    arena.scratch_bytes.extend_from_slice(&[42u8; 256]);
}

#[test]
fn frame_arena_zero_steady_state_allocations() {
    let mut arena = FrameArena::new();

    // Warm-up frames
    for _ in 0..2 {
        simulate_frame(&mut arena);
    }

    ALLOC_COUNT.store(0, Ordering::SeqCst);
    DEALLOC_COUNT.store(0, Ordering::SeqCst);
    COUNTING.store(true, Ordering::SeqCst);

    for _ in 0..100 {
        simulate_frame(&mut arena);
    }

    COUNTING.store(false, Ordering::SeqCst);
    assert_eq!(ALLOC_COUNT.load(Ordering::SeqCst), 0);
    assert_eq!(DEALLOC_COUNT.load(Ordering::SeqCst), 0);
}
