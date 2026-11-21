mod common;

use zerovg::math::{EdgeSeg, FrameArena, NO_CHUNK, Point, TileCmd, TileCmdKind, TileSeg};

install_counting_alloc!();

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

    for _ in 0..2 {
        simulate_frame(&mut arena);
    }

    assert_zero_allocs(|| {
        for _ in 0..100 {
            simulate_frame(&mut arena);
        }
    });
}
