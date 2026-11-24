mod common;

use zerovg::geom::flatten::{DEFAULT_TOLERANCE, flatten_path_for_fill};
use zerovg::geom::{FillRule, PathBuilder};
use zerovg::math::{
    Affine2D, EdgeSeg, FrameArena, NO_CHUNK, Point, Rect, TILE_SIZE, TileCmd, TileCmdKind, TileSeg,
};
use zerovg::raster::{CoverageSpan, TileFillParams, TileRasterScratch};
use zerovg::tile::TileBinner;

install_counting_alloc!();

static SERIAL_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

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
    let _guard = SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
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

#[test]
fn flatten_and_tile_binner_zero_steady_state_allocations() {
    let _guard = SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut pb = PathBuilder::new();
    pb.push_circle(96.0, 64.0, 58.0);
    pb.push_rounded_rect(Rect::new(12.5, 10.25, 180.75, 110.5), 16.0, 12.0);
    let path = pb.finish();

    let mut arena = FrameArena::new();
    let mut binner = TileBinner::new();
    let mut scratch = TileRasterScratch::new();
    let mut checksum = 0u64;

    let mut run_frame = |arena: &mut FrameArena, binner: &mut TileBinner| {
        arena.reset();
        binner.reset(256, 160);
        let range =
            flatten_path_for_fill(&path, Affine2D::IDENTITY, 256, 160, DEFAULT_TOLERANCE, arena);
        binner.bin_path(range, FillRule::NonZero, 0, 0, arena);
        for tid in 0..binner.tile_count() {
            for cmd in binner.tile_cmds(tid, arena) {
                if cmd.kind == TileCmdKind::PathFill {
                    let params = TileFillParams::from_cmd(cmd, TILE_SIZE, TILE_SIZE);
                    scratch.rasterize_tile_spans(params, arena, |span| match span {
                        CoverageSpan::Solid { x0, x1, .. } => {
                            checksum = checksum.wrapping_add((x1 - x0) as u64);
                        }
                        CoverageSpan::Fractional { cov, .. } => {
                            checksum = checksum.wrapping_add(cov.len() as u64);
                        }
                    });
                }
            }
        }
    };

    for _ in 0..2 {
        run_frame(&mut arena, &mut binner);
    }

    assert_zero_allocs(|| {
        for _ in 0..100 {
            run_frame(&mut arena, &mut binner);
        }
    });
    assert!(checksum > 0);
}
