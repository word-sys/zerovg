#[path = "../common/mod.rs"]
mod common;

use zerovg::math::float::sincos_f32;
use zerovg::{
    Affine2D, AntialiasMode, Color, Context, DitherMode, FillRule, Path, PathBuilder, Point, Rect,
    StrokeStyle, Surface,
};

install_counting_alloc!();

static SERIAL_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

struct TigerLayer {
    color: Color,
    fill_rule: FillRule,
    path: Path,
}

// Multi-layer Ghostscript-inspired vector Tiger head built from cubic Beziers & polygons.
fn build_tiger_scene() -> Vec<TigerLayer> {
    let mut layers = Vec::new();

    let mut push_path = |color: Color, fill_rule: FillRule, f: &dyn Fn(&mut PathBuilder)| {
        let mut pb = PathBuilder::new();
        f(&mut pb);
        layers.push(TigerLayer {
            color,
            fill_rule,
            path: pb.finish(),
        });
    };

    // Ears (outer dark silhouette + inner warm fur)
    push_path(Color::from_hex(0x1B1410), FillRule::NonZero, &|pb| {
        pb.move_to(56.0, 96.0)
            .cubic_to(32.0, 42.0, 68.0, 18.0, 114.0, 54.0)
            .close();
        pb.move_to(200.0, 96.0)
            .cubic_to(224.0, 42.0, 188.0, 18.0, 142.0, 54.0)
            .close();
    });
    push_path(Color::from_hex(0xE89B4C), FillRule::NonZero, &|pb| {
        pb.move_to(64.0, 88.0)
            .cubic_to(48.0, 54.0, 74.0, 36.0, 104.0, 60.0)
            .close();
        pb.move_to(192.0, 88.0)
            .cubic_to(208.0, 54.0, 182.0, 36.0, 152.0, 60.0)
            .close();
    });

    // Main head silhouette (rich tiger orange)
    push_path(Color::from_hex(0xD96B1F), FillRule::NonZero, &|pb| {
        pb.move_to(128.0, 36.0)
            .cubic_to(186.0, 36.0, 224.0, 78.0, 218.0, 134.0)
            .cubic_to(214.0, 174.0, 184.0, 216.0, 128.0, 224.0)
            .cubic_to(72.0, 216.0, 42.0, 174.0, 38.0, 134.0)
            .cubic_to(32.0, 78.0, 70.0, 36.0, 128.0, 36.0)
            .close();
    });

    // Cheek fur ruffs & warm highlight mask
    push_path(Color::from_hex(0xF5E6D3), FillRule::NonZero, &|pb| {
        pb.move_to(42.0, 142.0)
            .cubic_to(24.0, 154.0, 38.0, 188.0, 74.0, 198.0)
            .cubic_to(96.0, 204.0, 108.0, 224.0, 128.0, 224.0)
            .cubic_to(148.0, 224.0, 160.0, 204.0, 182.0, 198.0)
            .cubic_to(218.0, 188.0, 232.0, 154.0, 214.0, 142.0)
            .cubic_to(194.0, 164.0, 164.0, 154.0, 128.0, 162.0)
            .cubic_to(92.0, 154.0, 62.0, 164.0, 42.0, 142.0)
            .close();
    });

    // Forehead & cheek black stripes (curved tapered wedges)
    push_path(Color::from_hex(0x16110E), FillRule::NonZero, &|pb| {
        // Center crown M-mark stripes
        pb.move_to(128.0, 38.0)
            .line_to(137.0, 68.0)
            .line_to(128.0, 88.0)
            .line_to(119.0, 68.0)
            .close();
        for &side in &[-1.0f32, 1.0f32] {
            let sx = |x: f32| 128.0 + side * x;
            // Upper brow stripe
            pb.move_to(sx(14.0), 50.0)
                .cubic_to(sx(38.0), 52.0, sx(58.0), 66.0, sx(74.0), 82.0)
                .cubic_to(sx(52.0), 76.0, sx(30.0), 66.0, sx(10.0), 64.0)
                .close();
            // Mid temple stripe
            pb.move_to(sx(82.0), 92.0)
                .cubic_to(sx(64.0), 98.0, sx(52.0), 104.0, sx(42.0), 112.0)
                .cubic_to(sx(60.0), 112.0, sx(74.0), 106.0, sx(88.0), 104.0)
                .close();
            // Lower cheek stripes
            pb.move_to(sx(88.0), 126.0)
                .cubic_to(sx(66.0), 130.0, sx(54.0), 138.0, sx(42.0), 146.0)
                .cubic_to(sx(64.0), 146.0, sx(78.0), 140.0, sx(90.0), 138.0)
                .close();
            pb.move_to(sx(82.0), 156.0)
                .cubic_to(sx(62.0), 162.0, sx(48.0), 172.0, sx(38.0), 180.0)
                .cubic_to(sx(60.0), 178.0, sx(74.0), 172.0, sx(84.0), 166.0)
                .close();
        }
    });

    // Eyes (dark outline, amber iris, black vertical slit pupil, specular highlight)
    push_path(Color::from_hex(0x14100C), FillRule::NonZero, &|pb| {
        for &side in &[-1.0f32, 1.0f32] {
            let sx = |x: f32| 128.0 + side * x;
            pb.move_to(sx(18.0), 118.0)
                .cubic_to(sx(26.0), 100.0, sx(52.0), 100.0, sx(62.0), 114.0)
                .cubic_to(sx(52.0), 130.0, sx(26.0), 130.0, sx(18.0), 118.0)
                .close();
        }
    });
    push_path(Color::from_hex(0xF5B82E), FillRule::NonZero, &|pb| {
        pb.push_ellipse(90.0, 115.0, 14.0, 10.5);
        pb.push_ellipse(166.0, 115.0, 14.0, 10.5);
    });
    push_path(Color::from_hex(0x0B0908), FillRule::NonZero, &|pb| {
        pb.push_ellipse(90.0, 115.0, 4.5, 9.5);
        pb.push_ellipse(166.0, 115.0, 4.5, 9.5);
    });
    push_path(Color::WHITE, FillRule::NonZero, &|pb| {
        pb.push_circle(86.0, 111.0, 2.8);
        pb.push_circle(162.0, 111.0, 2.8);
    });

    // Nose bridge & pink-terracotta nose
    push_path(Color::from_hex(0xC85A4B), FillRule::NonZero, &|pb| {
        pb.move_to(110.0, 146.0)
            .cubic_to(118.0, 140.0, 138.0, 140.0, 146.0, 146.0)
            .cubic_to(146.0, 158.0, 136.0, 166.0, 128.0, 168.0)
            .cubic_to(120.0, 166.0, 110.0, 158.0, 110.0, 146.0)
            .close();
    });

    // Open roaring mouth + fangs + tongue
    push_path(Color::from_hex(0x3B0D11), FillRule::NonZero, &|pb| {
        pb.move_to(96.0, 176.0)
            .cubic_to(112.0, 166.0, 144.0, 166.0, 160.0, 176.0)
            .cubic_to(156.0, 206.0, 144.0, 214.0, 128.0, 214.0)
            .cubic_to(112.0, 214.0, 100.0, 206.0, 96.0, 176.0)
            .close();
    });
    push_path(Color::from_hex(0xE06072), FillRule::NonZero, &|pb| {
        pb.move_to(112.0, 198.0)
            .cubic_to(120.0, 188.0, 136.0, 188.0, 144.0, 198.0)
            .cubic_to(140.0, 212.0, 116.0, 212.0, 112.0, 198.0)
            .close();
    });
    push_path(Color::from_hex(0xFFFDF8), FillRule::NonZero, &|pb| {
        // Upper fangs
        pb.move_to(104.0, 174.0)
            .line_to(112.0, 172.0)
            .line_to(109.0, 192.0)
            .close();
        pb.move_to(152.0, 174.0)
            .line_to(144.0, 172.0)
            .line_to(147.0, 192.0)
            .close();
        // Lower fangs
        pb.move_to(110.0, 206.0)
            .line_to(116.0, 207.0)
            .line_to(114.0, 194.0)
            .close();
        pb.move_to(146.0, 206.0)
            .line_to(140.0, 207.0)
            .line_to(142.0, 194.0)
            .close();
    });

    // Whiskers (thin tapered polygon ribbons)
    push_path(Color::from_hex(0xF8F2EA), FillRule::NonZero, &|pb| {
        for &side in &[-1.0f32, 1.0f32] {
            for &(y0, y1) in &[(160.0f32, 150.0f32), (168.0, 168.0), (176.0, 186.0)] {
                let x0 = 128.0 + side * 34.0;
                let x1 = 128.0 + side * 108.0;
                pb.move_to(x0, y0 - 1.2)
                    .line_to(x1, y1 - 0.3)
                    .line_to(x1, y1 + 0.3)
                    .line_to(x0, y0 + 1.2)
                    .close();
            }
        }
    });

    // Subtle translucent vignette ring using EvenOdd
    push_path(Color::new(0.0, 0.0, 0.0, 0.18), FillRule::EvenOdd, &|pb| {
        pb.push_rect(Rect::new(0.0, 0.0, 256.0, 256.0));
        pb.push_circle(128.0, 128.0, 118.0);
    });

    layers
}

fn build_star_spiral_torture_paths() -> (Path, Path, Path) {
    // 1. 7-point self-intersecting star
    let mut spb = PathBuilder::new();
    let cx = 128.0f32;
    let cy = 128.0f32;
    let r = 104.0f32;
    let mut pts = [Point::ZERO; 7];
    for (i, pt) in pts.iter_mut().enumerate() {
        let ang = -core::f32::consts::FRAC_PI_2
            + (i as f32) * 2.0 * core::f32::consts::PI / 7.0;
        let (s, c) = sincos_f32(ang);
        *pt = Point::new(cx + r * c, cy + r * s);
    }
    spb.move_to(pts[0].x, pts[0].y);
    for k in 1..7 {
        let idx = (k * 3) % 7;
        spb.line_to(pts[idx].x, pts[idx].y);
    }
    spb.close();
    let star = spb.finish();

    // 2. Multi-turn Archimedean ribbon spiral (closed filled curve strip)
    let mut spiral_pb = PathBuilder::new();
    let steps = 96usize;
    let mut outer = Vec::with_capacity(steps + 1);
    let mut inner = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = (i as f32) / (steps as f32);
        let ang = t * 4.0 * core::f32::consts::TAU;
        let rad = 8.0 + t * 92.0;
        let w = 2.0 + t * 4.5;
        let (s, c) = sincos_f32(ang);
        outer.push(Point::new(cx + (rad + w) * c, cy + (rad + w) * s));
        inner.push(Point::new(cx + (rad - w) * c, cy + (rad - w) * s));
    }
    spiral_pb.move_to(outer[0].x, outer[0].y);
    for &p in &outer[1..] {
        spiral_pb.line_to(p.x, p.y);
    }
    for &p in inner.iter().rev() {
        spiral_pb.line_to(p.x, p.y);
    }
    spiral_pb.close();
    let spiral = spiral_pb.finish();

    // 3. Concentric cubic petal rosette
    let mut rose_pb = PathBuilder::new();
    for k in 0..12 {
        let a0 = (k as f32) * (core::f32::consts::TAU / 12.0);
        let a1 = a0 + core::f32::consts::TAU / 12.0;
        let (s0, c0) = sincos_f32(a0);
        let (s1, c1) = sincos_f32(a1);
        rose_pb
            .move_to(cx, cy)
            .cubic_to(
                cx + 75.0 * c0,
                cy + 75.0 * s0,
                cx + 75.0 * c1,
                cy + 75.0 * s1,
                cx,
                cy,
            )
            .close();
    }
    let rosette = rose_pb.finish();

    (star, spiral, rosette)
}

#[test]
fn tiger_and_torture_scenes_png_and_pinned_sha256() {
    let _guard = SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let tiger = build_tiger_scene();
    let mut surf = Surface::new_rgba8(256, 256).unwrap();
    {
        let mut ctx = Context::new(&mut surf);
        ctx.set_dither_mode(DitherMode::BlueNoise);
        ctx.clear(Color::from_hex(0x203040));
        for layer in &tiger {
            ctx.fill_with_paint(&layer.path, layer.fill_rule, layer.color);
        }
    }

    // Verify key tiger anatomical sample regions (orange forehead, amber iris, dark pupil, pink nose)
    let forehead = surf.pixel_at_f32(128, 95).unwrap();
    assert!(forehead.r > 0.5 && forehead.g > 0.1 && forehead.b < 0.05);
    let pupil = surf.pixel_at_f32(90, 115).unwrap();
    assert!(pupil.r < 0.02 && pupil.g < 0.02 && pupil.b < 0.02);
    let nose = surf.pixel_at_f32(128, 152).unwrap();
    assert!(nose.r > 0.45 && nose.g < 0.15);

    let tiger_hash = common::sha256_hex(surf.data());
    assert_eq!(
        tiger_hash,
        "baca22886eb9f6cf2a01ffe01e3686b1d02637aaf6fc35b0947655c7aa1f7ea6",
        "tiger framebuffer SHA-256 drifted: {tiger_hash}"
    );

    #[cfg(feature = "codec-png")]
    {
        let png_bytes = surf.encode_png().unwrap();
        assert_eq!(&png_bytes[..8], b"\x89PNG\r\n\x1a\n");
        let decoded = Surface::from_png_bytes(&png_bytes).unwrap();
        assert_eq!(decoded.width(), surf.width());
        assert_eq!(decoded.height(), surf.height());
        assert_eq!(decoded.data(), surf.data());
    }

    // Star / spiral / cubic rosette torture scene
    let (star, spiral, rosette) = build_star_spiral_torture_paths();
    let mut torture = Surface::new_rgba8(256, 256).unwrap();
    {
        let mut ctx = Context::new(&mut torture);
        ctx.set_dither_mode(DitherMode::BlueNoise);
        ctx.clear(Color::from_hex(0x0E1118));
        ctx.fill_with_paint(&star, FillRule::EvenOdd, Color::from_hex(0xFFB703));
        ctx.fill_with_paint(&spiral, FillRule::NonZero, Color::new(0.12, 0.72, 0.95, 0.75));
        ctx.fill_with_paint(&rosette, FillRule::NonZero, Color::new(0.95, 0.25, 0.45, 0.45));
        ctx.set_paint(Color::WHITE);
        ctx.stroke(&star, &StrokeStyle::new(1.5));
    }

    let torture_hash = common::sha256_hex(torture.data());
    assert_eq!(
        torture_hash,
        "e184e55c38c1da8a5ee73fbd285d77eb28c7a340294d32002c5503d10139b129",
        "torture framebuffer SHA-256 drifted: {torture_hash}"
    );

    #[cfg(feature = "codec-png")]
    {
        let png_bytes = torture.encode_png().unwrap();
        let decoded = Surface::from_png_bytes(&png_bytes).unwrap();
        assert_eq!(decoded.data(), torture.data());
    }
}

#[test]
fn cairo_stateful_matches_path_builder_bit_exact() {
    let _guard = SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let mut s1 = Surface::new_rgba8(160, 160).unwrap();
    {
        let mut ctx = Context::new(&mut s1);
        ctx.set_dither_mode(DitherMode::InterleavedGradientNoise);
        ctx.clear(Color::from_hex(0x182230));
        ctx.save();
        ctx.translate(16.0, 14.0);
        ctx.rotate(0.2);
        ctx.scale(0.9, 1.1);

        ctx.move_to(12.0, 18.0);
        ctx.rel_line_to(50.0, 6.0);
        ctx.curve_to(80.0, 30.0, 75.0, 80.0, 40.0, 90.0);
        ctx.rel_line_to(-28.0, -18.0);
        ctx.close_path();

        ctx.set_paint(Color::from_hex(0xE63946));
        ctx.set_fill_rule(FillRule::NonZero);
        ctx.fill_preserve();

        ctx.set_paint(Color::new(1.0, 1.0, 1.0, 0.35));
        ctx.fill_current();

        ctx.arc(92.0, 72.0, 26.0, 0.25, 4.75);
        ctx.arc_negative(92.0, 72.0, 12.0, 4.75, 0.25);
        ctx.close_path();
        ctx.set_paint(Color::from_hex(0x2A9D8F));
        ctx.set_fill_rule(FillRule::EvenOdd);
        ctx.fill_current();

        ctx.rectangle(10.0, 102.0, 90.0, 16.0);
        ctx.set_paint(Color::from_hex(0xF4A261));
        ctx.set_antialias(AntialiasMode::None);
        ctx.fill_current();
        ctx.restore();
    }

    let mut s2 = Surface::new_rgba8(160, 160).unwrap();
    {
        let mut ctx = Context::new(&mut s2);
        ctx.set_dither_mode(DitherMode::InterleavedGradientNoise);
        ctx.clear(Color::from_hex(0x182230));
        ctx.set_transform(
            Affine2D::translate(16.0, 14.0)
                .pre_concat(Affine2D::rotate(0.2))
                .pre_concat(Affine2D::scale(0.9, 1.1)),
        );

        let mut pb1 = PathBuilder::new();
        pb1.move_to(12.0, 18.0)
            .rel_line_to(50.0, 6.0)
            .cubic_to(80.0, 30.0, 75.0, 80.0, 40.0, 90.0)
            .rel_line_to(-28.0, -18.0)
            .close();
        let p1 = pb1.finish();
        ctx.fill_with_paint(&p1, FillRule::NonZero, Color::from_hex(0xE63946));
        ctx.fill_with_paint(&p1, FillRule::NonZero, Color::new(1.0, 1.0, 1.0, 0.35));

        let mut pb2 = PathBuilder::new();
        pb2.arc_center(92.0, 72.0, 26.0, 0.25, 4.75, false)
            .arc_center(92.0, 72.0, 12.0, 4.75, 0.25, true)
            .close();
        let p2 = pb2.finish();
        ctx.fill_with_paint(&p2, FillRule::EvenOdd, Color::from_hex(0x2A9D8F));

        let mut pb3 = PathBuilder::new();
        pb3.push_rect(Rect::from_xywh(10.0, 102.0, 90.0, 16.0));
        let p3 = pb3.finish();
        ctx.set_antialias(AntialiasMode::None);
        ctx.fill_with_paint(&p3, FillRule::NonZero, Color::from_hex(0xF4A261));
    }

    assert_eq!(s1.data(), s2.data());
}

#[test]
fn zero_steady_state_allocations_full_context_pipeline() {
    let _guard = SERIAL_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let tiger = build_tiger_scene();
    let mut surf = Surface::new_rgba8(256, 256).unwrap();
    let mut ctx = Context::new(&mut surf);
    ctx.set_dither_mode(DitherMode::BlueNoise);

    let render_frame = |ctx: &mut Context<'_>| {
        ctx.clear(Color::from_hex(0x203040));
        for layer in &tiger {
            ctx.fill_with_paint(&layer.path, layer.fill_rule, layer.color);
        }
        // Also exercise Cairo stateful fill_preserve + fill_current inside the 0-alloc loop
        ctx.move_to(12.0, 12.0);
        ctx.rel_line_to(24.0, 0.0);
        ctx.rel_line_to(-12.0, 20.0);
        ctx.close_path();
        ctx.set_paint(Color::from_hex(0x00CC88));
        ctx.fill_preserve();
        ctx.set_paint(Color::new(1.0, 1.0, 1.0, 0.25));
        ctx.fill_current();
        ctx.flush();
    };

    // 2 warm-up frames
    render_frame(&mut ctx);
    render_frame(&mut ctx);

    // 20 steady-state frames must execute with strictly 0 heap allocations or deallocations
    assert_zero_allocs(|| {
        for _ in 0..20 {
            render_frame(&mut ctx);
        }
    });
}
