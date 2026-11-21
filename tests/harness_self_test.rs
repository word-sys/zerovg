mod common;

use common::{
    Sha256, Xoshiro256PlusPlus, ciede2000_lab, compute_max_and_mean_delta_e2000, compute_ssim,
    sha256_hex,
};

#[test]
fn sha256_nist_vectors() {
    assert_eq!(
        sha256_hex(b""),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    assert_eq!(
        sha256_hex(b"abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
    assert_eq!(
        sha256_hex(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
    );

    // Streaming 1,000,000 'a' bytes vector
    let mut h = Sha256::new();
    let chunk = [b'a'; 1000];
    for _ in 0..1000 {
        h.update(&chunk);
    }
    let digest = h.finalize();
    let hex: String = digest.iter().map(|b| format!("{b:02x}")).collect();
    assert_eq!(
        hex,
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"
    );
}

#[test]
fn prng_deterministic_and_range() {
    let mut r1 = Xoshiro256PlusPlus::seed_from_u64(42);
    let mut r2 = Xoshiro256PlusPlus::seed_from_u64(42);
    for _ in 0..1000 {
        assert_eq!(r1.next_u64(), r2.next_u64());
    }
    for _ in 0..1000 {
        let f = r1.next_f32_unit();
        assert!((0.0..1.0).contains(&f));
    }
}

#[test]
fn ssim_and_ciede2000_reference_checks() {
    // Sharma, Wu, Dalal (2005) Table 1 pair #1: expected 2.0425
    let de1 = ciede2000_lab((50.0, 2.6772, -79.7751), (50.0, 0.0, -82.7485));
    assert!((de1 - 2.0425).abs() < 1e-4, "de1={de1}");

    // Sharma Table 1 pair #2: expected 2.8615
    let de2 = ciede2000_lab((50.0, 3.1571, -77.2803), (50.0, 0.0, -82.7485));
    assert!((de2 - 2.8615).abs() < 1e-4, "de2={de2}");

    // Identical images: SSIM == 1.0, Delta-E == 0.0
    let w = 16u32;
    let h = 16u32;
    let mut img_a = vec![0u8; (w * h * 4) as usize];
    for y in 0..h {
        for x in 0..w {
            let idx = ((y * w + x) * 4) as usize;
            img_a[idx] = (x * 16) as u8;
            img_a[idx + 1] = (y * 16) as u8;
            img_a[idx + 2] = 128;
            img_a[idx + 3] = 255;
        }
    }
    assert!((compute_ssim(&img_a, &img_a, w, h) - 1.0).abs() < 1e-12);
    assert_eq!(compute_max_and_mean_delta_e2000(&img_a, &img_a, w, h), (0.0, 0.0));

    // Spatial perturbation drops SSIM and raises Delta-E
    let mut img_b = img_a.clone();
    for (i, px) in img_b.chunks_exact_mut(4).enumerate() {
        let delta: i16 = if (i % 2) == 0 { 60 } else { -60 };
        px[0] = (px[0] as i16 + delta).clamp(0, 255) as u8;
        px[1] = (px[1] as i16 - delta).clamp(0, 255) as u8;
    }
    let ssim = compute_ssim(&img_a, &img_b, w, h);
    let (max_de, mean_de) = compute_max_and_mean_delta_e2000(&img_a, &img_b, w, h);
    assert!(ssim < 0.85, "ssim={ssim}");
    assert!(max_de > 15.0 && mean_de > 10.0, "max={max_de}, mean={mean_de}");

    // Alpha-only difference is detected by Delta-E
    let mut img_c = img_a.clone();
    img_c[3] = 128;
    let (max_de_a, _) = compute_max_and_mean_delta_e2000(&img_a, &img_c, w, h);
    assert!(max_de_a > 10.0, "max_de_a={max_de_a}");
}
