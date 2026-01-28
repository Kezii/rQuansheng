use image::GenericImageView;
use std::{
    error::Error,
    io::{self, BufWriter, Write},
    path::PathBuf,
};

const WIDTH: usize = 128;
const HEIGHT: usize = 64;
const PAGES: usize = HEIGHT / 8;
const FB_SIZE: usize = WIDTH * PAGES;

fn main() -> Result<(), Box<dyn Error>> {
    // Opzioni da CLI rimosse: usiamo sempre i valori "di default".
    let input: PathBuf = PathBuf::from("../splash.png");
    let threshold: u8 = 128;
    let invert = false;

    let img = image::open(&input)
        .map_err(|e| format!("impossibile aprire l'immagine {:?}: {e}", input))?;

    let (src_w, src_h) = img.dimensions();
    let mut bin = binarize_to_gray_1bit(&img, threshold, invert);

    if src_w != WIDTH as u32 || src_h != HEIGHT as u32 {
        // Ridimensiona *dopo* binarizzazione per preservare davvero 1-bit (0/255) con nearest.
        bin = image::imageops::resize(
            &bin,
            WIDTH as u32,
            HEIGHT as u32,
            image::imageops::FilterType::Nearest,
        );
    }

    let fb = gray_1bit_to_st7565_page_buffer(&bin);

    let mut writer = BufWriter::new(io::stdout().lock());

    write_rust_array(&mut writer, &fb)?;

    Ok(())
}

fn binarize_to_gray_1bit(
    img: &image::DynamicImage,
    threshold: u8,
    invert: bool,
) -> image::GrayImage {
    let rgba = img.to_rgba8();
    let (w, h) = rgba.dimensions();
    let mut out = image::GrayImage::new(w, h);

    // Per “rispettare” 1-bit, produciamo solo 0/255.
    for y in 0..h {
        for x in 0..w {
            let p = rgba.get_pixel(x, y).0;
            let (r, g, b, a) = (p[0], p[1], p[2], p[3]);

            // Se trasparente => OFF (bianco)
            let mut on = if a < 128 {
                false
            } else {
                // luminanza ~ Rec.709 in interi: (54*r + 183*g + 19*b) / 256
                let lum = ((r as u16 * 54) + (g as u16 * 183) + (b as u16 * 19)) >> 8;
                (lum as u8) < threshold
            };

            if invert {
                on = !on;
            }

            out.put_pixel(x, y, image::Luma([if on { 0u8 } else { 255u8 }]));
        }
    }

    out
}

fn gray_1bit_to_st7565_page_buffer(gray: &image::GrayImage) -> Vec<u8> {
    let mut fb = vec![0u8; FB_SIZE];

    debug_assert_eq!(gray.width(), WIDTH as u32);
    debug_assert_eq!(gray.height(), HEIGHT as u32);

    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let v = gray.get_pixel(x as u32, y as u32).0[0];
            let on = v < 128;
            if on {
                let page = y / 8;
                let bit = (y % 8) as u8;
                fb[page * WIDTH + x] |= 1u8 << bit;
            }
        }
    }

    fb
}

fn write_rust_array(mut w: impl Write, fb: &[u8]) -> io::Result<()> {
    writeln!(w, "pub const SPLASH_FB: [u8; {FB_SIZE}] = [")?;
    for (i, b) in fb.iter().enumerate() {
        write!(w, "0x{:02x},", b)?;
        if (i + 1) % 16 == 0 {
            writeln!(w)?;
        } else {
            write!(w, " ")?;
        }
    }
    writeln!(w, "];")?;
    Ok(())
}
