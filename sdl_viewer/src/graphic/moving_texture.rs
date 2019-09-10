use crate::graphic::uniform::GlUniform;
use crate::graphic::GlProgram;
use crate::opengl;
use crate::opengl::types::{GLint, GLuint};
use arrayvec::ArrayVec;
use cgmath::Vector2;
use image::{GenericImage, GenericImageView, ImageBuffer, LumaA, Pixel, Rgba};
use num_integer::Integer;
use std::ffi::c_void;
use std::marker::PhantomData;
use std::rc::Rc;

/// A square texture that is incrementally loaded. Incrementality is achieved
/// by using wraparound indexing and overwriting the parts that moved off the
/// texture with the newly loaded parts.
/// See https://developer.nvidia.com/gpugems/GPUGems2/gpugems2_chapter02.html,
/// section 2.4 for an illustration.
/// Also publishes an "xyz_texture_offset" uniform whose unit is pixels (not UV
/// coordinates).
pub struct GlTexture<P: Pixel> {
    id: GLuint,
    gl: Rc<opengl::Gl>,
    size: i32,
    u_texture_offset: GlUniform<Vector2<i32>>,
    pixel_type: PhantomData<P>,
}

/// Specifies the offsets and sizes in the destination, as well as the image to
/// be pasted there.
#[derive(Debug)]
struct UpdateRegion<P: Pixel> {
    xoff: i32,
    yoff: i32,
    width: i32,
    height: i32,
    pixels: ImageBuffer<P, Vec<P::Subpixel>>,
}

pub trait TextureFormat: Pixel {
    const INTERNALFORMAT: GLint;
    const FORMAT: GLuint;
    const DTYPE: GLuint;
}

impl TextureFormat for LumaA<f32> {
    const INTERNALFORMAT: GLint = opengl::RG32F as GLint;
    const FORMAT: GLuint = opengl::RG;
    const DTYPE: GLuint = opengl::FLOAT;
}

impl TextureFormat for Rgba<u8> {
    const INTERNALFORMAT: GLint = opengl::RGBA8 as GLint;
    const FORMAT: GLuint = opengl::RGBA;
    const DTYPE: GLuint = opengl::UNSIGNED_BYTE;
}

impl<P> GlTexture<P>
where
    P: TextureFormat,
    P: Pixel + 'static + std::fmt::Debug,
    P::Subpixel: 'static + std::fmt::Debug,
{
    pub fn new(
        program: &GlProgram,
        gl: Rc<opengl::Gl>,
        name: &str,
        size: i32,
        pixels: ImageBuffer<P, Vec<P::Subpixel>>,
    ) -> Self {
        let mut id = 0;
        unsafe {
            gl.UseProgram(program.id);
            let loc =
                gl.GetUniformLocation(program.id, (name.to_string() + "\0").as_ptr() as *const i8);
            gl.Uniform1i(loc, 0);

            gl.GenTextures(1, &mut id);
            gl.BindTexture(opengl::TEXTURE_2D, id);
            gl.TexParameteri(
                opengl::TEXTURE_2D,
                opengl::TEXTURE_WRAP_S,
                opengl::REPEAT as i32,
            );
            gl.TexParameteri(
                opengl::TEXTURE_2D,
                opengl::TEXTURE_WRAP_T,
                opengl::REPEAT as i32,
            );
            let color: [f32; 4] = [-1000.0, -1000.0, -1000.0, 1.0];
            gl.TexParameterfv(
                opengl::TEXTURE_2D,
                opengl::TEXTURE_BORDER_COLOR,
                color.as_ptr(),
            );
            gl.TexParameteri(
                opengl::TEXTURE_2D,
                opengl::TEXTURE_MIN_FILTER,
                opengl::NEAREST as i32,
            );
            gl.TexParameteri(
                opengl::TEXTURE_2D,
                opengl::TEXTURE_MAG_FILTER,
                opengl::NEAREST as i32,
            );
            gl.TexImage2D(
                opengl::TEXTURE_2D,
                0,
                P::INTERNALFORMAT,
                size,
                size,
                0,
                P::FORMAT,
                P::DTYPE,
                pixels.into_raw().as_ptr() as *const c_void,
            );
        }

        let u_texture_offset = GlUniform::new(
            &program,
            &(name.to_string() + "_texture_offset"),
            Vector2::new(0, 0),
        );

        GlTexture {
            id,
            gl,
            size,
            u_texture_offset,
            pixel_type: PhantomData,
        }
    }

    fn incremental_update_regions(
        &mut self,
        delta_x: i32,
        delta_y: i32,
        mut vert_strip: ImageBuffer<P, Vec<P::Subpixel>>,
        mut hori_strip: ImageBuffer<P, Vec<P::Subpixel>>,
    ) -> ArrayVec<[UpdateRegion<P>; 4]> {
        // width/height are always positive, but if the delta is negative, the
        // start of the new region is actually offset + delta, not offset
        let width = delta_x.abs();
        let height = delta_y.abs();
        let xoff = if delta_x > 0 {
            self.u_texture_offset.value.x
        } else {
            (self.u_texture_offset.value.x + delta_x).mod_floor(&self.size)
        };
        let yoff = if delta_y > 0 {
            self.u_texture_offset.value.y
        } else {
            (self.u_texture_offset.value.y + delta_y).mod_floor(&self.size)
        };

        // u_texture_offset.value is always in [0; self.size)
        let x = (self.u_texture_offset.value.x + delta_x).mod_floor(&self.size);
        let y = (self.u_texture_offset.value.y + delta_y).mod_floor(&self.size);
        self.u_texture_offset.value = Vector2::new(x, y);

        // There is a maximum of 4 update regions
        let mut regions: ArrayVec<[UpdateRegion<P>; 4]> = ArrayVec::new();

        if width + xoff > self.size {
            // Wraparound the right texture edge – split into two vertical strips.
            // Illustration:

            // <- width ->
            // +---+-+
            // |   | |
            // |   | |
            // | 1 |2|  vert_strip
            // |   | |
            // |   | |
            // +---+-+

            // <- self.size ->
            // +-+-------+---+
            // | |       |   |
            // | |       |   |
            // |2|       | 1 |  texture
            // | |       |   |
            // | |       |   |
            // +-+-------+---+
            //           ^
            //           |
            //           xoff

            let width_1 = self.size - xoff;
            let region_1 = UpdateRegion {
                xoff,
                yoff: 0,
                width: width_1,
                height: self.size,
                pixels: rotate_y(
                    vert_strip.sub_image(0, 0, width_1 as u32, self.size as u32),
                    self.u_texture_offset.value.y as u32,
                ),
            };
            let width_2 = width - width_1;
            let region_2 = UpdateRegion {
                xoff: 0,
                yoff: 0,
                width: width_2,
                height: self.size,
                pixels: rotate_y(
                    vert_strip.sub_image(width_1 as u32, 0, width_2 as u32, self.size as u32),
                    self.u_texture_offset.value.y as u32,
                ),
            };
            regions.push(region_1);
            regions.push(region_2);
        } else if width != 0 {
            let region = UpdateRegion {
                xoff,
                yoff: 0,
                width,
                height: self.size,
                pixels: rotate_y(vert_strip, self.u_texture_offset.value.y as u32),
            };
            regions.push(region);
        }

        if height + yoff > self.size {
            // Wraparound the upper texture edge – split into two horizontal strips.
            // Imagine the same illustration as above, but rotated counterclockwise by
            // 90 degrees and with xoff => yoff and width => height.
            let height_1 = self.size - yoff;
            let region_1 = UpdateRegion {
                xoff: 0,
                yoff,
                width: self.size,
                height: height_1,
                pixels: rotate_x(
                    hori_strip.sub_image(0, 0, self.size as u32, height_1 as u32),
                    self.u_texture_offset.value.x as u32,
                ),
            };
            let height_2 = height - height_1;
            let region_2 = UpdateRegion {
                xoff: 0,
                yoff: 0,
                width: self.size,
                height: height_2,
                pixels: rotate_x(
                    hori_strip.sub_image(0, height_1 as u32, self.size as u32, height_2 as u32),
                    self.u_texture_offset.value.x as u32,
                ),
            };
            regions.push(region_1);
            regions.push(region_2);
        } else if height != 0 {
            let region = UpdateRegion {
                xoff: 0,
                yoff,
                width: self.size,
                height,
                pixels: rotate_x(hori_strip, self.u_texture_offset.value.x as u32),
            };
            regions.push(region);
        }

        regions
    }

    pub fn incremental_update(
        &mut self,
        delta_x: i32,
        delta_y: i32,
        vert_strip: ImageBuffer<P, Vec<P::Subpixel>>,
        hori_strip: ImageBuffer<P, Vec<P::Subpixel>>,
    ) {
        if delta_x == 0 && delta_y == 0 {
            return;
        }
        let regions = self.incremental_update_regions(delta_x, delta_y, vert_strip, hori_strip);

        for r in regions {
            let buf =
                ImageBuffer::from_raw(r.width as u32, r.height as u32, r.pixels.clone().into_raw())
                    .unwrap();

            unsafe {
                self.gl.BindTexture(opengl::TEXTURE_2D, self.id);

                self.gl.TexSubImage2D(
                    opengl::TEXTURE_2D,
                    0,
                    r.xoff,
                    r.yoff,
                    r.width,
                    r.height,
                    P::FORMAT,
                    P::DTYPE,
                    r.pixels.into_raw().as_ptr() as *const c_void,
                );
            }
        }
    }

    pub fn submit(&mut self) {
        unsafe {
            self.u_texture_offset.submit();

            self.gl.ActiveTexture(opengl::TEXTURE0);
            self.gl.BindTexture(opengl::TEXTURE_2D, self.id);
        }
    }
}

fn rotate_x<I: GenericImageView>(
    img: I,
    amount: u32,
) -> ImageBuffer<I::Pixel, Vec<<I::Pixel as Pixel>::Subpixel>>
where
    I::Pixel: 'static + std::fmt::Debug,
    <I::Pixel as Pixel>::Subpixel: 'static,
{
    let mut result = ImageBuffer::new(img.width(), img.height());
    result.copy_from(
        &img.view(img.width() - amount, 0, amount, img.height()),
        0,
        0,
    );
    result.copy_from(
        &img.view(0, 0, img.width() - amount, img.height()),
        amount,
        0,
    );
    result
}

// moves the interval [0, height-amount] to [amount, height]
// and [height-amount, height] to [0, amount]
fn rotate_y<I: GenericImageView>(
    img: I,
    amount: u32,
) -> ImageBuffer<I::Pixel, Vec<<I::Pixel as Pixel>::Subpixel>>
where
    I::Pixel: 'static,
    <I::Pixel as Pixel>::Subpixel: 'static,
{
    let mut result = ImageBuffer::new(img.width(), img.height());
    result.copy_from(
        &img.view(0, img.height() - amount, img.width(), amount),
        0,
        0,
    );
    result.copy_from(
        &img.view(0, 0, img.width(), img.height() - amount),
        0,
        amount,
    );
    result
}
