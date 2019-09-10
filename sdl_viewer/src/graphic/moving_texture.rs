use crate::graphic::uniform::GlUniform;
use crate::graphic::GlProgram;
use crate::opengl;
use crate::opengl::types::{GLint, GLuint};
use arrayvec::ArrayVec;
use cgmath::Vector2;
use image::{GenericImageView, ImageBuffer, LumaA, Pixel, Rgba, SubImage};
use num_integer::Integer;
use std::convert::{TryFrom, TryInto};
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
/// I'm told that a more efficient alternative would be to use a PBO (pixel
/// buffer object), so a future version could use that instead.
pub struct GlTexture<P: Pixel> {
    id: GLuint,
    gl: Rc<opengl::Gl>,
    size: i32,
    u_texture_offset: GlUniform<Vector2<i32>>,
    pixel_type: PhantomData<P>,
}

/// Specifies the offsets in the destination, as well as the image to be pasted
/// there.
struct UpdateRegion<'a, P: Pixel + 'static> {
    x: i32,
    y: i32,
    pixels: SubImage<&'a ImageBuffer<P, Vec<P::Subpixel>>>,
}

impl<'a, P: Pixel + 'static> UpdateRegion<'a, P> {
    // Splits an image with offset into update regions
    //
    //     +---+-+
    //     | 3 |4|
    //     +---+-+
    //     |   | |
    //     | 1 |2|
    //     |   | |
    //     +---+-+
    //        |
    //        V
    // +-+-------+---+
    // | |       |   |
    // |2|       | 1 |
    // | |       |   |
    // +-+       +---+
    // |4|       | 3 |
    // +-+-------+---+
    pub fn new_regions(
        xoff: i32,
        yoff: i32,
        size: i32,
        pixels: &'a ImageBuffer<P, Vec<P::Subpixel>>,
    ) -> [UpdateRegion<'a, P>; 4] {
        // Do subtractions in signed numbers to avoid overflow
        let width: i32 = pixels.width().try_into().unwrap();
        let height: i32 = pixels.height().try_into().unwrap();
        let width_1_3 = width.min(size - xoff);
        let width_2_4 = width - width_1_3;
        let height_3_4 = height.min(size - yoff);
        let height_1_2 = height - height_3_4;
        // The image crate likes unsigned ints
        let width_1_3 = u32::try_from(width_1_3).unwrap();
        let width_2_4 = u32::try_from(width_2_4).unwrap();
        let height_3_4 = u32::try_from(height_3_4).unwrap();
        let height_1_2 = u32::try_from(height_1_2).unwrap();
        [
            UpdateRegion {
                x: xoff,
                y: yoff,
                pixels: pixels.view(0, 0, width_1_3, height_1_2),
            },
            UpdateRegion {
                x: 0,
                y: yoff,
                pixels: pixels.view(width_1_3, 0, width_2_4, height_1_2),
            },
            UpdateRegion {
                x: xoff,
                y: 0,
                pixels: pixels.view(0, height_1_2, width_1_3, height_3_4),
            },
            UpdateRegion {
                x: 0,
                y: 0,
                pixels: pixels.view(width_1_3, height_1_2, width_2_4, height_3_4),
            },
        ]
    }
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

            // Enable wraparound addressing
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

    // Let's say the texture window has moved left and up. The new pixels that
    // are now inside the window are passed as a vertical and a horizontal
    // strip. Here they are in the texture window:
    // +-----+------------+
    // |#####|############| <- hori_strip
    // +-----|------------+
    // |#####|            |
    // |#####|            |
    // |#####|            |
    // +-----+------------+
    //    ^
    //    |
    //   vert_strip
    //
    // You can see that they are redundant where they overlap. This is not
    // handled to keep the code from becoming more complex. Also, either may
    // be empty, which is the case for purely horizontal/vertical shifts.

    // These strips have to be inserted correctly into the OpenGL texture,
    // which starts not at 0 but at texture_offset because of wrapping indexing.
    // Updating that texture from such a strip can typically not be done in one
    // step, but requires breaking it into up to four subregions. This is
    // because OpenGL textures can be read with wrapping indexing, but not
    // written with wrapping indexing, so we need to do it ourselves.
    // Let's look at the vertical strip:

    // <- width ->
    // +---+-+
    // | 3 |4|
    // +---+-+
    // |   | |  vert_strip
    // | 1 |2|
    // |   | |
    // +---+-+

    // Wraparound at the right texture edge causes the split into regions 1/3
    // vs 2/4, and wraparound at the upper texture edge causes the split into
    // regions 1/2 vs 3/4.
    // Here's the texture with the vert_strip placed correctly:
    //
    // <- self.size ->
    // +-+-------+---+
    // | |       |   |
    // |2|       | 1 |
    // | |       |   |
    // +-+       +---+ <- texture_offset.y
    // |4|       | 3 |
    // +-+-------+---+
    //           ^
    //           |
    //           xoff
    //
    // xoff is not equal to texture_offset.x if the texture moved right.

    // The way this is handled is by creating regions as if they always need
    // splitting, and skipping empty regions that result from that.
    pub fn incremental_update(
        &mut self,
        delta_x: i32,
        delta_y: i32,
        vert_strip: ImageBuffer<P, Vec<P::Subpixel>>,
        hori_strip: ImageBuffer<P, Vec<P::Subpixel>>,
    ) {
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

        let vert_regions = UpdateRegion::new_regions(xoff, y, self.size, &vert_strip);
        let hori_regions = UpdateRegion::new_regions(x, yoff, self.size, &hori_strip);
        let regions = ArrayVec::from(vert_regions)
            .into_iter()
            .chain(ArrayVec::from(hori_regions));

        unsafe {
            self.gl.BindTexture(opengl::TEXTURE_2D, self.id);
            for mut r in regions {
                let width = i32::try_from(r.pixels.width()).unwrap();
                let height = i32::try_from(r.pixels.height()).unwrap();
                let image = r.pixels.to_image();
                self.gl.TexSubImage2D(
                    opengl::TEXTURE_2D,
                    0,
                    r.x,
                    r.y,
                    width,
                    height,
                    P::FORMAT,
                    P::DTYPE,
                    image.into_raw().as_ptr() as *const c_void,
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
