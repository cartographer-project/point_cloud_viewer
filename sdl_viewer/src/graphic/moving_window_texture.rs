use crate::graphic::uniform::GlUniform;
use crate::graphic::GlProgram;
use crate::opengl;
use crate::opengl::types::{GLint, GLuint};
use cgmath::Vector2;
use image::{GenericImageView, ImageBuffer, LumaA, Pixel, Rgba, SubImage};
use num_integer::Integer;
use std::convert::{TryFrom, TryInto};
use std::ffi::{c_void, CString};
use std::marker::PhantomData;
use std::rc::Rc;

/// A square texture that functions like a moving window into a larger texture.
/// Incrementality is achieved by using wraparound indexing and overwriting the
/// parts that moved off the texture with the newly loaded parts.
/// See `https://developer.nvidia.com/gpugems/GPUGems2/gpugems2_chapter02.html`,
/// section 2.4 for an illustration, but note that this page does not describe
/// the present implementation, which is simpler.
/// Also publishes an `<texture_name>_texture_offset` uniform whose unit is
/// pixels (not UV coordinates).
///
/// I'm told that a more efficient alternative would be to use a PBO (pixel
/// buffer object), so a future version could use that instead.
pub struct GlMovingWindowTexture<P: Pixel> {
    id: GLuint,
    gl: Rc<opengl::Gl>,
    size: u32,
    u_texture_offset: GlUniform<Vector2<i32>>,
    pixel_type: PhantomData<P>,
}

/// Specifies the offsets in the destination, as well as the image to be pasted
/// there.
struct UpdateRegion<'a, P: Pixel + 'static> {
    x: u32,
    y: u32,
    pixels: SubImage<&'a ImageBuffer<P, Vec<P::Subpixel>>>,
}

impl<'a, P: Pixel + 'static> UpdateRegion<'a, P> {
    /// Splits an image with offset into update regions. See
    /// [`incremental_update()`] for a detailed explanation.
    /// This works not only for full-width/full-height strips, but
    /// here are vertical (left) and horizontal (right) strips
    /// visualized:
    ///
    /// ```text              
    ///     +---+-+          
    ///     | 3 |4|              
    ///     +---+-+              +---+---------+
    ///     |   | |              | 3 |    4    |
    ///     | 1 |2|              +---+---------+
    ///     |   | |              | 1 |    2    |
    ///     +---+-+              +---+---------+
    ///        |                        |
    ///        V                        V
    /// +-+-------+---+          +---------+---+
    /// | |       |   |          |    2    | 1 |
    /// |2|       | 1 |          +---------+---+
    /// | |       |   |          |             |
    /// +-+       +---+          +---------+---+
    /// |4|       | 3 |          |    4    | 3 |
    /// +-+-------+---+          +---------+---+
    /// ```                      
    ///
    /// x is right and y is up, so the origin of the image is the lower left
    /// corner of region 1.

    pub fn new_regions(
        xoff: u32,
        yoff: u32,
        size: u32,
        pixels: &'a ImageBuffer<P, Vec<P::Subpixel>>,
    ) -> [UpdateRegion<'a, P>; 4] {
        // Check invariant
        debug_assert!(xoff < size && yoff <= size);
        // These variables are named after the regions they apply to. For
        // instance, regions 1 and 3 share the same width.
        let width_1_3 = pixels.width().min(size - xoff);
        let width_2_4 = pixels.width() - width_1_3;
        let height_1_2 = pixels.height().min(size - yoff);
        let height_3_4 = pixels.height() - height_1_2;
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

impl<P> GlMovingWindowTexture<P>
where
    P: TextureFormat + 'static,
    P::Subpixel: 'static + std::fmt::Debug,
{
    pub fn new(
        program: &GlProgram,
        gl: Rc<opengl::Gl>,
        name: &str,
        size: u32,
        pixels: ImageBuffer<P, Vec<P::Subpixel>>,
    ) -> Self {
        let mut id = 0;
        let name_c = CString::new(name).unwrap();
        unsafe {
            gl.UseProgram(program.id);
            let loc = gl.GetUniformLocation(program.id, name_c.as_ptr());
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

            // Intended for exact pixel fetching, no interpolation needed
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
            // For an explanation of the different parameters, see
            // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml
            gl.TexImage2D(
                opengl::TEXTURE_2D,
                0, // level
                P::INTERNALFORMAT,
                size as i32, // width
                size as i32, // height
                0,           // border
                P::FORMAT,
                P::DTYPE,
                pixels.into_raw().as_ptr() as *const c_void,
            );
        }

        let u_texture_offset = GlUniform::new(
            &program,
            &format!("{}_texture_offset", name),
            Vector2::new(0, 0),
        );

        GlMovingWindowTexture {
            id,
            gl,
            size,
            u_texture_offset,
            pixel_type: PhantomData,
        }
    }

    /// Let's say the texture window has moved left and up. The new pixels that
    /// are now inside the window are passed as a vertical and a horizontal
    /// strip. Here they are in the texture window:
    /// ```text
    /// +-----+------------+
    /// |#####|############| <- hori_strip
    /// +-----|------------+
    /// |#####|            |
    /// |#####|            |
    /// |#####|            |
    /// +-----+------------+
    ///    ^
    ///    |
    ///   vert_strip
    /// ```
    ///
    /// You can see that they are redundant where they overlap. This is not
    /// handled to keep the code from becoming more complex. Also, either may
    /// be empty, which is the case for purely horizontal/vertical shifts.
    ///
    /// These strips have to be inserted correctly into the OpenGL texture,
    /// which starts not at 0 but at `texture_offset` because of wrapping indexing.
    /// Updating that texture from such a strip can typically not be done in one
    /// step, but requires breaking it into up to four subregions. This is
    /// because OpenGL textures can be read with wrapping indexing, but not
    /// written with wrapping indexing, so we need to do it ourselves.
    /// Let's look at the vertical strip:
    ///
    /// ```text
    /// <- width ->
    /// +---+-+
    /// | 3 |4|
    /// +---+-+
    /// |   | |  vert_strip
    /// | 1 |2|
    /// |   | |
    /// +---+-+
    /// ```
    ///
    /// Wraparound at the right texture edge causes the split into regions 1/3
    /// vs 2/4, and wraparound at the upper texture edge causes the split into
    /// regions 1/2 vs 3/4.
    /// Here's the texture with the vert_strip placed correctly:
    ///
    /// ```text
    /// <- self.size ->
    /// +-+-------+---+
    /// | |       |   |
    /// |2|       | 1 |
    /// | |       |   |
    /// +-+       +---+ <- texture_offset.y
    /// |4|       | 3 |
    /// +-+-------+---+
    ///           ^
    ///           |
    ///           xoff
    /// ```
    ///
    /// `xoff` is not equal to `texture_offset.x` if the texture moved right.
    ///
    /// The way this is handled is by creating regions as if they always need
    /// splitting, and skipping empty regions that result from that.
    pub fn incremental_update(
        &mut self,
        delta_x: i32,
        delta_y: i32,
        vert_strip: ImageBuffer<P, Vec<P::Subpixel>>,
        hori_strip: ImageBuffer<P, Vec<P::Subpixel>>,
    ) {
        // Calculate the updated texture offset, which is always in [0; self.size)
        let x_after_update =
            (self.u_texture_offset.value.x + delta_x).mod_floor(&i32::try_from(self.size).unwrap());
        let y_after_update =
            (self.u_texture_offset.value.y + delta_y).mod_floor(&i32::try_from(self.size).unwrap());
        let texture_offset_after_update = Vector2::new(x_after_update, y_after_update);

        // The vertical region's x coordinate needs to be the "lower" (but not
        // really, because of wraparound, so don't use min) of the old and new
        // horizontal texture offset. Why? If delta_x > 0 and we use the new
        // offset, we'll overwrite the wrong data. The y coordinate is just the
        // new vertical texture offset however, which may not seem intuitive.
        // You can convince yourself by staring at the illustration in section
        // 2.4 of the page linked above,
        // https://developer.nvidia.com/gpugems/GPUGems2/gpugems2_chapter02.html,
        // remembering that y is up and recalling where the L shape originates.
        // If that doesn't help, draw an image with named pixels and a texture
        // window in it and perform a few updates in different directions,
        // noting where the L-shaped region (aka vert_strip + hori_strip) are
        // placed in the texture.
        let vert_x = if delta_x > 0 {
            self.u_texture_offset.value.x
        } else {
            x_after_update
        };
        let hori_y = if delta_y > 0 {
            self.u_texture_offset.value.y
        } else {
            y_after_update
        };
        let vert_regions = UpdateRegion::new_regions(
            vert_x.try_into().unwrap(),
            y_after_update.try_into().unwrap(),
            self.size,
            &vert_strip,
        );
        let hori_regions = UpdateRegion::new_regions(
            x_after_update.try_into().unwrap(),
            hori_y.try_into().unwrap(),
            self.size,
            &hori_strip,
        );
        let regions = vert_regions.iter().chain(hori_regions.iter());

        // Now we can overwrite the old texture offset
        self.u_texture_offset.value = texture_offset_after_update;

        unsafe {
            self.gl.BindTexture(opengl::TEXTURE_2D, self.id);
            for r in regions {
                let width = i32::try_from(r.pixels.width()).unwrap();
                let height = i32::try_from(r.pixels.height()).unwrap();
                if height * width == 0 {
                    continue;
                }
                let image = r.pixels.to_image();
                // For an explanation of the different parameters, see
                // https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glTexSubImage2D.xhtml
                self.gl.TexSubImage2D(
                    opengl::TEXTURE_2D,
                    0, // level
                    i32::try_from(r.x).unwrap(),
                    i32::try_from(r.y).unwrap(),
                    width,
                    height,
                    P::FORMAT,
                    P::DTYPE,
                    image.into_raw().as_ptr() as *const c_void,
                );
            }
        }
    }

    /// Updates the offset and binds the texture
    pub fn submit(&mut self) {
        unsafe {
            self.u_texture_offset.submit();

            self.gl.ActiveTexture(opengl::TEXTURE0);
            self.gl.BindTexture(opengl::TEXTURE_2D, self.id);
        }
    }
}

/// Trait to associate pixel formats with OpenGL constants
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

#[cfg(test)]
mod tests {
    use super::*;
    use image::GenericImage;

    #[test]
    fn test_regions() {
        let test_src = ImageBuffer::from_fn(16, 16, |x, y| Rgba::<u8>([x as u8, y as u8, 0, 255]));
        let regions = UpdateRegion::new_regions(4, 7, 16, &test_src);

        let mut dest = ImageBuffer::from_pixel(16, 16, Rgba::<u8>([0, 0, 0, 0]));
        for r in &regions {
            dest.copy_from(&r.pixels, r.x as u32, r.y as u32);
        }
        let reference = ImageBuffer::from_fn(16, 16, |x, y| {
            Rgba::<u8>([(x + 16 - 4) as u8 % 16, (y + 16 - 7) as u8 % 16, 0, 255])
        });
        assert!(dest.pixels().eq(reference.pixels()));
    }
}
