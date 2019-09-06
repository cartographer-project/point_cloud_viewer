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

pub struct GlTexture<P: Pixel> {
    id: GLuint,
    gl: Rc<opengl::Gl>,
    size: i32,
    u_texture_offset: GlUniform<Vector2<i32>>,
    pixel_type: PhantomData<P>,
    pub debug_tex: ImageBuffer<P, Vec<P::Subpixel>>,
}

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
        let debug_tex = pixels.clone();
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
            debug_tex,
        }
    }

    fn incremental_update_regions(
        &mut self,
        delta_x: i32,
        delta_y: i32,
        mut vert_strip: ImageBuffer<P, Vec<P::Subpixel>>,
        mut hori_strip: ImageBuffer<P, Vec<P::Subpixel>>,
    ) -> [Option<UpdateRegion<P>>; 4] {
        let mut regions: [Option<UpdateRegion<P>>; 4] = [None, None, None, None];

        // normalize to positive deltas
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

        if width + xoff > self.size {
            // wraparound – split into two vertical strips
            let right_width = self.size - xoff;
            let left_width = width - right_width;
            regions[0] = Some(UpdateRegion {
                xoff,
                yoff: 0,
                width: right_width,
                height: self.size,
                pixels: rotate_y(
                    vert_strip.sub_image(0, 0, right_width as u32, self.size as u32),
                    self.u_texture_offset.value.y as u32,
                ),
            });
            regions[1] = Some(UpdateRegion {
                xoff: 0,
                yoff: 0,
                width: left_width,
                height: self.size,
                pixels: rotate_y(
                    vert_strip.sub_image(
                        right_width as u32,
                        0,
                        left_width as u32,
                        self.size as u32,
                    ),
                    self.u_texture_offset.value.y as u32,
                ),
            });
        } else if width != 0 {
            regions[0] = Some(UpdateRegion {
                xoff,
                yoff: 0,
                width,
                height: self.size,
                pixels: rotate_y(vert_strip, self.u_texture_offset.value.y as u32),
            });
        }

        if height + yoff > self.size {
            // wraparound – split into two horizontal strips
            let upper_height = self.size - yoff;
            let lower_height = height - upper_height;
            regions[2] = Some(UpdateRegion {
                xoff: 0,
                yoff,
                width: self.size,
                height: upper_height,
                pixels: rotate_x(
                    hori_strip.sub_image(0, 0, self.size as u32, upper_height as u32),
                    self.u_texture_offset.value.x as u32,
                ),
            });
            regions[3] = Some(UpdateRegion {
                xoff: 0,
                yoff: 0,
                width: self.size,
                height: lower_height,
                pixels: rotate_x(
                    hori_strip.sub_image(
                        0,
                        upper_height as u32,
                        self.size as u32,
                        lower_height as u32,
                    ),
                    self.u_texture_offset.value.x as u32,
                ),
            });
        } else if height != 0 {
            regions[2] = Some(UpdateRegion {
                xoff: 0,
                yoff,
                width: self.size,
                height,
                pixels: rotate_x(hori_strip, self.u_texture_offset.value.x as u32),
            });
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
        let regions = ArrayVec::from(
            self.incremental_update_regions(delta_x, delta_y, vert_strip, hori_strip),
        );

        for reg in regions {
            let r = match reg {
                Some(r) => r,
                None => continue,
            };

            let buf =
                ImageBuffer::from_raw(r.width as u32, r.height as u32, r.pixels.clone().into_raw())
                    .unwrap();
            self.debug_tex.copy_from(&buf, r.xoff as u32, r.yoff as u32);

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

pub fn debug<P: AsRef<std::path::Path>>(tex: &ImageBuffer<LumaA<f32>, Vec<f32>>, name: P) {
    if tex.width() * tex.height() == 0 {
        return;
    }
    let first_channel: Vec<_> = tex
        .clone()
        .into_raw()
        .into_iter()
        .enumerate()
        .filter(|(i, _)| i % 2 == 0)
        .map(|(_, px)| (px * 100.0) as u8)
        .collect();
    use image::GrayImage;
    let dbg_image: GrayImage =
        ImageBuffer::from_raw(tex.width(), tex.height(), first_channel).unwrap();
    dbg_image.save(name).unwrap();
}
