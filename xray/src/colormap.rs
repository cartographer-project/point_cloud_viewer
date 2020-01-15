use point_viewer::color::Color;

// Implementation of matlab's jet colormap from here:
// https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
pub struct Jet;

pub trait Colormap: Send {
    fn for_value_unchecked(&self, val: f32) -> Color<u8>;

    fn for_value(&self, val: f32) -> Color<u8> {
        assert!(0. <= val);
        assert!(val <= 1.);
        self.for_value_unchecked(val)
    }
}

impl Jet {
    fn red(&self, gray: f32) -> f32 {
        self.base(gray - 0.5)
    }

    fn green(&self, gray: f32) -> f32 {
        self.base(gray)
    }

    fn blue(&self, gray: f32) -> f32 {
        self.base(gray + 0.5)
    }

    fn base(&self, val: f32) -> f32 {
        if val <= -0.75 {
            0.
        } else if val <= -0.25 {
            self.interpolate(val, 0.0, -0.75, 1.0, -0.25)
        } else if val <= 0.25 {
            1.0
        } else if val <= 0.75 {
            self.interpolate(val, 1.0, 0.25, 0.0, 0.75)
        } else {
            0.0
        }
    }

    fn interpolate(&self, val: f32, y0: f32, x0: f32, y1: f32, x1: f32) -> f32 {
        (val - x0) * (y1 - y0) / (x1 - x0) + y0
    }
}

impl Colormap for Jet {
    fn for_value_unchecked(&self, val: f32) -> Color<u8> {
        Color {
            red: self.red(val),
            green: self.green(val),
            blue: self.blue(val),
            alpha: 1.,
        }
        .to_u8()
    }
}

pub const PURPLISH: Color<f32> = Color {
    red: 0.8,
    green: 0.8,
    blue: 1.0,
    alpha: 1.0,
};

// Interpolate from that color to black
pub struct Monochrome(pub Color<f32>);

impl Colormap for Monochrome {
    fn for_value_unchecked(&self, val: f32) -> Color<u8> {
        Color {
            red: (1.0 - val) * self.0.red,
            green: (1.0 - val) * self.0.green,
            blue: (1.0 - val) * self.0.blue,
            alpha: 1.0,
        }
        .to_u8()
    }
}
