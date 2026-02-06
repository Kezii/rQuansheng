use core::fmt::Write;
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::{DrawTarget, Point, Primitive, Size},
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;

use embedded_graphics::Drawable;

pub enum FontSizes {
    VerySmall,
    VerySmallInverse,
    VerySmallNumbers,
    MidNumbers,
    LargeNumbers,
}

impl FontSizes {
    pub fn get_font_style(&self) -> MonoTextStyle<BinaryColor> {
        match self {
            FontSizes::VerySmall => MonoTextStyle::new(
                &embedded_graphics::mono_font::ascii::FONT_6X12,
                BinaryColor::On,
            ),
            FontSizes::VerySmallInverse => MonoTextStyle::new(
                &embedded_graphics::mono_font::ascii::FONT_6X12,
                BinaryColor::Off,
            ),
            FontSizes::VerySmallNumbers => {
                MonoTextStyle::new(&crate::fonts_generated::PROFONT_10_POINT, BinaryColor::On)
            }
            FontSizes::MidNumbers => {
                MonoTextStyle::new(&crate::fonts_generated::PROFONT_14_POINT, BinaryColor::On)
            }
            FontSizes::LargeNumbers => {
                MonoTextStyle::new(&crate::fonts_generated::PROFONT_24_POINT, BinaryColor::On)
            }
        }
    }
}
pub enum UiWidgetType {
    Nothing,
    Text(String<8>, FontSizes),
    Rectangle(Size),
}

pub const MAX_TEXT_LENGTH: usize = 8;

impl UiWidgetType {
    pub fn write<F>(font_size: FontSizes, writer: F) -> Self
    where
        F: FnOnce(&mut String<MAX_TEXT_LENGTH>) -> core::fmt::Result,
    {
        let mut text = String::<MAX_TEXT_LENGTH>::new();
        writer(&mut text).ok();
        Self::Text(text, font_size)
    }

    pub fn draw<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
        pos: Point,
    ) -> Result<(), D::Error> {
        match &self {
            UiWidgetType::Text(text, font_size) => {
                Text::new(text, pos, font_size.get_font_style()).draw(display)?;
            }
            UiWidgetType::Rectangle(size) => {
                Rectangle::new(pos, *size)
                    .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
                    .draw(display)?;
            }
            UiWidgetType::Nothing => {}
        }
        Ok(())
    }
}

pub struct UiWidget {
    pub ty: UiWidgetType,
    pub pos: Point,
}

impl UiWidget {
    pub fn new(pos: Point, ty: UiWidgetType) -> Self {
        Self { ty, pos }
    }

    pub fn draw<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
    ) -> Result<(), D::Error> {
        self.ty.draw(display, self.pos)?;
        Ok(())
    }
}

pub struct UiLineLayout<'a> {
    pub y: i32,
    pub widgets: &'a [(u16, UiWidgetType)], // offset from the previous widget
}

impl<'a> UiLineLayout<'a> {
    pub fn new(y: i32, widgets: &'a [(u16, UiWidgetType)]) -> Self {
        Self { y, widgets }
    }

    pub fn draw<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
    ) -> Result<(), D::Error> {
        let mut x = 0;
        for (offset, widget) in self.widgets {
            widget.draw(display, Point::new(x + *offset as i32, self.y))?;
            x += *offset as i32;
        }
        Ok(())
    }
}
