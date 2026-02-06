use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoFont, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::{DrawTarget, Point, Primitive, Size},
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;

use embedded_graphics::Drawable;

#[derive(Clone)]
pub enum FontSizes {
    VerySmall,
    VerySmallNumbers,
    MidNumbers,
    LargeNumbers,
}

impl FontSizes {
    pub fn get_font_style(&self) -> &'static MonoFont<'static> {
        match self {
            FontSizes::VerySmall => &embedded_graphics::mono_font::ascii::FONT_6X12,
            FontSizes::VerySmallNumbers => &crate::fonts_generated::PROFONT_10_POINT,
            FontSizes::MidNumbers => &crate::fonts_generated::PROFONT_14_POINT,
            FontSizes::LargeNumbers => &crate::fonts_generated::PROFONT_24_POINT,
        }
    }
}

#[derive(Clone)]
pub struct FlexText<const N: usize> {
    pub text: String<N>,
    pub rel_offset: Option<u16>,
    pub abs_offset: Option<i32>,
    pub font_size: Option<FontSizes>,
    pub visible: bool,
}

impl<const N: usize> FlexText<N> {
    pub fn write<F>(writer: F) -> Self
    where
        F: FnOnce(&mut String<N>) -> core::fmt::Result,
    {
        let mut text = String::<N>::new();
        writer(&mut text).ok();
        Self {
            text,
            rel_offset: None,
            abs_offset: None,
            font_size: None,
            visible: true,
        }
    }

    pub fn with_rel_offset(mut self, offset: u16) -> Self {
        self.rel_offset = Some(offset);
        self
    }
    pub fn with_abs_offset(mut self, offset: i32) -> Self {
        self.abs_offset = Some(offset);
        self
    }

    pub fn with_font_size(mut self, font_size: FontSizes) -> Self {
        self.font_size = Some(font_size);
        self
    }

    pub fn with_visible(mut self, visible: bool) -> Self {
        self.visible = visible;
        self
    }

    pub fn from_dbg<T: core::fmt::Debug>(value: T) -> Self {
        let mut text = String::<N>::new();
        write!(text, "{:?}", value).ok();
        Self {
            text,
            rel_offset: None,
            abs_offset: None,
            font_size: None,
            visible: true,
        }
    }
}

impl<const N: usize> From<&str> for FlexText<N> {
    fn from(s: &str) -> Self {
        let mut text = String::<N>::new();
        text.push_str(s).ok();
        Self {
            text,
            rel_offset: None,
            abs_offset: None,
            font_size: None,
            visible: true,
        }
    }
}

pub const MAX_TEXT_LENGTH: usize = 8;

pub enum UiWidgetType {
    Nothing,
    Text(FlexText<MAX_TEXT_LENGTH>, FontSizes),
    Rectangle(Size),
}

impl UiWidgetType {
    pub fn write<F>(font_size: FontSizes, writer: F) -> Self
    where
        F: FnOnce(&mut String<MAX_TEXT_LENGTH>) -> core::fmt::Result,
    {
        Self::Text(FlexText::write(writer), font_size)
    }

    pub fn draw<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
        pos: Point,
    ) -> Result<(), D::Error> {
        match &self {
            UiWidgetType::Text(text, font_size) => {
                let font = font_size.get_font_style();
                let font_style = MonoTextStyle::new(font, BinaryColor::On);

                Text::new(text.text.as_str(), pos, font_style).draw(display)?;
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

pub struct UiTextLineLayout<'a> {
    pub y: i32,
    pub font_size: FontSizes,
    pub widgets: &'a [FlexText<MAX_TEXT_LENGTH>], // offset from the previous widget
}

impl<'a> UiTextLineLayout<'a> {
    pub fn new(y: i32, font_size: FontSizes, texts: &'a [FlexText<MAX_TEXT_LENGTH>]) -> Self {
        Self {
            y,
            font_size,
            widgets: texts,
        }
    }

    pub fn draw<D: DrawTarget<Color = BinaryColor>>(
        &self,
        display: &mut D,
    ) -> Result<(), D::Error> {
        let mut x = 0;
        for widget in self.widgets {
            let pos_x = x + widget.rel_offset.unwrap_or(0) as i32;

            if !widget.visible {
                continue;
            }

            x += widget.rel_offset.unwrap_or(0) as i32;

            UiWidget {
                ty: UiWidgetType::Text(
                    widget.clone(),
                    widget.font_size.clone().unwrap_or(self.font_size.clone()),
                ),
                pos: Point::new(widget.abs_offset.unwrap_or(pos_x), self.y),
            }
            .draw(display)?;
        }
        Ok(())
    }
}
