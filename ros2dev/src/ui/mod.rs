//! Terminal UI module
//!
//! This module provides all UI rendering functionality including:
//! - Theme system with customizable colors and styles
//! - Icon set with Unicode and ASCII support
//! - View rendering functions

pub mod theme;
pub mod icons;
mod render;

pub use render::draw;
pub use theme::Theme;
pub use icons::Icons;

/// Initialize the UI system with config settings
pub fn init(config: &crate::core::Config) {
    theme::init_from_config(&config.ui.theme);
    icons::set_icons(icons::Icons::unicode());
}
