//! Theme system for consistent UI styling
//!
//! Provides color palettes, border styles, and visual constants
//! that can be customized via the config system.

use ratatui::style::{Color, Modifier, Style};
use ratatui::widgets::BorderType;

/// Color palette for the application
#[derive(Debug, Clone)]
pub struct ColorPalette {
    // Primary colors
    pub primary: Color,
    pub secondary: Color,
    pub accent: Color,

    // Status colors
    pub success: Color,
    pub warning: Color,
    pub error: Color,
    pub info: Color,

    // Text colors
    pub text: Color,
    pub text_muted: Color,
    pub text_dim: Color,

    // Background colors
    pub bg: Color,
    pub bg_secondary: Color,
    pub bg_highlight: Color,
    pub bg_selected: Color,

    // Border colors
    pub border: Color,
    pub border_focused: Color,
    pub border_active: Color,
}

impl ColorPalette {
    /// Dark theme (default)
    pub fn dark() -> Self {
        Self {
            // Primary colors
            primary: Color::Cyan,
            secondary: Color::Yellow,
            accent: Color::Magenta,

            // Status colors
            success: Color::Green,
            warning: Color::Yellow,
            error: Color::Red,
            info: Color::Blue,

            // Text colors
            text: Color::White,
            text_muted: Color::Gray,
            text_dim: Color::DarkGray,

            // Background colors
            bg: Color::Reset,
            bg_secondary: Color::Rgb(30, 30, 30),
            bg_highlight: Color::Rgb(50, 50, 60),
            bg_selected: Color::Rgb(40, 60, 80),

            // Border colors
            border: Color::DarkGray,
            border_focused: Color::Cyan,
            border_active: Color::Yellow,
        }
    }

    /// Light theme
    pub fn light() -> Self {
        Self {
            // Primary colors
            primary: Color::Blue,
            secondary: Color::Rgb(200, 120, 0),
            accent: Color::Magenta,

            // Status colors
            success: Color::Rgb(0, 150, 0),
            warning: Color::Rgb(200, 150, 0),
            error: Color::Rgb(200, 0, 0),
            info: Color::Blue,

            // Text colors
            text: Color::Black,
            text_muted: Color::DarkGray,
            text_dim: Color::Gray,

            // Background colors
            bg: Color::White,
            bg_secondary: Color::Rgb(240, 240, 240),
            bg_highlight: Color::Rgb(220, 230, 240),
            bg_selected: Color::Rgb(200, 220, 255),

            // Border colors
            border: Color::Gray,
            border_focused: Color::Blue,
            border_active: Color::Rgb(200, 120, 0),
        }
    }
}

impl Default for ColorPalette {
    fn default() -> Self {
        Self::dark()
    }
}

/// Theme configuration with all styling options
#[derive(Debug, Clone)]
pub struct Theme {
    pub colors: ColorPalette,
    pub border_type: BorderType,
    pub border_type_focused: BorderType,
}

impl Theme {
    pub fn dark() -> Self {
        Self {
            colors: ColorPalette::dark(),
            border_type: BorderType::Rounded,
            border_type_focused: BorderType::Double,
        }
    }

    pub fn light() -> Self {
        Self {
            colors: ColorPalette::light(),
            border_type: BorderType::Rounded,
            border_type_focused: BorderType::Double,
        }
    }

    // ===== Style Builders =====

    /// Default text style
    pub fn text(&self) -> Style {
        Style::default().fg(self.colors.text)
    }

    /// Muted text style (labels, hints)
    pub fn text_muted(&self) -> Style {
        Style::default().fg(self.colors.text_muted)
    }

    /// Dim text style (very subtle)
    pub fn text_dim(&self) -> Style {
        Style::default().fg(self.colors.text_dim)
    }

    /// Primary accent style (headers, active items)
    pub fn primary(&self) -> Style {
        Style::default().fg(self.colors.primary)
    }

    /// Secondary accent style (icons, highlights)
    pub fn secondary(&self) -> Style {
        Style::default().fg(self.colors.secondary)
    }

    /// Success style
    pub fn success(&self) -> Style {
        Style::default().fg(self.colors.success)
    }

    /// Warning style
    pub fn warning(&self) -> Style {
        Style::default().fg(self.colors.warning)
    }

    /// Error style
    pub fn error(&self) -> Style {
        Style::default().fg(self.colors.error)
    }

    /// Info style
    pub fn info(&self) -> Style {
        Style::default().fg(self.colors.info)
    }

    /// Selected/highlighted item style
    pub fn selected(&self) -> Style {
        Style::default()
            .fg(self.colors.primary)
            .bg(self.colors.bg_selected)
            .add_modifier(Modifier::BOLD)
    }

    /// List highlight style (for ratatui List widget)
    pub fn list_highlight(&self) -> Style {
        Style::default()
            .fg(self.colors.text)
            .bg(self.colors.bg_selected)
            .add_modifier(Modifier::BOLD)
    }

    /// Tab highlight style
    pub fn tab_highlight(&self) -> Style {
        Style::default()
            .fg(self.colors.secondary)
            .bg(self.colors.bg_highlight)
            .add_modifier(Modifier::BOLD)
    }

    /// Border style for normal blocks
    pub fn border(&self) -> Style {
        Style::default().fg(self.colors.border)
    }

    /// Border style for focused blocks
    pub fn border_focused(&self) -> Style {
        Style::default().fg(self.colors.border_focused)
    }

    /// Border style for active/selected blocks
    pub fn border_active(&self) -> Style {
        Style::default().fg(self.colors.border_active)
    }

    /// Input field style (inactive)
    pub fn input(&self) -> Style {
        Style::default()
            .fg(self.colors.text)
            .bg(self.colors.bg_secondary)
    }

    /// Input field style (focused)
    pub fn input_focused(&self) -> Style {
        Style::default()
            .fg(self.colors.text)
            .bg(self.colors.bg_highlight)
    }

    /// Status bar style
    pub fn status_bar(&self) -> Style {
        Style::default()
            .fg(self.colors.text)
            .bg(self.colors.bg_secondary)
    }

    /// Header/title style
    pub fn title(&self) -> Style {
        Style::default()
            .fg(self.colors.primary)
            .add_modifier(Modifier::BOLD)
    }

    /// Logo primary part
    pub fn logo_primary(&self) -> Style {
        Style::default()
            .fg(self.colors.primary)
            .add_modifier(Modifier::BOLD)
    }

    /// Logo secondary part
    pub fn logo_secondary(&self) -> Style {
        Style::default().fg(self.colors.secondary)
    }
}

impl Default for Theme {
    fn default() -> Self {
        Self::dark()
    }
}

/// Global theme instance - can be changed at runtime
use std::sync::RwLock;
static CURRENT_THEME: RwLock<Option<Theme>> = RwLock::new(None);

/// Get the current theme (defaults to dark)
pub fn current() -> Theme {
    CURRENT_THEME.read().unwrap().clone().unwrap_or_default()
}

/// Set the current theme
pub fn set_theme(theme: Theme) {
    *CURRENT_THEME.write().unwrap() = Some(theme);
}

/// Initialize theme from config
pub fn init_from_config(config_theme: &crate::core::Theme) {
    let theme = match config_theme {
        crate::core::Theme::Dark => Theme::dark(),
        crate::core::Theme::Light => Theme::light(),
        crate::core::Theme::Custom(_) => Theme::dark(), // TODO: implement custom theme
    };
    set_theme(theme);
}
