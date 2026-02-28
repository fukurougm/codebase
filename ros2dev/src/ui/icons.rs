//! Icon system using Unicode characters
//!
//! Provides consistent iconography throughout the UI.

/// Icon set for the application
#[derive(Debug, Clone)]
pub struct Icons {
    // Navigation
    pub arrow_right: &'static str,
    pub arrow_left: &'static str,
    pub arrow_up: &'static str,
    pub arrow_down: &'static str,
    pub chevron: &'static str,
    pub chevron_inactive: &'static str,
    pub nested: &'static str,

    // Status indicators
    pub running: &'static str,
    pub stopped: &'static str,
    pub pending: &'static str,
    pub success: &'static str,
    pub error: &'static str,
    pub warning: &'static str,

    // Actions
    pub add: &'static str,
    pub remove: &'static str,
    pub edit: &'static str,
    pub refresh: &'static str,
    pub settings: &'static str,
    pub build: &'static str,
    pub clean: &'static str,

    // Items
    pub dashboard: &'static str,
    pub package: &'static str,
    pub node: &'static str,
    pub launch: &'static str,
    pub topic: &'static str,
    pub service: &'static str,
    pub action: &'static str,
    pub process: &'static str,
    pub file: &'static str,
    pub folder: &'static str,

    // Misc
    pub home: &'static str,
    pub help: &'static str,
    pub info: &'static str,
    pub bullet: &'static str,
    pub check: &'static str,
    pub cross: &'static str,
    pub star: &'static str,
    pub time: &'static str,
    pub separator: &'static str,
    pub interface: &'static str,
    pub save: &'static str,
    pub ui_settings: &'static str,
    pub process_settings: &'static str,
    pub user: &'static str,
    pub link: &'static str,
}

impl Icons {
    /// Unicode icon set (default)
    pub fn unicode() -> Self {
        Self {
            // Navigation
            arrow_right: "→",
            arrow_left: "←",
            arrow_up: "▲",
            arrow_down: "▼",
            chevron: "❯",
            chevron_inactive: " ",
            nested: "↳",

            // Status indicators
            running: "●",
            stopped: "○",
            pending: "◐",
            success: "✓",
            error: "✗",
            warning: "⚠",

            // Actions
            add: "⊕",
            remove: "⊖",
            edit: "✎",
            refresh: "⟲",
            settings: "⚙",
            build: "⚒",
            clean: "⌫",

            // Items
            dashboard: "◈",
            package: "▣",
            node: "◆",
            launch: "▶",
            topic: "◇",
            service: "⚡",
            action: "↯",
            process: "⊡",
            file: "▪",
            folder: "▬",

            // Misc
            home: "⌂",
            help: "?",
            info: "ℹ",
            bullet: "•",
            check: "✓",
            cross: "✗",
            star: "★",
            time: "⧗",
            separator: "›",
            interface: "⊞",
            save: "⊙",
            ui_settings: "◐",
            process_settings: "⚙",
            user: "♠",
            link: "↗",
        }
    }

    /// ASCII fallback for limited terminals
    pub fn ascii() -> Self {
        Self {
            // Navigation
            arrow_right: "->",
            arrow_left: "<-",
            arrow_up: "^",
            arrow_down: "v",
            chevron: ">",
            chevron_inactive: " ",
            nested: "+-",

            // Status indicators
            running: "*",
            stopped: "o",
            pending: "~",
            success: "+",
            error: "x",
            warning: "!",

            // Actions
            add: "[+]",
            remove: "[-]",
            edit: "[e]",
            refresh: "[r]",
            settings: "[s]",
            build: "[b]",
            clean: "[c]",

            // Items
            dashboard: "[D]",
            package: "[P]",
            node: "[N]",
            launch: "[L]",
            topic: "[T]",
            service: "[S]",
            action: "[A]",
            process: "[*]",
            file: "-",
            folder: "+",

            // Misc
            home: "~",
            help: "?",
            info: "i",
            bullet: "*",
            check: "+",
            cross: "x",
            star: "*",
            time: "@",
            separator: ">",
            interface: "[I]",
            save: "[S]",
            ui_settings: "[U]",
            process_settings: "[P]",
            user: "@",
            link: ">",
        }
    }

    /// Get icon for MenuItem
    pub fn menu_icon(&self, item: &crate::app::MenuItem) -> &'static str {
        use crate::app::MenuItem;
        match item {
            MenuItem::Dashboard => self.dashboard,
            MenuItem::Packages => self.package,
            MenuItem::Nodes => self.node,
            MenuItem::LaunchFiles => self.launch,
            MenuItem::Topics => self.topic,
            MenuItem::Services => self.service,
            MenuItem::Actions => self.action,
            MenuItem::ProcessManager => self.process,
            MenuItem::CreatePackage => self.add,
            MenuItem::CreateNode => self.edit,
            MenuItem::CreateLaunch => self.file,
            MenuItem::Build => self.build,
            MenuItem::Clean => self.clean,
            MenuItem::Settings => self.settings,
        }
    }

    /// Get status icon with appropriate symbol
    pub fn status_icon(&self, status: &crate::process::ProcessStatus) -> &'static str {
        use crate::process::ProcessStatus;
        match status {
            ProcessStatus::Pending => self.pending,
            ProcessStatus::Running => self.running,
            ProcessStatus::Stopped => self.stopped,
            ProcessStatus::Exited(code) => {
                if *code == 0 {
                    self.success
                } else {
                    self.warning
                }
            }
            ProcessStatus::Failed(_) => self.error,
        }
    }

    /// Get settings category icon
    pub fn settings_icon(&self, category: &crate::app::SettingsCategory) -> &'static str {
        use crate::app::SettingsCategory;
        match category {
            SettingsCategory::Ui => self.ui_settings,
            SettingsCategory::Process => self.process_settings,
            SettingsCategory::SavedCommands => self.save,
            SettingsCategory::About => self.info,
        }
    }
}

impl Default for Icons {
    fn default() -> Self {
        Self::unicode()
    }
}

/// Global icons instance
use std::sync::RwLock;
static CURRENT_ICONS: RwLock<Option<Icons>> = RwLock::new(None);

/// Get the current icon set
pub fn current() -> Icons {
    CURRENT_ICONS.read().unwrap().clone().unwrap_or_default()
}

/// Set the current icon set
pub fn set_icons(icons: Icons) {
    *CURRENT_ICONS.write().unwrap() = Some(icons);
}

/// Highlight symbol for list selection
pub const HIGHLIGHT_SYMBOL: &str = "❯ ";

/// Highlight symbol for ASCII mode
pub const HIGHLIGHT_SYMBOL_ASCII: &str = "> ";
