//! Configuration management with YAML persistence

use anyhow::Result;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

/// Application configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Saved commands for quick access
    #[serde(default)]
    pub saved_commands: Vec<SavedCommand>,

    /// Default workspace path
    #[serde(default)]
    pub default_workspace: Option<PathBuf>,

    /// UI preferences
    #[serde(default)]
    pub ui: UiPreferences,

    /// Process preferences
    #[serde(default)]
    pub process: ProcessPreferences,
}

/// A saved command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SavedCommand {
    /// Display name
    pub name: String,
    /// The command to execute
    pub command: String,
    /// Optional description
    #[serde(default)]
    pub description: Option<String>,
    /// Whether to auto-start on launch
    #[serde(default)]
    pub auto_start: bool,
}

/// UI preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UiPreferences {
    /// Auto-scroll logs
    #[serde(default = "default_true")]
    pub auto_scroll: bool,

    /// Number of log lines to display
    #[serde(default = "default_log_lines")]
    pub log_lines: usize,

    /// Show timestamps in logs
    #[serde(default)]
    pub show_timestamps: bool,

    /// Color theme
    #[serde(default)]
    pub theme: Theme,
}

/// Process preferences
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProcessPreferences {
    /// Maximum log buffer size per process
    #[serde(default = "default_log_buffer_size")]
    pub log_buffer_size: usize,

    /// Auto-restart failed processes
    #[serde(default)]
    pub auto_restart: bool,

    /// Maximum auto-restart attempts
    #[serde(default = "default_max_restarts")]
    pub max_restarts: u32,

    /// Restart delay in seconds
    #[serde(default = "default_restart_delay")]
    pub restart_delay_secs: u64,
}

/// Color theme
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Theme {
    #[default]
    Dark,
    Light,
    Custom(CustomTheme),
}

/// Custom theme colors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomTheme {
    pub background: String,
    pub foreground: String,
    pub highlight: String,
    pub error: String,
    pub warning: String,
    pub success: String,
}

// Default value functions for serde
fn default_true() -> bool {
    true
}

fn default_log_lines() -> usize {
    50
}

fn default_log_buffer_size() -> usize {
    10000
}

fn default_max_restarts() -> u32 {
    3
}

fn default_restart_delay() -> u64 {
    2
}

impl Default for Config {
    fn default() -> Self {
        Self {
            saved_commands: Vec::new(),
            default_workspace: None,
            ui: UiPreferences::default(),
            process: ProcessPreferences::default(),
        }
    }
}

impl Default for UiPreferences {
    fn default() -> Self {
        Self {
            auto_scroll: true,
            log_lines: 50,
            show_timestamps: false,
            theme: Theme::Dark,
        }
    }
}

impl Default for ProcessPreferences {
    fn default() -> Self {
        Self {
            log_buffer_size: 10000,
            auto_restart: false,
            max_restarts: 3,
            restart_delay_secs: 2,
        }
    }
}

impl Config {
    /// Get the default config file path
    pub fn default_path() -> PathBuf {
        dirs::config_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("ros2dev")
            .join("config.yaml")
    }

    /// Load configuration from file
    pub fn load(path: Option<PathBuf>) -> Result<Self> {
        let path = path.unwrap_or_else(Self::default_path);

        if path.exists() {
            let content = std::fs::read_to_string(&path)?;
            let config: Config = serde_yaml::from_str(&content)?;
            Ok(config)
        } else {
            Ok(Self::default())
        }
    }

    /// Save configuration to file
    pub fn save(&self, path: Option<PathBuf>) -> Result<()> {
        let path = path.unwrap_or_else(Self::default_path);

        // Create directory if it doesn't exist
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let content = serde_yaml::to_string(self)?;
        std::fs::write(&path, content)?;

        Ok(())
    }

    /// Add a saved command
    pub fn add_saved_command(&mut self, name: String, command: String, description: Option<String>) {
        self.saved_commands.push(SavedCommand {
            name,
            command,
            description,
            auto_start: false,
        });
    }

    /// Remove a saved command by name
    pub fn remove_saved_command(&mut self, name: &str) {
        self.saved_commands.retain(|c| c.name != name);
    }

    /// Get commands that should auto-start
    pub fn auto_start_commands(&self) -> Vec<&SavedCommand> {
        self.saved_commands.iter().filter(|c| c.auto_start).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn test_default_config() {
        let config = Config::default();
        assert!(config.saved_commands.is_empty());
        assert!(config.ui.auto_scroll);
    }

    #[test]
    fn test_save_and_load() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("config.yaml");

        let mut config = Config::default();
        config.add_saved_command(
            "test".to_string(),
            "echo hello".to_string(),
            Some("Test command".to_string()),
        );

        config.save(Some(path.clone())).unwrap();

        let loaded = Config::load(Some(path)).unwrap();
        assert_eq!(loaded.saved_commands.len(), 1);
        assert_eq!(loaded.saved_commands[0].name, "test");
    }
}
