//! ROS 2 environment detection

use std::path::PathBuf;

/// ROS 2 environment information
#[derive(Debug, Clone, Default)]
pub struct Ros2Environment {
    /// ROS distribution name (e.g., "humble", "iron", "jazzy")
    pub distro: Option<String>,
    /// ROS domain ID
    pub domain_id: Option<u32>,
    /// RMW implementation
    pub rmw_implementation: Option<String>,
    /// Whether ROS 2 is sourced
    pub is_sourced: bool,
    /// ROS 2 installation path
    pub ros_root: Option<PathBuf>,
}

impl Ros2Environment {
    /// Detect the ROS 2 environment from environment variables
    pub fn detect() -> Self {
        let distro = std::env::var("ROS_DISTRO").ok();
        let domain_id = std::env::var("ROS_DOMAIN_ID")
            .ok()
            .and_then(|s| s.parse().ok());
        let rmw_implementation = std::env::var("RMW_IMPLEMENTATION").ok();
        let ros_root = std::env::var("AMENT_PREFIX_PATH")
            .ok()
            .and_then(|s| s.split(':').next().map(PathBuf::from));

        let is_sourced = distro.is_some();

        Self {
            distro,
            domain_id,
            rmw_implementation,
            is_sourced,
            ros_root,
        }
    }

    /// Get available ROS 2 distributions
    pub fn available_distros() -> Vec<String> {
        let mut distros = Vec::new();
        let ros_path = std::path::Path::new("/opt/ros");

        if ros_path.exists() {
            if let Ok(entries) = std::fs::read_dir(ros_path) {
                for entry in entries.flatten() {
                    if entry.path().is_dir() {
                        if let Some(name) = entry.file_name().to_str() {
                            if entry.path().join("setup.bash").exists() {
                                distros.push(name.to_string());
                            }
                        }
                    }
                }
            }
        }

        distros.sort();
        distros
    }

    /// Get display string
    pub fn display(&self) -> String {
        if let Some(distro) = &self.distro {
            let mut info = format!("ROS 2 {}", distro);
            if let Some(domain) = self.domain_id {
                info.push_str(&format!(" (domain: {})", domain));
            }
            if let Some(rmw) = &self.rmw_implementation {
                info.push_str(&format!(" [{}]", rmw));
            }
            info
        } else {
            "ROS 2 not sourced".to_string()
        }
    }

    /// Get the setup script path for this environment
    pub fn get_setup_script(&self) -> Option<PathBuf> {
        if let Some(distro) = &self.distro {
            let path = PathBuf::from(format!("/opt/ros/{}/setup.bash", distro));
            if path.exists() {
                return Some(path);
            }
        }
        None
    }
}
