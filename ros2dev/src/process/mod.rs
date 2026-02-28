//! Process management module
//!
//! This module handles spawning, monitoring, and managing
//! concurrent processes for ROS 2 development tasks.

mod types;
mod manager;

pub use types::*;
pub use manager::*;
