//! Core application modules
//!
//! This module contains the core functionality of the application:
//! - Configuration management
//! - Event handling
//! - IPC (Inter-Process Communication)
//! - Log buffering

mod config;
mod events;
mod log_buffer;

// ipc depends on process types, so we need to handle this carefully
// ProcessId is re-exported from the process module
pub mod ipc;

pub use config::*;
pub use events::*;
pub use ipc::*;
pub use log_buffer::*;
