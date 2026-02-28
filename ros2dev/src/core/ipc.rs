//! Inter-process communication types for process manager

use uuid::Uuid;

/// Unique identifier for a process (re-exported for convenience)
pub type ProcessId = Uuid;

/// Commands sent to the process manager
#[derive(Debug, Clone)]
pub enum ProcessCommand {
    /// Start a new process
    Start {
        id: ProcessId,
        name: String,
        command: String,
    },
    /// Stop a running process
    Stop { id: ProcessId },
    /// Restart a process
    Restart {
        id: ProcessId,
        name: String,
        command: String,
    },
    /// Stop all processes
    StopAll,
    /// Send signal to a process
    Signal { id: ProcessId, signal: i32 },
}

/// Events emitted by the process manager
#[derive(Debug, Clone)]
pub enum ProcessEvent {
    /// Process has started
    Started { id: ProcessId, pid: u32 },
    /// Process produced output
    Output {
        id: ProcessId,
        line: String,
        is_stderr: bool,
    },
    /// Process has exited
    Exited { id: ProcessId, exit_code: i32 },
    /// Error occurred
    Error { id: ProcessId, error: String },
}
