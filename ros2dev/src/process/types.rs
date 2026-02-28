//! Process types and status management

use chrono::{DateTime, Local};

use crate::core::LogBuffer;

// Re-export ProcessId from core::ipc to avoid duplication
pub use crate::core::ipc::ProcessId;

/// Status of a managed process
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProcessStatus {
    /// Process is pending (not yet started)
    Pending,
    /// Process is running
    Running,
    /// Process was stopped by user
    Stopped,
    /// Process exited with a code
    Exited(i32),
    /// Process failed with an error
    Failed(String),
}

impl ProcessStatus {
    /// Get a short string representation
    pub fn as_str(&self) -> &str {
        match self {
            ProcessStatus::Pending => "Pending",
            ProcessStatus::Running => "Running",
            ProcessStatus::Stopped => "Stopped",
            ProcessStatus::Exited(_) => "Exited",
            ProcessStatus::Failed(_) => "Failed",
        }
    }

    /// Check if the process is running
    pub fn is_running(&self) -> bool {
        matches!(self, ProcessStatus::Running)
    }

    /// Check if the process can be started
    pub fn can_start(&self) -> bool {
        matches!(
            self,
            ProcessStatus::Pending
                | ProcessStatus::Stopped
                | ProcessStatus::Exited(_)
                | ProcessStatus::Failed(_)
        )
    }
}

impl std::fmt::Display for ProcessStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProcessStatus::Pending => write!(f, "⏳ Pending"),
            ProcessStatus::Running => write!(f, "🟢 Running"),
            ProcessStatus::Stopped => write!(f, "⏹️  Stopped"),
            ProcessStatus::Exited(code) => {
                if *code == 0 {
                    write!(f, "✅ Exited (0)")
                } else {
                    write!(f, "⚠️  Exited ({})", code)
                }
            }
            ProcessStatus::Failed(msg) => write!(f, "❌ Failed: {}", msg),
        }
    }
}

/// A managed process
#[derive(Debug)]
pub struct Process {
    /// Unique identifier
    pub id: ProcessId,
    /// Display name
    pub name: String,
    /// The command being executed
    pub command: String,
    /// Current status
    pub status: ProcessStatus,
    /// Process ID (if running)
    pub pid: Option<u32>,
    /// When the process was created
    pub created_at: DateTime<Local>,
    /// When the process was last started
    pub started_at: Option<DateTime<Local>>,
    /// When the process ended
    pub ended_at: Option<DateTime<Local>>,
    /// Log buffer for this process
    pub log_buffer: LogBuffer,
    /// Number of restarts
    pub restart_count: u32,
}

impl Process {
    /// Create a new process
    pub fn new(id: ProcessId, name: String, command: String) -> Self {
        Self {
            id,
            name,
            command,
            status: ProcessStatus::Pending,
            pid: None,
            created_at: Local::now(),
            started_at: None,
            ended_at: None,
            log_buffer: LogBuffer::new(5000),
            restart_count: 0,
        }
    }

    /// Get the uptime string
    pub fn uptime(&self) -> Option<String> {
        if let Some(started) = self.started_at {
            if self.status.is_running() {
                let duration = Local::now().signed_duration_since(started);
                return Some(format_duration(duration));
            }
        }
        None
    }

    /// Get the runtime string (total time from start to end)
    pub fn runtime(&self) -> Option<String> {
        if let (Some(started), Some(ended)) = (self.started_at, self.ended_at) {
            let duration = ended.signed_duration_since(started);
            return Some(format_duration(duration));
        }
        None
    }
}

/// Format a duration as a human-readable string
fn format_duration(duration: chrono::Duration) -> String {
    let total_seconds = duration.num_seconds();
    let hours = total_seconds / 3600;
    let minutes = (total_seconds % 3600) / 60;
    let seconds = total_seconds % 60;

    if hours > 0 {
        format!("{}h {}m {}s", hours, minutes, seconds)
    } else if minutes > 0 {
        format!("{}m {}s", minutes, seconds)
    } else {
        format!("{}s", seconds)
    }
}

/// Information about a running process handle
#[derive(Debug)]
pub struct ProcessHandle {
    /// Process ID
    pub id: ProcessId,
    /// Child process handle
    pub child: tokio::process::Child,
    /// Task handles for stdout/stderr readers
    pub reader_tasks: Vec<tokio::task::JoinHandle<()>>,
}

impl ProcessHandle {
    /// Create a new process handle
    pub fn new(
        id: ProcessId,
        child: tokio::process::Child,
        reader_tasks: Vec<tokio::task::JoinHandle<()>>,
    ) -> Self {
        Self {
            id,
            child,
            reader_tasks,
        }
    }

    /// Kill the process
    pub async fn kill(&mut self) -> anyhow::Result<()> {
        self.child.kill().await?;
        
        // Abort reader tasks
        for task in &self.reader_tasks {
            task.abort();
        }
        
        Ok(())
    }
}
