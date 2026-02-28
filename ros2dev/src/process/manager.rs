//! Process manager for spawning and managing concurrent processes

use anyhow::Result;
use std::collections::HashMap;
use std::path::PathBuf;
use std::process::Stdio;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tokio::sync::mpsc;
use uuid::Uuid;

use crate::core::ProcessEvent;
use super::{ProcessHandle, ProcessId};

/// Manages concurrent processes
pub struct ProcessManager {
    /// Map of running process handles
    handles: HashMap<ProcessId, ProcessHandle>,
    /// Channel to send events to the app
    event_tx: mpsc::UnboundedSender<ProcessEvent>,
    /// Working directory for processes
    working_dir: PathBuf,
}

impl ProcessManager {
    /// Create a new process manager
    pub fn new(event_tx: mpsc::UnboundedSender<ProcessEvent>, working_dir: PathBuf) -> Self {
        Self {
            handles: HashMap::new(),
            event_tx,
            working_dir,
        }
    }

    /// Spawn a new process
    pub async fn spawn_process(&mut self, name: &str, command: &str) -> Result<ProcessId> {
        let id = Uuid::new_v4();
        
        // Parse the command
        let parts: Vec<&str> = command.split_whitespace().collect();
        if parts.is_empty() {
            return Err(anyhow::anyhow!("Empty command"));
        }

        let program = parts[0];
        let args = &parts[1..];

        // Check if we need to source ROS 2 setup
        let (shell_program, shell_args) = if self.needs_ros2_env(command) {
            // Wrap in bash to source ROS 2 setup
            let setup_script = self.find_ros2_setup();
            let full_command = if let Some(setup) = setup_script {
                format!("source {} && {}", setup.display(), command)
            } else {
                command.to_string()
            };
            ("bash", vec!["-c".to_string(), full_command])
        } else {
            (program, args.iter().map(|s| s.to_string()).collect())
        };

        // Spawn the process
        let mut child = Command::new(shell_program)
            .args(&shell_args)
            .current_dir(&self.working_dir)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .stdin(Stdio::null())
            .kill_on_drop(true)
            .spawn()?;

        let pid = child.id().unwrap_or(0);

        // Send started event
        let _ = self.event_tx.send(ProcessEvent::Started { id, pid });

        // Setup stdout reader
        let stdout = child.stdout.take();
        let stderr = child.stderr.take();

        let mut reader_tasks = Vec::new();

        // Spawn stdout reader task
        if let Some(stdout) = stdout {
            let tx = self.event_tx.clone();
            let task = tokio::spawn(async move {
                let reader = BufReader::new(stdout);
                let mut lines = reader.lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    if tx
                        .send(ProcessEvent::Output {
                            id,
                            line,
                            is_stderr: false,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
            });
            reader_tasks.push(task);
        }

        // Spawn stderr reader task
        if let Some(stderr) = stderr {
            let tx = self.event_tx.clone();
            let task = tokio::spawn(async move {
                let reader = BufReader::new(stderr);
                let mut lines = reader.lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    if tx
                        .send(ProcessEvent::Output {
                            id,
                            line,
                            is_stderr: true,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
            });
            reader_tasks.push(task);
        }

        // Spawn wait task to handle process exit
        let tx = self.event_tx.clone();
        let wait_task = tokio::spawn(async move {
            // We need to wait on the child, but we don't have ownership here
            // This is handled differently - we'll check status in the main loop
        });
        drop(wait_task);

        // Store the handle
        let handle = ProcessHandle::new(id, child, reader_tasks);
        self.handles.insert(id, handle);

        // Spawn a task to watch for process exit
        self.spawn_exit_watcher(id);

        Ok(id)
    }

    /// Spawn a task to watch for process exit
    fn spawn_exit_watcher(&mut self, id: ProcessId) {
        if let Some(handle) = self.handles.get_mut(&id) {
            let tx = self.event_tx.clone();
            
            // We can't move the child out, so we'll use a different approach
            // Check the status periodically in on_tick instead
            let _ = tx; // Suppress unused warning
        }
    }

    /// Check for exited processes
    pub async fn check_processes(&mut self) {
        let mut exited = Vec::new();

        for (id, handle) in &mut self.handles {
            match handle.child.try_wait() {
                Ok(Some(status)) => {
                    let exit_code = status.code().unwrap_or(-1);
                    let _ = self.event_tx.send(ProcessEvent::Exited {
                        id: *id,
                        exit_code,
                    });
                    exited.push(*id);
                }
                Ok(None) => {
                    // Still running
                }
                Err(e) => {
                    let _ = self.event_tx.send(ProcessEvent::Error {
                        id: *id,
                        error: e.to_string(),
                    });
                    exited.push(*id);
                }
            }
        }

        // Remove exited processes from handles
        for id in exited {
            self.handles.remove(&id);
        }
    }

    /// Stop a process
    pub async fn stop_process(&mut self, id: ProcessId) -> Result<()> {
        if let Some(mut handle) = self.handles.remove(&id) {
            handle.kill().await?;
            let _ = self.event_tx.send(ProcessEvent::Exited { id, exit_code: -1 });
        }
        Ok(())
    }

    /// Restart a process
    pub async fn restart_process(&mut self, id: ProcessId, name: &str, command: &str) -> Result<()> {
        // Stop the old process if running
        self.stop_process(id).await?;
        
        // Start a new process with the same ID
        self.spawn_process_with_id(id, name, command).await?;
        
        Ok(())
    }

    /// Spawn a process with a specific ID
    async fn spawn_process_with_id(&mut self, id: ProcessId, _name: &str, command: &str) -> Result<()> {
        // Parse the command
        let parts: Vec<&str> = command.split_whitespace().collect();
        if parts.is_empty() {
            return Err(anyhow::anyhow!("Empty command"));
        }

        let program = parts[0];
        let args = &parts[1..];

        // Check if we need to source ROS 2 setup
        let (shell_program, shell_args) = if self.needs_ros2_env(command) {
            let setup_script = self.find_ros2_setup();
            let full_command = if let Some(setup) = setup_script {
                format!("source {} && {}", setup.display(), command)
            } else {
                command.to_string()
            };
            ("bash", vec!["-c".to_string(), full_command])
        } else {
            (program, args.iter().map(|s| s.to_string()).collect())
        };

        // Spawn the process
        let mut child = Command::new(shell_program)
            .args(&shell_args)
            .current_dir(&self.working_dir)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .stdin(Stdio::null())
            .kill_on_drop(true)
            .spawn()?;

        let pid = child.id().unwrap_or(0);

        // Send started event
        let _ = self.event_tx.send(ProcessEvent::Started { id, pid });

        // Setup readers
        let stdout = child.stdout.take();
        let stderr = child.stderr.take();

        let mut reader_tasks = Vec::new();

        if let Some(stdout) = stdout {
            let tx = self.event_tx.clone();
            let task = tokio::spawn(async move {
                let reader = BufReader::new(stdout);
                let mut lines = reader.lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    if tx
                        .send(ProcessEvent::Output {
                            id,
                            line,
                            is_stderr: false,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
            });
            reader_tasks.push(task);
        }

        if let Some(stderr) = stderr {
            let tx = self.event_tx.clone();
            let task = tokio::spawn(async move {
                let reader = BufReader::new(stderr);
                let mut lines = reader.lines();
                while let Ok(Some(line)) = lines.next_line().await {
                    if tx
                        .send(ProcessEvent::Output {
                            id,
                            line,
                            is_stderr: true,
                        })
                        .is_err()
                    {
                        break;
                    }
                }
            });
            reader_tasks.push(task);
        }

        let handle = ProcessHandle::new(id, child, reader_tasks);
        self.handles.insert(id, handle);

        Ok(())
    }

    /// Stop all processes
    pub async fn stop_all(&mut self) -> Result<()> {
        let ids: Vec<ProcessId> = self.handles.keys().copied().collect();
        for id in ids {
            self.stop_process(id).await?;
        }
        Ok(())
    }

    /// Check if a command needs ROS 2 environment
    fn needs_ros2_env(&self, command: &str) -> bool {
        command.starts_with("ros2 ")
            || command.starts_with("colcon ")
            || command.contains("ros2 ")
            || command.contains("colcon ")
    }

    /// Find the ROS 2 setup script
    fn find_ros2_setup(&self) -> Option<PathBuf> {
        // First check for local workspace setup
        let local_setup = self.working_dir.join("install/setup.bash");
        if local_setup.exists() {
            return Some(local_setup);
        }

        // Check for ROS 2 installations
        let ros2_paths = [
            "/opt/ros/jazzy/setup.bash",
            "/opt/ros/iron/setup.bash",
            "/opt/ros/humble/setup.bash",
            "/opt/ros/rolling/setup.bash",
        ];

        for path in &ros2_paths {
            let path = PathBuf::from(path);
            if path.exists() {
                return Some(path);
            }
        }

        // Check environment variable
        if let Ok(distro) = std::env::var("ROS_DISTRO") {
            let path = PathBuf::from(format!("/opt/ros/{}/setup.bash", distro));
            if path.exists() {
                return Some(path);
            }
        }

        None
    }

    /// Get the number of running processes
    pub fn running_count(&self) -> usize {
        self.handles.len()
    }
}
