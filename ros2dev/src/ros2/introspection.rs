//! ROS 2 runtime introspection (topics, services, nodes)

use anyhow::Result;
use std::process::Command;
use std::path::Path;

/// A ROS 2 topic
#[derive(Debug, Clone)]
pub struct Ros2Topic {
    pub name: String,
    pub msg_type: String,
    pub publishers: u32,
    pub subscribers: u32,
}

/// A ROS 2 service
#[derive(Debug, Clone)]
pub struct Ros2Service {
    pub name: String,
    pub srv_type: String,
}

/// A ROS 2 action
#[derive(Debug, Clone)]
pub struct Ros2Action {
    pub name: String,
    pub action_type: String,
}

/// A running ROS 2 node
#[derive(Debug, Clone)]
pub struct Ros2Node {
    pub name: String,
    pub namespace: String,
}

/// Interface type info
#[derive(Debug, Clone)]
pub struct Ros2Interface {
    pub name: String,
    pub interface_type: InterfaceType,
    pub definition: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InterfaceType {
    Message,
    Service,
    Action,
}

/// ROS 2 introspection helper
pub struct Ros2Introspection {
    workspace_path: Option<std::path::PathBuf>,
}

impl Ros2Introspection {
    pub fn new(workspace_path: Option<std::path::PathBuf>) -> Self {
        Self { workspace_path }
    }

    /// Build the ros2 command with proper sourcing
    fn ros2_command(&self) -> Command {
        let mut cmd = Command::new("bash");
        
        let source_cmd = if let Some(ws) = &self.workspace_path {
            let local_setup = ws.join("install/setup.bash");
            if local_setup.exists() {
                format!("source {} 2>/dev/null; ", local_setup.display())
            } else {
                Self::get_ros2_source()
            }
        } else {
            Self::get_ros2_source()
        };

        cmd.arg("-c");
        cmd.env("PYTHONUNBUFFERED", "1");
        
        // Store source_cmd for later use
        cmd.env("ROS2_SOURCE_CMD", source_cmd);
        cmd
    }

    fn get_ros2_source() -> String {
        // Try to find ROS 2 setup
        let distros = ["jazzy", "iron", "humble", "rolling"];
        for distro in &distros {
            let path = format!("/opt/ros/{}/setup.bash", distro);
            if Path::new(&path).exists() {
                return format!("source {} 2>/dev/null; ", path);
            }
        }
        String::new()
    }

    fn run_ros2_command(&self, args: &str) -> Result<String> {
        let source_cmd = if let Some(ws) = &self.workspace_path {
            let local_setup = ws.join("install/setup.bash");
            if local_setup.exists() {
                format!("source {} 2>/dev/null; ", local_setup.display())
            } else {
                Self::get_ros2_source()
            }
        } else {
            Self::get_ros2_source()
        };

        let full_cmd = format!("{} ros2 {}", source_cmd, args);
        
        let output = Command::new("bash")
            .arg("-c")
            .arg(&full_cmd)
            .output()?;

        Ok(String::from_utf8_lossy(&output.stdout).to_string())
    }

    /// List all topics
    pub fn list_topics(&self) -> Result<Vec<Ros2Topic>> {
        let output = self.run_ros2_command("topic list -v")?;
        let mut topics = Vec::new();

        // Format: " * /topic_name [msg/Type] N publishers" or " * /topic_name [msg/Type] N subscribers"
        for line in output.lines() {
            let line = line.trim();
            
            // Skip headers like "Published topics:" and "Subscribed topics:"
            if line.ends_with(':') || line.is_empty() {
                continue;
            }
            
            if line.starts_with("* ") {
                let content = line.trim_start_matches("* ");
                
                // Parse format: /topic_name [msg/Type] N publishers/subscribers
                // Find the opening bracket for msg type
                if let Some(bracket_start) = content.find('[') {
                    let topic_name = content[..bracket_start].trim().to_string();
                    
                    // Find the closing bracket
                    if let Some(bracket_end) = content.find(']') {
                        let msg_type = content[bracket_start + 1..bracket_end].to_string();
                        
                        // Parse count after ] 
                        let after_bracket = &content[bracket_end + 1..].trim();
                        let pub_count = after_bracket
                            .split_whitespace()
                            .next()
                            .and_then(|s| s.parse::<u32>().ok())
                            .unwrap_or(0);
                        
                        let is_publisher = after_bracket.contains("publisher");
                        
                        // Check if we already have this topic
                        if let Some(existing) = topics.iter_mut().find(|t: &&mut Ros2Topic| t.name == topic_name) {
                            if is_publisher {
                                existing.publishers = pub_count;
                            } else {
                                existing.subscribers = pub_count;
                            }
                        } else {
                            topics.push(Ros2Topic {
                                name: topic_name,
                                msg_type,
                                publishers: if is_publisher { pub_count } else { 0 },
                                subscribers: if !is_publisher { pub_count } else { 0 },
                            });
                        }
                    }
                }
            }
        }

        // If verbose failed, try simple list
        if topics.is_empty() {
            let output = self.run_ros2_command("topic list")?;
            for line in output.lines() {
                let name = line.trim();
                if !name.is_empty() && name.starts_with('/') {
                    topics.push(Ros2Topic {
                        name: name.to_string(),
                        msg_type: String::new(),
                        publishers: 0,
                        subscribers: 0,
                    });
                }
            }
        }

        topics.sort_by(|a, b| a.name.cmp(&b.name));
        topics.dedup_by(|a, b| a.name == b.name);
        
        Ok(topics)
    }

    /// Get topic info
    pub fn topic_info(&self, topic: &str) -> Result<String> {
        self.run_ros2_command(&format!("topic info {} -v", topic))
    }

    /// Echo topic (returns a command to run)
    pub fn topic_echo_cmd(&self, topic: &str) -> String {
        format!("ros2 topic echo {}", topic)
    }

    /// Publish to topic (returns a command)
    pub fn topic_pub_cmd(&self, topic: &str, msg_type: &str, data: &str) -> String {
        format!("ros2 topic pub {} {} \"{}\" --once", topic, msg_type, data)
    }

    /// List all services
    pub fn list_services(&self) -> Result<Vec<Ros2Service>> {
        let output = self.run_ros2_command("service list -t")?;
        let mut services = Vec::new();

        for line in output.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            
            // Format: /service_name [service/Type]
            let parts: Vec<&str> = line.splitn(2, ' ').collect();
            let name = parts[0].to_string();
            let srv_type = if parts.len() > 1 {
                parts[1].trim_matches(|c| c == '[' || c == ']').to_string()
            } else {
                String::new()
            };

            services.push(Ros2Service { name, srv_type });
        }

        Ok(services)
    }

    /// Call service (returns a command)
    pub fn service_call_cmd(&self, service: &str, srv_type: &str, data: &str) -> String {
        format!("ros2 service call {} {} \"{}\"", service, srv_type, data)
    }

    /// List all actions
    pub fn list_actions(&self) -> Result<Vec<Ros2Action>> {
        let output = self.run_ros2_command("action list -t")?;
        let mut actions = Vec::new();

        for line in output.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            let parts: Vec<&str> = line.splitn(2, ' ').collect();
            let name = parts[0].to_string();
            let action_type = if parts.len() > 1 {
                parts[1].trim_matches(|c| c == '[' || c == ']').to_string()
            } else {
                String::new()
            };

            actions.push(Ros2Action { name, action_type });
        }

        Ok(actions)
    }

    /// List running nodes
    pub fn list_nodes(&self) -> Result<Vec<Ros2Node>> {
        let output = self.run_ros2_command("node list")?;
        let mut nodes = Vec::new();

        for line in output.lines() {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            // Parse namespace and name from full path like /namespace/node_name
            let parts: Vec<&str> = line.rsplitn(2, '/').collect();
            let (name, namespace) = if parts.len() == 2 {
                (parts[0].to_string(), format!("/{}", parts[1].trim_start_matches('/')))
            } else {
                (line.trim_start_matches('/').to_string(), "/".to_string())
            };

            nodes.push(Ros2Node { name, namespace });
        }

        Ok(nodes)
    }

    /// Get node info
    pub fn node_info(&self, node: &str) -> Result<String> {
        self.run_ros2_command(&format!("node info {}", node))
    }

    /// Get interface definition
    pub fn interface_show(&self, interface: &str) -> Result<String> {
        self.run_ros2_command(&format!("interface show {}", interface))
    }

    /// List all message types
    pub fn list_msg_types(&self) -> Result<Vec<String>> {
        let output = self.run_ros2_command("interface list -m")?;
        Ok(output.lines()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect())
    }

    /// List all service types
    pub fn list_srv_types(&self) -> Result<Vec<String>> {
        let output = self.run_ros2_command("interface list -s")?;
        Ok(output.lines()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect())
    }
}
