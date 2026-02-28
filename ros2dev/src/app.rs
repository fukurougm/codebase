//! Application state and main logic

use anyhow::Result;
use crossterm::event::{KeyCode, KeyEvent, KeyModifiers, MouseEvent};
use std::path::PathBuf;
use tokio::sync::mpsc;

use crate::core::{Config, ProcessEvent, LogBuffer};
use crate::process::{Process, ProcessId, ProcessStatus, ProcessManager};
use crate::ros2::{
    Ros2Action, Ros2Environment, Ros2Introspection, Ros2Node, Ros2Package, 
    Ros2Service, Ros2Templates, Ros2Topic, Ros2Workspace,
};

/// Menu categories for grouping items
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuCategory {
    Observation,  // Monitoring & viewing
    Management,   // Process management
    Workspace,    // Project creation & build
    System,       // Configuration
}

impl MenuCategory {
    pub fn label(&self) -> &'static str {
        match self {
            MenuCategory::Observation => "OBSERVATION",
            MenuCategory::Management => "MANAGEMENT",
            MenuCategory::Workspace => "WORKSPACE",
            MenuCategory::System => "SYSTEM",
        }
    }
    
    pub fn description(&self) -> &'static str {
        match self {
            MenuCategory::Observation => "Monitoring & viewing",
            MenuCategory::Management => "Process management",
            MenuCategory::Workspace => "Project creation & build",
            MenuCategory::System => "Configuration",
        }
    }
}

/// Main menu items
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuItem {
    Dashboard,
    Packages,
    Nodes,
    LaunchFiles,
    Topics,
    Services,
    Actions,
    ProcessManager,
    CreatePackage,
    CreateNode,
    CreateLaunch,
    Build,
    Clean,
    Settings,
}

impl MenuItem {
    pub fn all() -> Vec<MenuItem> {
        vec![
            MenuItem::Dashboard,
            MenuItem::Packages,
            MenuItem::Nodes,
            MenuItem::LaunchFiles,
            MenuItem::Topics,
            MenuItem::Services,
            MenuItem::Actions,
            MenuItem::ProcessManager,
            MenuItem::CreatePackage,
            MenuItem::CreateNode,
            MenuItem::CreateLaunch,
            MenuItem::Build,
            MenuItem::Clean,
            MenuItem::Settings,
        ]
    }

    pub fn icon(&self) -> &str {
        match self {
            MenuItem::Dashboard => "📊",
            MenuItem::Packages => "📦",
            MenuItem::Nodes => "🔲",
            MenuItem::LaunchFiles => "🚀",
            MenuItem::Topics => "📡",
            MenuItem::Services => "🔧",
            MenuItem::Actions => "⚡",
            MenuItem::ProcessManager => "⚙️",
            MenuItem::CreatePackage => "➕",
            MenuItem::CreateNode => "📝",
            MenuItem::CreateLaunch => "📄",
            MenuItem::Build => "🔨",
            MenuItem::Clean => "🧹",
            MenuItem::Settings => "⚙️",
        }
    }

    pub fn label(&self) -> &str {
        match self {
            MenuItem::Dashboard => "Dashboard",
            MenuItem::Packages => "Packages",
            MenuItem::Nodes => "Nodes",
            MenuItem::LaunchFiles => "Launch Files",
            MenuItem::Topics => "Topics",
            MenuItem::Services => "Services",
            MenuItem::Actions => "Actions",
            MenuItem::ProcessManager => "Process Manager",
            MenuItem::CreatePackage => "Create Package",
            MenuItem::CreateNode => "Create Node",
            MenuItem::CreateLaunch => "Create Launch File",
            MenuItem::Build => "Build Workspace",
            MenuItem::Clean => "Clean Workspace",
            MenuItem::Settings => "Settings",
        }
    }

    pub fn description(&self) -> &str {
        match self {
            MenuItem::Dashboard => "View workspace overview and status",
            MenuItem::Packages => "Browse and manage ROS 2 packages",
            MenuItem::Nodes => "Explore and run ROS 2 nodes",
            MenuItem::LaunchFiles => "View and execute launch files",
            MenuItem::Topics => "Monitor and interact with topics",
            MenuItem::Services => "Browse and call services",
            MenuItem::Actions => "View available actions",
            MenuItem::ProcessManager => "Manage running processes",
            MenuItem::CreatePackage => "Create a new ROS 2 package",
            MenuItem::CreateNode => "Generate node template (C++/Python)",
            MenuItem::CreateLaunch => "Create a new launch file",
            MenuItem::Build => "Build workspace with colcon",
            MenuItem::Clean => "Remove build artifacts",
            MenuItem::Settings => "Configure ros2dev settings",
        }
    }
    
    pub fn category(&self) -> MenuCategory {
        match self {
            MenuItem::Dashboard | MenuItem::Packages | MenuItem::Nodes |
            MenuItem::LaunchFiles | MenuItem::Topics | MenuItem::Services |
            MenuItem::Actions => MenuCategory::Observation,
            
            MenuItem::ProcessManager => MenuCategory::Management,
            
            MenuItem::CreatePackage | MenuItem::CreateNode | MenuItem::CreateLaunch |
            MenuItem::Build | MenuItem::Clean => MenuCategory::Workspace,
            
            MenuItem::Settings => MenuCategory::System,
        }
    }
    
    /// Get all menu items grouped by category
    pub fn grouped() -> Vec<(MenuCategory, Vec<MenuItem>)> {
        vec![
            (MenuCategory::Observation, vec![
                MenuItem::Dashboard,
                MenuItem::Packages,
                MenuItem::Nodes,
                MenuItem::LaunchFiles,
                MenuItem::Topics,
                MenuItem::Services,
                MenuItem::Actions,
            ]),
            (MenuCategory::Management, vec![
                MenuItem::ProcessManager,
            ]),
            (MenuCategory::Workspace, vec![
                MenuItem::CreatePackage,
                MenuItem::CreateNode,
                MenuItem::CreateLaunch,
                MenuItem::Build,
                MenuItem::Clean,
            ]),
            (MenuCategory::System, vec![
                MenuItem::Settings,
            ]),
        ]
    }
}

/// Current view/screen
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum View {
    MainMenu,
    Dashboard,
    PackageExplorer,
    PackageDetail(PackageDetailState), // Interactive package detail view
    NodeExplorer,
    LaunchExplorer,
    TopicExplorer,
    ServiceExplorer,
    ActionExplorer,
    ProcessManager,
    CreatePackage(CreatePackageState),
    CreateNode(CreateNodeState),
    CreateLaunch(CreateLaunchState),
    CreateSavedCommand(CreateSavedCommandState), // New saved command wizard
    EditSavedCommand(EditSavedCommandState),     // Edit existing saved command
    EditProcessCommand(EditProcessCommandState), // Edit process command
    TopicSubscriber(String),        // topic name
    TopicPublisher(String, String), // topic name, msg type  
    ServiceClient(ServiceClientState), // interactive service call
    InterfaceView(String, String),  // title, interface definition
    Settings(SettingsState),        // Settings page
    Help,
    Confirm(String, ConfirmAction),
}

/// Settings category
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettingsCategory {
    Ui,
    Process,
    SavedCommands,
    About,
}

impl SettingsCategory {
    pub fn all() -> &'static [SettingsCategory] {
        &[SettingsCategory::Ui, SettingsCategory::Process, SettingsCategory::SavedCommands, SettingsCategory::About]
    }
    
    pub fn name(&self) -> &'static str {
        match self {
            SettingsCategory::Ui => "UI Settings",
            SettingsCategory::Process => "Process Settings",
            SettingsCategory::SavedCommands => "Saved Commands",
            SettingsCategory::About => "About",
        }
    }
    
    pub fn icon(&self) -> &'static str {
        match self {
            SettingsCategory::Ui => "🎨",
            SettingsCategory::Process => "⚙️",
            SettingsCategory::SavedCommands => "💾",
            SettingsCategory::About => "ℹ️",
        }
    }
}

/// State for settings view
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SettingsState {
    pub category: SettingsCategory,
    pub selected_item: usize,
    pub editing: bool,
    pub input_buffer: String,
    pub category_focused: bool, // true = focus on category sidebar, false = focus on items
}

impl Default for SettingsState {
    fn default() -> Self {
        Self {
            category: SettingsCategory::Ui,
            selected_item: 0,
            editing: false,
            input_buffer: String::new(),
            category_focused: true,
        }
    }
}

impl SettingsState {
    pub fn items_for_category(&self) -> Vec<SettingsItem> {
        match self.category {
            SettingsCategory::Ui => vec![
                SettingsItem::Bool { key: "auto_scroll".into(), label: "Auto Scroll Logs".into(), description: "Automatically scroll to bottom of logs".into() },
                SettingsItem::Number { key: "log_lines".into(), label: "Log Lines".into(), description: "Number of log lines to display".into(), min: 10, max: 500 },
                SettingsItem::Bool { key: "show_timestamps".into(), label: "Show Timestamps".into(), description: "Display timestamps in log output".into() },
                SettingsItem::Choice { key: "theme".into(), label: "Theme".into(), description: "Color theme".into(), options: vec!["Dark".into(), "Light".into()] },
            ],
            SettingsCategory::Process => vec![
                SettingsItem::Number { key: "log_buffer_size".into(), label: "Log Buffer Size".into(), description: "Maximum log entries per process".into(), min: 1000, max: 100000 },
                SettingsItem::Bool { key: "auto_restart".into(), label: "Auto Restart".into(), description: "Automatically restart failed processes".into() },
                SettingsItem::Number { key: "max_restarts".into(), label: "Max Restarts".into(), description: "Maximum restart attempts".into(), min: 0, max: 10 },
                SettingsItem::Number { key: "restart_delay_secs".into(), label: "Restart Delay (s)".into(), description: "Delay between restart attempts".into(), min: 1, max: 60 },
            ],
            SettingsCategory::SavedCommands => vec![], // Handled separately by draw_saved_commands
            SettingsCategory::About => vec![], // Handled separately by draw_about
        }
    }
}

/// Settings item types
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SettingsItem {
    Bool { key: String, label: String, description: String },
    Number { key: String, label: String, description: String, min: usize, max: usize },
    Choice { key: String, label: String, description: String, options: Vec<String> },
    Text { key: String, label: String, description: String },
    Action { key: String, label: String, description: String },
}

/// State for creating a new saved command
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreateSavedCommandState {
    pub name: String,
    pub command: String,
    pub description: String,
    pub auto_start: bool,
    pub active_field: usize, // 0=name, 1=command, 2=description, 3=auto_start
    pub field_cursor: usize, // Cursor position within active field
}

impl Default for CreateSavedCommandState {
    fn default() -> Self {
        Self {
            name: String::new(),
            command: String::new(),
            description: String::new(),
            auto_start: false,
            active_field: 0,
            field_cursor: 0,
        }
    }
}

impl CreateSavedCommandState {
    pub fn field_count() -> usize {
        4
    }
    
    pub fn field_name(index: usize) -> &'static str {
        match index {
            0 => "Name",
            1 => "Command",
            2 => "Description",
            3 => "Auto Start",
            _ => "",
        }
    }
    
    pub fn is_valid(&self) -> bool {
        !self.name.trim().is_empty() && !self.command.trim().is_empty()
    }
}

/// State for editing an existing saved command
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EditSavedCommandState {
    pub index: usize, // Index in saved_commands vec
    pub name: String,
    pub command: String,
    pub description: String,
    pub auto_start: bool,
    pub active_field: usize,
    pub field_cursor: usize, // Cursor position within active field
}

impl EditSavedCommandState {
    pub fn from_command(index: usize, cmd: &crate::core::SavedCommand) -> Self {
        Self {
            index,
            name: cmd.name.clone(),
            command: cmd.command.clone(),
            description: cmd.description.clone().unwrap_or_default(),
            auto_start: cmd.auto_start,
            active_field: 0,
            field_cursor: 0,
        }
    }
    
    pub fn field_count() -> usize {
        4
    }
    
    pub fn field_name(index: usize) -> &'static str {
        CreateSavedCommandState::field_name(index)
    }
    
    pub fn is_valid(&self) -> bool {
        !self.name.trim().is_empty() && !self.command.trim().is_empty()
    }
}

/// State for editing a process command
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EditProcessCommandState {
    pub process_index: usize,
    pub name: String,
    pub command: String,
    pub active_field: usize, // 0=name, 1=command
    pub field_cursor: usize, // Cursor position within active field
}

impl EditProcessCommandState {
    pub fn from_process(index: usize, process: &crate::process::Process) -> Self {
        Self {
            process_index: index,
            name: process.name.clone(),
            command: process.command.clone(),
            active_field: 0,
            field_cursor: 0,
        }
    }
    
    pub fn field_count() -> usize {
        2
    }
    
    pub fn is_valid(&self) -> bool {
        !self.name.trim().is_empty() && !self.command.trim().is_empty()
    }
}

/// State for package detail view (folder-like explorer)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PackageDetailState {
    pub package_name: String,
    pub package_path: std::path::PathBuf,
    pub build_type: String,
    pub nodes: Vec<String>,
    pub launch_files: Vec<(String, std::path::PathBuf)>, // (display_name, full_path)
    pub selected_tab: usize,   // 0 = Nodes, 1 = Launch Files, 2 = + New Node, 3 = + New Launch
    pub selected_item: usize,
}

impl PackageDetailState {
    pub fn new(pkg: &crate::ros2::Ros2Package) -> Self {
        let launch_files: Vec<(String, std::path::PathBuf)> = pkg.launch_files.iter()
            .map(|p| {
                let name = p.file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| p.to_string_lossy().to_string());
                (name, p.clone())
            })
            .collect();
        
        Self {
            package_name: pkg.name.clone(),
            package_path: pkg.path.clone(),
            build_type: pkg.build_type.to_string(),
            nodes: pkg.nodes.clone(),
            launch_files,
            selected_tab: 0,
            selected_item: 0,
        }
    }
    
    pub fn current_items(&self) -> Vec<String> {
        match self.selected_tab {
            0 => self.nodes.clone(),
            1 => self.launch_files.iter().map(|(name, _)| name.clone()).collect(),
            _ => vec![],
        }
    }
    
    pub fn items_count(&self) -> usize {
        match self.selected_tab {
            0 => self.nodes.len(),
            1 => self.launch_files.len(),
            _ => 0,
        }
    }
    
    pub fn tab_count() -> usize {
        4  // Nodes, Launch Files, + New Node, + New Launch
    }
}

/// Input mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputMode {
    Normal,
    Search,
    Input,
    LogSelect, 
}

/// State for package creation wizard
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreatePackageState {
    pub step: usize,
    pub name: String,
    pub build_type: usize, // 0=ament_python, 1=ament_cmake
    pub description: String,
    pub maintainer: String,
    pub email: String,
    pub dependencies: String,
    pub initial_node: String,
    pub active_field: usize,
    pub field_cursor: usize, // Cursor position within active field
}

impl Default for CreatePackageState {
    fn default() -> Self {
        Self {
            step: 0,
            name: String::new(),
            build_type: 0,
            description: String::new(),
            maintainer: whoami::username(),
            email: String::new(),
            dependencies: "rclpy std_msgs".to_string(),
            initial_node: String::new(),
            active_field: 0,
            field_cursor: 0,
        }
    }
}

/// State for node creation
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreateNodeState {
    pub package: String,
    pub node_name: String,
    pub node_type: usize, // 0=Python, 1=C++
    pub template: usize,  // 0=Basic, 1=Subscriber, 2=Publisher, 3=Service
    pub active_field: usize,
    pub field_cursor: usize, // Cursor position within active field
}

impl Default for CreateNodeState {
    fn default() -> Self {
        Self {
            package: String::new(),
            node_name: String::new(),
            node_type: 0,
            template: 0,
            active_field: 0,
            field_cursor: 0,
        }
    }
}

/// State for launch file creation
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreateLaunchState {
    pub package: String,
    pub name: String,
    pub format: usize, // 0=Python, 1=XML
    pub selected_nodes: Vec<(String, String)>, // (package, node)
    pub active_field: usize,
    pub field_cursor: usize, // Cursor position within active field
}

impl Default for CreateLaunchState {
    fn default() -> Self {
        Self {
            package: String::new(),
            name: String::new(),
            format: 0,
            selected_nodes: Vec::new(),
            active_field: 0,
            field_cursor: 0,
        }
    }
}

/// State for service client call
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceClientState {
    pub service_name: String,
    pub srv_type: String,
    pub interface_definition: String,
    pub fields: Vec<ServiceField>,
    pub active_field: usize,
}

/// Service request field
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceField {
    pub name: String,
    pub field_type: String,
    pub value: String,
    pub indent: usize,
}

impl ServiceClientState {
    pub fn new(service_name: String, srv_type: String, interface_def: &str) -> Self {
        let fields = Self::parse_interface_fields(interface_def);
        Self {
            service_name,
            srv_type,
            interface_definition: interface_def.to_string(),
            fields,
            active_field: 0,
        }
    }

    /// Parse interface definition to extract request fields
    fn parse_interface_fields(interface_def: &str) -> Vec<ServiceField> {
        let mut fields = Vec::new();
        let mut in_request = true;
        
        for line in interface_def.lines() {
            let trimmed = line.trim();
            
            // Skip empty lines and comments
            if trimmed.is_empty() || trimmed.starts_with('#') {
                continue;
            }
            
            // Check for separator (--- means response section starts)
            if trimmed == "---" {
                in_request = false;
                continue;
            }
            
            // Only parse request fields
            if !in_request {
                continue;
            }
            
            // Count leading spaces for indent
            let indent = line.len() - line.trim_start().len();
            
            // Parse field: "type name" or "type name default_value"
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 2 {
                let field_type = parts[0].to_string();
                let name = parts[1].to_string();
                
                // Skip constants (have = in them)
                if !trimmed.contains('=') {
                    let default_value = Self::default_value_for_type(&field_type);
                    fields.push(ServiceField {
                        name,
                        field_type,
                        value: default_value,
                        indent: indent / 2,
                    });
                }
            }
        }
        
        fields
    }
    
    /// Get default value for a ROS 2 type
    fn default_value_for_type(field_type: &str) -> String {
        match field_type {
            "bool" => "false".to_string(),
            "int8" | "int16" | "int32" | "int64" |
            "uint8" | "uint16" | "uint32" | "uint64" |
            "byte" => "0".to_string(),
            "float32" | "float64" => "0.0".to_string(),
            "string" => "".to_string(),
            t if t.ends_with("[]") => "[]".to_string(),
            _ => "{}".to_string(), // Nested message
        }
    }

    /// Build YAML string from field values
    pub fn to_yaml(&self) -> String {
        if self.fields.is_empty() {
            return "{}".to_string();
        }
        
        let mut yaml_parts = Vec::new();
        for field in &self.fields {
            let value = if field.value.is_empty() {
                Self::default_value_for_type(&field.field_type)
            } else {
                // Quote strings if needed
                if field.field_type == "string" && !field.value.starts_with('"') {
                    format!("'{}'", field.value)
                } else {
                    field.value.clone()
                }
            };
            yaml_parts.push(format!("{}: {}", field.name, value));
        }
        
        format!("{{{}}}", yaml_parts.join(", "))
    }
}

/// Confirm action types
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfirmAction {
    StopProcess(ProcessId),
    StopAllProcesses,
    DeleteProcess(ProcessId),
    DeletePackage(String),
    DeleteSavedCommand(usize), // Index of command to delete
    CleanWorkspace,
    Quit,
}

/// Explorer item - generic for different explorers
#[derive(Debug, Clone)]
pub enum ExplorerItem {
    Package(Ros2Package),
    Node(String, String),      // package, node
    LaunchFile(String, PathBuf), // package, path
    Topic(Ros2Topic),
    Service(Ros2Service),
    Action(Ros2Action),
    RunningNode(Ros2Node),
}

/// Main application state
pub struct App {
    // Core state
    pub should_quit: bool,
    pub view: View,
    pub previous_view: Option<Box<View>>, // Store previous view for confirm dialogs
    pub input_mode: InputMode,
    pub workspace_path: PathBuf,

    // ROS 2 state
    pub ros2_env: Ros2Environment,
    pub ros2_workspace: Option<Ros2Workspace>,
    pub ros2_introspection: Ros2Introspection,

    // Menu state
    pub menu_items: Vec<MenuItem>,
    pub selected_menu: usize,

    // Explorer state
    pub explorer_items: Vec<ExplorerItem>,
    pub selected_item: usize,
    pub search_query: String,
    pub search_cursor: usize,

    // Process manager state
    pub processes: Vec<Process>,
    pub selected_process: usize,
    pub log_buffer: LogBuffer,
    pub log_scroll: usize,
    pub log_select_start: Option<usize>,  // Start of selection (line index)
    pub log_select_end: usize,            // End of selection (cursor line)

    // Live data (topics, services, etc.)
    pub topics: Vec<Ros2Topic>,
    pub services: Vec<Ros2Service>,
    pub actions: Vec<Ros2Action>,
    pub running_nodes: Vec<Ros2Node>,

    // Input state
    pub input_buffer: String,
    pub input_cursor: usize,
    pub suggestions: Vec<(String, String)>, // (name, command)
    pub selected_suggestion: usize,
    pub show_suggestions: bool,

    // Status
    pub status_message: Option<String>,
    pub status_timestamp: std::time::Instant,
    pub is_loading: bool,

    // Config
    pub config: Config,

    // Process management
    process_manager: ProcessManager,
    event_rx: mpsc::UnboundedReceiver<ProcessEvent>,
}

impl App {
    pub async fn new(workspace_path: PathBuf, config_path: Option<PathBuf>) -> Result<Self> {
        let config = Config::load(config_path)?;
        let (event_tx, event_rx) = mpsc::unbounded_channel();
        let process_manager = ProcessManager::new(event_tx, workspace_path.clone());
        let ros2_env = Ros2Environment::detect();
        let ros2_workspace = Ros2Workspace::scan(&workspace_path).ok();
        let ros2_introspection = Ros2Introspection::new(Some(workspace_path.clone()));

        Ok(Self {
            should_quit: false,
            view: View::MainMenu,
            previous_view: None,
            input_mode: InputMode::Normal,
            workspace_path,

            ros2_env,
            ros2_workspace,
            ros2_introspection,

            menu_items: MenuItem::all(),
            selected_menu: 0,

            explorer_items: Vec::new(),
            selected_item: 0,
            search_query: String::new(),
            search_cursor: 0,

            processes: Vec::new(),
            selected_process: 0,
            log_buffer: LogBuffer::new(10000),
            log_scroll: 0,
            log_select_start: None,
            log_select_end: 0,

            topics: Vec::new(),
            services: Vec::new(),
            actions: Vec::new(),
            running_nodes: Vec::new(),

            input_buffer: String::new(),
            input_cursor: 0,
            suggestions: Vec::new(),
            selected_suggestion: 0,
            show_suggestions: false,

            status_message: None,
            status_timestamp: std::time::Instant::now(),
            is_loading: false,

            config,
            process_manager,
            event_rx,
        })
    }

    /// Handle tick events
    pub async fn on_tick(&mut self) -> Result<()> {
        // Process pending events
        while let Ok(event) = self.event_rx.try_recv() {
            self.handle_process_event(event).await?;
        }

        // Clear old status messages
        if self.status_message.is_some()
            && self.status_timestamp.elapsed() > std::time::Duration::from_secs(5)
        {
            self.status_message = None;
        }

        Ok(())
    }

    /// Handle process events
    async fn handle_process_event(&mut self, event: ProcessEvent) -> Result<()> {
        match event {
            ProcessEvent::Started { id, pid } => {
                let name = self.processes.iter().find(|p| p.id == id).map(|p| p.name.clone());
                if let Some(process) = self.processes.iter_mut().find(|p| p.id == id) {
                    process.status = ProcessStatus::Running;
                    process.pid = Some(pid);
                }
                if let Some(name) = name {
                    self.set_status(format!("Process '{}' started (PID: {})", name, pid));
                }
            }
            ProcessEvent::Output { id, line, is_stderr } => {
                if let Some(process) = self.processes.iter_mut().find(|p| p.id == id) {
                    process.log_buffer.push_line(&line, is_stderr);

                    if self.selected_process < self.processes.len()
                        && self.processes[self.selected_process].id == id
                    {
                        self.log_buffer.push_line(&line, is_stderr);
                        if self.log_scroll >= self.log_buffer.len().saturating_sub(1) {
                            self.log_scroll = self.log_buffer.len().saturating_sub(1);
                        }
                    }
                }
            }
            ProcessEvent::Exited { id, exit_code } => {
                let name = self.processes.iter().find(|p| p.id == id).map(|p| p.name.clone());
                if let Some(process) = self.processes.iter_mut().find(|p| p.id == id) {
                    process.status = ProcessStatus::Exited(exit_code);
                    process.pid = None;
                }
                if let Some(name) = name {
                    self.set_status(format!("Process '{}' exited with code {}", name, exit_code));
                }
            }
            ProcessEvent::Error { id, error } => {
                let name = self.processes.iter().find(|p| p.id == id).map(|p| p.name.clone());
                if let Some(process) = self.processes.iter_mut().find(|p| p.id == id) {
                    process.status = ProcessStatus::Failed(error.clone());
                }
                if let Some(name) = name {
                    self.set_status(format!("Process '{}' error: {}", name, error));
                }
            }
        }
        Ok(())
    }

    /// Handle key events
    pub async fn on_key(&mut self, key: KeyEvent) -> Result<()> {
        // Global shortcuts
        if key.modifiers.contains(KeyModifiers::CONTROL) {
            match key.code {
                KeyCode::Char('c') | KeyCode::Char('q') => {
                    if self.has_running_processes() {
                        self.show_confirm(
                            "Stop all processes and quit?".to_string(),
                            ConfirmAction::Quit,
                        );
                    } else {
                        self.should_quit = true;
                    }
                    return Ok(());
                }
                KeyCode::Char('r') => {
                    self.refresh_data().await?;
                    return Ok(());
                }
                _ => {}
            }
        }

        match &self.view.clone() {
            View::MainMenu => self.handle_main_menu_key(key).await?,
            View::Dashboard => self.handle_dashboard_key(key).await?,
            View::PackageExplorer => self.handle_explorer_key(key, MenuItem::Packages).await?,
            View::PackageDetail(_) => self.handle_package_detail_key(key).await?,
            View::NodeExplorer => self.handle_explorer_key(key, MenuItem::Nodes).await?,
            View::LaunchExplorer => self.handle_explorer_key(key, MenuItem::LaunchFiles).await?,
            View::TopicExplorer => self.handle_explorer_key(key, MenuItem::Topics).await?,
            View::ServiceExplorer => self.handle_explorer_key(key, MenuItem::Services).await?,
            View::ActionExplorer => self.handle_explorer_key(key, MenuItem::Actions).await?,
            View::ProcessManager => self.handle_process_manager_key(key).await?,
            View::CreatePackage(_) => self.handle_create_package_key(key).await?,
            View::CreateNode(_) => self.handle_create_node_key(key).await?,
            View::CreateLaunch(_) => self.handle_create_launch_key(key).await?,
            View::CreateSavedCommand(_) => self.handle_create_saved_command_key(key).await?,
            View::EditSavedCommand(_) => self.handle_edit_saved_command_key(key).await?,
            View::EditProcessCommand(_) => self.handle_edit_process_command_key(key).await?,
            View::TopicSubscriber(topic) => {
                let t = topic.clone();
                self.handle_subscriber_key(key, &t).await?;
            }
            View::TopicPublisher(topic, msg_type) => {
                let t = topic.clone();
                let m = msg_type.clone();
                self.handle_publisher_key(key, &t, &m).await?;
            }
            View::ServiceClient(_) => self.handle_service_client_key(key).await?,
            View::InterfaceView(_, _) => self.handle_interface_view_key(key),
            View::Settings(_) => self.handle_settings_key(key).await?,
            View::Help => self.handle_help_key(key),
            View::Confirm(_, _) => self.handle_confirm_key(key).await?,
        }

        Ok(())
    }

    /// Handle main menu navigation
    async fn handle_main_menu_key(&mut self, key: KeyEvent) -> Result<()> {
        match self.input_mode {
            InputMode::Normal => match key.code {
                KeyCode::Char('q') | KeyCode::Esc => {
                    if self.has_running_processes() {
                        self.show_confirm(
                            "Stop all processes and quit?".to_string(),
                            ConfirmAction::Quit,
                        );
                    } else {
                        self.should_quit = true;
                    }
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    if self.selected_menu > 0 {
                        self.selected_menu -= 1;
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if self.selected_menu < self.menu_items.len() - 1 {
                        self.selected_menu += 1;
                    }
                }
                KeyCode::Enter => {
                    self.activate_menu_item().await?;
                }
                KeyCode::Char('/') => {
                    self.input_mode = InputMode::Search;
                    self.search_query.clear();
                    self.search_cursor = 0;
                }
                KeyCode::Char('?') | KeyCode::F(1) => {
                    self.view = View::Help;
                }
                KeyCode::Char('1') => self.selected_menu = 0,
                KeyCode::Char('2') => self.selected_menu = 1,
                KeyCode::Char('3') => self.selected_menu = 2,
                KeyCode::Char('4') => self.selected_menu = 3,
                KeyCode::Char('5') => self.selected_menu = 4,
                KeyCode::Char('6') => self.selected_menu = 5,
                KeyCode::Char('7') => self.selected_menu = 6,
                KeyCode::Char('8') => self.selected_menu = 7,
                KeyCode::Char('9') => self.selected_menu = 8,
                _ => {}
            },
            InputMode::Search => match key.code {
                KeyCode::Esc => {
                    self.input_mode = InputMode::Normal;
                    self.search_query.clear();
                }
                KeyCode::Enter => {
                    self.input_mode = InputMode::Normal;
                    // Filter menu items by search
                    if let Some(idx) = self.menu_items.iter().position(|item| {
                        item.label().to_lowercase().contains(&self.search_query.to_lowercase())
                    }) {
                        self.selected_menu = idx;
                    }
                }
                KeyCode::Char(c) => {
                    self.search_query.insert(self.search_cursor, c);
                    self.search_cursor += 1;
                }
                KeyCode::Backspace => {
                    if self.search_cursor > 0 {
                        self.search_cursor -= 1;
                        self.search_query.remove(self.search_cursor);
                    }
                }
                _ => {}
            },
            _ => {}
        }
        Ok(())
    }

    /// Activate selected menu item
    async fn activate_menu_item(&mut self) -> Result<()> {
        match self.menu_items[self.selected_menu] {
            MenuItem::Dashboard => {
                self.view = View::Dashboard;
            }
            MenuItem::Packages => {
                self.load_packages();
                self.view = View::PackageExplorer;
            }
            MenuItem::Nodes => {
                self.load_nodes();
                self.view = View::NodeExplorer;
            }
            MenuItem::LaunchFiles => {
                self.load_launch_files();
                self.view = View::LaunchExplorer;
            }
            MenuItem::Topics => {
                self.load_topics().await;
                self.view = View::TopicExplorer;
            }
            MenuItem::Services => {
                self.load_services().await;
                self.view = View::ServiceExplorer;
            }
            MenuItem::Actions => {
                self.load_actions().await;
                self.view = View::ActionExplorer;
            }
            MenuItem::ProcessManager => {
                self.view = View::ProcessManager;
            }
            MenuItem::CreatePackage => {
                self.view = View::CreatePackage(CreatePackageState::default());
            }
            MenuItem::CreateNode => {
                self.view = View::CreateNode(CreateNodeState::default());
            }
            MenuItem::CreateLaunch => {
                self.view = View::CreateLaunch(CreateLaunchState::default());
            }
            MenuItem::Build => {
                self.run_build().await?;
            }
            MenuItem::Clean => {
                self.show_confirm(
                    "Clean workspace? This will remove build/, install/, and log/ directories.".to_string(),
                    ConfirmAction::CleanWorkspace,
                );
            }
            MenuItem::Settings => {
                self.view = View::Settings(SettingsState::default());
            }
        }
        Ok(())
    }

    /// Handle dashboard key events
    async fn handle_dashboard_key(&mut self, key: KeyEvent) -> Result<()> {
        match key.code {
            KeyCode::Esc | KeyCode::Char('q') => {
                self.view = View::MainMenu;
            }
            KeyCode::Char('r') => {
                self.refresh_data().await?;
            }
            KeyCode::Char('?') | KeyCode::F(1) => {
                self.view = View::Help;
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle explorer key events (generic for all explorers)
    async fn handle_explorer_key(&mut self, key: KeyEvent, explorer_type: MenuItem) -> Result<()> {
        // Check for Ctrl modifier for background execution
        let is_ctrl = key.modifiers.contains(crossterm::event::KeyModifiers::CONTROL);
        
        match self.input_mode {
            InputMode::Normal => match key.code {
                KeyCode::Esc | KeyCode::Char('q') => {
                    self.view = View::MainMenu;
                    self.search_query.clear();
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    if self.selected_item > 0 {
                        self.selected_item -= 1;
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    let items = self.filtered_explorer_items();
                    if self.selected_item < items.len().saturating_sub(1) {
                        self.selected_item += 1;
                    }
                }
                KeyCode::Enter => {
                    self.activate_explorer_item_with_mode(explorer_type, is_ctrl).await?;
                }
                KeyCode::Char('/') => {
                    self.input_mode = InputMode::Search;
                    self.search_cursor = self.search_query.len();
                }
                KeyCode::Char('r') => {
                    self.refresh_explorer(explorer_type).await?;
                }
                KeyCode::Char('e') => {
                    // Echo/Subscribe for topics
                    if explorer_type == MenuItem::Topics {
                        if let Some(ExplorerItem::Topic(topic)) = self.filtered_explorer_items().get(self.selected_item) {
                            let name = topic.name.clone();
                            self.view = View::TopicSubscriber(name);
                        }
                    }
                }
                KeyCode::Char('p') => {
                    // Publish for topics
                    if explorer_type == MenuItem::Topics {
                        if let Some(ExplorerItem::Topic(topic)) = self.filtered_explorer_items().get(self.selected_item) {
                            let name = topic.name.clone();
                            let msg_type = topic.msg_type.clone();
                            self.view = View::TopicPublisher(name, msg_type);
                        }
                    }
                }
                KeyCode::Char('c') => {
                    // Call for services
                    if explorer_type == MenuItem::Services {
                        if let Some(ExplorerItem::Service(service)) = self.filtered_explorer_items().get(self.selected_item) {
                            let name = service.name.clone();
                            let srv_type = service.srv_type.clone();
                            // Get interface definition
                            let interface_def = self.ros2_introspection
                                .interface_show(&srv_type)
                                .unwrap_or_default();
                            let state = ServiceClientState::new(name, srv_type, &interface_def);
                            self.view = View::ServiceClient(state);
                        }
                    }
                }
                KeyCode::Char('b') => {
                    // Build package (for Packages and Nodes explorers)
                    if explorer_type == MenuItem::Packages {
                        if let Some(ExplorerItem::Package(pkg)) = self.filtered_explorer_items().get(self.selected_item) {
                            let pkg_name = pkg.name.clone();
                            self.build_package(&pkg_name).await?;
                        }
                    } else if explorer_type == MenuItem::Nodes {
                        if let Some(ExplorerItem::Node(pkg_name, _)) = self.filtered_explorer_items().get(self.selected_item) {
                            let pkg_name = pkg_name.clone();
                            self.build_package(&pkg_name).await?;
                        }
                    }
                }
                KeyCode::Char('i') => {
                    // Show interface info
                    self.show_item_info().await?;
                }
                KeyCode::Char('?') | KeyCode::F(1) => {
                    self.view = View::Help;
                }
                _ => {}
            },
            InputMode::Search => match key.code {
                KeyCode::Esc => {
                    self.input_mode = InputMode::Normal;
                }
                KeyCode::Enter => {
                    self.input_mode = InputMode::Normal;
                    self.selected_item = 0;
                }
                KeyCode::Char(c) => {
                    self.search_query.insert(self.search_cursor, c);
                    self.search_cursor += 1;
                    self.selected_item = 0;
                }
                KeyCode::Backspace => {
                    if self.search_cursor > 0 {
                        self.search_cursor -= 1;
                        self.search_query.remove(self.search_cursor);
                        self.selected_item = 0;
                    }
                }
                KeyCode::Left => {
                    if self.search_cursor > 0 {
                        self.search_cursor -= 1;
                    }
                }
                KeyCode::Right => {
                    if self.search_cursor < self.search_query.len() {
                        self.search_cursor += 1;
                    }
                }
                _ => {}
            },
            _ => {}
        }
        Ok(())
    }

    /// Activate selected explorer item
    async fn activate_explorer_item(&mut self, explorer_type: MenuItem) -> Result<()> {
        self.activate_explorer_item_with_mode(explorer_type, false).await
    }

    /// Activate selected explorer item with optional background mode (Ctrl+Enter)
    async fn activate_explorer_item_with_mode(&mut self, explorer_type: MenuItem, background: bool) -> Result<()> {
        let items = self.filtered_explorer_items();
        if let Some(item) = items.get(self.selected_item) {
            match item {
                ExplorerItem::Package(pkg) => {
                    // Open package detail view (folder-like explorer)
                    let state = PackageDetailState::new(pkg);
                    self.view = View::PackageDetail(state);
                }
                ExplorerItem::Node(package, node) => {
                    // Run the node
                    let cmd = format!("ros2 run {} {}", package, node);
                    if background {
                        self.run_in_background(&cmd).await?;
                    } else {
                        self.add_command(&cmd).await?;
                        self.view = View::ProcessManager;
                    }
                }
                ExplorerItem::LaunchFile(package, path) => {
                    // Launch the file
                    let filename = path.file_name().unwrap_or_default().to_string_lossy();
                    let cmd = format!("ros2 launch {} {}", package, filename);
                    if background {
                        self.run_in_background(&cmd).await?;
                    } else {
                        self.add_command(&cmd).await?;
                        self.view = View::ProcessManager;
                    }
                }
                ExplorerItem::Topic(topic) => {
                    // Subscribe to topic
                    self.view = View::TopicSubscriber(topic.name.clone());
                }
                ExplorerItem::Service(service) => {
                    // Call service with interactive client
                    let interface_def = self.ros2_introspection.interface_show(&service.srv_type)
                        .unwrap_or_default();
                    let state = ServiceClientState::new(
                        service.name.clone(),
                        service.srv_type.clone(),
                        &interface_def,
                    );
                    self.view = View::ServiceClient(state);
                }
                ExplorerItem::Action(action) => {
                    self.set_status(format!("Action: {} [{}]", action.name, action.action_type));
                }
                ExplorerItem::RunningNode(node) => {
                    // Show node info
                    if let Ok(info) = self.ros2_introspection.node_info(&format!("{}{}", node.namespace, node.name)) {
                        self.set_status(info.lines().next().unwrap_or("").to_string());
                    }
                }
            }
        }
        Ok(())
    }

    /// Handle process manager key events
    async fn handle_process_manager_key(&mut self, key: KeyEvent) -> Result<()> {
        // Handle LogSelect mode - for selecting and copying log lines
        if self.input_mode == InputMode::LogSelect {
            match key.code {
                KeyCode::Esc | KeyCode::Char('q') => {
                    self.input_mode = InputMode::Normal;
                    self.log_select_start = None;
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    if self.log_select_end > 0 {
                        self.log_select_end -= 1;
                        // Auto scroll
                        if self.log_select_end < self.log_scroll {
                            self.log_scroll = self.log_select_end;
                        }
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    let max = self.log_buffer.len().saturating_sub(1);
                    if self.log_select_end < max {
                        self.log_select_end += 1;
                        // Auto scroll (assuming ~20 visible lines)
                        if self.log_select_end > self.log_scroll + 15 {
                            self.log_scroll = self.log_select_end.saturating_sub(15);
                        }
                    }
                }
                KeyCode::Char(' ') | KeyCode::Char('v') => {
                    // Start/toggle selection
                    if self.log_select_start.is_none() {
                        self.log_select_start = Some(self.log_select_end);
                    } else {
                        self.log_select_start = None;
                    }
                }
                KeyCode::Char('y') | KeyCode::Char('c') | KeyCode::Enter => {
                    // Copy selected lines to clipboard
                    self.copy_selected_logs()?;
                    self.input_mode = InputMode::Normal;
                    self.log_select_start = None;
                }
                KeyCode::Char('Y') | KeyCode::Char('C') => {
                    // Copy all logs
                    self.copy_all_logs()?;
                    self.input_mode = InputMode::Normal;
                    self.log_select_start = None;
                }
                KeyCode::PageUp => {
                    self.log_select_end = self.log_select_end.saturating_sub(10);
                    self.log_scroll = self.log_scroll.saturating_sub(10);
                }
                KeyCode::PageDown => {
                    let max = self.log_buffer.len().saturating_sub(1);
                    self.log_select_end = (self.log_select_end + 10).min(max);
                    self.log_scroll = (self.log_scroll + 10).min(max);
                }
                KeyCode::Home => {
                    self.log_select_end = 0;
                    self.log_scroll = 0;
                }
                KeyCode::End => {
                    self.log_select_end = self.log_buffer.len().saturating_sub(1);
                    self.log_scroll = self.log_select_end.saturating_sub(15);
                }
                _ => {}
            }
            return Ok(());
        }
        
        // Handle input mode separately - only Escape exits
        if self.input_mode == InputMode::Input {
            match key.code {
                KeyCode::Esc => {
                    self.input_mode = InputMode::Normal;
                    self.input_buffer.clear();
                    self.input_cursor = 0;
                    self.show_suggestions = false;
                    self.selected_suggestion = 0;
                }
                KeyCode::Enter => {
                    if self.show_suggestions && !self.suggestions.is_empty() {
                        // Use selected suggestion
                        let (name, cmd) = self.suggestions[self.selected_suggestion].clone();
                        self.input_buffer.clear();
                        self.input_cursor = 0;
                        self.show_suggestions = false;
                        self.input_mode = InputMode::Normal;
                        // Run the command, handle error gracefully
                        if let Err(e) = self.add_command_with_name(&cmd, &name).await {
                            self.set_status(format!("Error: {}", e));
                        }
                    } else if !self.input_buffer.is_empty() {
                        let cmd = self.input_buffer.clone();
                        self.input_buffer.clear();
                        self.input_cursor = 0;
                        self.input_mode = InputMode::Normal;
                        if let Err(e) = self.add_command(&cmd).await {
                            self.set_status(format!("Error: {}", e));
                        }
                    } else {
                        self.input_mode = InputMode::Normal;
                    }
                }
                KeyCode::Tab => {
                    // Toggle suggestions or cycle through them
                    if !self.show_suggestions {
                        self.update_suggestions();
                        self.show_suggestions = !self.suggestions.is_empty();
                        self.selected_suggestion = 0;
                    } else if !self.suggestions.is_empty() {
                        self.selected_suggestion = (self.selected_suggestion + 1) % self.suggestions.len();
                    }
                }
                KeyCode::BackTab => {
                    // Cycle suggestions backwards
                    if self.show_suggestions && !self.suggestions.is_empty() {
                        if self.selected_suggestion > 0 {
                            self.selected_suggestion -= 1;
                        } else {
                            self.selected_suggestion = self.suggestions.len() - 1;
                        }
                    }
                }
                KeyCode::Up => {
                    if self.show_suggestions && !self.suggestions.is_empty() {
                        if self.selected_suggestion > 0 {
                            self.selected_suggestion -= 1;
                        } else {
                            self.selected_suggestion = self.suggestions.len() - 1;
                        }
                    }
                }
                KeyCode::Down => {
                    if self.show_suggestions && !self.suggestions.is_empty() {
                        self.selected_suggestion = (self.selected_suggestion + 1) % self.suggestions.len();
                    }
                }
                KeyCode::Char(c) => {
                    self.input_buffer.insert(self.input_cursor, c);
                    self.input_cursor += 1;
                    self.update_suggestions();
                    if !self.suggestions.is_empty() {
                        self.show_suggestions = true;
                        self.selected_suggestion = 0;
                    }
                }
                KeyCode::Backspace => {
                    if self.input_cursor > 0 {
                        self.input_cursor -= 1;
                        self.input_buffer.remove(self.input_cursor);
                        self.update_suggestions();
                        self.show_suggestions = !self.suggestions.is_empty();
                        if self.selected_suggestion >= self.suggestions.len() {
                            self.selected_suggestion = self.suggestions.len().saturating_sub(1);
                        }
                    }
                }
                KeyCode::Delete => {
                    if self.input_cursor < self.input_buffer.len() {
                        self.input_buffer.remove(self.input_cursor);
                        self.update_suggestions();
                    }
                }
                KeyCode::Left => {
                    if self.input_cursor > 0 {
                        self.input_cursor -= 1;
                    }
                }
                KeyCode::Right => {
                    if self.input_cursor < self.input_buffer.len() {
                        self.input_cursor += 1;
                    }
                }
                KeyCode::Home => {
                    self.input_cursor = 0;
                }
                KeyCode::End => {
                    self.input_cursor = self.input_buffer.len();
                }
                _ => {}
            }
            return Ok(());
        }

        // Normal mode handling
        match key.code {
            KeyCode::Esc | KeyCode::Char('q') => {
                self.view = View::MainMenu;
            }
            KeyCode::Up | KeyCode::Char('k') => {
                if self.selected_process > 0 {
                    self.selected_process -= 1;
                    self.update_log_buffer();
                }
            }
            KeyCode::Down | KeyCode::Char('j') => {
                if self.selected_process < self.processes.len().saturating_sub(1) {
                    self.selected_process += 1;
                    self.update_log_buffer();
                }
            }
            KeyCode::Char('n') | KeyCode::Char('a') => {
                self.input_mode = InputMode::Input;
                self.input_buffer.clear();
                self.input_cursor = 0;
                self.update_suggestions();
                self.show_suggestions = !self.suggestions.is_empty();
                self.selected_suggestion = 0;
            }
            KeyCode::Char('e') => {
                // Edit selected process command
                if let Some(process) = self.processes.get(self.selected_process) {
                    let state = EditProcessCommandState::from_process(self.selected_process, process);
                    self.view = View::EditProcessCommand(state);
                }
            }
            KeyCode::Char('s') => {
                self.toggle_selected_process().await?;
            }
            KeyCode::Char('r') => {
                self.restart_selected_process().await?;
            }
            KeyCode::Char('d') | KeyCode::Delete => {
                if let Some(process) = self.processes.get(self.selected_process) {
                    let id = process.id;
                    let name = process.name.clone();
                    self.show_confirm(
                        format!("Delete process '{}'?", name),
                        ConfirmAction::DeleteProcess(id),
                    );
                }
            }
            KeyCode::Char('S') => {
                if self.has_running_processes() {
                    self.show_confirm(
                        "Stop all processes?".to_string(),
                        ConfirmAction::StopAllProcesses,
                    );
                }
            }
            KeyCode::PageUp => {
                self.log_scroll = self.log_scroll.saturating_sub(20);
            }
            KeyCode::PageDown => {
                let max = self.log_buffer.len().saturating_sub(1);
                self.log_scroll = (self.log_scroll + 20).min(max);
            }
            KeyCode::Home => {
                self.log_scroll = 0;
            }
            KeyCode::End => {
                self.log_scroll = self.log_buffer.len().saturating_sub(1);
            }
            KeyCode::Char('o') | KeyCode::Char('c') => {
                // Enter log select mode for copying output
                if !self.log_buffer.is_empty() {
                    self.input_mode = InputMode::LogSelect;
                    self.log_select_end = self.log_scroll;
                    self.log_select_start = None;
                    self.set_status("Select mode: ↑↓ move, Space start selection, y/Enter copy, Y copy all, Esc cancel".to_string());
                }
            }
            KeyCode::Char('?') | KeyCode::F(1) => {
                self.view = View::Help;
            }
            _ => {}
        }
        Ok(())
    }

    /// Copy selected log lines to clipboard
    fn copy_selected_logs(&mut self) -> Result<()> {
        let lines: Vec<&str> = self.log_buffer.lines()
            .map(|l| l.content.as_str())
            .collect();
        
        if lines.is_empty() {
            return Ok(());
        }
        
        let (start, end) = if let Some(sel_start) = self.log_select_start {
            (sel_start.min(self.log_select_end), sel_start.max(self.log_select_end))
        } else {
            (self.log_select_end, self.log_select_end)
        };
        
        let selected: Vec<&str> = lines.iter()
            .skip(start)
            .take(end - start + 1)
            .copied()
            .collect();
        
        let text = selected.join("\n");
        self.copy_to_clipboard(&text)?;
        
        let count = end - start + 1;
        self.set_status(format!("Copied {} line{} to clipboard", count, if count > 1 { "s" } else { "" }));
        Ok(())
    }

    /// Copy all log lines to clipboard
    fn copy_all_logs(&mut self) -> Result<()> {
        let lines: Vec<&str> = self.log_buffer.lines()
            .map(|l| l.content.as_str())
            .collect();
        
        if lines.is_empty() {
            self.set_status("No logs to copy".to_string());
            return Ok(());
        }
        
        let text = lines.join("\n");
        self.copy_to_clipboard(&text)?;
        
        self.set_status(format!("Copied all {} lines to clipboard", lines.len()));
        Ok(())
    }

    /// Copy text to system clipboard using arboard crate
    fn copy_to_clipboard(&self, text: &str) -> Result<()> {
        use arboard::Clipboard;
        
        let mut clipboard = Clipboard::new()
            .map_err(|e| anyhow::anyhow!("Failed to access clipboard: {}", e))?;
        
        clipboard.set_text(text.to_string())
            .map_err(|e| anyhow::anyhow!("Failed to copy to clipboard: {}", e))?;
        
        Ok(())
    }

    /// Handle package detail view (folder-like explorer for nodes and launch files)
    async fn handle_package_detail_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state data first to avoid borrow issues
        let state_data = if let View::PackageDetail(state) = &self.view {
            Some((
                state.package_name.clone(),
                state.package_path.clone(),
                state.selected_tab,
                state.selected_item,
                state.items_count(),
                state.nodes.clone(),
                state.launch_files.clone(),
            ))
        } else {
            return Ok(());
        };
        
        let (pkg_name, pkg_path, selected_tab, selected_item, items_count, nodes, launch_files) = state_data.unwrap();

        match key.code {
            KeyCode::Esc | KeyCode::Char('q') => {
                self.view = View::PackageExplorer;
            }
            KeyCode::Tab => {
                // Cycle through tabs: 0 -> 1 -> 2 -> 3 -> 0
                if let View::PackageDetail(ref mut state) = self.view {
                    state.selected_tab = (state.selected_tab + 1) % PackageDetailState::tab_count();
                    state.selected_item = 0;
                }
            }
            KeyCode::BackTab => {
                // Reverse cycle through tabs
                if let View::PackageDetail(ref mut state) = self.view {
                    state.selected_tab = if state.selected_tab == 0 {
                        PackageDetailState::tab_count() - 1
                    } else {
                        state.selected_tab - 1
                    };
                    state.selected_item = 0;
                }
            }
            KeyCode::Right => {
                if let View::PackageDetail(ref mut state) = self.view {
                    if state.selected_tab < PackageDetailState::tab_count() - 1 {
                        state.selected_tab += 1;
                        state.selected_item = 0;
                    }
                }
            }
            KeyCode::Left => {
                if let View::PackageDetail(ref mut state) = self.view {
                    if state.selected_tab > 0 {
                        state.selected_tab -= 1;
                        state.selected_item = 0;
                    }
                }
            }
            KeyCode::Up | KeyCode::Char('k') => {
                if let View::PackageDetail(ref mut state) = self.view {
                    if state.selected_item > 0 {
                        state.selected_item -= 1;
                    }
                }
            }
            KeyCode::Down | KeyCode::Char('j') => {
                if let View::PackageDetail(ref mut state) = self.view {
                    let max = state.items_count().saturating_sub(1);
                    if state.selected_item < max {
                        state.selected_item += 1;
                    }
                }
            }
            KeyCode::Enter => {
                match selected_tab {
                    0 => {
                        // Run node
                        if let Some(node) = nodes.get(selected_item) {
                            let cmd = format!("ros2 run {} {}", pkg_name, node);
                            self.add_command(&cmd).await?;
                            self.view = View::ProcessManager;
                        } else if items_count == 0 {
                            self.set_status("No nodes to run".to_string());
                        }
                    }
                    1 => {
                        // Launch file
                        if let Some((filename, _)) = launch_files.get(selected_item) {
                            let cmd = format!("ros2 launch {} {}", pkg_name, filename);
                            self.add_command(&cmd).await?;
                            self.view = View::ProcessManager;
                        } else if items_count == 0 {
                            self.set_status("No launch files to run".to_string());
                        }
                    }
                    2 => {
                        // + New Node - open create node wizard
                        let state = CreateNodeState {
                            package: pkg_name.clone(),
                            node_name: String::new(),
                            node_type: 0,
                            template: 0,
                            active_field: 1, // Start at node_name field
                            field_cursor: 0,
                        };
                        self.view = View::CreateNode(state);
                    }
                    3 => {
                        // + New Launch - open create launch wizard
                        let state = CreateLaunchState {
                            package: pkg_name.clone(),
                            name: String::new(),
                            format: 0,
                            selected_nodes: Vec::new(),
                            active_field: 1, // Start at name field
                            field_cursor: 0,
                        };
                        self.view = View::CreateLaunch(state);
                    }
                    _ => {}
                }
            }
            KeyCode::Char('R') => {
                // Run in background (Shift+R)
                match selected_tab {
                    0 => {
                        if let Some(node) = nodes.get(selected_item) {
                            let cmd = format!("ros2 run {} {}", pkg_name, node);
                            self.run_in_background(&cmd).await?;
                        }
                    }
                    1 => {
                        if let Some((filename, _)) = launch_files.get(selected_item) {
                            let cmd = format!("ros2 launch {} {}", pkg_name, filename);
                            self.run_in_background(&cmd).await?;
                        }
                    }
                    _ => {}
                }
            }
            KeyCode::Char('b') => {
                // Build package
                self.build_package(&pkg_name).await?;
            }
            KeyCode::Char('?') | KeyCode::F(1) => {
                self.view = View::Help;
            }
            _ => {}
        }
        Ok(())
    }

    /// Run command in background and show notification (without switching to ProcessManager)
    async fn run_in_background(&mut self, command: &str) -> Result<()> {
        self.add_command(command).await?;
        self.set_status(format!("🚀 Started in background: {}", command));
        // Don't switch view - stay in current view
        Ok(())
    }

    /// Handle create package wizard
    async fn handle_create_package_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state data first to avoid borrow issues
        let state_data = if let View::CreatePackage(state) = &self.view {
            Some((
                state.active_field,
                state.build_type,
                state.name.clone(),
                state.description.clone(),
                state.maintainer.clone(),
                state.email.clone(),
                state.dependencies.clone(),
                state.initial_node.clone(),
            ))
        } else {
            None
        };

        if state_data.is_none() {
            return Ok(());
        }

        match key.code {
            KeyCode::Esc => {
                self.view = View::MainMenu;
            }
            KeyCode::Tab => {
                if let View::CreatePackage(ref mut state) = self.view {
                    state.active_field = (state.active_field + 1) % 7;
                    state.field_cursor = 0;
                }
            }
            KeyCode::BackTab => {
                if let View::CreatePackage(ref mut state) = self.view {
                    state.active_field = if state.active_field == 0 { 6 } else { state.active_field - 1 };
                    state.field_cursor = 0;
                }
            }
            KeyCode::Up => {
                if let View::CreatePackage(ref mut state) = self.view {
                    if state.active_field == 1 {
                        state.build_type = if state.build_type == 0 { 1 } else { 0 };
                    } else {
                        state.active_field = if state.active_field == 0 { 6 } else { state.active_field - 1 };
                        state.field_cursor = 0;
                    }
                }
            }
            KeyCode::Down => {
                if let View::CreatePackage(ref mut state) = self.view {
                    if state.active_field == 1 {
                        state.build_type = if state.build_type == 0 { 1 } else { 0 };
                    } else {
                        state.active_field = (state.active_field + 1) % 7;
                        state.field_cursor = 0;
                    }
                }
            }
            KeyCode::Enter => {
                // Create the package with cloned data
                let (_, build_type_idx, name, description, maintainer, email, dependencies, initial_node) = state_data.unwrap();
                let src_path = self.workspace_path.join("src");
                let build_type = if build_type_idx == 0 { "ament_python" } else { "ament_cmake" };
                let deps: Vec<String> = dependencies
                    .split_whitespace()
                    .map(|s| s.to_string())
                    .collect();
                let initial_node_opt = if initial_node.is_empty() { None } else { Some(initial_node.as_str()) };

                match Ros2Templates::create_package(
                    &src_path,
                    &name,
                    build_type,
                    &description,
                    &maintainer,
                    &email,
                    &deps,
                    initial_node_opt,
                ) {
                    Ok(_) => {
                        self.set_status(format!("Package '{}' created successfully!", name));
                        self.ros2_workspace = Ros2Workspace::scan(&self.workspace_path).ok();
                        self.view = View::MainMenu;
                    }
                    Err(e) => {
                        self.set_status(format!("Failed to create package: {}", e));
                    }
                }
            }
            KeyCode::Char(c) => {
                if let View::CreatePackage(ref mut state) = self.view {
                    let field = match state.active_field {
                        0 => &mut state.name,
                        2 => &mut state.description,
                        3 => &mut state.maintainer,
                        4 => &mut state.email,
                        5 => &mut state.dependencies,
                        6 => &mut state.initial_node,
                        _ => return Ok(()),
                    };
                    field.insert(state.field_cursor, c);
                    state.field_cursor += 1;
                }
            }
            KeyCode::Backspace => {
                if let View::CreatePackage(ref mut state) = self.view {
                    if state.field_cursor > 0 {
                        state.field_cursor -= 1;
                        let field = match state.active_field {
                            0 => &mut state.name,
                            2 => &mut state.description,
                            3 => &mut state.maintainer,
                            4 => &mut state.email,
                            5 => &mut state.dependencies,
                            6 => &mut state.initial_node,
                            _ => return Ok(()),
                        };
                        field.remove(state.field_cursor);
                    }
                }
            }
            KeyCode::Delete => {
                if let View::CreatePackage(ref mut state) = self.view {
                    let field = match state.active_field {
                        0 => &mut state.name,
                        2 => &mut state.description,
                        3 => &mut state.maintainer,
                        4 => &mut state.email,
                        5 => &mut state.dependencies,
                        6 => &mut state.initial_node,
                        _ => return Ok(()),
                    };
                    if state.field_cursor < field.len() {
                        field.remove(state.field_cursor);
                    }
                }
            }
            KeyCode::Left => {
                if let View::CreatePackage(ref mut state) = self.view {
                    if state.field_cursor > 0 {
                        state.field_cursor -= 1;
                    }
                }
            }
            KeyCode::Right => {
                if let View::CreatePackage(ref mut state) = self.view {
                    let field_len = match state.active_field {
                        0 => state.name.len(),
                        2 => state.description.len(),
                        3 => state.maintainer.len(),
                        4 => state.email.len(),
                        5 => state.dependencies.len(),
                        6 => state.initial_node.len(),
                        _ => 0,
                    };
                    if state.field_cursor < field_len {
                        state.field_cursor += 1;
                    }
                }
            }
            KeyCode::Home => {
                if let View::CreatePackage(ref mut state) = self.view {
                    state.field_cursor = 0;
                }
            }
            KeyCode::End => {
                if let View::CreatePackage(ref mut state) = self.view {
                    state.field_cursor = match state.active_field {
                        0 => state.name.len(),
                        2 => state.description.len(),
                        3 => state.maintainer.len(),
                        4 => state.email.len(),
                        5 => state.dependencies.len(),
                        6 => state.initial_node.len(),
                        _ => 0,
                    };
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle create node wizard
    async fn handle_create_node_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state data first to avoid borrow issues
        let state_data = if let View::CreateNode(state) = &self.view {
            Some((
                state.active_field,
                state.package.clone(),
                state.node_name.clone(),
                state.node_type,
            ))
        } else {
            None
        };

        if state_data.is_none() {
            return Ok(());
        }

        match key.code {
            KeyCode::Esc => {
                self.view = View::MainMenu;
            }
            KeyCode::Tab => {
                if let View::CreateNode(ref mut state) = self.view {
                    state.active_field = (state.active_field + 1) % 4;
                    state.field_cursor = 0;
                }
            }
            KeyCode::BackTab => {
                if let View::CreateNode(ref mut state) = self.view {
                    state.active_field = if state.active_field == 0 { 3 } else { state.active_field - 1 };
                    state.field_cursor = 0;
                }
            }
            KeyCode::Up | KeyCode::Down => {
                if let View::CreateNode(ref mut state) = self.view {
                    match state.active_field {
                        2 => state.node_type = if state.node_type == 0 { 1 } else { 0 },
                        3 => state.template = (state.template + 1) % 4,
                        _ => {}
                    }
                }
            }
            KeyCode::Enter => {
                let (_, package, node_name, node_type) = state_data.unwrap();
                // Get package info
                let pkg_info = self.ros2_workspace.as_ref()
                    .and_then(|ws| ws.get_package(&package))
                    .map(|pkg| (pkg.path.clone(), pkg.name.clone()));

                if let Some((pkg_path, pkg_name)) = pkg_info {
                    let class_name = Ros2Templates::to_class_name(&node_name);
                    let content = match node_type {
                        0 => Ros2Templates::python_node(&node_name, &class_name),
                        _ => Ros2Templates::cpp_node(&node_name, &class_name),
                    };
                    
                    let file_path = if node_type == 0 {
                        pkg_path.join(&pkg_name).join(format!("{}.py", node_name))
                    } else {
                        pkg_path.join("src").join(format!("{}.cpp", node_name))
                    };

                    match std::fs::write(&file_path, content) {
                        Ok(_) => {
                            self.set_status(format!("Node '{}' created at {:?}", node_name, file_path));
                            self.view = View::MainMenu;
                        }
                        Err(e) => {
                            self.set_status(format!("Failed to create node: {}", e));
                        }
                    }
                } else {
                    self.set_status("Package not found".to_string());
                }
            }
            KeyCode::Char(c) => {
                if let View::CreateNode(ref mut state) = self.view {
                    let field = match state.active_field {
                        0 => &mut state.package,
                        1 => &mut state.node_name,
                        _ => return Ok(()),
                    };
                    field.insert(state.field_cursor, c);
                    state.field_cursor += 1;
                }
            }
            KeyCode::Backspace => {
                if let View::CreateNode(ref mut state) = self.view {
                    if state.field_cursor > 0 {
                        state.field_cursor -= 1;
                        let field = match state.active_field {
                            0 => &mut state.package,
                            1 => &mut state.node_name,
                            _ => return Ok(()),
                        };
                        field.remove(state.field_cursor);
                    }
                }
            }
            KeyCode::Delete => {
                if let View::CreateNode(ref mut state) = self.view {
                    let field = match state.active_field {
                        0 => &mut state.package,
                        1 => &mut state.node_name,
                        _ => return Ok(()),
                    };
                    if state.field_cursor < field.len() {
                        field.remove(state.field_cursor);
                    }
                }
            }
            KeyCode::Left => {
                if let View::CreateNode(ref mut state) = self.view {
                    if state.field_cursor > 0 {
                        state.field_cursor -= 1;
                    }
                }
            }
            KeyCode::Right => {
                if let View::CreateNode(ref mut state) = self.view {
                    let field_len = match state.active_field {
                        0 => state.package.len(),
                        1 => state.node_name.len(),
                        _ => 0,
                    };
                    if state.field_cursor < field_len {
                        state.field_cursor += 1;
                    }
                }
            }
            KeyCode::Home => {
                if let View::CreateNode(ref mut state) = self.view {
                    state.field_cursor = 0;
                }
            }
            KeyCode::End => {
                if let View::CreateNode(ref mut state) = self.view {
                    state.field_cursor = match state.active_field {
                        0 => state.package.len(),
                        1 => state.node_name.len(),
                        _ => 0,
                    };
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle create launch file wizard
    async fn handle_create_launch_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state data first to avoid borrow issues
        let state_data = if let View::CreateLaunch(state) = &self.view {
            Some((
                state.package.clone(),
                state.name.clone(),
                state.format,
                state.selected_nodes.clone(),
                state.active_field,
            ))
        } else {
            None
        };

        if state_data.is_none() {
            return Ok(());
        }

        match key.code {
            KeyCode::Esc => {
                self.view = View::MainMenu;
            }
            KeyCode::Tab => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    state.active_field = (state.active_field + 1) % 3;
                    state.field_cursor = 0;
                }
            }
            KeyCode::BackTab => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    state.active_field = if state.active_field == 0 { 2 } else { state.active_field - 1 };
                    state.field_cursor = 0;
                }
            }
            KeyCode::Up | KeyCode::Down => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    if state.active_field == 2 {
                        state.format = if state.format == 0 { 1 } else { 0 };
                    }
                }
            }
            KeyCode::Enter => {
                let (package, name, format, selected_nodes, _) = state_data.unwrap();
                // Get package path
                let pkg_path = self.ros2_workspace.as_ref()
                    .and_then(|ws| ws.get_package(&package))
                    .map(|pkg| pkg.path.clone());

                if let Some(pkg_path) = pkg_path {
                    let nodes: Vec<(&str, &str)> = selected_nodes
                        .iter()
                        .map(|(p, n)| (p.as_str(), n.as_str()))
                        .collect();

                    let (content, ext) = if format == 0 {
                        (Ros2Templates::launch_py(&package, &nodes), "launch.py")
                    } else {
                        (Ros2Templates::launch_xml(&nodes), "launch.xml")
                    };

                    let launch_dir = pkg_path.join("launch");
                    std::fs::create_dir_all(&launch_dir).ok();
                    let file_path = launch_dir.join(format!("{}.{}", name, ext));

                    match std::fs::write(&file_path, content) {
                        Ok(_) => {
                            self.set_status(format!("Launch file created at {:?}", file_path));
                            self.view = View::MainMenu;
                        }
                        Err(e) => {
                            self.set_status(format!("Failed to create launch file: {}", e));
                        }
                    }
                } else {
                    self.set_status("Package not found".to_string());
                }
            }
            KeyCode::Char(c) => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    let field = match state.active_field {
                        0 => &mut state.package,
                        1 => &mut state.name,
                        _ => return Ok(()),
                    };
                    field.insert(state.field_cursor, c);
                    state.field_cursor += 1;
                }
            }
            KeyCode::Backspace => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    if state.field_cursor > 0 {
                        state.field_cursor -= 1;
                        let field = match state.active_field {
                            0 => &mut state.package,
                            1 => &mut state.name,
                            _ => return Ok(()),
                        };
                        field.remove(state.field_cursor);
                    }
                }
            }
            KeyCode::Delete => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    let field = match state.active_field {
                        0 => &mut state.package,
                        1 => &mut state.name,
                        _ => return Ok(()),
                    };
                    if state.field_cursor < field.len() {
                        field.remove(state.field_cursor);
                    }
                }
            }
            KeyCode::Left => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    if state.field_cursor > 0 {
                        state.field_cursor -= 1;
                    }
                }
            }
            KeyCode::Right => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    let field_len = match state.active_field {
                        0 => state.package.len(),
                        1 => state.name.len(),
                        _ => 0,
                    };
                    if state.field_cursor < field_len {
                        state.field_cursor += 1;
                    }
                }
            }
            KeyCode::Home => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    state.field_cursor = 0;
                }
            }
            KeyCode::End => {
                if let View::CreateLaunch(ref mut state) = self.view {
                    state.field_cursor = match state.active_field {
                        0 => state.package.len(),
                        1 => state.name.len(),
                        _ => 0,
                    };
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle topic subscriber view
    async fn handle_subscriber_key(&mut self, key: KeyEvent, topic: &str) -> Result<()> {
        match key.code {
            KeyCode::Esc | KeyCode::Char('q') => {
                self.view = View::TopicExplorer;
            }
            KeyCode::Enter => {
                // Start echo process
                let cmd = self.ros2_introspection.topic_echo_cmd(topic);
                self.add_command(&cmd).await?;
                self.view = View::ProcessManager;
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle topic publisher view
    async fn handle_publisher_key(&mut self, key: KeyEvent, topic: &str, msg_type: &str) -> Result<()> {
        match key.code {
            KeyCode::Esc | KeyCode::Char('q') => {
                self.view = View::TopicExplorer;
            }
            KeyCode::Enter => {
                // Publish message
                let cmd = self.ros2_introspection.topic_pub_cmd(topic, msg_type, "{}");
                self.add_command(&cmd).await?;
                self.view = View::ProcessManager;
            }
            KeyCode::Char(c) => {
                self.input_buffer.push(c);
            }
            KeyCode::Backspace => {
                self.input_buffer.pop();
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle service client view with interactive field input
    async fn handle_service_client_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state first to avoid borrow issues
        let state_data = if let View::ServiceClient(state) = &self.view {
            Some((
                state.service_name.clone(),
                state.srv_type.clone(),
                state.fields.len(),
                state.active_field,
            ))
        } else {
            return Ok(());
        };

        let (service_name, srv_type, field_count, active_field) = state_data.unwrap();

        match key.code {
            KeyCode::Esc => {
                self.view = View::ServiceExplorer;
            }
            KeyCode::Tab | KeyCode::Down => {
                if let View::ServiceClient(ref mut state) = self.view {
                    if field_count > 0 {
                        state.active_field = (state.active_field + 1) % field_count;
                    }
                }
            }
            KeyCode::Up => {
                if let View::ServiceClient(ref mut state) = self.view {
                    if field_count > 0 {
                        state.active_field = state.active_field.checked_sub(1).unwrap_or(field_count - 1);
                    }
                }
            }
            KeyCode::Enter => {
                // Call service with current field values
                let yaml = if let View::ServiceClient(ref state) = self.view {
                    state.to_yaml()
                } else {
                    "{}".to_string()
                };
                let cmd = self.ros2_introspection.service_call_cmd(&service_name, &srv_type, &yaml);
                self.add_command(&cmd).await?;
                self.view = View::ProcessManager;
            }
            KeyCode::Char(c) => {
                if let View::ServiceClient(ref mut state) = self.view {
                    if let Some(field) = state.fields.get_mut(active_field) {
                        field.value.push(c);
                    }
                }
            }
            KeyCode::Backspace => {
                if let View::ServiceClient(ref mut state) = self.view {
                    if let Some(field) = state.fields.get_mut(active_field) {
                        field.value.pop();
                    }
                }
            }
            KeyCode::Delete => {
                if let View::ServiceClient(ref mut state) = self.view {
                    if let Some(field) = state.fields.get_mut(active_field) {
                        field.value.clear();
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle interface view (read-only)
    fn handle_interface_view_key(&mut self, key: KeyEvent) {
        match key.code {
            KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('i') => {
                // Return to previous explorer
                self.view = self.previous_view.take().map(|v| *v).unwrap_or(View::MainMenu);
            }
            _ => {}
        }
    }

    /// Handle help view
    fn handle_help_key(&mut self, key: KeyEvent) {
        match key.code {
            KeyCode::Esc | KeyCode::Char('q') | KeyCode::Char('?') | KeyCode::F(1) => {
                self.view = View::MainMenu;
            }
            _ => {}
        }
    }

    /// Handle confirm dialog
    async fn handle_confirm_key(&mut self, key: KeyEvent) -> Result<()> {
        if let View::Confirm(_, ref action) = self.view.clone() {
            match key.code {
                KeyCode::Char('y') | KeyCode::Char('Y') | KeyCode::Enter => {
                    self.execute_confirm_action(action).await?;
                    // Return to previous view or MainMenu
                    self.view = self.previous_view.take().map(|v| *v).unwrap_or(View::MainMenu);
                }
                KeyCode::Char('n') | KeyCode::Char('N') | KeyCode::Esc => {
                    // Return to previous view or MainMenu
                    self.view = self.previous_view.take().map(|v| *v).unwrap_or(View::MainMenu);
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Execute confirmed action
    async fn execute_confirm_action(&mut self, action: &ConfirmAction) -> Result<()> {
        match action {
            ConfirmAction::StopProcess(id) => {
                self.process_manager.stop_process(*id).await?;
            }
            ConfirmAction::StopAllProcesses => {
                self.process_manager.stop_all().await?;
            }
            ConfirmAction::DeleteProcess(id) => {
                self.process_manager.stop_process(*id).await?;
                self.processes.retain(|p| p.id != *id);
                if self.selected_process >= self.processes.len() && !self.processes.is_empty() {
                    self.selected_process = self.processes.len() - 1;
                }
            }
            ConfirmAction::DeletePackage(_name) => {
                // TODO: Implement package deletion
            }
            ConfirmAction::DeleteSavedCommand(idx) => {
                if *idx < self.config.saved_commands.len() {
                    self.config.saved_commands.remove(*idx);
                    self.config.save(None)?;
                    self.set_status("Command deleted".to_string());
                    // Update selection in settings view
                    self.view = View::Settings(SettingsState {
                        category: SettingsCategory::SavedCommands,
                        selected_item: if *idx > 0 { *idx } else { 0 },
                        editing: false,
                        input_buffer: String::new(),
                        category_focused: false,
                    });
                }
            }
            ConfirmAction::CleanWorkspace => {
                self.run_clean().await?;
            }
            ConfirmAction::Quit => {
                self.process_manager.stop_all().await?;
                self.should_quit = true;
            }
        }
        Ok(())
    }

    /// Handle settings key events
    async fn handle_settings_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state info first to avoid borrow issues
        let (is_editing, category_focused, category, selected_item, input_buffer) = {
            if let View::Settings(ref state) = self.view {
                (state.editing, state.category_focused, state.category, state.selected_item, state.input_buffer.clone())
            } else {
                return Ok(());
            }
        };
        
        let categories = SettingsCategory::all();
        let items = {
            if let View::Settings(ref state) = self.view {
                state.items_for_category()
            } else {
                return Ok(());
            }
        };
        
        // In editing mode
        if is_editing {
            match key.code {
                KeyCode::Esc => {
                    if let View::Settings(ref mut state) = self.view {
                        state.editing = false;
                        state.input_buffer.clear();
                    }
                }
                KeyCode::Enter => {
                    // Apply the edited value
                    let state_clone = if let View::Settings(ref state) = self.view {
                        state.clone()
                    } else {
                        return Ok(());
                    };
                    self.apply_setting_change(&state_clone)?;
                    if let View::Settings(ref mut state) = self.view {
                        state.editing = false;
                        state.input_buffer.clear();
                    }
                    self.config.save(None)?;
                    self.set_status("Settings saved".to_string());
                }
                KeyCode::Char(c) => {
                    if let View::Settings(ref mut state) = self.view {
                        state.input_buffer.push(c);
                    }
                }
                KeyCode::Backspace => {
                    if let View::Settings(ref mut state) = self.view {
                        state.input_buffer.pop();
                    }
                }
                _ => {}
            }
            return Ok(());
        }
        
        // In category sidebar
        if category_focused {
            match key.code {
                KeyCode::Esc | KeyCode::Char('q') => {
                    self.view = View::MainMenu;
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    if let View::Settings(ref mut state) = self.view {
                        let current_idx = categories.iter().position(|c| *c == state.category).unwrap_or(0);
                        if current_idx > 0 {
                            state.category = categories[current_idx - 1];
                            state.selected_item = 0;
                        }
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if let View::Settings(ref mut state) = self.view {
                        let current_idx = categories.iter().position(|c| *c == state.category).unwrap_or(0);
                        if current_idx < categories.len() - 1 {
                            state.category = categories[current_idx + 1];
                            state.selected_item = 0;
                        }
                    }
                }
                KeyCode::Right | KeyCode::Enter | KeyCode::Char('l') => {
                    // For SavedCommands, items is empty but we still want to focus right
                    let can_focus_right = !items.is_empty() || category == SettingsCategory::SavedCommands;
                    if can_focus_right {
                        if let View::Settings(ref mut state) = self.view {
                            state.category_focused = false;
                        }
                    }
                }
                KeyCode::Char('?') | KeyCode::F(1) => {
                    self.view = View::Help;
                }
                _ => {}
            }
        } else {
            // In items list
            match key.code {
                KeyCode::Esc | KeyCode::Left | KeyCode::Char('h') => {
                    if let View::Settings(ref mut state) = self.view {
                        state.category_focused = true;
                    }
                }
                KeyCode::Char('q') => {
                    self.view = View::MainMenu;
                }
                KeyCode::Up | KeyCode::Char('k') => {
                    if let View::Settings(ref mut state) = self.view {
                        if state.selected_item > 0 {
                            state.selected_item -= 1;
                        }
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if let View::Settings(ref mut state) = self.view {
                        let max_items = if state.category == SettingsCategory::SavedCommands {
                            // +1 for "Add Command" action, plus saved commands count
                            1 + self.config.saved_commands.len()
                        } else {
                            items.len()
                        };
                        if state.selected_item < max_items.saturating_sub(1) {
                            state.selected_item += 1;
                        }
                    }
                }
                KeyCode::Enter | KeyCode::Char(' ') => {
                    // Toggle/edit the selected item or handle saved command actions
                    self.toggle_or_edit_setting()?;
                }
                KeyCode::Char('e') => {
                    // Edit saved command (only for SavedCommands category)
                    self.edit_saved_command_at_selection()?;
                }
                KeyCode::Char('d') => {
                    // Delete saved command (only for SavedCommands category)
                    self.delete_saved_command_at_selection()?;
                }
                KeyCode::Char('r') | KeyCode::Char('R') => {
                    // Run saved command (only for SavedCommands category)
                    self.run_saved_command_at_selection().await?;
                }
                KeyCode::Char('?') | KeyCode::F(1) => {
                    self.view = View::Help;
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Toggle boolean settings or enter edit mode for other types
    fn toggle_or_edit_setting(&mut self) -> Result<()> {
        if let View::Settings(ref mut state) = self.view {
            // Handle SavedCommands category separately
            if state.category == SettingsCategory::SavedCommands {
                if state.selected_item == 0 {
                    // "Add Command" action
                    self.view = View::CreateSavedCommand(CreateSavedCommandState::default());
                } else {
                    // Edit an existing command
                    let cmd_idx = state.selected_item - 1;
                    if cmd_idx < self.config.saved_commands.len() {
                        let cmd = &self.config.saved_commands[cmd_idx];
                        self.view = View::EditSavedCommand(EditSavedCommandState::from_command(cmd_idx, cmd));
                    }
                }
                return Ok(());
            }
            
            let items = state.items_for_category();
            if state.selected_item >= items.len() {
                return Ok(());
            }
            
            let item = &items[state.selected_item];
            match item {
                SettingsItem::Bool { key, .. } => {
                    // Toggle boolean directly
                    match (state.category, key.as_str()) {
                        (SettingsCategory::Ui, "auto_scroll") => {
                            self.config.ui.auto_scroll = !self.config.ui.auto_scroll;
                        }
                        (SettingsCategory::Ui, "show_timestamps") => {
                            self.config.ui.show_timestamps = !self.config.ui.show_timestamps;
                        }
                        (SettingsCategory::Process, "auto_restart") => {
                            self.config.process.auto_restart = !self.config.process.auto_restart;
                        }
                        _ => {}
                    }
                    self.config.save(None)?;
                    self.set_status("Settings saved".to_string());
                }
                SettingsItem::Number { key, .. } => {
                    // Enter edit mode with current value
                    let current = match (state.category, key.as_str()) {
                        (SettingsCategory::Ui, "log_lines") => self.config.ui.log_lines.to_string(),
                        (SettingsCategory::Process, "log_buffer_size") => self.config.process.log_buffer_size.to_string(),
                        (SettingsCategory::Process, "max_restarts") => self.config.process.max_restarts.to_string(),
                        (SettingsCategory::Process, "restart_delay_secs") => self.config.process.restart_delay_secs.to_string(),
                        _ => String::new(),
                    };
                    state.input_buffer = current;
                    state.editing = true;
                }
                SettingsItem::Choice { key, options, .. } => {
                    // Cycle through options
                    match (state.category, key.as_str()) {
                        (SettingsCategory::Ui, "theme") => {
                            let current_idx = match &self.config.ui.theme {
                                crate::core::Theme::Dark => 0,
                                crate::core::Theme::Light => 1,
                                _ => 0,
                            };
                            let next_idx = (current_idx + 1) % options.len();
                            self.config.ui.theme = if next_idx == 0 {
                                crate::core::Theme::Dark
                            } else {
                                crate::core::Theme::Light
                            };
                        }
                        _ => {}
                    }
                    self.config.save(None)?;
                    self.set_status("Settings saved".to_string());
                }
                SettingsItem::Action { key, .. } => {
                    // Handle action items
                    if key == "add_command" {
                        self.view = View::CreateSavedCommand(CreateSavedCommandState::default());
                    }
                }
                SettingsItem::Text { .. } => {
                    state.input_buffer.clear();
                    state.editing = true;
                }
            }
        }
        Ok(())
    }

    /// Apply a setting change from edit mode
    fn apply_setting_change(&mut self, state: &SettingsState) -> Result<()> {
        let items = state.items_for_category();
        if state.selected_item >= items.len() {
            return Ok(());
        }
        
        let item = &items[state.selected_item];
        match item {
            SettingsItem::Number { key, min, max, .. } => {
                if let Ok(value) = state.input_buffer.parse::<usize>() {
                    let clamped = value.clamp(*min, *max);
                    match (state.category, key.as_str()) {
                        (SettingsCategory::Ui, "log_lines") => {
                            self.config.ui.log_lines = clamped;
                        }
                        (SettingsCategory::Process, "log_buffer_size") => {
                            self.config.process.log_buffer_size = clamped;
                        }
                        (SettingsCategory::Process, "max_restarts") => {
                            self.config.process.max_restarts = clamped as u32;
                        }
                        (SettingsCategory::Process, "restart_delay_secs") => {
                            self.config.process.restart_delay_secs = clamped as u64;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Edit a saved command at current selection
    fn edit_saved_command_at_selection(&mut self) -> Result<()> {
        if let View::Settings(ref state) = self.view {
            if state.category != SettingsCategory::SavedCommands {
                return Ok(());
            }
            // selected_item 0 is "Add Command", 1+ are actual commands
            if state.selected_item > 0 {
                let cmd_idx = state.selected_item - 1;
                if cmd_idx < self.config.saved_commands.len() {
                    let cmd = &self.config.saved_commands[cmd_idx];
                    self.view = View::EditSavedCommand(EditSavedCommandState::from_command(cmd_idx, cmd));
                }
            }
        }
        Ok(())
    }

    /// Delete a saved command at current selection
    fn delete_saved_command_at_selection(&mut self) -> Result<()> {
        if let View::Settings(ref state) = self.view {
            if state.category != SettingsCategory::SavedCommands {
                return Ok(());
            }
            // selected_item 0 is "Add Command", 1+ are actual commands
            if state.selected_item > 0 {
                let cmd_idx = state.selected_item - 1;
                if cmd_idx < self.config.saved_commands.len() {
                    let cmd_name = self.config.saved_commands[cmd_idx].name.clone();
                    self.show_confirm(
                        format!("Delete saved command '{}'?", cmd_name),
                        ConfirmAction::DeleteSavedCommand(cmd_idx),
                    );
                }
            }
        }
        Ok(())
    }

    /// Run a saved command at current selection
    async fn run_saved_command_at_selection(&mut self) -> Result<()> {
        if let View::Settings(ref state) = self.view {
            if state.category != SettingsCategory::SavedCommands {
                return Ok(());
            }
            // selected_item 0 is "Add Command", 1+ are actual commands
            if state.selected_item > 0 {
                let cmd_idx = state.selected_item - 1;
                if cmd_idx < self.config.saved_commands.len() {
                    let cmd = self.config.saved_commands[cmd_idx].clone();
                    self.run_saved_command(&cmd).await?;
                }
            }
        }
        Ok(())
    }

    /// Run a saved command
    async fn run_saved_command(&mut self, cmd: &crate::core::SavedCommand) -> Result<()> {
        let id = self.process_manager.spawn_process(&cmd.name, &cmd.command).await?;
        self.processes.push(Process::new(id, cmd.name.clone(), cmd.command.clone()));
        self.set_status(format!("Started: {}", cmd.name));
        self.view = View::ProcessManager;
        Ok(())
    }

    /// Handle create saved command wizard key events
    async fn handle_create_saved_command_key(&mut self, key: KeyEvent) -> Result<()> {
        if let View::CreateSavedCommand(ref mut state) = self.view {
            match key.code {
                KeyCode::Esc => {
                    // Return to settings
                    self.view = View::Settings(SettingsState {
                        category: SettingsCategory::SavedCommands,
                        selected_item: 0,
                        editing: false,
                        input_buffer: String::new(),
                        category_focused: false,
                    });
                }
                KeyCode::Tab => {
                    if let View::CreateSavedCommand(ref mut state) = self.view {
                        state.active_field = (state.active_field + 1) % CreateSavedCommandState::field_count();
                        state.field_cursor = 0;
                    }
                }
                KeyCode::BackTab | KeyCode::Up => {
                    if let View::CreateSavedCommand(ref mut state) = self.view {
                        if state.active_field > 0 {
                            state.active_field -= 1;
                        } else {
                            state.active_field = CreateSavedCommandState::field_count() - 1;
                        }
                        state.field_cursor = 0;
                    }
                }
                KeyCode::Enter => {
                    if state.active_field == 3 {
                        // Toggle auto_start
                        state.auto_start = !state.auto_start;
                    } else if state.is_valid() {
                        // Save the command
                        let new_cmd = crate::core::SavedCommand {
                            name: state.name.trim().to_string(),
                            command: state.command.trim().to_string(),
                            description: if state.description.trim().is_empty() {
                                None
                            } else {
                                Some(state.description.trim().to_string())
                            },
                            auto_start: state.auto_start,
                        };
                        self.config.saved_commands.push(new_cmd);
                        self.config.save(None)?;
                        self.set_status("Command saved".to_string());
                        self.view = View::Settings(SettingsState {
                            category: SettingsCategory::SavedCommands,
                            selected_item: self.config.saved_commands.len(), // Select the new command
                            editing: false,
                            input_buffer: String::new(),
                            category_focused: false,
                        });
                    } else {
                        self.set_status("Name and Command are required".to_string());
                    }
                }
                KeyCode::Char(' ') if state.active_field == 3 => {
                    // Toggle auto_start with space
                    state.auto_start = !state.auto_start;
                }
                KeyCode::Char(c) => {
                    match state.active_field {
                        0 => {
                            state.name.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        1 => {
                            state.command.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        2 => {
                            state.description.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        _ => {}
                    }
                }
                KeyCode::Backspace => {
                    match state.active_field {
                        0 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.name.remove(state.field_cursor);
                            }
                        }
                        1 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.command.remove(state.field_cursor);
                            }
                        }
                        2 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.description.remove(state.field_cursor);
                            }
                        }
                        _ => {}
                    }
                }
                KeyCode::Delete => {
                    match state.active_field {
                        0 => {
                            if state.field_cursor < state.name.len() {
                                state.name.remove(state.field_cursor);
                            }
                        }
                        1 => {
                            if state.field_cursor < state.command.len() {
                                state.command.remove(state.field_cursor);
                            }
                        }
                        2 => {
                            if state.field_cursor < state.description.len() {
                                state.description.remove(state.field_cursor);
                            }
                        }
                        _ => {}
                    }
                }
                KeyCode::Left => {
                    if state.active_field < 3 && state.field_cursor > 0 {
                        state.field_cursor -= 1;
                    }
                }
                KeyCode::Right => {
                    if state.active_field < 3 {
                        let field_len = match state.active_field {
                            0 => state.name.len(),
                            1 => state.command.len(),
                            2 => state.description.len(),
                            _ => 0,
                        };
                        if state.field_cursor < field_len {
                            state.field_cursor += 1;
                        }
                    }
                }
                KeyCode::Home => {
                    if state.active_field < 3 {
                        state.field_cursor = 0;
                    }
                }
                KeyCode::End => {
                    if state.active_field < 3 {
                        state.field_cursor = match state.active_field {
                            0 => state.name.len(),
                            1 => state.command.len(),
                            2 => state.description.len(),
                            _ => 0,
                        };
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Handle edit saved command wizard key events
    async fn handle_edit_saved_command_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state data first to avoid borrow issues
        let (index, name, command, description, auto_start, active_field, is_valid) = {
            if let View::EditSavedCommand(ref state) = self.view {
                (state.index, state.name.clone(), state.command.clone(), 
                 state.description.clone(), state.auto_start, state.active_field, state.is_valid())
            } else {
                return Ok(());
            }
        };
        
        match key.code {
            KeyCode::Esc => {
                // Return to settings
                self.view = View::Settings(SettingsState {
                    category: SettingsCategory::SavedCommands,
                    selected_item: index + 1, // +1 for "Add Command" item
                    editing: false,
                    input_buffer: String::new(),
                    category_focused: false,
                });
            }
            KeyCode::Tab | KeyCode::Down => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    state.active_field = (state.active_field + 1) % EditSavedCommandState::field_count();
                    state.field_cursor = 0;
                }
            }
            KeyCode::BackTab | KeyCode::Up => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    if state.active_field > 0 {
                        state.active_field -= 1;
                    } else {
                        state.active_field = EditSavedCommandState::field_count() - 1;
                    }
                    state.field_cursor = 0;
                }
            }
            KeyCode::Enter => {
                if active_field == 3 {
                    // Toggle auto_start
                    if let View::EditSavedCommand(ref mut state) = self.view {
                        state.auto_start = !state.auto_start;
                    }
                } else if is_valid {
                    // Update the command
                    if index < self.config.saved_commands.len() {
                        self.config.saved_commands[index] = crate::core::SavedCommand {
                            name: name.trim().to_string(),
                            command: command.trim().to_string(),
                            description: if description.trim().is_empty() {
                                None
                            } else {
                                Some(description.trim().to_string())
                            },
                            auto_start,
                        };
                        self.config.save(None)?;
                        self.set_status("Command updated".to_string());
                    }
                    self.view = View::Settings(SettingsState {
                        category: SettingsCategory::SavedCommands,
                        selected_item: index + 1,
                        editing: false,
                        input_buffer: String::new(),
                        category_focused: false,
                    });
                } else {
                    self.set_status("Name and Command are required".to_string());
                }
            }
            KeyCode::Char(' ') if active_field == 3 => {
                // Toggle auto_start with space
                if let View::EditSavedCommand(ref mut state) = self.view {
                    state.auto_start = !state.auto_start;
                }
            }
            KeyCode::Char(c) => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    match state.active_field {
                        0 => {
                            state.name.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        1 => {
                            state.command.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        2 => {
                            state.description.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        _ => {}
                    }
                }
            }
            KeyCode::Backspace => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    match state.active_field {
                        0 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.name.remove(state.field_cursor);
                            }
                        }
                        1 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.command.remove(state.field_cursor);
                            }
                        }
                        2 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.description.remove(state.field_cursor);
                            }
                        }
                        _ => {}
                    }
                }
            }
            KeyCode::Delete => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    match state.active_field {
                        0 => {
                            if state.field_cursor < state.name.len() {
                                state.name.remove(state.field_cursor);
                            }
                        }
                        1 => {
                            if state.field_cursor < state.command.len() {
                                state.command.remove(state.field_cursor);
                            }
                        }
                        2 => {
                            if state.field_cursor < state.description.len() {
                                state.description.remove(state.field_cursor);
                            }
                        }
                        _ => {}
                    }
                }
            }
            KeyCode::Left => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    if state.active_field < 3 && state.field_cursor > 0 {
                        state.field_cursor -= 1;
                    }
                }
            }
            KeyCode::Right => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    if state.active_field < 3 {
                        let field_len = match state.active_field {
                            0 => state.name.len(),
                            1 => state.command.len(),
                            2 => state.description.len(),
                            _ => 0,
                        };
                        if state.field_cursor < field_len {
                            state.field_cursor += 1;
                        }
                    }
                }
            }
            KeyCode::Home => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    if state.active_field < 3 {
                        state.field_cursor = 0;
                    }
                }
            }
            KeyCode::End => {
                if let View::EditSavedCommand(ref mut state) = self.view {
                    if state.active_field < 3 {
                        state.field_cursor = match state.active_field {
                            0 => state.name.len(),
                            1 => state.command.len(),
                            2 => state.description.len(),
                            _ => 0,
                        };
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle edit process command key events
    async fn handle_edit_process_command_key(&mut self, key: KeyEvent) -> Result<()> {
        // Extract state data first to avoid borrow issues
        let (process_index, name, command, active_field, is_valid) = {
            if let View::EditProcessCommand(ref state) = self.view {
                (state.process_index, state.name.clone(), state.command.clone(), 
                 state.active_field, state.is_valid())
            } else {
                return Ok(());
            }
        };
        
        match key.code {
            KeyCode::Esc => {
                // Return to process manager
                self.view = View::ProcessManager;
            }
            KeyCode::Tab | KeyCode::Down => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    state.active_field = (state.active_field + 1) % EditProcessCommandState::field_count();
                    state.field_cursor = 0;
                }
            }
            KeyCode::BackTab | KeyCode::Up => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    if state.active_field > 0 {
                        state.active_field -= 1;
                    } else {
                        state.active_field = EditProcessCommandState::field_count() - 1;
                    }
                    state.field_cursor = 0;
                }
            }
            KeyCode::Enter => {
                if is_valid {
                    // Update the process
                    if process_index < self.processes.len() {
                        self.processes[process_index].name = name.trim().to_string();
                        self.processes[process_index].command = command.trim().to_string();
                        self.set_status("Process updated".to_string());
                    }
                    self.view = View::ProcessManager;
                } else {
                    self.set_status("Name and Command are required".to_string());
                }
            }
            KeyCode::Char(c) => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    match state.active_field {
                        0 => {
                            state.name.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        1 => {
                            state.command.insert(state.field_cursor, c);
                            state.field_cursor += 1;
                        }
                        _ => {}
                    }
                }
            }
            KeyCode::Backspace => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    match state.active_field {
                        0 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.name.remove(state.field_cursor);
                            }
                        }
                        1 => {
                            if state.field_cursor > 0 {
                                state.field_cursor -= 1;
                                state.command.remove(state.field_cursor);
                            }
                        }
                        _ => {}
                    }
                }
            }
            KeyCode::Delete => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    match state.active_field {
                        0 => {
                            if state.field_cursor < state.name.len() {
                                state.name.remove(state.field_cursor);
                            }
                        }
                        1 => {
                            if state.field_cursor < state.command.len() {
                                state.command.remove(state.field_cursor);
                            }
                        }
                        _ => {}
                    }
                }
            }
            KeyCode::Left => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    if state.active_field < 2 && state.field_cursor > 0 {
                        state.field_cursor -= 1;
                    }
                }
            }
            KeyCode::Right => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    if state.active_field < 2 {
                        let field_len = match state.active_field {
                            0 => state.name.len(),
                            1 => state.command.len(),
                            _ => 0,
                        };
                        if state.field_cursor < field_len {
                            state.field_cursor += 1;
                        }
                    }
                }
            }
            KeyCode::Home => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    if state.active_field < 2 {
                        state.field_cursor = 0;
                    }
                }
            }
            KeyCode::End => {
                if let View::EditProcessCommand(ref mut state) = self.view {
                    if state.active_field < 2 {
                        state.field_cursor = match state.active_field {
                            0 => state.name.len(),
                            1 => state.command.len(),
                            _ => 0,
                        };
                    }
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Handle mouse events
    pub fn on_mouse(&mut self, _event: MouseEvent) {
        // TODO: Implement mouse support
    }

    /// Handle resize
    pub fn on_resize(&mut self, _width: u16, _height: u16) {
        // Handled by ratatui
    }

    // === Data loading methods ===

    fn load_packages(&mut self) {
        self.explorer_items.clear();
        self.selected_item = 0;
        if let Some(ws) = &self.ros2_workspace {
            for pkg in &ws.packages {
                self.explorer_items.push(ExplorerItem::Package(pkg.clone()));
            }
        }
    }

    fn load_nodes(&mut self) {
        self.explorer_items.clear();
        self.selected_item = 0;
        if let Some(ws) = &self.ros2_workspace {
            for (pkg, node) in ws.all_nodes() {
                self.explorer_items.push(ExplorerItem::Node(pkg.name.clone(), node.to_string()));
            }
        }
    }

    fn load_launch_files(&mut self) {
        self.explorer_items.clear();
        self.selected_item = 0;
        if let Some(ws) = &self.ros2_workspace {
            for (pkg, path) in ws.all_launch_files() {
                self.explorer_items.push(ExplorerItem::LaunchFile(pkg.name.clone(), path.clone()));
            }
        }
    }

    async fn load_topics(&mut self) {
        self.explorer_items.clear();
        self.selected_item = 0;
        self.is_loading = true;
        
        if let Ok(topics) = self.ros2_introspection.list_topics() {
            self.topics = topics;
            for topic in &self.topics {
                self.explorer_items.push(ExplorerItem::Topic(topic.clone()));
            }
        }
        self.is_loading = false;
    }

    async fn load_services(&mut self) {
        self.explorer_items.clear();
        self.selected_item = 0;
        self.is_loading = true;
        
        if let Ok(services) = self.ros2_introspection.list_services() {
            self.services = services;
            for service in &self.services {
                self.explorer_items.push(ExplorerItem::Service(service.clone()));
            }
        }
        self.is_loading = false;
    }

    async fn load_actions(&mut self) {
        self.explorer_items.clear();
        self.selected_item = 0;
        self.is_loading = true;
        
        if let Ok(actions) = self.ros2_introspection.list_actions() {
            self.actions = actions;
            for action in &self.actions {
                self.explorer_items.push(ExplorerItem::Action(action.clone()));
            }
        }
        self.is_loading = false;
    }

    async fn refresh_explorer(&mut self, explorer_type: MenuItem) -> Result<()> {
        match explorer_type {
            MenuItem::Packages => {
                self.ros2_workspace = Ros2Workspace::scan(&self.workspace_path).ok();
                self.load_packages();
            }
            MenuItem::Nodes => {
                self.ros2_workspace = Ros2Workspace::scan(&self.workspace_path).ok();
                self.load_nodes();
            }
            MenuItem::LaunchFiles => {
                self.ros2_workspace = Ros2Workspace::scan(&self.workspace_path).ok();
                self.load_launch_files();
            }
            MenuItem::Topics => self.load_topics().await,
            MenuItem::Services => self.load_services().await,
            MenuItem::Actions => self.load_actions().await,
            _ => {}
        }
        self.set_status("Refreshed".to_string());
        Ok(())
    }

    async fn refresh_data(&mut self) -> Result<()> {
        self.ros2_workspace = Ros2Workspace::scan(&self.workspace_path).ok();
        self.set_status("Workspace refreshed".to_string());
        Ok(())
    }

    async fn show_item_info(&mut self) -> Result<()> {
        let items = self.filtered_explorer_items();
        if let Some(item) = items.get(self.selected_item) {
            match item {
                ExplorerItem::Topic(topic) => {
                    let msg_type = &topic.msg_type;
                    if !msg_type.is_empty() {
                        let interface_def = self.ros2_introspection
                            .interface_show(msg_type)
                            .unwrap_or_else(|_| format!("Could not load interface for {}", msg_type));
                        self.previous_view = Some(Box::new(self.view.clone()));
                        self.view = View::InterfaceView(
                            format!("Message: {}", msg_type),
                            interface_def,
                        );
                    } else {
                        self.set_status("No message type available".to_string());
                    }
                }
                ExplorerItem::Service(service) => {
                    let srv_type = &service.srv_type;
                    let interface_def = self.ros2_introspection
                        .interface_show(srv_type)
                        .unwrap_or_else(|_| format!("Could not load interface for {}", srv_type));
                    self.previous_view = Some(Box::new(self.view.clone()));
                    self.view = View::InterfaceView(
                        format!("Service: {}", srv_type),
                        interface_def,
                    );
                }
                ExplorerItem::Package(pkg) => {
                    let info = format!(
                        "Package: {}\nVersion: {}\nPath: {:?}\nBuild: {}\nNodes: {:?}",
                        pkg.name, pkg.version, pkg.path, pkg.build_type, pkg.nodes
                    );
                    self.set_status(info.lines().take(3).collect::<Vec<_>>().join(" | "));
                }
                _ => {}
            };
        }
        Ok(())
    }

    // === Process management ===

    pub async fn add_command(&mut self, command: &str) -> Result<()> {
        let name = command
            .split_whitespace()
            .take(3)
            .collect::<Vec<_>>()
            .join(" ");

        let id = self.process_manager.spawn_process(&name, command).await?;

        let process = Process::new(id, name, command.to_string());
        self.processes.push(process);
        self.selected_process = self.processes.len() - 1;
        self.update_log_buffer();

        Ok(())
    }

    pub async fn add_command_with_name(&mut self, command: &str, name: &str) -> Result<()> {
        let id = self.process_manager.spawn_process(name, command).await?;

        let process = Process::new(id, name.to_string(), command.to_string());
        self.processes.push(process);
        self.selected_process = self.processes.len() - 1;
        self.update_log_buffer();

        Ok(())
    }

    /// Update suggestions based on current input buffer
    fn update_suggestions(&mut self) {
        self.suggestions.clear();
        let input_lower = self.input_buffer.to_lowercase();
        
        for saved in &self.config.saved_commands {
            // Match against name or command
            if saved.name.to_lowercase().contains(&input_lower) 
                || saved.command.to_lowercase().contains(&input_lower)
                || input_lower.is_empty() 
            {
                self.suggestions.push((saved.name.clone(), saved.command.clone()));
            }
        }
        
        // Limit to 4 suggestions
        self.suggestions.truncate(4);
    }

    async fn toggle_selected_process(&mut self) -> Result<()> {
        if let Some(process) = self.processes.get(self.selected_process) {
            let id = process.id;
            match process.status {
                ProcessStatus::Running => {
                    self.process_manager.stop_process(id).await?;
                }
                ProcessStatus::Stopped | ProcessStatus::Exited(_) | ProcessStatus::Failed(_) => {
                    let cmd = process.command.clone();
                    let name = process.name.clone();
                    self.process_manager.restart_process(id, &name, &cmd).await?;
                }
                ProcessStatus::Pending => {}
            }
        }
        Ok(())
    }

    async fn restart_selected_process(&mut self) -> Result<()> {
        if let Some(process) = self.processes.get(self.selected_process) {
            let id = process.id;
            let cmd = process.command.clone();
            let name = process.name.clone();
            self.process_manager.restart_process(id, &name, &cmd).await?;
        }
        Ok(())
    }

    async fn run_build(&mut self) -> Result<()> {
        self.add_command("colcon build --symlink-install").await?;
        self.view = View::ProcessManager;
        Ok(())
    }

    async fn build_package(&mut self, package_name: &str) -> Result<()> {
        let cmd = format!("colcon build --symlink-install --packages-select {}", package_name);
        self.add_command(&cmd).await?;
        self.set_status(format!("Building package: {}", package_name));
        self.view = View::ProcessManager;
        Ok(())
    }

    async fn run_clean(&mut self) -> Result<()> {
        self.add_command("rm -rf build install log").await?;
        self.view = View::ProcessManager;
        Ok(())
    }

    fn update_log_buffer(&mut self) {
        self.log_buffer.clear();
        if let Some(process) = self.processes.get(self.selected_process) {
            for line in process.log_buffer.lines() {
                self.log_buffer.push_line(&line.content, line.is_stderr);
            }
        }
        self.log_scroll = self.log_buffer.len().saturating_sub(1);
    }

    // === Helpers ===

    pub fn filtered_explorer_items(&self) -> Vec<ExplorerItem> {
        if self.search_query.is_empty() {
            return self.explorer_items.clone();
        }

        let query = self.search_query.to_lowercase();
        self.explorer_items
            .iter()
            .filter(|item| {
                let name = match item {
                    ExplorerItem::Package(pkg) => &pkg.name,
                    ExplorerItem::Node(_, node) => node,
                    ExplorerItem::LaunchFile(_, path) => {
                        return path.to_string_lossy().to_lowercase().contains(&query);
                    }
                    ExplorerItem::Topic(topic) => &topic.name,
                    ExplorerItem::Service(service) => &service.name,
                    ExplorerItem::Action(action) => &action.name,
                    ExplorerItem::RunningNode(node) => &node.name,
                };
                name.to_lowercase().contains(&query)
            })
            .cloned()
            .collect()
    }

    fn has_running_processes(&self) -> bool {
        self.processes.iter().any(|p| p.status == ProcessStatus::Running)
    }

    /// Show confirm dialog while preserving current view
    fn show_confirm(&mut self, message: String, action: ConfirmAction) {
        self.previous_view = Some(Box::new(self.view.clone()));
        self.view = View::Confirm(message, action);
    }

    fn set_status(&mut self, message: String) {
        self.status_message = Some(message);
        self.status_timestamp = std::time::Instant::now();
    }

    pub async fn cleanup(&mut self) -> Result<()> {
        self.process_manager.stop_all().await?;
        Ok(())
    }
}
