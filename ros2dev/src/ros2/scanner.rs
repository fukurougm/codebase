//! ROS 2 workspace scanning

use anyhow::Result;
use quick_xml::de::from_str;
use serde::Deserialize;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

/// Scanned ROS 2 workspace information
#[derive(Debug, Clone, Default)]
pub struct Ros2Workspace {
    /// Path to the workspace
    pub path: PathBuf,
    /// List of packages
    pub packages: Vec<Ros2Package>,
    /// Whether the workspace has been built
    pub is_built: bool,
}

impl Ros2Workspace {
    /// Scan a workspace for packages
    pub fn scan(path: &Path) -> Result<Self> {
        let path = path.canonicalize()?;
        let mut packages = Vec::new();

        let src_path = path.join("src");
        let search_path = if src_path.exists() {
            src_path
        } else {
            path.clone()
        };

        for entry in WalkDir::new(&search_path)
            .follow_links(true)
            .max_depth(5)
            .into_iter()
            .filter_map(|e| e.ok())
        {
            if entry.file_name() == "package.xml" {
                if let Some(pkg) = Ros2Package::from_package_xml(entry.path()) {
                    packages.push(pkg);
                }
            }
        }

        let is_built = path.join("install").exists() || path.join("build").exists();
        packages.sort_by(|a, b| a.name.cmp(&b.name));

        Ok(Self {
            path,
            packages,
            is_built,
        })
    }

    /// Get package by name
    pub fn get_package(&self, name: &str) -> Option<&Ros2Package> {
        self.packages.iter().find(|p| p.name == name)
    }

    /// Get all launch files across packages
    pub fn all_launch_files(&self) -> Vec<(&Ros2Package, &PathBuf)> {
        self.packages
            .iter()
            .flat_map(|pkg| pkg.launch_files.iter().map(move |lf| (pkg, lf)))
            .collect()
    }

    /// Get all nodes across packages
    pub fn all_nodes(&self) -> Vec<(&Ros2Package, &str)> {
        self.packages
            .iter()
            .flat_map(|pkg| pkg.nodes.iter().map(move |n| (pkg, n.as_str())))
            .collect()
    }
}

/// A ROS 2 package
#[derive(Debug, Clone)]
pub struct Ros2Package {
    /// Package name
    pub name: String,
    /// Package version
    pub version: String,
    /// Package description
    pub description: String,
    /// Package path
    pub path: PathBuf,
    /// Build type (ament_cmake, ament_python, cmake)
    pub build_type: BuildType,
    /// List of node executables
    pub nodes: Vec<String>,
    /// Launch files
    pub launch_files: Vec<PathBuf>,
    /// Dependencies
    pub dependencies: Vec<String>,
    /// Maintainer
    pub maintainer: Option<String>,
}

/// Build type of a package
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuildType {
    AmentCmake,
    AmentPython,
    Cmake,
    Unknown,
}

impl std::fmt::Display for BuildType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BuildType::AmentCmake => write!(f, "ament_cmake"),
            BuildType::AmentPython => write!(f, "ament_python"),
            BuildType::Cmake => write!(f, "cmake"),
            BuildType::Unknown => write!(f, "unknown"),
        }
    }
}

/// XML structure for package.xml
#[derive(Debug, Deserialize)]
struct PackageXml {
    name: String,
    #[serde(default)]
    version: String,
    #[serde(default)]
    description: String,
    #[serde(default)]
    maintainer: Option<String>,
    #[serde(rename = "buildtool_depend", default)]
    buildtool_depend: Vec<String>,
    #[serde(rename = "build_depend", default)]
    build_depend: Vec<String>,
    #[serde(rename = "exec_depend", default)]
    exec_depend: Vec<String>,
    #[serde(rename = "depend", default)]
    depend: Vec<String>,
}

impl Ros2Package {
    /// Parse a package from package.xml
    pub fn from_package_xml(path: &Path) -> Option<Self> {
        let content = std::fs::read_to_string(path).ok()?;
        let pkg_xml: PackageXml = from_str(&content).ok()?;

        let pkg_path = path.parent()?.to_path_buf();

        let build_type = if pkg_xml.buildtool_depend.iter().any(|d| d == "ament_cmake") {
            BuildType::AmentCmake
        } else if pkg_xml.buildtool_depend.iter().any(|d| d == "ament_python") {
            BuildType::AmentPython
        } else if pkg_xml.buildtool_depend.iter().any(|d| d == "cmake") {
            BuildType::Cmake
        } else if pkg_path.join("setup.py").exists() {
            BuildType::AmentPython
        } else if pkg_path.join("CMakeLists.txt").exists() {
            BuildType::AmentCmake
        } else {
            BuildType::Unknown
        };

        let nodes = Self::find_nodes(&pkg_path, &build_type, &pkg_xml.name);
        let launch_files = Self::find_launch_files(&pkg_path);

        let mut dependencies = Vec::new();
        dependencies.extend(pkg_xml.build_depend);
        dependencies.extend(pkg_xml.exec_depend);
        dependencies.extend(pkg_xml.depend);
        dependencies.sort();
        dependencies.dedup();

        Some(Self {
            name: pkg_xml.name,
            version: pkg_xml.version,
            description: pkg_xml.description,
            path: pkg_path,
            build_type,
            nodes,
            launch_files,
            dependencies,
            maintainer: pkg_xml.maintainer,
        })
    }

    /// Find executable nodes in a package
    fn find_nodes(pkg_path: &Path, build_type: &BuildType, pkg_name: &str) -> Vec<String> {
        let mut nodes = Vec::new();

        match build_type {
            BuildType::AmentPython => {
                // Try setup.py first - look for console_scripts entry points
                let setup_py = pkg_path.join("setup.py");
                if setup_py.exists() {
                    if let Ok(content) = std::fs::read_to_string(&setup_py) {
                        // Parse entry_points console_scripts section
                        // Format: 'node_name = package.module:main'
                        nodes.extend(Self::parse_console_scripts(&content));
                    }
                }

                // Try setup.cfg as alternative
                let setup_cfg = pkg_path.join("setup.cfg");
                if setup_cfg.exists() {
                    if let Ok(content) = std::fs::read_to_string(&setup_cfg) {
                        nodes.extend(Self::parse_setup_cfg_scripts(&content));
                    }
                }

                // Fallback: look for node_*.py files in package directory
                let py_pkg = pkg_path.join(pkg_name);
                if py_pkg.exists() {
                    if let Ok(entries) = std::fs::read_dir(&py_pkg) {
                        for entry in entries.flatten() {
                            let name = entry.file_name();
                            let name_str = name.to_string_lossy();
                            if name_str.starts_with("node_") && name_str.ends_with(".py") {
                                let node_name = name_str
                                    .trim_start_matches("node_")
                                    .trim_end_matches(".py");
                                if !nodes.contains(&node_name.to_string()) {
                                    nodes.push(node_name.to_string());
                                }
                            }
                        }
                    }
                }
            }
            BuildType::AmentCmake | BuildType::Cmake => {
                let cmake = pkg_path.join("CMakeLists.txt");
                if cmake.exists() {
                    if let Ok(content) = std::fs::read_to_string(&cmake) {
                        // Match add_executable(name ...)
                        let re = regex::Regex::new(r"add_executable\s*\(\s*(\w+)").ok();
                        if let Some(re) = re {
                            for cap in re.captures_iter(&content) {
                                if let Some(name) = cap.get(1) {
                                    let node_name = name.as_str();
                                    // Filter out common non-node executables
                                    if !["test", "tests", "gtest", "benchmark"].contains(&node_name) {
                                        nodes.push(node_name.to_string());
                                    }
                                }
                            }
                        }
                        
                        // Also check rosidl_generate_interfaces for C++ services/messages
                        // and find ros2 nodes from install(TARGETS ...)
                    }
                }
            }
            BuildType::Unknown => {}
        }

        nodes.sort();
        nodes.dedup();
        nodes
    }

    /// Parse console_scripts from setup.py content
    fn parse_console_scripts(content: &str) -> Vec<String> {
        let mut nodes = Vec::new();
        
        // Look for entry_points section with console_scripts
        // Common formats:
        // entry_points={'console_scripts': ['node = pkg.mod:main', ...]}
        // entry_points={"console_scripts": ["node = pkg.mod:main", ...]}
        
        // Find the console_scripts section
        let content_normalized = content.replace('\n', " ").replace('\r', " ");
        
        // Pattern 1: Look for console_scripts list
        if let Some(start) = content_normalized.find("console_scripts") {
            // Find the opening bracket after console_scripts
            let after_cs = &content_normalized[start..];
            if let Some(bracket_start) = after_cs.find('[') {
                let list_start = &after_cs[bracket_start + 1..];
                // Find matching closing bracket
                let mut depth = 1;
                let mut end_idx = 0;
                for (i, c) in list_start.chars().enumerate() {
                    match c {
                        '[' => depth += 1,
                        ']' => {
                            depth -= 1;
                            if depth == 0 {
                                end_idx = i;
                                break;
                            }
                        }
                        _ => {}
                    }
                }
                
                if end_idx > 0 {
                    let scripts_content = &list_start[..end_idx];
                    // Extract node names from entries like 'node_name = package.module:main'
                    // or "node_name = package.module:main"
                    let re = regex::Regex::new(r#"['"](\w+)\s*=\s*[\w.]+:\w+['"]"#).ok();
                    if let Some(re) = re {
                        for cap in re.captures_iter(scripts_content) {
                            if let Some(name) = cap.get(1) {
                                nodes.push(name.as_str().to_string());
                            }
                        }
                    }
                }
            }
        }
        
        nodes
    }

    /// Parse console_scripts from setup.cfg
    fn parse_setup_cfg_scripts(content: &str) -> Vec<String> {
        let mut nodes = Vec::new();
        let mut in_console_scripts = false;
        
        for line in content.lines() {
            let trimmed = line.trim();
            
            // Check for section header
            if trimmed.starts_with('[') {
                in_console_scripts = trimmed == "[options.entry_points]" || 
                                     trimmed.contains("console_scripts");
                continue;
            }
            
            if in_console_scripts {
                // Line format: node_name = package.module:main
                if let Some(eq_pos) = trimmed.find('=') {
                    let name = trimmed[..eq_pos].trim();
                    // Validate it looks like a node name (alphanumeric with underscores)
                    if !name.is_empty() && 
                       !name.starts_with('#') &&
                       name.chars().all(|c| c.is_alphanumeric() || c == '_') {
                        nodes.push(name.to_string());
                    }
                }
            }
        }
        
        nodes
    }

    /// Find launch files in a package
    fn find_launch_files(pkg_path: &Path) -> Vec<PathBuf> {
        let mut launch_files = Vec::new();

        let launch_dir = pkg_path.join("launch");
        if launch_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(&launch_dir) {
                for entry in entries.flatten() {
                    let path = entry.path();
                    if let Some(ext) = path.extension() {
                        let ext_str = ext.to_string_lossy();
                        if ext_str == "py" || ext_str == "xml" || ext_str == "yaml" || ext_str == "launch" {
                            let name = path.file_name().unwrap_or_default().to_string_lossy();
                            if name.contains("launch") || ext_str == "launch" {
                                launch_files.push(path);
                            }
                        }
                    }
                }
            }
        }

        launch_files.sort();
        launch_files
    }
}
