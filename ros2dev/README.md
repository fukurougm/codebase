# ros2dev

Interactive TUI (Terminal User Interface) for ROS 2 development, written in Rust using the ratatui framework.

## Features

- **Interactive Main Menu** - Navigation dengan arrow keys, menu grouping by categories (OBSERVATION, MANAGEMENT, WORKSPACE, SYSTEM)
- **Package Explorer** - Browse packages, view details, quick build/run
- **Node Explorer** - List nodes, run/stop individual nodes, view details
- **Launch Files Explorer** - Browse launch files, run directly
- **Topics** - Real-time topic list, subscriber/publisher interfaces
- **Services** - List services, service call tester
- **Process Manager** - Manage concurrent processes, log selection, clipboard copy
- **Create Package Wizard** - Interactive package creation (ament_python/cmake)
- **Create Node Template** - Generate Python/C++ node templates
- **Create Launch File** - Generate Python/XML launch files
- **Build Tools** - colcon integration, workspace cleaning
- **Settings** - Categorized settings (UI, Process, Saved Commands, About page)
- **Clipboard Support** - Cross-platform clipboard (arboard), no external dependencies
- **CI/CD Pipeline** - GitHub Actions multi-platform releases (Linux, macOS, Windows)

## Installation

### From source

```bash
cd tools/ros2dev-rust
cargo build --release
```

The binary will be at `target/release/ros2dev`.

### Install to PATH

```bash
cargo install --path .
```

### Pre-built Binaries

Download from [GitHub Releases](https://github.com/fukurougm/ros2dev/releases) for your platform.

### Installer script

- **Download & run:**

```bash
curl -fsSL https://raw.githubusercontent.com/fukurougm/codebase/ros2dev/master/scripts/install_linux.sh -o /tmp/install_ros2dev.sh
chmod +x /tmp/install_ros2dev.sh
/tmp/install_ros2dev.sh
```

- **One-liner:**

```bash
curl -fsSL https://raw.githubusercontent.com/fukurougm/codebase/ros2dev/master/scripts/install_linux.sh | bash -s -- v0.1.0
```

## Usage

```bash
# Run in current workspace
ros2dev

# Specify workspace path
ros2dev --workspace /path/to/ros2_ws

# Enable debug logging
ros2dev --debug
```

## Keybindings

### Navigation
| Key | Action |
|-----|--------|
| `↑/k`, `↓/j` | Navigate up/down |
| `Tab` | Switch focus between panes |
| `Enter` | Select / Confirm |
| `Esc` | Go back / Cancel |

### Process Management
| Key | Action |
|-----|--------|
| `n` / `a` | Add new command |
| `s` | Start/Stop selected process |
| `r` | Restart selected process |
| `d` / `Del` | Delete selected process |
| `S` | Stop all processes |
| `o` | Enter log select mode |
| `y` | Copy selected logs |
| `Y` | Copy all logs |

### Log Selection Mode
| Key | Action |
|-----|--------|
| `↑/↓` | Move selection |
| `Space` | Toggle line selection |
| `y` | Copy selected lines |
| `Y` | Copy all lines |
| `Esc` | Exit selection mode |

### Build & Tools
| Key | Action |
|-----|--------|
| `b` | Build workspace |
| `c` | Clean workspace |
| `p` | Open package browser |

### Settings
| Key | Action |
|-----|--------|
| `←/→` | Switch between sidebar and content |
| `Space` | Toggle boolean settings |
| `Enter` | Edit text/number settings |

### General
| Key | Action |
|-----|--------|
| `?` / `F1` | Show help |
| `q` | Quit |
| `Ctrl+C` | Force quit |
| `Ctrl+R` | Refresh workspace |

## Configuration

Configuration is stored at `~/.config/ros2dev/config.yaml`:

```yaml
saved_commands:
  - name: "talker"
    command: "ros2 run demo_nodes_cpp talker"
    auto_start: false
  - name: "listener"
    command: "ros2 run demo_nodes_cpp listener"
    auto_start: false

ui:
  auto_scroll: true
  log_lines: 50
  show_timestamps: false
  theme: "dark"

process:
  log_buffer_size: 10000
  auto_restart: false
  max_restarts: 3
  restart_delay_secs: 5
```

## Architecture

```
src/
├── main.rs           # Entry point, CLI, terminal setup
├── app.rs            # Application state & logic, menu categories
├── ui/
│   ├── mod.rs        # UI module exports
│   ├── render.rs     # TUI rendering dengan ratatui
│   ├── theme.rs      # Color themes dan styling
│   └── icons.rs      # Unicode icons untuk UI
├── events.rs         # Keyboard/mouse event handling
├── process/
│   ├── mod.rs        # Process module
│   ├── manager.rs    # Concurrent process management
│   └── types.rs      # Process types and status
├── core/
│   ├── mod.rs        # Core utilities
│   ├── config.rs     # Configuration management
│   ├── log_buffer.rs # Circular log buffer
│   └── ipc.rs        # Inter-process communication
└── ros2/
    ├── mod.rs
    ├── detection.rs    # ROS 2 environment detection
    ├── introspection.rs # Runtime introspection (topics, services, nodes)
    ├── scanner.rs      # Workspace scanning
    └── templates.rs    # Package/node/launch templates
```

## Tech Stack

- **Rust** - Systems programming language
- **Ratatui** - Terminal UI framework
- **Tokio** - Async runtime
- **Arboard** - Cross-platform clipboard
- **Serde** - Serialization
- **Chrono** - Date/time handling

## Requirements

- Rust 1.70+
- ROS 2 (Humble, Iron, Jazzy, or Rolling)
- A terminal with Unicode support

## Development

### Setup

```bash
git clone <repository>
cd ros2dev-rust
cargo build
```

### Testing

```bash
cargo test
cargo clippy
cargo fmt --check
```

### Release

```bash
# Create tag
git tag v0.1.0
git push origin v0.1.0

# GitHub Actions will automatically build and release
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT - see [LICENSE](LICENSE) file for details.

## Author

**@fukurougm** - [GitHub](https://github.com/fukurougm)

