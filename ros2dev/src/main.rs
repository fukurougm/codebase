//! ros2dev - ROS 2 Development TUI Tool with Process Management
//!
//! A terminal user interface for managing ROS 2 development workflows,
//! running multiple commands concurrently, and monitoring their output.

// Core modules (config, events, IPC, logging)
mod core;
// Process management
mod process;
// ROS 2 integration
mod ros2;
// Main application logic
mod app;
// Terminal UI rendering
mod ui;

use anyhow::Result;
use clap::Parser;
use crossterm::{
    event::{DisableMouseCapture, EnableMouseCapture},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use ratatui::prelude::*;
use std::io;
use std::path::PathBuf;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

// Use the new App
use app::App;
use core::EventHandler;

/// ROS 2 Development TUI Tool
#[derive(Parser, Debug)]
#[command(name = "ros2dev")]
#[command(author = "Fukuro Strategy Team")]
#[command(version = "0.1.0")]
#[command(about = "Interactive TUI for ROS 2 development with process management")]
struct Cli {
    /// Path to the ROS 2 workspace
    #[arg(short, long, default_value = ".")]
    workspace: PathBuf,

    /// Config file path
    #[arg(short, long)]
    config: Option<PathBuf>,

    /// Enable debug logging to file
    #[arg(short, long)]
    debug: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    // Setup logging
    if cli.debug {
        let log_dir = dirs::data_local_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("ros2dev")
            .join("logs");
        std::fs::create_dir_all(&log_dir)?;

        let log_file = std::fs::File::create(log_dir.join("ros2dev.log"))?;
        let file_layer = tracing_subscriber::fmt::layer()
            .with_writer(log_file)
            .with_ansi(false);

        tracing_subscriber::registry()
            .with(file_layer)
            .with(tracing_subscriber::EnvFilter::new("ros2dev=debug"))
            .init();
    }

    // Initialize terminal
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    // Create app and event handler
    let workspace = cli.workspace.canonicalize().unwrap_or(cli.workspace);
    let config_path = cli.config;
    
    let mut app = App::new(workspace, config_path).await?;
    
    // Initialize UI theme and icons
    ui::init(&app.config);
    
    let event_handler = EventHandler::new(250); // 250ms tick rate

    // Run the main loop
    let result = run_app(&mut terminal, &mut app, event_handler).await;

    // Restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;

    // Handle any errors
    if let Err(e) = result {
        eprintln!("Error: {e:?}");
        return Err(e);
    }

    Ok(())
}

async fn run_app<B: Backend>(
    terminal: &mut Terminal<B>,
    app: &mut App,
    mut event_handler: EventHandler,
) -> Result<()> {
    loop {
        // Draw UI
        terminal.draw(|frame| ui::draw(frame, app))?;

        // Handle events
        match event_handler.next().await? {
            core::Event::Tick => {
                app.on_tick().await?;
            }
            core::Event::Key(key_event) => {
                app.on_key(key_event).await?;
            }
            core::Event::Mouse(mouse_event) => {
                app.on_mouse(mouse_event);
            }
            core::Event::Resize(width, height) => {
                app.on_resize(width, height);
            }
        }

        // Check if we should quit
        if app.should_quit {
            // Cleanup all processes before quitting
            app.cleanup().await?;
            break;
        }
    }

    Ok(())
}
