//! Terminal UI rendering with ratatui

use std::path::Path;

use ratatui::{
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style, Stylize},
    text::{Line, Span, Text},
    widgets::{
        Block, BorderType, Borders, Clear, List, ListItem, ListState, Paragraph, Scrollbar,
        ScrollbarOrientation, ScrollbarState, Tabs, Wrap,
    },
    Frame,
};

/// Make a path relative to workspace if possible
fn make_relative_path(command: &str, workspace_path: &Path) -> String {
    let workspace_str = workspace_path.to_string_lossy();
    if command.contains(workspace_str.as_ref()) {
        command.replace(workspace_str.as_ref(), ".")
    } else {
        command.to_string()
    }
}

/// Create a text string with cursor at the specified position
fn format_text_with_cursor(text: &str, cursor_pos: usize) -> String {
    let cursor_pos = cursor_pos.min(text.len());
    let (before_cursor, after_cursor) = text.split_at(cursor_pos);
    format!("{}┃{}", before_cursor, after_cursor)
}

/// Create spans for text with cursor highlighting at the specified position
fn create_text_with_cursor_spans(text: &str, cursor_pos: usize, style: Style, cursor_style: Style) -> Vec<Span<'static>> {
    let cursor_pos = cursor_pos.min(text.len());
    let (before_cursor, after_cursor) = text.split_at(cursor_pos);
    
    vec![
        Span::styled(before_cursor.to_string(), style),
        Span::styled("┃".to_string(), cursor_style),
        Span::styled(after_cursor.to_string(), style),
    ]
}

/// Helper function to wrap text with cursor into multiple lines
fn wrap_text_with_cursor(text: &str, cursor_pos: usize, width: usize) -> Vec<Line<'static>> {
    use ratatui::style::Color;
    
    let cursor_pos = cursor_pos.min(text.len());
    let (before_cursor, after_cursor) = text.split_at(cursor_pos);
    
    let mut lines = Vec::new();
    let mut current_line_spans = Vec::new();
    let mut current_width = 0;
    let mut cursor_placed = false;
    
    // Process text before cursor
    for ch in before_cursor.chars() {
        if ch == '\n' || current_width >= width {
            lines.push(Line::from(current_line_spans));
            current_line_spans = Vec::new();
            current_width = 0;
            if ch != '\n' {
                current_line_spans.push(Span::raw(ch.to_string()));
                current_width += 1;
            }
        } else {
            current_line_spans.push(Span::raw(ch.to_string()));
            current_width += 1;
        }
    }
    
    // Place cursor
    if !cursor_placed {
        current_line_spans.push(Span::styled("┃", Style::default().fg(Color::Yellow)));
        cursor_placed = true;
    }
    
    // Process text after cursor
    for ch in after_cursor.chars() {
        if ch == '\n' || current_width >= width {
            lines.push(Line::from(current_line_spans));
            current_line_spans = Vec::new();
            current_width = 0;
            if ch != '\n' {
                current_line_spans.push(Span::raw(ch.to_string()));
                current_width += 1;
            }
        } else {
            current_line_spans.push(Span::raw(ch.to_string()));
            current_width += 1;
        }
    }
    
    // Add the last line if it has content
    if !current_line_spans.is_empty() || lines.is_empty() {
        lines.push(Line::from(current_line_spans));
    }
    
    lines
}

use crate::app::{
    App, ConfirmAction, CreateLaunchState, CreateNodeState, CreatePackageState, 
    CreateSavedCommandState, EditSavedCommandState, EditProcessCommandState, ExplorerItem,
    InputMode, MenuCategory, MenuItem, PackageDetailState, ServiceClientState, SettingsCategory, 
    SettingsItem, SettingsState, View,
};
use crate::process::ProcessStatus;

use super::theme;
use super::icons;

/// Main draw function
pub fn draw(f: &mut Frame, app: &App) {
    let t = theme::current();
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),  // Header
            Constraint::Min(10),    // Main content
            Constraint::Length(1),  // Status bar
        ])
        .split(f.area());

    draw_header(f, app, chunks[0]);
    
    match &app.view {
        View::MainMenu => draw_main_menu(f, app, chunks[1]),
        View::Dashboard => draw_dashboard(f, app, chunks[1]),
        View::PackageExplorer | View::NodeExplorer | View::LaunchExplorer |
        View::TopicExplorer | View::ServiceExplorer | View::ActionExplorer => {
            draw_explorer(f, app, chunks[1])
        }
        View::PackageDetail(state) => draw_package_detail(f, state, chunks[1]),
        View::ProcessManager => draw_process_manager(f, app, chunks[1]),
        View::CreatePackage(state) => draw_create_package(f, app, state.clone(), chunks[1]),
        View::CreateNode(state) => draw_create_node(f, app, state.clone(), chunks[1]),
        View::CreateLaunch(state) => draw_create_launch(f, app, state.clone(), chunks[1]),
        View::CreateSavedCommand(state) => draw_create_saved_command(f, state, chunks[1]),
        View::EditSavedCommand(state) => draw_edit_saved_command(f, state, chunks[1]),
        View::EditProcessCommand(state) => draw_edit_process_command(f, app, state, chunks[1]),
        View::TopicSubscriber(topic) => draw_topic_subscriber(f, topic, chunks[1]),
        View::TopicPublisher(topic, msg_type) => draw_topic_publisher(f, app, topic, msg_type, chunks[1]),
        View::ServiceClient(state) => draw_service_client(f, state, chunks[1]),
        View::InterfaceView(title, definition) => draw_interface_view(f, title, definition, chunks[1]),
        View::Settings(state) => draw_settings(f, app, state, chunks[1]),
        View::Help => draw_help(f, chunks[1]),
        View::Confirm(msg, action) => {
            // Draw underlying view based on previous state
            if let Some(prev) = &app.previous_view {
                match prev.as_ref() {
                    View::ProcessManager => draw_process_manager(f, app, chunks[1]),
                    View::PackageDetail(state) => draw_package_detail(f, state, chunks[1]),
                    _ => draw_main_menu(f, app, chunks[1]),
                }
            } else {
                draw_main_menu(f, app, chunks[1]);
            }
            draw_confirm_dialog(f, msg, action.clone(), f.area());
        }
    }

    draw_status_bar(f, app, chunks[2]);
}

/// Draw header with logo and environment info
fn draw_header(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Length(20),  // Logo
            Constraint::Min(20),     // Environment
            Constraint::Length(25),  // Time
        ])
        .split(area);

    // Logo
    let logo = Paragraph::new(Text::from(vec![
        Line::from(vec![
            Span::styled("ros2dev", t.logo_primary()),
            Span::raw(" "),
            Span::styled("rust", t.logo_secondary()),
        ]),
    ]))
    .block(Block::default()
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border()))
    .alignment(Alignment::Center);
    f.render_widget(logo, chunks[0]);

    // Environment info
    let env_text = if app.ros2_env.is_sourced {
        format!(
            "{} ROS 2 {} {} {} packages",
            icons.success,
            app.ros2_env.distro.clone().unwrap_or_default(),
            icons.bullet,
            app.ros2_workspace.as_ref().map(|w| w.packages.len()).unwrap_or(0)
        )
    } else {
        format!("{} ROS 2 not sourced", icons.warning)
    };

    let env = Paragraph::new(env_text)
        .style(if app.ros2_env.is_sourced { t.success() } else { t.error() })
        .block(Block::default()
            .borders(Borders::ALL)
            .border_type(t.border_type)
            .border_style(t.border()))
        .alignment(Alignment::Center);
    f.render_widget(env, chunks[1]);

    // Current time
    let time = chrono::Local::now().format("%H:%M:%S").to_string();
    let time_widget = Paragraph::new(format!("{} {}", icons.time, time))
        .style(t.text())
        .block(Block::default()
            .borders(Borders::ALL)
            .border_type(t.border_type)
            .border_style(t.border()))
        .alignment(Alignment::Center);
    f.render_widget(time_widget, chunks[2]);
}

/// Draw main menu (Claude Code style)
fn draw_main_menu(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(40),  // Menu
            Constraint::Percentage(60),  // Preview
        ])
        .split(area);

    // Build grouped menu items
    let mut items: Vec<ListItem> = Vec::new();
    let mut item_index = 0;
    let grouped = MenuItem::grouped();
    
    for (cat_idx, (category, menu_items)) in grouped.iter().enumerate() {
        // Add category header
        // if cat_idx > 0 {
        //     items.push(ListItem::new(Line::from(""))); // spacer
        // }
        
        items.push(ListItem::new(Line::from(vec![
            Span::styled(
                format!(" {} ", category.label()),
                Style::default().fg(t.colors.text_muted).add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                format!("─ {}", category.description()),
                Style::default().fg(t.colors.text_dim),
            ),
        ])));
        
        // Add menu items in this category
        for item in menu_items {
            let icon = icons.menu_icon(item);
            let is_selected = item_index == app.selected_menu;
            let content = Line::from(vec![
                Span::styled(
                    format!("   {} ", icon),
                    t.secondary(),
                ),
                Span::styled(
                    item.label(),
                    if is_selected { t.primary().add_modifier(Modifier::BOLD) } else { t.text() },
                ),
            ]);
            items.push(ListItem::new(content));
            item_index += 1;
        }
    }

    let menu_block = Block::default()
        .title(Line::from(vec![
            Span::styled(" ", Style::default()),
            Span::styled("Menu", t.title()),
            Span::styled(" ", Style::default()),
        ]))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused())
        .padding(ratatui::widgets::Padding::new(1, 1, 0, 0));

    // Calculate the list position accounting for category headers and spacers
    let mut list_position = 0;
    let mut remaining = app.selected_menu;
    for (cat_idx, (_, menu_items)) in grouped.iter().enumerate() {
        // if cat_idx > 0 {
        //     list_position += 1; // spacer
        // }
        list_position += 1; // category header
        
        if remaining < menu_items.len() {
            list_position += remaining;
            break;
        }
        remaining -= menu_items.len();
        list_position += menu_items.len();
    }

    let mut list_state = ListState::default();
    list_state.select(Some(list_position));

    let menu = List::new(items)
        .block(menu_block)
        .highlight_style(t.list_highlight())
        .highlight_symbol(icons::HIGHLIGHT_SYMBOL);

    f.render_stateful_widget(menu, chunks[0], &mut list_state);

    // Preview/Description panel
    let selected_item = &app.menu_items[app.selected_menu];
    let selected_icon = icons.menu_icon(selected_item);
    let preview_text = vec![
        Line::from(""),
        Line::from(vec![
            Span::styled(
                format!(" {}  ", selected_icon),
                t.secondary().add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                selected_item.label(),
                t.primary().add_modifier(Modifier::BOLD),
            ),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::raw(" "),
            Span::styled(selected_item.description(), t.text()),
        ]),
        Line::from(""),
        Line::from(""),
        Line::from(vec![
            Span::styled(" Keyboard Shortcuts:", t.text_muted().add_modifier(Modifier::BOLD)),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("   Enter", t.success()),
            Span::styled("   → Select", t.text_muted()),
        ]),
        Line::from(vec![
            Span::styled("   /", t.success()),
            Span::styled("       → Search", t.text_muted()),
        ]),
        Line::from(vec![
            Span::styled("   ?", t.success()),
            Span::styled("       → Help", t.text_muted()),
        ]),
        Line::from(vec![
            Span::styled("   Ctrl+R", t.success()),
            Span::styled(" → Refresh", t.text_muted()),
        ]),
        Line::from(vec![
            Span::styled("   q/Esc", t.success()),
            Span::styled("  → Quit", t.text_muted()),
        ]),
    ];

    let preview = Paragraph::new(preview_text)
        .block(
            Block::default()
                .title(Line::from(vec![
                    Span::styled(" ", Style::default()),
                    Span::styled("Preview", t.title()),
                    Span::styled(" ", Style::default()),
                ]))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border()),
        )
        .wrap(Wrap { trim: true });

    f.render_widget(preview, chunks[1]);

    // Search bar (if in search mode)
    if app.input_mode == InputMode::Search {
        draw_search_bar(f, app, area);
    }
}

/// Draw search bar popup
fn draw_search_bar(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let search_area = centered_rect(60, 3, area);
    f.render_widget(Clear, search_area);
    
    let search_text = format!("/{}", app.search_query);
    let search = Paragraph::new(search_text)
        .style(t.secondary())
        .block(
            Block::default()
                .title(" Search ")
                .borders(Borders::ALL)
                .border_type(t.border_type_focused)
                .border_style(t.border_active()),
        );
    
    f.render_widget(search, search_area);
    
    // Show cursor
    f.set_cursor_position((
        search_area.x + 2 + app.search_cursor as u16,
        search_area.y + 1,
    ));
}

/// Draw dashboard
fn draw_dashboard(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(8),  // Stats
            Constraint::Min(5),     // Info
        ])
        .split(area);

    // Stats cards
    let stats_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(25),
            Constraint::Percentage(25),
            Constraint::Percentage(25),
            Constraint::Percentage(25),
        ])
        .split(chunks[0]);

    let package_count = app.ros2_workspace.as_ref().map(|w| w.packages.len()).unwrap_or(0);
    let node_count = app.ros2_workspace.as_ref().map(|w| w.all_nodes().len()).unwrap_or(0);
    let running_count = app.processes.iter().filter(|p| p.status == ProcessStatus::Running).count();
    let topic_count = app.topics.len();

    draw_stat_card(f, &format!("{} Packages", icons.package), package_count, t.colors.info, stats_chunks[0]);
    draw_stat_card(f, &format!("{} Nodes", icons.node), node_count, t.colors.success, stats_chunks[1]);
    draw_stat_card(f, &format!("{} Running", icons.running), running_count, t.colors.warning, stats_chunks[2]);
    draw_stat_card(f, &format!("{} Topics", icons.topic), topic_count, t.colors.primary, stats_chunks[3]);

    // Workspace info
    let ws_info = if let Some(ws) = &app.ros2_workspace {
        vec![
            Line::from(vec![
                Span::styled(" Workspace: ", t.text_muted()),
                Span::styled(
                    app.workspace_path.display().to_string(),
                    t.text(),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled(format!(" {} Packages:", icons.package), t.primary().add_modifier(Modifier::BOLD)),
            ]),
        ]
        .into_iter()
        .chain(ws.packages.iter().take(10).map(|pkg| {
            Line::from(vec![
                Span::styled(format!("   {} ", icons.bullet), t.text_muted()),
                Span::styled(&pkg.name, t.text()),
                Span::styled(format!(" ({})", pkg.build_type), t.text_dim()),
            ])
        }))
        .collect()
    } else {
        vec![Line::from(vec![
            Span::styled(format!(" {} No ROS 2 workspace found", icons.warning), t.error()),
        ])]
    };

    let info = Paragraph::new(ws_info)
        .block(
            Block::default()
                .title(Line::from(vec![
                    Span::styled(" ", Style::default()),
                    Span::styled("Workspace Info", t.title()),
                    Span::styled(" ", Style::default()),
                ]))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border()),
        )
        .wrap(Wrap { trim: true });
    
    f.render_widget(info, chunks[1]);
}

/// Draw stat card
fn draw_stat_card(f: &mut Frame, title: &str, value: usize, color: Color, area: Rect) {
    let t = theme::current();
    let content = vec![
        Line::from(""),
        Line::from(vec![Span::styled(
            format!("{}", value),
            Style::default().fg(color).add_modifier(Modifier::BOLD),
        )]),
        Line::from(""),
        Line::from(vec![Span::styled(title, t.text_muted())]),
    ];

    let card = Paragraph::new(content)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(Style::default().fg(color)),
        )
        .alignment(Alignment::Center);
    
    f.render_widget(card, area);
}

/// Draw explorer view (generic for packages, nodes, topics, etc.)
fn draw_explorer(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(50),  // List
            Constraint::Percentage(50),  // Details
        ])
        .split(area);

    let title = match app.view {
        View::PackageExplorer => format!("{} Packages", icons.package),
        View::NodeExplorer => format!("{} Nodes", icons.node),
        View::LaunchExplorer => format!("{} Launch Files", icons.launch),
        View::TopicExplorer => format!("{} Topics", icons.topic),
        View::ServiceExplorer => format!("{} Services", icons.service),
        View::ActionExplorer => format!("{} Actions", icons.action),
        _ => "Explorer".to_string(),
    };

    // Filter items
    let filtered_items = app.filtered_explorer_items();

    // Build list items
    let items: Vec<ListItem> = filtered_items
        .iter()
        .enumerate()
        .map(|(i, item)| {
            let (icon, name, detail) = match item {
                ExplorerItem::Package(pkg) => (icons.package, pkg.name.clone(), pkg.build_type.to_string()),
                ExplorerItem::Node(pkg, node) => (icons.node, node.clone(), pkg.clone()),
                ExplorerItem::LaunchFile(pkg, path) => {
                    let filename = path.file_name().unwrap_or_default().to_string_lossy().to_string();
                    (icons.launch, filename, pkg.clone())
                }
                ExplorerItem::Topic(topic) => (icons.topic, topic.name.clone(), topic.msg_type.clone()),
                ExplorerItem::Service(service) => (icons.service, service.name.clone(), service.srv_type.clone()),
                ExplorerItem::Action(action) => (icons.action, action.name.clone(), action.action_type.clone()),
                ExplorerItem::RunningNode(node) => (icons.running, node.name.clone(), node.namespace.clone()),
            };

            let is_selected = i == app.selected_item;
            let content = Line::from(vec![
                Span::styled(format!(" {} ", icon), t.secondary()),
                Span::styled(
                    name,
                    if is_selected { t.primary().add_modifier(Modifier::BOLD) } else { t.text() },
                ),
                Span::raw(" "),
                Span::styled(detail, t.text_dim()),
            ]);
            ListItem::new(content)
        })
        .collect();

    let search_hint = if app.search_query.is_empty() {
        "[/ Search]".to_string()
    } else {
        format!("[Filter: {}]", app.search_query)
    };

    let list_block = Block::default()
        .title(Line::from(vec![
            Span::styled(format!(" {} ", title), t.title()),
            Span::styled(
                format!("({}) ", filtered_items.len()),
                t.text_muted(),
            ),
            Span::styled(search_hint, t.text_dim()),
        ]))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());

    let mut list_state = ListState::default();
    if !filtered_items.is_empty() {
        list_state.select(Some(app.selected_item.min(filtered_items.len() - 1)));
    }

    let list = List::new(items)
        .block(list_block)
        .highlight_style(t.list_highlight())
        .highlight_symbol(icons::HIGHLIGHT_SYMBOL);

    f.render_stateful_widget(list, chunks[0], &mut list_state);

    // Details panel
    let detail_content = if let Some(item) = filtered_items.get(app.selected_item) {
        match item {
            ExplorerItem::Package(pkg) => vec![
                Line::from(vec![Span::styled(" Package Details", t.title())]),
                Line::from(""),
                Line::from(vec![
                    Span::styled(" Name: ", t.text_muted()),
                    Span::styled(&pkg.name, t.text()),
                ]),
                Line::from(vec![
                    Span::styled(" Version: ", t.text_muted()),
                    Span::styled(&pkg.version, t.text()),
                ]),
                Line::from(vec![
                    Span::styled(" Build: ", t.text_muted()),
                    Span::styled(pkg.build_type.to_string(), t.success()),
                ]),
                Line::from(vec![
                    Span::styled(" Path: ", t.text_muted()),
                    Span::styled(pkg.path.display().to_string(), t.secondary()),
                ]),
                Line::from(""),
                Line::from(vec![Span::styled(" Actions:", t.secondary())]),
                Line::from(vec![Span::styled("   [Enter] Run node", t.text())]),
                Line::from(vec![Span::styled("   [b] Build package", t.text())]),
                Line::from(vec![Span::styled("   [r] Refresh", t.text())]),
                Line::from(""),
                Line::from(vec![Span::styled(format!(" {} Nodes:", icons.node), t.primary())]),
            ]
            .into_iter()
            .chain(pkg.nodes.iter().map(|n| {
                Line::from(vec![Span::styled(format!("   {} {}", icons.bullet, n), t.text())])
            }))
            .collect(),
            ExplorerItem::Topic(topic) => {
                let mut lines = vec![
                    Line::from(vec![Span::styled(" Topic Details", t.title())]),
                    Line::from(""),
                    Line::from(vec![
                        Span::styled(" Name: ", t.text_muted()),
                        Span::styled(&topic.name, t.text()),
                    ]),
                    Line::from(vec![
                        Span::styled(" Type: ", t.text_muted()),
                        Span::styled(&topic.msg_type, t.success()),
                    ]),
                    Line::from(""),
                    Line::from(vec![Span::styled(" Actions:", t.secondary())]),
                    Line::from(vec![Span::styled("   [e] Echo/Subscribe", t.text())]),
                    Line::from(vec![Span::styled("   [p] Publish message", t.text())]),
                    Line::from(vec![Span::styled("   [i] Show interface", t.text())]),
                ];
                if topic.publishers > 0 {
                    lines.push(Line::from(""));
                    lines.push(Line::from(vec![Span::styled(
                        format!(" {} Publishers: {}", icons.running, topic.publishers),
                        t.primary(),
                    )]));
                }
                lines
            }
            ExplorerItem::Service(service) => vec![
                Line::from(vec![Span::styled(" Service Details", t.title())]),
                Line::from(""),
                Line::from(vec![
                    Span::styled(" Name: ", t.text_muted()),
                    Span::styled(&service.name, t.text()),
                ]),
                Line::from(vec![
                    Span::styled(" Type: ", t.text_muted()),
                    Span::styled(&service.srv_type, t.success()),
                ]),
                Line::from(""),
                Line::from(vec![Span::styled(" Actions:", t.secondary())]),
                Line::from(vec![Span::styled("   [c] Call service", t.text())]),
                Line::from(vec![Span::styled("   [i] Show interface", t.text())]),
            ],
            _ => vec![Line::from(Span::styled(" Select an item to view details", t.text_muted()))],
        }
    } else {
        vec![Line::from(Span::styled(" No items", t.text_muted()))]
    };

    let details = Paragraph::new(detail_content)
        .block(
            Block::default()
                .title(Line::from(vec![
                    Span::styled(" ", Style::default()),
                    Span::styled("Details", t.title()),
                    Span::styled(" ", Style::default()),
                ]))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border()),
        )
        .wrap(Wrap { trim: true });

    f.render_widget(details, chunks[1]);

    // Search bar (if in search mode)
    if app.input_mode == InputMode::Search {
        draw_search_bar(f, app, area);
    }
}

/// Draw package detail view (folder-like explorer for nodes and launch files)
fn draw_package_detail(f: &mut Frame, state: &PackageDetailState, area: Rect) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(4),  // Header
            Constraint::Length(3),  // Tabs
            Constraint::Min(10),    // Content
            Constraint::Length(3),  // Help
        ])
        .margin(1)
        .split(area);

    let t = theme::current();
    let icons = icons::current();

    let outer_block = Block::default()
        .title(format!(" {} {} ", icons.package, state.package_name))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());
    f.render_widget(outer_block, area);

    // Header with package info
    let header = vec![
        Line::from(vec![
            Span::styled("Package: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.package_name, Style::default().fg(t.colors.warning).bold()),
            Span::styled("  │  Type: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.build_type, Style::default().fg(t.colors.success)),
        ]),
        Line::from(vec![
            Span::styled("Path: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(
                state.package_path.to_string_lossy().to_string(),
                Style::default().fg(t.colors.text_dim),
            ),
        ]),
    ];
    let header_widget = Paragraph::new(header);
    f.render_widget(header_widget, chunks[0]);

    // Tabs with 4 items
    let tab_titles = vec![
        format!(" {} Nodes ({}) ", icons.node, state.nodes.len()),
        format!(" {} Launch ({}) ", icons.launch, state.launch_files.len()),
        format!(" {} New Node ", icons.add),
        format!(" {} New Launch ", icons.add),
    ];
    let tabs = Tabs::new(tab_titles)
        .select(state.selected_tab)
        .style(Style::default().fg(t.colors.text_muted))
        .highlight_style(Style::default().fg(t.colors.warning).bold().bg(t.colors.bg_highlight))
        .divider(Span::styled("│", Style::default().fg(t.colors.text_muted)));
    f.render_widget(tabs, chunks[1]);

    // Content based on selected tab
    let items: Vec<ListItem> = match state.selected_tab {
        0 => {
            // Nodes
            if state.nodes.is_empty() {
                vec![ListItem::new(Line::from(vec![
                    Span::styled("  No nodes found. Press ", Style::default().fg(t.colors.text_muted)),
                    Span::styled("Tab", Style::default().fg(t.colors.warning)),
                    Span::styled(" to create one.", Style::default().fg(t.colors.text_muted)),
                ]))]
            } else {
                state.nodes
                    .iter()
                    .enumerate()
                    .map(|(i, node)| {
                        let is_selected = i == state.selected_item;
                        let content = Line::from(vec![
                            Span::styled(
                                if is_selected { icons::HIGHLIGHT_SYMBOL } else { "   " },
                                Style::default().fg(t.colors.warning),
                            ),
                            Span::styled(format!("{} ", icons.node), Style::default().fg(t.colors.primary)),
                            Span::styled(
                                node,
                                if is_selected {
                                    Style::default().fg(t.colors.warning).bold()
                                } else {
                                    Style::default().fg(t.colors.text)
                                },
                            ),
                        ]);
                        ListItem::new(content)
                    })
                    .collect()
            }
        }
        1 => {
            // Launch files
            if state.launch_files.is_empty() {
                vec![ListItem::new(Line::from(vec![
                    Span::styled("  No launch files found. Press ", Style::default().fg(t.colors.text_muted)),
                    Span::styled("Tab", Style::default().fg(t.colors.warning)),
                    Span::styled(" to create one.", Style::default().fg(t.colors.text_muted)),
                ]))]
            } else {
                state.launch_files
                    .iter()
                    .enumerate()
                    .map(|(i, (name, _path))| {
                        let is_selected = i == state.selected_item;
                        let ext_color = if name.ends_with(".py") {
                            t.colors.success
                        } else if name.ends_with(".xml") {
                            Color::Magenta
                        } else if name.ends_with(".yaml") || name.ends_with(".yml") {
                            t.colors.warning
                        } else {
                            t.colors.text
                        };
                        let content = Line::from(vec![
                            Span::styled(
                                if is_selected { icons::HIGHLIGHT_SYMBOL } else { "   " },
                                Style::default().fg(t.colors.warning),
                            ),
                            Span::styled(format!("{} ", icons.launch), Style::default().fg(t.colors.success)),
                            Span::styled(
                                name,
                                if is_selected {
                                    Style::default().fg(t.colors.warning).bold()
                                } else {
                                    Style::default().fg(ext_color)
                                },
                            ),
                        ]);
                        ListItem::new(content)
                    })
                    .collect()
            }
        }
        2 => {
            // + New Node tab
            vec![
                ListItem::new(Line::from("")),
                ListItem::new(Line::from(vec![
                    Span::styled("  Press ", Style::default().fg(t.colors.text_muted)),
                    Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
                    Span::styled(" to create a new node in package ", Style::default().fg(t.colors.text_muted)),
                    Span::styled(&state.package_name, Style::default().fg(t.colors.warning)),
                ])),
                ListItem::new(Line::from("")),
                ListItem::new(Line::from(vec![
                    Span::styled("  This will open the node creation wizard.", Style::default().fg(t.colors.text_dim)),
                ])),
            ]
        }
        3 => {
            // + New Launch tab
            vec![
                ListItem::new(Line::from("")),
                ListItem::new(Line::from(vec![
                    Span::styled("  Press ", Style::default().fg(t.colors.text_muted)),
                    Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
                    Span::styled(" to create a new launch file in package ", Style::default().fg(t.colors.text_muted)),
                    Span::styled(&state.package_name, Style::default().fg(t.colors.warning)),
                ])),
                ListItem::new(Line::from("")),
                ListItem::new(Line::from(vec![
                    Span::styled("  This will open the launch file creation wizard.", Style::default().fg(t.colors.text_dim)),
                ])),
            ]
        }
        _ => vec![],
    };

    let content_block = Block::default()
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(Style::default().fg(t.colors.secondary));
    
    let mut list_state = ListState::default();
    if state.items_count() > 0 && state.selected_tab < 2 {
        list_state.select(Some(state.selected_item.min(state.items_count().saturating_sub(1))));
    }

    let list = List::new(items)
        .block(content_block)
        .highlight_style(Style::default().bg(t.colors.bg_highlight));

    f.render_stateful_widget(list, chunks[2], &mut list_state);

    // Help bar - different based on tab
    let help = if state.selected_tab < 2 {
        Paragraph::new(Line::from(vec![
            Span::styled("[Enter]", Style::default().fg(t.colors.success).bold()),
            Span::styled(" Run  ", Style::default().fg(t.colors.text)),
            Span::styled("[R]", Style::default().fg(t.colors.secondary).bold()),
            Span::styled(" Run Background  ", Style::default().fg(t.colors.text)),
            Span::styled("[Tab/←→]", Style::default().fg(t.colors.warning).bold()),
            Span::styled(" Switch Tab  ", Style::default().fg(t.colors.text)),
            Span::styled("[b]", Style::default().fg(Color::Magenta).bold()),
            Span::styled(" Build  ", Style::default().fg(t.colors.text)),
            Span::styled("[Esc]", Style::default().fg(t.colors.error).bold()),
            Span::styled(" Back", Style::default().fg(t.colors.text)),
        ]))
    } else {
        Paragraph::new(Line::from(vec![
            Span::styled("[Enter]", Style::default().fg(t.colors.success).bold()),
            Span::styled(" Create  ", Style::default().fg(t.colors.text)),
            Span::styled("[Tab/←→]", Style::default().fg(t.colors.warning).bold()),
            Span::styled(" Switch Tab  ", Style::default().fg(t.colors.text)),
            Span::styled("[Esc]", Style::default().fg(t.colors.error).bold()),
            Span::styled(" Back", Style::default().fg(t.colors.text)),
        ]))
    };
    f.render_widget(help.alignment(Alignment::Center), chunks[3]);
}

/// Draw process manager
fn draw_process_manager(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(35),  // Process list
            Constraint::Percentage(65),  // Logs
        ])
        .split(area);

    // Process list
    let items: Vec<ListItem> = app
        .processes
        .iter()
        .enumerate()
        .map(|(i, process)| {
            let status_icon = icons.status_icon(&process.status);
            let status_color = match &process.status {
                ProcessStatus::Pending => t.colors.warning,
                ProcessStatus::Running => t.colors.success,
                ProcessStatus::Stopped => t.colors.error,
                ProcessStatus::Exited(0) => t.colors.success,
                ProcessStatus::Exited(_) => t.colors.error,
                ProcessStatus::Failed(_) => t.colors.error,
            };

            let is_selected = i == app.selected_process;
            let content = Line::from(vec![
                Span::styled(format!(" {} ", status_icon), Style::default().fg(status_color)),
                Span::styled(
                    &process.name,
                    if is_selected { t.primary().add_modifier(Modifier::BOLD) } else { t.text() },
                ),
            ]);
            ListItem::new(content)
        })
        .collect();

    let process_block = Block::default()
        .title(Line::from(vec![
            Span::styled(format!(" {} Processes ", icons.process), t.title()),
            Span::styled(
                format!("({})", app.processes.len()),
                t.text_muted(),
            ),
        ]))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());

    let mut list_state = ListState::default();
    if !app.processes.is_empty() {
        list_state.select(Some(app.selected_process));
    }

    let process_list = List::new(items)
        .block(process_block)
        .highlight_style(t.list_highlight())
        .highlight_symbol(icons::HIGHLIGHT_SYMBOL);

    f.render_stateful_widget(process_list, chunks[0], &mut list_state);

    // Log viewer
    let log_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),  // Process info
            Constraint::Min(5),     // Logs
            Constraint::Length(3),  // Input
        ])
        .split(chunks[1]);

    // Process info
    if let Some(process) = app.processes.get(app.selected_process) {
        let status_text = match &process.status {
            ProcessStatus::Pending => format!("{} Pending", icons.pending),
            ProcessStatus::Running => format!("{} Running (PID: {})", icons.running, process.pid.unwrap_or(0)),
            ProcessStatus::Stopped => format!("{} Stopped", icons.stopped),
            ProcessStatus::Exited(code) => format!("{} Exited ({})", if *code == 0 { icons.success } else { icons.error }, code),
            ProcessStatus::Failed(err) => format!("{} Failed: {}", icons.error, err),
        };

        // Make command path relative to workspace
        let display_command = make_relative_path(&process.command, &app.workspace_path);

        let info = Paragraph::new(vec![
            Line::from(vec![
                Span::styled(" Command: ", t.text_muted()),
                Span::styled(display_command, t.secondary()),
            ]),
            Line::from(vec![
                Span::styled(" Status: ", t.text_muted()),
                Span::styled(status_text, t.text()),
            ]),
        ])
        .block(Block::default()
            .borders(Borders::ALL)
            .border_type(t.border_type)
            .border_style(t.border()));
        
        f.render_widget(info, log_chunks[0]);
    }

    // Logs
    let is_log_select = app.input_mode == InputMode::LogSelect;
    let (sel_start, sel_end) = if is_log_select {
        let start = app.log_select_start.unwrap_or(app.log_select_end);
        (start.min(app.log_select_end), start.max(app.log_select_end))
    } else {
        (0, 0)
    };
    
    let log_lines: Vec<Line> = app
        .log_buffer
        .lines()
        .enumerate()
        .map(|(i, line)| {
            let base_style = if line.is_stderr { t.error() } else { t.text() };
            
            if is_log_select {
                // Highlight current cursor line
                if i == app.log_select_end {
                    let cursor_style = Style::default().fg(t.colors.bg).bg(t.colors.primary);
                    return Line::styled(format!("{} {}", icons.chevron, &line.content), cursor_style);
                }
                // Highlight selected range
                if app.log_select_start.is_some() && i >= sel_start && i <= sel_end {
                    let sel_style = Style::default().fg(t.colors.text).bg(t.colors.bg_highlight);
                    return Line::styled(format!("  {}", &line.content), sel_style);
                }
                // Non-selected line in select mode
                Line::styled(format!("  {}", &line.content), base_style)
            } else {
                Line::styled(&line.content, base_style)
            }
        })
        .collect();

    let log_title = if is_log_select {
        Line::from(vec![
            Span::styled(" Select Output ", Style::default().fg(t.colors.bg).bg(t.colors.warning)),
            Span::styled(
                format!(" (line {}/{})", app.log_select_end + 1, app.log_buffer.len()),
                t.text_muted(),
            ),
        ])
    } else {
        Line::from(vec![
            Span::styled(" Output ", t.title()),
            Span::styled(
                format!("({} lines)", app.log_buffer.len()),
                t.text_muted(),
            ),
        ])
    };

    let log_block = Block::default()
        .title(log_title)
        .borders(Borders::ALL)
        .border_type(if is_log_select { t.border_type_focused } else { t.border_type })
        .border_style(if is_log_select { t.border_focused() } else { t.border() });

    let logs = Paragraph::new(log_lines)
        .block(log_block)
        .scroll((app.log_scroll as u16, 0))
        .wrap(Wrap { trim: false });

    f.render_widget(logs, log_chunks[1]);

    // Scrollbar
    let scrollbar = Scrollbar::default()
        .orientation(ScrollbarOrientation::VerticalRight)
        .begin_symbol(Some(icons.arrow_up))
        .end_symbol(Some(icons.arrow_down));

    let mut scrollbar_state = ScrollbarState::new(app.log_buffer.len())
        .position(app.log_scroll);

    f.render_stateful_widget(
        scrollbar,
        log_chunks[1].inner(ratatui::layout::Margin {
            vertical: 1,
            horizontal: 0,
        }),
        &mut scrollbar_state,
    );

    // Input area
    let (input_text, input_style, input_border_type, input_border_style) = if is_log_select {
        let sel_info = if app.log_select_start.is_some() {
            let s = app.log_select_start.unwrap_or(app.log_select_end);
            let count = s.max(app.log_select_end) - s.min(app.log_select_end) + 1;
            format!(" ({} lines selected)", count)
        } else {
            String::new()
        };
        (
            format!("[↑↓] Move  [Space] Select  [y/Enter] Copy  [Y] Copy all  [Esc] Cancel{}", sel_info),
            Style::default().fg(t.colors.warning),
            t.border_type_focused,
            Style::default().fg(t.colors.warning),
        )
    } else if app.input_mode == InputMode::Input {
        (
            format!("{} {}", icons.chevron, app.input_buffer),
            t.secondary(),
            t.border_type_focused,
            t.border_active(),
        )
    } else {
        (
            format!("[n] New  [e] Edit  [s] Start/Stop  [r] Restart  [d] Delete  [o] Copy  [S] Stop all"),
            t.text_dim(),
            t.border_type,
            t.border(),
        )
    };

    let input = Paragraph::new(input_text)
        .style(input_style)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(input_border_type)
                .border_style(input_border_style),
        );

    f.render_widget(input, log_chunks[2]);

    // Cursor for input
    if app.input_mode == InputMode::Input {
        f.set_cursor_position((
            log_chunks[2].x + 3 + app.input_cursor as u16,
            log_chunks[2].y + 1,
        ));

        // Draw suggestions dropdown if active
        if app.show_suggestions && !app.suggestions.is_empty() {
            let suggestion_height = (app.suggestions.len() + 2).min(6) as u16; // max 4 items + 2 for border
            let suggestion_area = Rect {
                x: log_chunks[2].x,
                y: log_chunks[2].y.saturating_sub(suggestion_height),
                width: log_chunks[2].width,
                height: suggestion_height,
            };

            // Clear the area first
            f.render_widget(Clear, suggestion_area);

            let suggestion_items: Vec<Line> = app
                .suggestions
                .iter()
                .enumerate()
                .map(|(i, (name, cmd))| {
                    let is_selected = i == app.selected_suggestion;
                    let style = if is_selected {
                        Style::default().fg(t.colors.bg).bg(t.colors.warning)
                    } else {
                        Style::default().fg(t.colors.text)
                    };
                    let prefix = if is_selected { icons::HIGHLIGHT_SYMBOL } else { "  " };
                    Line::from(vec![
                        Span::styled(prefix, style),
                        Span::styled(format!("{} ", name), style.add_modifier(Modifier::BOLD)),
                        Span::styled(format!("{} {}", icons.bullet, cmd), style.fg(if is_selected { t.colors.bg } else { t.colors.text_muted })),
                    ])
                })
                .collect();

            let suggestions_block = Block::default()
                .title(format!(" {} Saved Commands (Tab/↑↓ to select) ", icons.save))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border_focused())
                .style(Style::default().bg(t.colors.bg));

            let suggestions_widget = Paragraph::new(suggestion_items)
                .block(suggestions_block);

            f.render_widget(suggestions_widget, suggestion_area);
        }
    }
}

/// Draw create package wizard
fn draw_create_package(f: &mut Frame, _app: &App, state: CreatePackageState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let block = Block::default()
        .title(format!(" {} Create New Package ", icons.add))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());

    let inner = block.inner(area);
    f.render_widget(block, area);

    let fields: Vec<(&str, String, usize)> = vec![
        ("Package Name", state.name.clone(), 0),
        ("Build Type", if state.build_type == 0 { "ament_python".to_string() } else { "ament_cmake".to_string() }, 1),
        ("Description", state.description.clone(), 2),
        ("Maintainer", state.maintainer.clone(), 3),
        ("Email", state.email.clone(), 4),
        ("Dependencies", state.dependencies.clone(), 5),
        ("Initial Node", state.initial_node.clone(), 6),
    ];

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints(
            std::iter::repeat(Constraint::Length(3))
                .take(fields.len())
                .chain(std::iter::once(Constraint::Length(3)))
                .collect::<Vec<_>>(),
        )
        .split(inner);

    for (i, (label, value, _)) in fields.iter().enumerate() {
        let is_active = state.active_field == i;
        let style = if is_active {
            Style::default().fg(t.colors.warning).bold()
        } else {
            Style::default().fg(t.colors.text)
        };

        let display_value = if i == 1 {
            // Build type selector
            format!("[{}] ament_python  [{}] ament_cmake",
                if state.build_type == 0 { icons.running } else { icons.stopped },
                if state.build_type == 1 { icons.running } else { icons.stopped })
        } else {
            value.to_string()
        };

        let field = if is_active && i != 1 {
            // Show cursor only for text fields, not build type selector
            let field_text = match i {
                0 => &state.name,
                2 => &state.description,
                3 => &state.maintainer,
                4 => &state.email,
                5 => &state.dependencies,
                6 => &state.initial_node,
                _ => "",
            };
            let cursor_pos = state.field_cursor.min(field_text.len());
            let (before_cursor, after_cursor) = field_text.split_at(cursor_pos);
            Paragraph::new(Line::from(vec![
                Span::styled(format!("{}: ", label), Style::default().fg(t.colors.text_muted)),
                Span::styled(before_cursor, style),
                Span::styled("┃", Style::default().fg(t.colors.warning).bg(t.colors.bg)),
                Span::styled(after_cursor, style),
            ]))
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .border_type(if is_active { BorderType::Double } else { t.border_type })
                    .border_style(if is_active {
                        Style::default().fg(t.colors.warning)
                    } else {
                        Style::default().fg(t.colors.border)
                    }),
            )
        } else {
            Paragraph::new(Line::from(vec![
                Span::styled(format!("{}: ", label), Style::default().fg(t.colors.text_muted)),
                Span::styled(display_value, style),
            ]))
        }
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(if is_active { BorderType::Double } else { t.border_type })
                .border_style(if is_active {
                    Style::default().fg(t.colors.warning)
                } else {
                    Style::default().fg(t.colors.border)
                }),
        );

        f.render_widget(field, chunks[i]);
    }

    // Create button
    let button = Paragraph::new(Line::from(vec![
        Span::styled("  [Enter] Create Package  ", Style::default().fg(t.colors.bg).bg(t.colors.success)),
        Span::raw("  "),
        Span::styled("  [Esc] Cancel  ", Style::default().fg(t.colors.bg).bg(t.colors.error)),
    ]))
    .alignment(Alignment::Center);
    
    f.render_widget(button, chunks[fields.len()]);
}

/// Draw create node wizard
fn draw_create_node(f: &mut Frame, _app: &App, state: CreateNodeState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let block = Block::default()
        .title(format!(" {} Create New Node ", icons.node))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());

    let inner = block.inner(area);
    f.render_widget(block, area);

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),  // Package
            Constraint::Length(3),  // Node name
            Constraint::Length(3),  // Type
            Constraint::Length(3),  // Template
            Constraint::Length(3),  // Button
            Constraint::Min(0),
        ])
        .split(inner);

    // Package field
    let pkg_field = if state.active_field == 0 {
        let cursor_pos = state.field_cursor.min(state.package.len());
        let (before_cursor, after_cursor) = state.package.split_at(cursor_pos);
        Paragraph::new(Line::from(vec![
            Span::styled("Package: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(before_cursor, Style::default().fg(t.colors.warning).bold()),
            Span::styled("┃", Style::default().fg(t.colors.warning).bg(t.colors.bg)),
            Span::styled(after_cursor, Style::default().fg(t.colors.warning).bold()),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Double)
                .border_style(Style::default().fg(t.colors.warning)),
        )
    } else {
        Paragraph::new(Line::from(vec![
            Span::styled("Package: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.package, Style::default().fg(t.colors.text)),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(Style::default().fg(t.colors.border)),
        )
    };
    f.render_widget(pkg_field, chunks[0]);

    // Node name field
    let name_field = if state.active_field == 1 {
        let cursor_pos = state.field_cursor.min(state.node_name.len());
        let (before_cursor, after_cursor) = state.node_name.split_at(cursor_pos);
        Paragraph::new(Line::from(vec![
            Span::styled("Node Name: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(before_cursor, Style::default().fg(t.colors.warning).bold()),
            Span::styled("┃", Style::default().fg(t.colors.warning).bg(t.colors.bg)),
            Span::styled(after_cursor, Style::default().fg(t.colors.warning).bold()),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Double)
                .border_style(Style::default().fg(t.colors.warning)),
        )
    } else {
        Paragraph::new(Line::from(vec![
            Span::styled("Node Name: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.node_name, Style::default().fg(t.colors.text)),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(Style::default().fg(t.colors.border)),
        )
    };
    f.render_widget(name_field, chunks[1]);

    // Type selector
    let type_text = format!("[{}] Python  [{}] C++",
        if state.node_type == 0 { icons.running } else { icons.stopped },
        if state.node_type == 1 { icons.running } else { icons.stopped });
    let type_field = Paragraph::new(Line::from(vec![
        Span::styled("Type: ", Style::default().fg(t.colors.text_muted)),
        Span::styled(type_text, Style::default().fg(t.colors.text)),
    ]))
    .block(
        Block::default()
            .borders(Borders::ALL)
            .border_type(if state.active_field == 2 { BorderType::Double } else { t.border_type })
            .border_style(if state.active_field == 2 {
                Style::default().fg(t.colors.warning)
            } else {
                Style::default().fg(t.colors.border)
            }),
    );
    f.render_widget(type_field, chunks[2]);

    // Template selector
    let templates = ["Basic", "Subscriber", "Publisher", "Service"];
    let template_text = templates
        .iter()
        .enumerate()
        .map(|(i, tpl)| format!("[{}] {}", if state.template == i { icons.running } else { icons.stopped }, tpl))
        .collect::<Vec<_>>()
        .join("  ");
    let template_field = Paragraph::new(Line::from(vec![
        Span::styled("Template: ", Style::default().fg(t.colors.text_muted)),
        Span::styled(template_text, Style::default().fg(t.colors.text)),
    ]))
    .block(
        Block::default()
            .borders(Borders::ALL)
            .border_type(if state.active_field == 3 { BorderType::Double } else { t.border_type })
            .border_style(if state.active_field == 3 {
                Style::default().fg(t.colors.warning)
            } else {
                Style::default().fg(t.colors.border)
            }),
    );
    f.render_widget(template_field, chunks[3]);

    // Button
    let button = Paragraph::new(Line::from(vec![
        Span::styled("  [Enter] Create Node  ", Style::default().fg(t.colors.bg).bg(t.colors.success)),
        Span::raw("  "),
        Span::styled("  [Esc] Cancel  ", Style::default().fg(t.colors.bg).bg(t.colors.error)),
    ]))
    .alignment(Alignment::Center);
    f.render_widget(button, chunks[4]);
}

/// Draw create launch wizard
fn draw_create_launch(f: &mut Frame, _app: &App, state: CreateLaunchState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let block = Block::default()
        .title(format!(" {} Create Launch File ", icons.launch))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());

    let inner = block.inner(area);
    f.render_widget(block, area);

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),  // Package
            Constraint::Length(3),  // Name
            Constraint::Length(3),  // Format
            Constraint::Length(3),  // Button
            Constraint::Min(0),
        ])
        .split(inner);

    // Package field
    let pkg_field = if state.active_field == 0 {
        let cursor_pos = state.field_cursor.min(state.package.len());
        let (before_cursor, after_cursor) = state.package.split_at(cursor_pos);
        Paragraph::new(Line::from(vec![
            Span::styled("Package: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(before_cursor, Style::default().fg(t.colors.warning).bold()),
            Span::styled("┃", Style::default().fg(t.colors.warning).bg(t.colors.bg)),
            Span::styled(after_cursor, Style::default().fg(t.colors.warning).bold()),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Double)
                .border_style(Style::default().fg(t.colors.warning)),
        )
    } else {
        Paragraph::new(Line::from(vec![
            Span::styled("Package: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.package, Style::default().fg(t.colors.text)),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(Style::default().fg(t.colors.border)),
        )
    };
    f.render_widget(pkg_field, chunks[0]);

    // Name field
    let name_field = if state.active_field == 1 {
        let cursor_pos = state.field_cursor.min(state.name.len());
        let (before_cursor, after_cursor) = state.name.split_at(cursor_pos);
        Paragraph::new(Line::from(vec![
            Span::styled("File Name: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(before_cursor, Style::default().fg(t.colors.warning).bold()),
            Span::styled("┃", Style::default().fg(t.colors.warning).bg(t.colors.bg)),
            Span::styled(after_cursor, Style::default().fg(t.colors.warning).bold()),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(BorderType::Double)
                .border_style(Style::default().fg(t.colors.warning)),
        )
    } else {
        Paragraph::new(Line::from(vec![
            Span::styled("File Name: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.name, Style::default().fg(t.colors.text)),
        ]))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(Style::default().fg(t.colors.border)),
        )
    };
    f.render_widget(name_field, chunks[1]);

    // Format selector
    let format_text = format!("[{}] Python (.launch.py)  [{}] XML (.launch.xml)",
        if state.format == 0 { icons.running } else { icons.stopped },
        if state.format == 1 { icons.running } else { icons.stopped });
    let format_field = Paragraph::new(Line::from(vec![
        Span::styled("Format: ", Style::default().fg(t.colors.text_muted)),
        Span::styled(format_text, Style::default().fg(t.colors.text)),
    ]))
    .block(
        Block::default()
            .borders(Borders::ALL)
            .border_type(if state.active_field == 2 { BorderType::Double } else { t.border_type })
            .border_style(if state.active_field == 2 {
                Style::default().fg(t.colors.warning)
            } else {
                Style::default().fg(t.colors.border)
            }),
    );
    f.render_widget(format_field, chunks[2]);

    // Button
    let button = Paragraph::new(Line::from(vec![
        Span::styled("  [Enter] Create Launch File  ", Style::default().fg(t.colors.bg).bg(t.colors.success)),
        Span::raw("  "),
        Span::styled("  [Esc] Cancel  ", Style::default().fg(t.colors.bg).bg(t.colors.error)),
    ]))
    .alignment(Alignment::Center);
    f.render_widget(button, chunks[3]);
}

/// Draw topic subscriber view
fn draw_topic_subscriber(f: &mut Frame, topic: &str, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let content = vec![
        Line::from(""),
        Line::from(vec![
            Span::styled(format!("{} Subscribe to Topic", icons.topic), Style::default().fg(t.colors.primary).bold()),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Topic: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(topic, Style::default().fg(t.colors.warning)),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Press ", Style::default().fg(t.colors.text_muted)),
            Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
            Span::styled(" to start echoing messages", Style::default().fg(t.colors.text_muted)),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Press ", Style::default().fg(t.colors.text_muted)),
            Span::styled("Esc", Style::default().fg(t.colors.error).bold()),
            Span::styled(" to cancel", Style::default().fg(t.colors.text_muted)),
        ]),
    ];

    let block = Paragraph::new(content)
        .block(
            Block::default()
                .title(format!(" {} Topic Subscriber ", icons.topic))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border_focused()),
        )
        .alignment(Alignment::Center);

    f.render_widget(block, area);
}

/// Draw topic publisher view
fn draw_topic_publisher(f: &mut Frame, app: &App, topic: &str, msg_type: &str, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let content = vec![
        Line::from(""),
        Line::from(vec![
            Span::styled(format!("{} Publish to Topic", icons.topic), Style::default().fg(t.colors.primary).bold()),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Topic: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(topic, Style::default().fg(t.colors.warning)),
        ]),
        Line::from(vec![
            Span::styled("Type: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(msg_type, Style::default().fg(t.colors.success)),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Message Data (YAML):", Style::default().fg(t.colors.text_muted)),
        ]),
        Line::from(vec![
            Span::styled(
                if app.input_buffer.is_empty() { "{}" } else { &app.input_buffer },
                Style::default().fg(t.colors.warning),
            ),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Press ", Style::default().fg(t.colors.text_muted)),
            Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
            Span::styled(" to publish", Style::default().fg(t.colors.text_muted)),
        ]),
    ];

    let block = Paragraph::new(content)
        .block(
            Block::default()
                .title(format!(" {} Topic Publisher ", icons.topic))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border_focused()),
        )
        .alignment(Alignment::Center);

    f.render_widget(block, area);
}

/// Draw service client view with interactive fields
fn draw_service_client(f: &mut Frame, state: &ServiceClientState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(5),  // Header
            Constraint::Min(10),    // Fields
            Constraint::Length(3),  // Buttons
        ])
        .margin(1)
        .split(area);

    let outer_block = Block::default()
        .title(format!(" {} Call Service ", icons.service))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(t.border_focused());
    f.render_widget(outer_block, area);

    // Header
    let header = vec![
        Line::from(vec![
            Span::styled("Service: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.service_name, Style::default().fg(t.colors.warning)),
        ]),
        Line::from(vec![
            Span::styled("Type: ", Style::default().fg(t.colors.text_muted)),
            Span::styled(&state.srv_type, Style::default().fg(t.colors.success)),
        ]),
        Line::from(""),
        Line::from(vec![
            Span::styled("Request Fields:", Style::default().fg(t.colors.primary).bold()),
        ]),
    ];
    let header_widget = Paragraph::new(header);
    f.render_widget(header_widget, chunks[0]);

    // Fields
    if state.fields.is_empty() {
        let no_fields = Paragraph::new(vec![
            Line::from(""),
            Line::from(vec![Span::styled(
                "No request fields (empty request)",
                Style::default().fg(t.colors.text_muted),
            )]),
        ])
        .alignment(Alignment::Center);
        f.render_widget(no_fields, chunks[1]);
    } else {
        let field_items: Vec<ListItem> = state.fields
            .iter()
            .enumerate()
            .map(|(i, field)| {
                let is_active = i == state.active_field;
                let indent = "  ".repeat(field.indent);
                
                let content = Line::from(vec![
                    Span::styled(
                        if is_active { icons::HIGHLIGHT_SYMBOL } else { "  " },
                        Style::default().fg(t.colors.warning),
                    ),
                    Span::styled(
                        format!("{}{}", indent, field.name),
                        Style::default().fg(t.colors.text),
                    ),
                    Span::styled(
                        format!(" ({}): ", field.field_type),
                        Style::default().fg(t.colors.text_muted),
                    ),
                    Span::styled(
                        if field.value.is_empty() {
                            format!("<{}>", field.field_type)
                        } else {
                            field.value.clone()
                        },
                        if is_active {
                            Style::default().fg(t.colors.warning).bold()
                        } else {
                            Style::default().fg(t.colors.primary)
                        },
                    ),
                    if is_active {
                        Span::styled("_", Style::default().fg(t.colors.warning))
                    } else {
                        Span::raw("")
                    },
                ]);
                ListItem::new(content)
            })
            .collect();

        let fields_list = List::new(field_items)
            .block(Block::default().borders(Borders::NONE));
        f.render_widget(fields_list, chunks[1]);
    }

    // Buttons
    let buttons = Paragraph::new(Line::from(vec![
        Span::styled("  [Enter] Call Service  ", Style::default().fg(t.colors.bg).bg(t.colors.success)),
        Span::raw("  "),
        Span::styled("  [Tab/↑↓] Navigate  ", Style::default().fg(t.colors.bg).bg(t.colors.secondary)),
        Span::raw("  "),
        Span::styled("  [Esc] Cancel  ", Style::default().fg(t.colors.bg).bg(t.colors.error)),
    ]))
    .alignment(Alignment::Center);
    f.render_widget(buttons, chunks[2]);
}

/// Draw interface definition view
fn draw_interface_view(f: &mut Frame, title: &str, definition: &str, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let lines: Vec<Line> = definition
        .lines()
        .map(|line| {
            let trimmed = line.trim();
            let style = if trimmed.starts_with('#') {
                Style::default().fg(t.colors.text_muted)
            } else if trimmed == "---" {
                Style::default().fg(t.colors.warning).bold()
            } else if trimmed.contains('=') {
                // Constants
                Style::default().fg(Color::Magenta)
            } else {
                Style::default().fg(t.colors.text)
            };
            Line::styled(line, style)
        })
        .collect();

    let content = Paragraph::new(lines)
        .block(
            Block::default()
                .title(format!(" {} {} ", icons.interface, title))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border_focused()),
        )
        .wrap(Wrap { trim: false });

    f.render_widget(content, area);

    // Draw hint at bottom
    let hint_area = Rect {
        x: area.x + 2,
        y: area.y + area.height - 2,
        width: area.width - 4,
        height: 1,
    };
    let hint = Paragraph::new(Line::from(vec![
        Span::styled("Press ", Style::default().fg(t.colors.text_muted)),
        Span::styled("Esc", Style::default().fg(t.colors.warning)),
        Span::styled(" or ", Style::default().fg(t.colors.text_muted)),
        Span::styled("q", Style::default().fg(t.colors.warning)),
        Span::styled(" to close", Style::default().fg(t.colors.text_muted)),
    ]));
    f.render_widget(hint, hint_area);
}

/// Draw settings page
fn draw_settings(f: &mut Frame, app: &App, state: &SettingsState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Length(24), // Category sidebar
            Constraint::Min(40),    // Settings content
        ])
        .split(area);
    
    // Draw category sidebar
    let categories = SettingsCategory::all();
    let category_items: Vec<ListItem> = categories
        .iter()
        .map(|cat| {
            let is_selected = *cat == state.category;
            let style = if is_selected {
                if state.category_focused {
                    t.list_highlight()
                } else {
                    Style::default().fg(t.colors.primary).bold()
                }
            } else {
                Style::default().fg(t.colors.text)
            };
            
            ListItem::new(Line::from(vec![
                Span::raw(" "),
                Span::raw(icons.settings_icon(cat)),
                Span::raw(" "),
                Span::styled(cat.name(), style),
            ]))
        })
        .collect();
    
    let category_list = List::new(category_items)
        .block(
            Block::default()
                .title(format!(" {} Categories ", icons.folder))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(if state.category_focused { t.border_focused() } else { t.border() }),
        );
    
    f.render_widget(category_list, chunks[0]);
    
    // Draw settings content
    let items = state.items_for_category();
    let content_block = Block::default()
        .title(format!(" {} ", state.category.name()))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(if !state.category_focused { t.border_focused() } else { t.border() });
    
    let content_area = content_block.inner(chunks[1]);
    f.render_widget(content_block, chunks[1]);
    
    if items.is_empty() {
        // Show saved commands
        if state.category == SettingsCategory::SavedCommands {
            draw_saved_commands(f, app, state, content_area);
        } else if state.category == SettingsCategory::About {
            draw_about(f, content_area);
        }
        return;
    }
    
    // Draw items
    let item_height = 3;
    let visible_items = (content_area.height / item_height) as usize;
    let start_idx = if state.selected_item >= visible_items {
        state.selected_item - visible_items + 1
    } else {
        0
    };
    
    for (i, item) in items.iter().enumerate().skip(start_idx).take(visible_items) {
        let y = content_area.y + ((i - start_idx) * item_height as usize) as u16;
        let item_area = Rect::new(content_area.x, y, content_area.width, item_height as u16);
        
        let is_selected = i == state.selected_item && !state.category_focused;
        draw_setting_item(f, app, state, item, is_selected, item_area);
    }
}

/// Draw a single setting item
fn draw_setting_item(f: &mut Frame, app: &App, state: &SettingsState, item: &SettingsItem, selected: bool, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let (key, label, description, value_str) = match item {
        SettingsItem::Bool { key, label, description } => {
            let value = match (state.category, key.as_str()) {
                (SettingsCategory::Ui, "auto_scroll") => app.config.ui.auto_scroll,
                (SettingsCategory::Ui, "show_timestamps") => app.config.ui.show_timestamps,
                (SettingsCategory::Process, "auto_restart") => app.config.process.auto_restart,
                _ => false,
            };
            let value_str = if value { format!("{} ON", icons.success) } else { format!("{} OFF", icons.error) };
            (key.clone(), label.clone(), description.clone(), value_str)
        }
        SettingsItem::Number { key, label, description, .. } => {
            let value = match (state.category, key.as_str()) {
                (SettingsCategory::Ui, "log_lines") => app.config.ui.log_lines,
                (SettingsCategory::Process, "log_buffer_size") => app.config.process.log_buffer_size,
                (SettingsCategory::Process, "max_restarts") => app.config.process.max_restarts as usize,
                (SettingsCategory::Process, "restart_delay_secs") => app.config.process.restart_delay_secs as usize,
                _ => 0,
            };
            
            // Check if we're editing this item
            let display = if selected && state.editing {
                format!("{}┃", state.input_buffer)
            } else {
                value.to_string()
            };
            (key.clone(), label.clone(), description.clone(), display)
        }
        SettingsItem::Choice { key, label, description, options } => {
            let value = match (state.category, key.as_str()) {
                (SettingsCategory::Ui, "theme") => {
                    match &app.config.ui.theme {
                        crate::core::Theme::Dark => "Dark",
                        crate::core::Theme::Light => "Light",
                        crate::core::Theme::Custom(_) => "Custom",
                    }
                }
                _ => options.first().map(|s| s.as_str()).unwrap_or(""),
            };
            (key.clone(), label.clone(), description.clone(), value.to_string())
        }
        SettingsItem::Action { key, label, description } => {
            (key.clone(), label.clone(), description.clone(), icons.chevron.to_string())
        }
        SettingsItem::Text { key, label, description } => {
            let display = if selected && state.editing {
                format!("{}┃", state.input_buffer)
            } else {
                "...".to_string()
            };
            (key.clone(), label.clone(), description.clone(), display)
        }
    };
    
    let bg_color = if selected { t.colors.bg_highlight } else { Color::Reset };
    let label_style = if selected {
        Style::default().fg(t.colors.primary).bold()
    } else {
        Style::default().fg(t.colors.text)
    };
    
    let value_color = match item {
        SettingsItem::Bool { key, .. } => {
            let is_on = match (state.category, key.as_str()) {
                (SettingsCategory::Ui, "auto_scroll") => app.config.ui.auto_scroll,
                (SettingsCategory::Ui, "show_timestamps") => app.config.ui.show_timestamps,
                (SettingsCategory::Process, "auto_restart") => app.config.process.auto_restart,
                _ => false,
            };
            if is_on { t.colors.success } else { t.colors.error }
        }
        _ => t.colors.warning,
    };
    
    // Line 1: Label and value
    let line1 = Line::from(vec![
        Span::raw("  "),
        Span::styled(&label, label_style),
        Span::raw(" ".repeat(area.width.saturating_sub(label.len() as u16 + value_str.len() as u16 + 6) as usize)),
        Span::styled(&value_str, Style::default().fg(value_color)),
        Span::raw("  "),
    ]);
    
    // Line 2: Description
    let line2 = Line::from(vec![
        Span::raw("  "),
        Span::styled(&description, Style::default().fg(t.colors.text_muted)),
    ]);
    
    let para = Paragraph::new(vec![line1, line2])
        .style(Style::default().bg(bg_color));
    
    f.render_widget(para, area);
}

/// Draw about section
fn draw_about(f: &mut Frame, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    // Get version from Cargo.toml at compile time
    let version = env!("CARGO_PKG_VERSION");
    let name = env!("CARGO_PKG_NAME");
    
    // Colors: Blue for ROS2, Orange for DEV
    let blue = Color::Rgb(66, 133, 244);   // ROS2 blue
    let orange = Color::Rgb(255, 106, 0);  // Rust orange
    let border_color = t.colors.text_muted;
    
    let mut lines: Vec<Line> = Vec::new();
    
    // Add some top padding
    lines.push(Line::from(""));
    
    // Border line
    lines.push(Line::from(Span::styled(
        "╔═══════════════════════════════════════════════════════════════╗",
        Style::default().fg(border_color),
    )));
    lines.push(Line::from(Span::styled(
        "║                                                               ║",
        Style::default().fg(border_color),
    )));
    
    // Row 1: ██████╗  ██████╗ ███████╗██████╗ | ██████╗ ███████╗██╗   ██╗
    lines.push(Line::from(vec![
        Span::styled(" ║  ", Style::default().fg(border_color)),
        Span::styled("██████╗ ██████╗ ███████╗██████╗ ", Style::default().fg(blue)),
        Span::styled("██████╗ ███████╗██╗   ██╗", Style::default().fg(orange)),
        Span::styled("    ║", Style::default().fg(border_color)),
    ]));
    
    // Row 2
    lines.push(Line::from(vec![
        Span::styled("║  ", Style::default().fg(border_color)),
        Span::styled("██╔══██╗██╔══██╗██╔════╝╚════██╗", Style::default().fg(blue)),
        Span::styled("██╔══██╗██╔════╝██║   ██║", Style::default().fg(orange)),
        Span::styled("    ║", Style::default().fg(border_color)),
    ]));
    
    // Row 3
    lines.push(Line::from(vec![
        Span::styled("║  ", Style::default().fg(border_color)),
        Span::styled("██████╔╝██║  ██║███████╗ █████╔╝", Style::default().fg(blue)),
        Span::styled("██║  ██║█████╗  ██║   ██║", Style::default().fg(orange)),
        Span::styled("    ║", Style::default().fg(border_color)),
    ]));
    
    // Row 4
    lines.push(Line::from(vec![
        Span::styled("║  ", Style::default().fg(border_color)),
        Span::styled("██╔══██╗██║  ██║╚════██║██╔═══╝ ", Style::default().fg(blue)),
        Span::styled("██║  ██║██╔══╝  ╚██╗ ██╔╝", Style::default().fg(orange)),
        Span::styled("    ║", Style::default().fg(border_color)),
    ]));
    
    // Row 5
    lines.push(Line::from(vec![
        Span::styled("║  ", Style::default().fg(border_color)),
        Span::styled("██║  ██║██████╔╝███████║███████╗", Style::default().fg(blue)),
        Span::styled("██████╔╝███████╗ ╚████╔╝ ", Style::default().fg(orange)),
        Span::styled("    ║", Style::default().fg(border_color)),
    ]));
    
    // Row 6
    lines.push(Line::from(vec![
        Span::styled("║  ", Style::default().fg(border_color)),
        Span::styled("╚═╝  ╚═╝╚═════╝ ╚══════╝╚══════╝", Style::default().fg(blue)),
        Span::styled("╚═════╝ ╚══════╝  ╚═══╝  ", Style::default().fg(orange)),
        Span::styled("    ║", Style::default().fg(border_color)),
    ]));
    
    lines.push(Line::from(Span::styled(
        "║                                                               ║",
        Style::default().fg(border_color),
    )));
    lines.push(Line::from(Span::styled(
        "╚═══════════════════════════════════════════════════════════════╝",
        Style::default().fg(border_color),
    )));
    
    lines.push(Line::from(""));
    lines.push(Line::from(""));
    
    // App info
    lines.push(Line::from(vec![
        Span::styled("    Application: ", Style::default().fg(t.colors.text_muted)),
        Span::styled(name, Style::default().fg(t.colors.text).bold()),
    ]));
    
    lines.push(Line::from(vec![
        Span::styled("    Version:     ", Style::default().fg(t.colors.text_muted)),
        Span::styled(format!("v{}", version), Style::default().fg(t.colors.success)),
    ]));
    
    lines.push(Line::from(""));
    
    // Description
    lines.push(Line::from(vec![
        Span::styled("    ", Style::default()),
        Span::styled(
            "A modern TUI (Terminal User Interface) for ROS2 development",
            Style::default().fg(t.colors.text),
        ),
    ]));
    lines.push(Line::from(vec![
        Span::styled("    ", Style::default()),
        Span::styled(
            "workflow. Manage packages, nodes, topics, services, and more",
            Style::default().fg(t.colors.text),
        ),
    ]));
    lines.push(Line::from(vec![
        Span::styled("    ", Style::default()),
        Span::styled(
            "from a single terminal interface.",
            Style::default().fg(t.colors.text),
        ),
    ]));
    
    lines.push(Line::from(""));
    lines.push(Line::from(""));
    
    // Maintainer section
    lines.push(Line::from(vec![
        Span::styled(format!("    {} ", icons.user), Style::default().fg(orange)),
        Span::styled("Maintainer", Style::default().fg(t.colors.text).bold()),
    ]));
    lines.push(Line::from(vec![
        Span::styled("       ", Style::default()),
        Span::styled("@fukurougm", Style::default().fg(orange).bold()),
    ]));
    
    lines.push(Line::from(""));
    
    // Links section
    lines.push(Line::from(vec![
        Span::styled(format!("    {} ", icons.link), Style::default().fg(blue)),
        Span::styled("Links", Style::default().fg(t.colors.text).bold()),
    ]));
    lines.push(Line::from(vec![
        Span::styled("       GitHub: ", Style::default().fg(t.colors.text_muted)),
        Span::styled("https://github.com/fukurougm/codebase/ros2dev", Style::default().fg(blue)),
    ]));
    
    lines.push(Line::from(""));
    lines.push(Line::from(""));
    
    // Tech stack
    lines.push(Line::from(vec![
        Span::styled(format!("    {} ", icons.package), Style::default().fg(orange)),
        Span::styled("Built with", Style::default().fg(t.colors.text).bold()),
    ]));
    lines.push(Line::from(vec![
        Span::styled("       ", Style::default()),
        Span::styled("Rust ", Style::default().fg(orange)),
        Span::styled("+ ", Style::default().fg(t.colors.text_muted)),
        Span::styled("Ratatui ", Style::default().fg(blue)),
        Span::styled("+ ", Style::default().fg(t.colors.text_muted)),
        Span::styled("Tokio ", Style::default().fg(t.colors.success)),
    ]));
    
    lines.push(Line::from(""));
    lines.push(Line::from(""));
    
    // License
    lines.push(Line::from(vec![
        Span::styled("    📜 ", Style::default()),
        Span::styled("License: ", Style::default().fg(t.colors.text_muted)),
        Span::styled("MIT", Style::default().fg(t.colors.text)),
    ]));
    
    let paragraph = Paragraph::new(lines)
        .alignment(ratatui::layout::Alignment::Center);
    
    f.render_widget(paragraph, area);
}

/// Draw saved commands section
fn draw_saved_commands(f: &mut Frame, app: &App, state: &SettingsState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let mut items: Vec<ListItem> = vec![];
    
    // Add "Add Command" action
    let add_style = if state.selected_item == 0 && !state.category_focused {
        Style::default().fg(t.colors.bg).bg(t.colors.success).bold()
    } else {
        Style::default().fg(t.colors.success)
    };
    items.push(ListItem::new(Line::from(vec![
        Span::raw("  "),
        Span::styled(format!("{} Add New Command", icons.add), add_style),
    ])));
    
    // List existing commands
    for (i, cmd) in app.config.saved_commands.iter().enumerate() {
        let is_selected = state.selected_item == i + 1 && !state.category_focused;
        let style = if is_selected {
            t.list_highlight()
        } else {
            Style::default().fg(t.colors.text)
        };
        
        let auto_icon = if cmd.auto_start { format!("{} ", icons.refresh) } else { "   ".to_string() };
        
        items.push(ListItem::new(Line::from(vec![
            Span::raw(auto_icon),
            Span::styled(&cmd.name, style),
            Span::raw(" - "),
            Span::styled(&cmd.command, Style::default().fg(t.colors.text_muted)),
        ])));
    }
    
    if app.config.saved_commands.is_empty() {
        items.push(ListItem::new(Line::from(vec![
            Span::raw("  "),
            Span::styled("No saved commands yet", Style::default().fg(t.colors.text_muted).italic()),
        ])));
    }
    
    let list = List::new(items);
    f.render_widget(list, area);
    
    // Draw hint at bottom
    if !state.category_focused && !app.config.saved_commands.is_empty() && state.selected_item > 0 {
        let hint_area = Rect::new(area.x, area.y + area.height.saturating_sub(2), area.width, 2);
        let hint = Paragraph::new(Line::from(vec![
            Span::styled("r", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Run  "),
            Span::styled("e", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Edit  "),
            Span::styled("d", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Delete"),
        ])).alignment(Alignment::Center);
        f.render_widget(hint, hint_area);
    }
}

/// Draw create saved command wizard
fn draw_create_saved_command(f: &mut Frame, state: &CreateSavedCommandState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let block = Block::default()
        .title(format!(" {} New Saved Command ", icons.add))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(Style::default().fg(t.colors.success));
    
    let inner = block.inner(area);
    f.render_widget(block, area);
    
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints([
            Constraint::Length(3), // Name
            Constraint::Length(3), // Command
            Constraint::Length(3), // Description
            Constraint::Length(3), // Auto Start
            Constraint::Min(1),    // Spacer
            Constraint::Length(2), // Hints
        ])
        .split(inner);
    
    // Name field
    let name_style = if state.active_field == 0 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let name_block = Block::default()
        .title(" Name (required) ")
        .borders(Borders::ALL)
        .border_style(name_style);
    let name_text = if state.active_field == 0 {
        format_text_with_cursor(&state.name, state.field_cursor)
    } else {
        state.name.clone()
    };
    let name_para = Paragraph::new(name_text).block(name_block);
    f.render_widget(name_para, chunks[0]);
    
    // Command field
    let cmd_style = if state.active_field == 1 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let cmd_block = Block::default()
        .title(" Command (required) ")
        .borders(Borders::ALL)
        .border_style(cmd_style);
    let cmd_text = if state.active_field == 1 {
        format_text_with_cursor(&state.command, state.field_cursor)
    } else {
        state.command.clone()
    };
    let cmd_para = Paragraph::new(cmd_text).block(cmd_block);
    f.render_widget(cmd_para, chunks[1]);
    
    // Description field
    let desc_style = if state.active_field == 2 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let desc_block = Block::default()
        .title(" Description (optional) ")
        .borders(Borders::ALL)
        .border_style(desc_style);
    let desc_text = if state.active_field == 2 {
        format_text_with_cursor(&state.description, state.field_cursor)
    } else if state.description.is_empty() {
        "...".to_string()
    } else {
        state.description.clone()
    };
    let desc_para = Paragraph::new(desc_text).block(desc_block);
    f.render_widget(desc_para, chunks[2]);
    
    // Auto Start toggle
    let auto_style = if state.active_field == 3 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let auto_block = Block::default()
        .title(" Auto Start ")
        .borders(Borders::ALL)
        .border_style(auto_style);
    let auto_text = if state.auto_start {
        format!("{} Yes - Run on app startup", icons.success)
    } else {
        format!("{} No", icons.error)
    };
    let auto_color = if state.auto_start { t.colors.success } else { t.colors.error };
    let auto_para = Paragraph::new(Span::styled(auto_text, Style::default().fg(auto_color)))
        .block(auto_block);
    f.render_widget(auto_para, chunks[3]);
    
    // Hints
    let valid = state.is_valid();
    let hints = if valid {
        Line::from(vec![
            Span::styled("Tab", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Next field  "),
            Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
            Span::raw(" Save  "),
            Span::styled("Esc", Style::default().fg(t.colors.warning).bold()),
            Span::raw(" Cancel"),
        ])
    } else {
        Line::from(vec![
            Span::styled("Tab", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Next field  "),
            Span::styled("Esc", Style::default().fg(t.colors.warning).bold()),
            Span::raw(" Cancel  "),
            Span::styled("(Fill required fields to save)", Style::default().fg(t.colors.text_muted)),
        ])
    };
    let hints_para = Paragraph::new(hints).alignment(Alignment::Center);
    f.render_widget(hints_para, chunks[5]);
}

/// Draw edit saved command wizard
fn draw_edit_saved_command(f: &mut Frame, state: &EditSavedCommandState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let block = Block::default()
        .title(format!(" {} Edit Command: {} ", icons.edit, state.name))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(Style::default().fg(t.colors.warning));
    
    let inner = block.inner(area);
    f.render_widget(block, area);
    
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints([
            Constraint::Length(3), // Name
            Constraint::Length(3), // Command
            Constraint::Length(3), // Description
            Constraint::Length(3), // Auto Start
            Constraint::Min(1),    // Spacer
            Constraint::Length(2), // Hints
        ])
        .split(inner);
    
    // Name field
    let name_style = if state.active_field == 0 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let name_block = Block::default()
        .title(" Name (required) ")
        .borders(Borders::ALL)
        .border_style(name_style);
    let name_text = if state.active_field == 0 {
        format_text_with_cursor(&state.name, state.field_cursor)
    } else {
        state.name.clone()
    };
    let name_para = Paragraph::new(name_text).block(name_block);
    f.render_widget(name_para, chunks[0]);
    
    // Command field
    let cmd_style = if state.active_field == 1 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let cmd_block = Block::default()
        .title(" Command (required) ")
        .borders(Borders::ALL)
        .border_style(cmd_style);
    let cmd_text = if state.active_field == 1 {
        format_text_with_cursor(&state.command, state.field_cursor)
    } else {
        state.command.clone()
    };
    let cmd_para = Paragraph::new(cmd_text).block(cmd_block);
    f.render_widget(cmd_para, chunks[1]);
    
    // Description field
    let desc_style = if state.active_field == 2 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let desc_block = Block::default()
        .title(" Description (optional) ")
        .borders(Borders::ALL)
        .border_style(desc_style);
    let desc_text = if state.active_field == 2 {
        format_text_with_cursor(&state.description, state.field_cursor)
    } else if state.description.is_empty() {
        "...".to_string()
    } else {
        state.description.clone()
    };
    let desc_para = Paragraph::new(desc_text).block(desc_block);
    f.render_widget(desc_para, chunks[2]);
    
    // Auto Start toggle
    let auto_style = if state.active_field == 3 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let auto_block = Block::default()
        .title(" Auto Start ")
        .borders(Borders::ALL)
        .border_style(auto_style);
    let auto_text = if state.auto_start {
        format!("{} Yes - Run on app startup", icons.success)
    } else {
        format!("{} No", icons.error)
    };
    let auto_color = if state.auto_start { t.colors.success } else { t.colors.error };
    let auto_para = Paragraph::new(Span::styled(auto_text, Style::default().fg(auto_color)))
        .block(auto_block);
    f.render_widget(auto_para, chunks[3]);
    
    // Hints
    let valid = state.is_valid();
    let hints = if valid {
        Line::from(vec![
            Span::styled("Tab", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Next field  "),
            Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
            Span::raw(" Save  "),
            Span::styled("Esc", Style::default().fg(t.colors.warning).bold()),
            Span::raw(" Cancel"),
        ])
    } else {
        Line::from(vec![
            Span::styled("Tab", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Next field  "),
            Span::styled("Esc", Style::default().fg(t.colors.warning).bold()),
            Span::raw(" Cancel  "),
            Span::styled("(Fill required fields to save)", Style::default().fg(t.colors.text_muted)),
        ])
    };
    let hints_para = Paragraph::new(hints).alignment(Alignment::Center);
    f.render_widget(hints_para, chunks[5]);
}

/// Draw edit process command dialog
fn draw_edit_process_command(f: &mut Frame, app: &App, state: &EditProcessCommandState, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let block = Block::default()
        .title(format!(" {} Edit Process Command ", icons.edit))
        .borders(Borders::ALL)
        .border_type(t.border_type)
        .border_style(Style::default().fg(t.colors.warning));
    
    let inner = block.inner(area);
    f.render_widget(block, area);
    
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .margin(1)
        .constraints([
            Constraint::Length(3), // Name
            Constraint::Min(5),    // Command (multi-line)
            Constraint::Length(1), // Spacer
            Constraint::Length(2), // Hints
        ])
        .split(inner);
    
    // Name field
    let name_style = if state.active_field == 0 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let name_block = Block::default()
        .title(" Name (required) ")
        .borders(Borders::ALL)
        .border_style(name_style);
    let name_text = if state.active_field == 0 {
        format_text_with_cursor(&state.name, state.field_cursor)
    } else {
        state.name.clone()
    };
    let name_para = Paragraph::new(name_text).block(name_block);
    f.render_widget(name_para, chunks[0]);
    
    // Command field - show relative path with text wrapping
    let display_command = make_relative_path(&state.command, &app.workspace_path);
    let cmd_style = if state.active_field == 1 {
        Style::default().fg(t.colors.primary)
    } else {
        Style::default().fg(t.colors.border)
    };
    let cmd_block = Block::default()
        .title(" Command (required) ")
        .borders(Borders::ALL)
        .border_style(cmd_style);
    
    let cmd_para = if state.active_field == 1 {
        // When editing, show full command with cursor and wrapping
        let cmd_area = cmd_block.inner(chunks[1]);
        let available_width = cmd_area.width.saturating_sub(2) as usize; // Account for padding
        let wrapped_lines = wrap_text_with_cursor(&state.command, state.field_cursor, available_width);
        Paragraph::new(wrapped_lines)
            .block(cmd_block)
            .wrap(Wrap { trim: false })
    } else {
        // When not editing, show display command (potentially relative path)
        Paragraph::new(display_command)
            .block(cmd_block)
            .wrap(Wrap { trim: false })
    };
    f.render_widget(cmd_para, chunks[1]);
    
    // Hints
    let valid = state.is_valid();
    let hints = if valid {
        Line::from(vec![
            Span::styled("Tab", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Next field  "),
            Span::styled("Enter", Style::default().fg(t.colors.success).bold()),
            Span::raw(" Save  "),
            Span::styled("Esc", Style::default().fg(t.colors.warning).bold()),
            Span::raw(" Cancel"),
        ])
    } else {
        Line::from(vec![
            Span::styled("Tab", Style::default().fg(t.colors.primary).bold()),
            Span::raw(" Next field  "),
            Span::styled("Esc", Style::default().fg(t.colors.warning).bold()),
            Span::raw(" Cancel  "),
            Span::styled("(Fill required fields to save)", Style::default().fg(t.colors.text_muted)),
        ])
    };
    let hints_para = Paragraph::new(hints).alignment(Alignment::Center);
    f.render_widget(hints_para, chunks[3]);
}

/// Draw help screen
fn draw_help(f: &mut Frame, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let help_text = vec![
        Line::from(""),
        Line::from(vec![Span::styled(
            format!("{} Keyboard Shortcuts", icons.help),
            Style::default().fg(t.colors.primary).bold(),
        )]),
        Line::from(""),
        Line::from(vec![Span::styled("Navigation", Style::default().fg(t.colors.warning).bold())]),
        Line::from("  ↑/k, ↓/j    Move selection up/down"),
        Line::from("  Enter       Select/activate item"),
        Line::from("  /           Open search"),
        Line::from("  Esc         Go back / Cancel"),
        Line::from("  Tab         Next field (wizards)"),
        Line::from(""),
        Line::from(vec![Span::styled("Explorers", Style::default().fg(t.colors.warning).bold())]),
        Line::from("  e           Echo topic (topics)"),
        Line::from("  p           Publish to topic"),
        Line::from("  c           Call service"),
        Line::from("  i           Show info/interface"),
        Line::from("  r           Refresh data"),
        Line::from(""),
        Line::from(vec![Span::styled("Process Manager", Style::default().fg(t.colors.warning).bold())]),
        Line::from("  n/a         New command"),
        Line::from("  s           Start/Stop process"),
        Line::from("  r           Restart process"),
        Line::from("  d           Delete process"),
        Line::from("  S           Stop all processes"),
        Line::from("  PgUp/PgDn   Scroll logs"),
        Line::from(""),
        Line::from(vec![Span::styled("Settings", Style::default().fg(t.colors.warning).bold())]),
        Line::from("  ←/h, →/l    Switch sidebar/items"),
        Line::from("  Enter/Space Toggle/edit setting"),
        Line::from("  Esc         Cancel edit / Go back"),
        Line::from(""),
        Line::from(vec![Span::styled("Global", Style::default().fg(t.colors.warning).bold())]),
        Line::from("  Ctrl+R      Refresh workspace"),
        Line::from("  Ctrl+C/q    Quit"),
        Line::from("  ?/F1        Show this help"),
        Line::from(""),
        Line::from(vec![Span::styled(
            "Press any key to close",
            Style::default().fg(t.colors.text_muted),
        )]),
    ];

    let help = Paragraph::new(help_text)
        .block(
            Block::default()
                .title(format!(" {} Help ", icons.help))
                .borders(Borders::ALL)
                .border_type(t.border_type)
                .border_style(t.border_focused()),
        )
        .wrap(Wrap { trim: true });

    f.render_widget(help, area);
}

/// Draw confirm dialog
fn draw_confirm_dialog(f: &mut Frame, message: &str, _action: ConfirmAction, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let dialog_area = centered_rect(60, 7, area);
    f.render_widget(Clear, dialog_area);

    let content = vec![
        Line::from(""),
        Line::from(vec![Span::styled(message, Style::default().fg(t.colors.warning))]),
        Line::from(""),
        Line::from(vec![
            Span::styled("  [Y] Yes  ", Style::default().fg(t.colors.bg).bg(t.colors.success)),
            Span::raw("  "),
            Span::styled("  [N] No  ", Style::default().fg(t.colors.bg).bg(t.colors.error)),
        ]),
    ];

    let dialog = Paragraph::new(content)
        .block(
            Block::default()
                .title(format!(" {} Confirm ", icons.warning))
                .borders(Borders::ALL)
                .border_type(BorderType::Double)
                .border_style(Style::default().fg(t.colors.warning)),
        )
        .alignment(Alignment::Center);

    f.render_widget(dialog, dialog_area);
}

/// Draw status bar with breadcrumbs
fn draw_status_bar(f: &mut Frame, app: &App, area: Rect) {
    let t = theme::current();
    let icons = icons::current();
    
    let sep = format!(" {} ", icons.separator);
    let primary = t.colors.primary;
    let muted = t.colors.text_muted;
    
    // Build breadcrumbs
    let breadcrumbs: Vec<Span> = match &app.view {
        View::MainMenu => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(primary)),
        ],
        View::Dashboard => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Dashboard", icons.dashboard), Style::default().fg(primary)),
        ],
        View::PackageExplorer => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Packages", icons.package), Style::default().fg(primary)),
        ],
        View::PackageDetail(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Packages", icons.package), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(&state.package_name, Style::default().fg(primary)),
        ],
        View::NodeExplorer => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Nodes", icons.node), Style::default().fg(primary)),
        ],
        View::LaunchExplorer => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Launch Files", icons.launch), Style::default().fg(primary)),
        ],
        View::TopicExplorer => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Topics", icons.topic), Style::default().fg(primary)),
        ],
        View::ServiceExplorer => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Services", icons.service), Style::default().fg(primary)),
        ],
        View::ActionExplorer => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Actions", icons.action), Style::default().fg(primary)),
        ],
        View::ProcessManager => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Process Manager", icons.process), Style::default().fg(primary)),
        ],
        View::CreatePackage(_) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Packages", icons.package), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Create Package", icons.add), Style::default().fg(primary)),
        ],
        View::CreateNode(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Packages", icons.package), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(&state.package, Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} New Node", icons.add), Style::default().fg(primary)),
        ],
        View::CreateLaunch(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Packages", icons.package), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(&state.package, Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} New Launch", icons.add), Style::default().fg(primary)),
        ],
        View::TopicSubscriber(topic) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Topics", icons.topic), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(topic, Style::default().fg(primary)),
        ],
        View::TopicPublisher(topic, _) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Topics", icons.topic), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(topic, Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled("Publish", Style::default().fg(primary)),
        ],
        View::ServiceClient(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Services", icons.service), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(&state.service_name, Style::default().fg(primary)),
        ],
        View::InterfaceView(title, _) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Interface", icons.interface), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(title, Style::default().fg(primary)),
        ],
        View::Help => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Help", icons.help), Style::default().fg(primary)),
        ],
        View::Settings(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Settings", icons.settings), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(state.category.name(), Style::default().fg(primary)),
        ],
        View::CreateSavedCommand(_) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Settings", icons.settings), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Saved Commands", icons.save), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} New Command", icons.add), Style::default().fg(t.colors.success)),
        ],
        View::EditSavedCommand(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Settings", icons.settings), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Saved Commands", icons.save), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} {}", icons.edit, state.name), Style::default().fg(t.colors.warning)),
        ],
        View::EditProcessCommand(state) => vec![
            Span::styled(format!("{} Home", icons.home), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} Process Manager", icons.process), Style::default().fg(muted)),
            Span::styled(&sep, Style::default().fg(muted)),
            Span::styled(format!("{} {}", icons.edit, state.name), Style::default().fg(t.colors.warning)),
        ],
        View::Confirm(_, _) => vec![
            Span::styled(format!("{} Confirm", icons.warning), Style::default().fg(t.colors.warning)),
        ],
    };

    // Show status message if present, otherwise show breadcrumbs
    let mut spans = vec![Span::raw(" ")];
    
    if let Some(msg) = &app.status_message {
        spans.push(Span::styled(format!("{} ", icons.info), Style::default().fg(t.colors.warning)));
        spans.push(Span::styled(msg, Style::default().fg(t.colors.warning)));
    } else {
        spans.extend(breadcrumbs);
    }
    
    // Add help hint on the right
    spans.push(Span::styled("  │  ", Style::default().fg(muted)));
    spans.push(Span::styled("?", Style::default().fg(t.colors.warning)));
    spans.push(Span::styled(" Help", Style::default().fg(muted)));

    let loading_indicator = if app.is_loading {
        Span::styled(format!("  {}", icons.pending), Style::default().fg(t.colors.warning))
    } else {
        Span::raw("")
    };
    spans.push(loading_indicator);

    // Process count indicator
    let running = app.processes.iter().filter(|p| matches!(p.status, ProcessStatus::Running)).count();
    if running > 0 {
        spans.push(Span::styled("  │  ", Style::default().fg(muted)));
        spans.push(Span::styled(format!("{} {} running", icons.running, running), Style::default().fg(t.colors.success)));
    }

    let status = Paragraph::new(Line::from(spans))
        .style(Style::default().bg(t.colors.bg_secondary));

    f.render_widget(status, area);
}

/// Draw input field helper
fn draw_input_field<'a>(label: &'a str, value: &'a str, active: bool) -> Paragraph<'a> {
    let t = theme::current();
    
    Paragraph::new(Line::from(vec![
        Span::styled(format!("{}: ", label), Style::default().fg(t.colors.text_muted)),
        Span::styled(
            value,
            if active {
                Style::default().fg(t.colors.warning).bold()
            } else {
                Style::default().fg(t.colors.text)
            },
        ),
        if active { Span::raw("_") } else { Span::raw("") },
    ]))
    .block(
        Block::default()
            .borders(Borders::ALL)
            .border_type(if active { BorderType::Double } else { t.border_type })
            .border_style(if active {
                Style::default().fg(t.colors.warning)
            } else {
                Style::default().fg(t.colors.border)
            }),
    )
}

/// Helper to create centered rect
fn centered_rect(percent_x: u16, height: u16, r: Rect) -> Rect {
    let popup_layout = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length((r.height.saturating_sub(height)) / 2),
            Constraint::Length(height),
            Constraint::Min(0),
        ])
        .split(r);

    Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage((100 - percent_x) / 2),
            Constraint::Percentage(percent_x),
            Constraint::Percentage((100 - percent_x) / 2),
        ])
        .split(popup_layout[1])[1]
}
