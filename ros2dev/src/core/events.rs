//! Event handling for keyboard, mouse, and tick events

use anyhow::Result;
use crossterm::event::{self, Event as CrosstermEvent, KeyEvent, MouseEvent};
use std::time::Duration;
use tokio::sync::mpsc;

/// Application events
#[derive(Debug)]
pub enum Event {
    /// Tick event for periodic updates
    Tick,
    /// Keyboard event
    Key(KeyEvent),
    /// Mouse event
    Mouse(MouseEvent),
    /// Terminal resize event
    Resize(u16, u16),
}

/// Event handler that polls for events
pub struct EventHandler {
    /// Event receiver
    rx: mpsc::UnboundedReceiver<Event>,
    /// Stop signal sender
    _stop_tx: mpsc::UnboundedSender<()>,
}

impl EventHandler {
    /// Create a new event handler with the given tick rate in milliseconds
    pub fn new(tick_rate_ms: u64) -> Self {
        let tick_rate = Duration::from_millis(tick_rate_ms);
        let (tx, rx) = mpsc::unbounded_channel();
        let (stop_tx, mut stop_rx) = mpsc::unbounded_channel();

        // Spawn event polling task
        tokio::spawn(async move {
            let mut last_tick = std::time::Instant::now();
            
            loop {
                // Check for stop signal
                if stop_rx.try_recv().is_ok() {
                    break;
                }

                // Calculate timeout until next tick
                let timeout = tick_rate
                    .checked_sub(last_tick.elapsed())
                    .unwrap_or(Duration::ZERO);

                // Poll for events
                if event::poll(timeout).unwrap_or(false) {
                    match event::read() {
                        Ok(CrosstermEvent::Key(key)) => {
                            if tx.send(Event::Key(key)).is_err() {
                                break;
                            }
                        }
                        Ok(CrosstermEvent::Mouse(mouse)) => {
                            if tx.send(Event::Mouse(mouse)).is_err() {
                                break;
                            }
                        }
                        Ok(CrosstermEvent::Resize(width, height)) => {
                            if tx.send(Event::Resize(width, height)).is_err() {
                                break;
                            }
                        }
                        Ok(_) => {}
                        Err(_) => break,
                    }
                }

                // Send tick event if enough time has passed
                if last_tick.elapsed() >= tick_rate {
                    if tx.send(Event::Tick).is_err() {
                        break;
                    }
                    last_tick = std::time::Instant::now();
                }
            }
        });

        Self {
            rx,
            _stop_tx: stop_tx,
        }
    }

    /// Get the next event
    pub async fn next(&mut self) -> Result<Event> {
        self.rx
            .recv()
            .await
            .ok_or_else(|| anyhow::anyhow!("Event channel closed"))
    }
}
