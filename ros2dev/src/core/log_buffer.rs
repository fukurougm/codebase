//! Circular log buffer for storing process output

use chrono::{DateTime, Local};
use std::collections::VecDeque;

/// A single log line
#[derive(Debug, Clone)]
pub struct LogLine {
    /// The content of the line
    pub content: String,
    /// Timestamp when the line was received
    pub timestamp: DateTime<Local>,
    /// Whether this line is from stderr
    pub is_stderr: bool,
}

/// Circular buffer for storing log lines
#[derive(Debug)]
pub struct LogBuffer {
    /// The log lines
    lines: VecDeque<LogLine>,
    /// Maximum number of lines to keep
    max_lines: usize,
}

impl LogBuffer {
    /// Create a new log buffer with the given capacity
    pub fn new(max_lines: usize) -> Self {
        Self {
            lines: VecDeque::with_capacity(max_lines),
            max_lines,
        }
    }

    /// Push a new line to the buffer
    pub fn push_line(&mut self, content: &str, is_stderr: bool) {
        // Remove oldest line if at capacity
        if self.lines.len() >= self.max_lines {
            self.lines.pop_front();
        }

        self.lines.push_back(LogLine {
            content: content.to_string(),
            timestamp: Local::now(),
            is_stderr,
        });
    }

    /// Get all lines
    pub fn lines(&self) -> impl Iterator<Item = &LogLine> {
        self.lines.iter()
    }

    /// Get lines in a range for display
    pub fn lines_range(&self, start: usize, count: usize) -> Vec<&LogLine> {
        self.lines.iter().skip(start).take(count).collect()
    }

    /// Get the number of lines
    pub fn len(&self) -> usize {
        self.lines.len()
    }

    /// Check if the buffer is empty
    pub fn is_empty(&self) -> bool {
        self.lines.is_empty()
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.lines.clear();
    }

    /// Search for lines containing the given pattern
    pub fn search(&self, pattern: &str) -> Vec<(usize, &LogLine)> {
        let pattern_lower = pattern.to_lowercase();
        self.lines
            .iter()
            .enumerate()
            .filter(|(_, line)| line.content.to_lowercase().contains(&pattern_lower))
            .collect()
    }

    /// Get the content as a vector of strings (for display)
    pub fn content_lines(&self) -> Vec<String> {
        self.lines.iter().map(|l| l.content.clone()).collect()
    }

    /// Export to a string
    pub fn export(&self) -> String {
        self.lines
            .iter()
            .map(|l| format!("[{}] {}", l.timestamp.format("%H:%M:%S"), l.content))
            .collect::<Vec<_>>()
            .join("\n")
    }
}

impl Default for LogBuffer {
    fn default() -> Self {
        Self::new(10000)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_and_get() {
        let mut buffer = LogBuffer::new(100);
        buffer.push_line("test line 1", false);
        buffer.push_line("test line 2", true);

        assert_eq!(buffer.len(), 2);
        
        let lines: Vec<_> = buffer.lines().collect();
        assert_eq!(lines[0].content, "test line 1");
        assert!(!lines[0].is_stderr);
        assert_eq!(lines[1].content, "test line 2");
        assert!(lines[1].is_stderr);
    }

    #[test]
    fn test_capacity_limit() {
        let mut buffer = LogBuffer::new(3);
        buffer.push_line("line 1", false);
        buffer.push_line("line 2", false);
        buffer.push_line("line 3", false);
        buffer.push_line("line 4", false);

        assert_eq!(buffer.len(), 3);
        
        let lines: Vec<_> = buffer.lines().collect();
        assert_eq!(lines[0].content, "line 2");
        assert_eq!(lines[2].content, "line 4");
    }

    #[test]
    fn test_search() {
        let mut buffer = LogBuffer::new(100);
        buffer.push_line("error: something failed", true);
        buffer.push_line("info: all good", false);
        buffer.push_line("error: another failure", true);

        let results = buffer.search("error");
        assert_eq!(results.len(), 2);
    }
}
