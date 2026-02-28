# Fukuro - Opensource Repository

This repository contains two main components built for the **Fukuro** robot soccer team. The first is a goal detection system that uses YOLO26n-seg instance segmentation models trained for two camera types - a front-facing camera and an omnidirectional camera - achieving over 99% mAP50 on both. The second is **ros2dev**, an interactive terminal UI (TUI) for ROS 2 development written in Rust, designed to make it easier to manage packages, nodes, topics, processes, and workspace builds directly from the terminal.

---

## Table of Contents

- [Repository Structure](#repository-structure)
- [Goal Detection](#goal-detection)
  - [Models](#models)
  - [Inference Example](#inference-example)
- [ros2dev](#ros2dev)
  - [Features](#features)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Requirements](#requirements)
- [License](#license)
- [Author](#author)

---

## Repository Structure

```
.
├── goal_detection/
│   ├── model_trained/
│   │   ├── front_train/        # Trained model for front camera
│   │   └── omni_train/         # Trained model for omni camera
│   ├── raw_photos/             # Raw training images
│   └── train_code/             # Jupyter notebooks for training
└── ros2dev/
    ├── src/                    # Rust source code
    ├── scripts/                # Install scripts
    └── Cargo.toml
```

---

## Goal Detection

Instance segmentation models trained with **YOLO26n-seg** to detect goals from two camera types.

### Models

| Model | Camera | Epochs | mAP50 | mAP50-95 | Weights |
|-------|--------|--------|-------|----------|---------|
| Front | Front camera | 300 | 99.5% | 97.7% | `front_train/weights/best_gawang_front.pt` |
| Omni  | Omni camera  | 120 | 99.1% | 87.3% | `omni_train/omni_goal_26/weights/best.pt` |

### Inference Example

```python
from ultralytics import YOLO

# Front camera
model = YOLO("goal_detection/model_trained/front_train/weights/best_gawang_front.pt")
results = model("image.jpg")

# Omni camera
model = YOLO("goal_detection/model_trained/omni_train/omni_goal_26/weights/best.pt")
results = model("omni_image.jpg")
```

---

## ROS2Dev

An interactive terminal UI for ROS 2 development built with Rust and [ratatui](https://github.com/ratatui-org/ratatui).

### Features

- Browse packages, nodes, topics, services, and launch files
- Manage and monitor running processes with live logs
- Create packages, nodes, and launch file templates
- Build and clean workspaces via colcon
- Clipboard support for copying logs
- Configurable settings with YAML persistence

### Installation

```bash
cd ros2dev
cargo build --release
```

Binary will be at `ros2dev/target/release/ros2dev`. Or install it to PATH:

```bash
cargo install --path ros2dev
```

Pre-built binaries are also available on [GitHub Releases](https://github.com/fukurougm/codebase/releases).

### Usage

```bash
ros2dev                               # Run in current directory
ros2dev --workspace /path/to/ros2_ws  # Specify workspace
ros2dev --debug                       # Enable debug logging
```

### Requirements

- Rust 1.70+
- ROS 2 (Humble, Iron, Jazzy, or Rolling)
- Terminal with Unicode support

---

## License

MIT - see [LICENSE](LICENSE) for details.

## Author

**Fukuro UGM** - [@fukurougm](https://github.com/fukurougm)
