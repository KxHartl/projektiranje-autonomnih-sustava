# Memory Rules (Cross-Agent)

## Goal
Store only durable, high-value facts so future agents can recover context with minimal token use.

## What to Store
- Build/test commands that are confirmed working.
- Stable architectural decisions and constraints.
- Interface contracts (topics, message types, APIs, file formats).
- Dataset/model assumptions that affect reproducibility.
- Performance baselines and known bottlenecks.

## What NOT to Store
- Verbose logs, stack traces, or full prompts.
- Temporary experiments without decisions.
- Duplicated facts already present in source index or docs.

## Domain Memory Checklist
### Python
- Python version, package manager, virtualenv policy.
- Lint/test commands and module structure conventions.

### C++
- Compiler + standard (e.g., GCC/Clang, C++17/C++20).
- CMake options, ABI-relevant decisions, dependency pins.

### ROS2
- ROS2 distro, workspace layout, `colcon` commands.
- Nodes/topics/services/actions contracts.
- QoS choices, frame conventions (`tf`), launch entrypoints.

### Computer Vision
- Camera model/calibration assumptions.
- Image formats, preprocessing, augmentation policy.
- Metrics (mAP, IoU, precision/recall, latency).

### Neural Networks
- Model family, loss functions, optimization settings.
- Training/validation split rules and seed/reproducibility policy.

### PyTorch
- Torch/CUDA versions and device policy.
- Dataloader shape contracts and checkpoint naming/loading rules.
- Mixed precision policy and gradient scaling decisions.

## Storage Targets
- Short-term session progress: `task/backlog.md`
- Durable distilled memory: `.agent/context/knowledge_base.md`
- Audit trail only: `.agent/logs/prompt_log.md`
