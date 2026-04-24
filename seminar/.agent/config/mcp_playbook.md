# MCP Playbook (Retrieval First, Token Minimal)

## Preferred Retrieval Order
1. Local project files (`task/`, `.agent/context/`, `sources/docs/index.md`).
2. MCP servers for targeted lookup (API docs, issue trackers, code search).
3. Web fetch as fallback when MCP/local sources are insufficient.

## MCP Usage Rules
- Query narrowly (symbol/path/topic specific), never broad scraping by default.
- Capture only the needed excerpt and immediately distill it to local memory.
- Do not duplicate large MCP outputs into logs.

## Domain-Specific MCP Priorities
### Python
- Runtime/library docs for exact API behavior and version-specific changes.

### C++
- Standard/library references and compiler/toolchain diagnostics.

### ROS2
- ROS2 docs + package API references for message/service/action contracts.
- Keep distro compatibility explicit in notes.

### Computer Vision
- Framework docs (OpenCV, dataset tools), model cards, metric definitions.

### Neural Networks / PyTorch
- Official docs for module/loss/optimizer behavior.
- CUDA/cuDNN compatibility matrix references when debugging training/runtime issues.

## MCP Output Distillation Template
- **Question**: what needed verification?
- **Source**: MCP server + reference path/link.
- **Answer**: 2-5 bullet facts.
- **Impact**: what changed in design/code/backlog.
