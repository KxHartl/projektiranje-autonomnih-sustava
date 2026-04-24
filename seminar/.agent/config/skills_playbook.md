# Skills Playbook (Reusable Execution Patterns)

## Purpose
Define reusable "skills" as compact workflows per domain so agents solve tasks consistently with less prompt overhead.

## Core Skills
1. **Read-Plan-Implement-Log**
2. **Debug-and-Isolate**
3. **Reproduce-and-Fix**
4. **Benchmark-and-Compare**
5. **Document-and-Handoff**

## Domain Skill Packs
### Python
- Environment bootstrap, dependency sync, lint/test quick loop.
- Module-level refactor with API compatibility checks.

### C++
- CMake configure/build skill.
- Compiler warning triage and ABI-safe refactor skill.

### ROS2
- `colcon` build/test skill.
- Node graph verification (topics/services/actions) skill.
- Launch orchestration and parameter validation skill.

### Computer Vision
- Data pipeline validation skill (I/O, preprocessing, augmentations).
- Inference quality + latency check skill.

### Neural Networks
- Training loop sanity skill (loss curve, gradient health, overfit mini-batch test).
- Evaluation and metric drift comparison skill.

### PyTorch
- Device precision strategy skill (CPU/CUDA, AMP).
- Checkpoint save/load integrity skill.
- Dataloader bottleneck diagnosis skill.

## Skill Output Contract
Every completed skill should update:
- `task/backlog.md` (status changes)
- `.agent/logs/action_history.md` (major actions)
- `.agent/context/knowledge_base.md` (durable takeaways)
