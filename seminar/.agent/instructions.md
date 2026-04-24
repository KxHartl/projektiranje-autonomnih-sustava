# Agent System Instructions

You are an AI assistant collaborating with a human on a technical project. This workspace is designed for multi-agent persistence and clarity.

## Core Directives

1.  **Read Before Acting**: Always start with `.agent/config/session_bootstrap.md` to minimize tokens and keep context consistent.
2.  **Maintain the Backlog**: Update `task/backlog.md` with your progress. Use `[ ]` for pending, `[/]` for in-progress, and `[x]` for completed tasks.
3.  **Preserve History**: Record major decisions, architectural changes, and "what was done" in `.agent/logs/action_history.md`.
4.  **Distill Knowledge**: When you finish a major milestone, update `.agent/context/knowledge_base.md` with a summary of the current state so future agents have quick context.
5.  **Source Integrity**: Always start from `sources/docs/index.md`, then open full source files only if needed.
6.  **Audit Separation**: `.agent/logs/prompt_log.md` is audit-only. Do not read it during normal implementation.
7.  **Domain Playbooks**: For Python/C++/ROS2/CV/NN/PyTorch workflows, use:
    - `.agent/config/memory_rules.md`
    - `.agent/config/mcp_playbook.md`
    - `.agent/config/skills_playbook.md`

## Workspace Structure
- `.agent/`: Your domain. Configs, logs, and persistent context.
- `.human/`: Notes and research by the human. Respect these but do not modify them unless asked.
- `task/`: The source of truth for "What" and "How".
- `work/`: Empty template workspace. Create project subfolders only when implementation starts.
- `sources/`: Data and documentation.

## Interaction Style
- Be concise and technical.
- Propose plans before making large changes.
- Always explain the rationale for architectural decisions.
