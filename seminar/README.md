# AI-Human Collaboration Template

This workspace is a template designed for projects where multiple AI agents (Gemini, Claude Code, Copilot, Cursor, etc.) collaborate with a human.

## Quick Start for AI Agents

1.  **Read Bootstrap**: Start with `.agent/config/session_bootstrap.md`.
2.  **Ground Truth**: Refer to `task/original_task.md`.
3.  **Check Status**: Review `task/backlog.md` and `task/roadmap.md`.
4.  **Source Index First**: Open `sources/docs/index.md` before reading full PDFs/docs.
5.  **Execute**: Implement inside `work/` (create subfolder only when work starts).
6.  **Log**: Update `.agent/logs/action_history.md` and `task/backlog.md` before finishing.

## Folder Structure

- **`.agent/`**: Context and instructions for AI.
  - `config/`: Agent-specific settings.
  - `context/`: Distilled knowledge base for cross-session continuity.
  - `logs/`: History of major actions and architectural decisions.
- **`.human/`**: A space for human thoughts, manual research, and private notes.
- **`task/`**: Centralized task management.
- **`work/`**: Empty implementation sandbox template.
- **`sources/`**: Reference materials (PDFs, docs, datasets).

## Pointer Files

- `.GEMINI`
- `.cursorrules`
- `.clinerules`
- `.copilot-instructions.md`
- `CLAUDE.md`

## Tracking Principles

- **Immutability**: `task/original_task.md` should never be deleted.
- **Persistence**: AI agents must distill their progress into `.agent/context/` to prevent "forgetting" between sessions.
- **Traceability**: All major logic changes should be reflected in the backlog.
- **Audit Separation**: `.agent/logs/prompt_log.md` exists only for audit, not as default read context.
