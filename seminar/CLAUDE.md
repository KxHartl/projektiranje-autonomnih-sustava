# Claude Code Pointer

## Mandatory Session Start
Read in this order:
1. `.agent/config/session_bootstrap.md`
2. `.agent/instructions.md`
3. `task/original_task.md`
4. `task/backlog.md`
5. `.agent/context/knowledge_base.md`
6. `sources/docs/index.md`

## Claude Code Rules
- Keep prompts compact and action-oriented.
- Ask for confirmation before destructive shell operations.
- Do not read `.agent/logs/prompt_log.md` unless the user explicitly requests an audit task.
- Use domain playbooks for implementation strategy:
  - `.agent/config/memory_rules.md`
  - `.agent/config/mcp_playbook.md`
  - `.agent/config/skills_playbook.md`
