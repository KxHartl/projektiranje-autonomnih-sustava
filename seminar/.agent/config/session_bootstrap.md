# Session Bootstrap (Token-Efficient)

## Minimal Read Order (default)
1. `.agent/instructions.md`
2. `task/original_task.md`
3. `task/backlog.md`
4. `.agent/context/knowledge_base.md`
5. `sources/docs/index.md`

Only if needed:
6. `task/roadmap.md`
7. Specific file in `work/`
8. Full source docs/PDFs from `sources/docs/`

## Hard Exclusions (unless explicitly requested)
- `.agent/logs/prompt_log.md` (audit-only)
- `sources/data/` raw files
- Any large source document not required for current decision

## Retrieval Strategy
1. Start from indexed summaries (`sources/docs/index.md`, `knowledge_base.md`).
2. Expand only to the exact section/file required to complete the task.
3. Write back distilled findings to `knowledge_base.md` to avoid repeated full reads.
