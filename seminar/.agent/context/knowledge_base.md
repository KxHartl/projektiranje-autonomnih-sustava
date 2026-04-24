# Knowledge Base

> **Svrha**: Sažetak trenutnog stanja projekta za brzi onboarding budućih AI sesija.
> **Pravilo agenta**: Ažuriraj ovaj dokument na kraju svake veće milestone-a.
> **Zadnja izmjena**: 2026-04-24 | **Agent**: GPT-5.3-Codex

---

## Pregled Projekta

**Projekt**: Dual Arm Robot — seminar za kolegij Projektiranje Autonomnih Sustava (FSB)
**Faza**: Phase 2 — razvoj počinje

## Trenutno Stanje

Workspace je inicijaliziran. Dodani su pointeri za Copilot i Claude Code, uveden je sources indeks i domain playbooki (memory/MCP/skills). `work/` je namjerno ostavljen prazan kao template.

## Ključne Arhitekturne Odluke

| Odluka | Razlog | Datum |
|--------|--------|-------|
| `work/` ostaje prazan dok razvoj ne krene | Template ostaje neutralan i bez implicitnog scope-a | 2026-04-24 |
| `.agent/instructions.md` je centralna instrukcija | Jedna točka istine za sve agente | 2026-04-24 |
| `sources/docs/` sadrži sve PDF materijale | Organizacija referentnih materijala | 2026-04-24 |
| `sources/docs/index.md` je obavezan prvi korak retrievala | Manja potrošnja tokena prije čitanja cijelih PDF-ova | 2026-04-24 |
| `prompt_log.md` je audit-only | Sprječava token waste i curenje nepotrebnog konteksta | 2026-04-24 |

## Poznate Pretpostavke i Ograničenja

- _Dodaj ovdje pretpostavke koje si napravio/la_

## Ključne Datoteke i Njihova Uloga

- `task/original_task.md` — neizmjenjivi originalni zadatak
- `task/backlog.md` — dinamički popis zadataka, ažuriraj svaku sesiju
- `.agent/logs/action_history.md` — log svih AI akcija
- `.agent/logs/prompt_log.md` — audit log (nije za standardni read)
- `.agent/config/session_bootstrap.md` — minimalni onboarding read-order
- `.agent/config/memory_rules.md` — memory pravila po domeni
- `.agent/config/mcp_playbook.md` — MCP retrieval strategija po domeni
- `.agent/config/skills_playbook.md` — reusable skill workflowi
- `work/` — prazan template workspace
- `sources/docs/` — PDF materijali s predavanja
- `sources/docs/index.md` — indeks izvora za token-efikasan retrieval

## Što Sljedeći Agent Treba Znati

_Dopuni ovu sekciju na kraju sesije sa sažetkom onoga što si napravio i gdje si stao/la._
