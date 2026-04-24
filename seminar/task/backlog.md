# Project Backlog

> **Pravilo**: Agent koji obavlja zadatak mora ažurirati ovaj backlog.
> Status: `[ ]` čeka, `[/]` u tijeku, `[x]` završeno.

---

## Phase 1: Inicijalizacija Workspacea
- [x] Kreirati strukturu direktorija
- [x] Definirati master instrukcije u `.agent/instructions.md`
- [x] Zapisati originalni zadatak u `task/original_task.md`
- [x] Kreirati agent-specifične pointer datoteke (`.GEMINI`, `.cursorrules`, `.clinerules`)
- [x] Konfigurirati `.gitignore` za AI/Human hybrid workflow
- [x] Kreirati `README.md`
- [x] Postaviti `work/` kao prazan template (bez pre-kreiranog projekta)
- [x] Reorganizirati `sources/` (PDFovi u `sources/docs/`)
- [x] Kreirati template za `prompt_log.md`
- [x] Kreirati template za `context/knowledge_base.md`
- [x] Dodati pointere za Copilot i Claude Code (`.copilot-instructions.md`, `CLAUDE.md`)
- [x] Dodati indeks izvora (`sources/docs/index.md`)
- [x] Dodati memory/MCP/skills playbooke u `.agent/config/`

## Phase 2: Razvoj (Dual Arm Robot)
- [ ] Analizirati zadatak iz `task/task.pdf`
- [ ] Definirati podjelu zadataka za dual-arm robot
- [ ] Kreirati `work/dual_arm_robot/` i implementirati kinematički model
- [ ] Testirati u simulatoru (Gazebo / Stage)
- [ ] Dokumentirati rezultate u `.agent/context/knowledge_base.md`

## Phase 3: Zaključak i Predaja
- [ ] Finalizirati izvještaj
- [ ] Provjeriti usklađenost s originalnim zadatkom
- [ ] Git tag finalne verzije
