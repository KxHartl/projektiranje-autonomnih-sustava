# Prompt Log

> **Svrha**: Praćenje svih promptova poslatih AI agentima — tko je tražio što, kada i koji model je odgovorio.
> **Format zapisa**: Dodaj novi entry na VRH datoteke (najnoviji gore).
> **Napomena**: Ovaj dokument je **audit-only** i ne ulazi u standardni onboarding kontekst agenata.

---

## Template za novi unos

```
## [DATUM] — [MODEL/AGENT]
**Sesija ID**: ...
**Korisnik**: khartl
**Prompt**:
> Ovdje kopiraj prompt koji si poslao agentu.

**Sažetak odgovora/akcije**:
- Što je agent napravio
- Koje datoteke je mijenjao

**Datoteke promijenjene**:
- `path/to/file.md`
```

---

## 2026-04-24 — Antigravity (Gemini 3 Flash → Claude Sonnet)
**Korisnik**: khartl
**Prompt #1**:
> Napravi detaljni prijedlog kako je ovo najbolje implementirati za AI-Human workspace template...

**Akcija**: Kreirana kompletna struktura workspacea — mape, instrukcije, backlog, roadmap, `.gitignore`, pointer datoteke.

**Prompt #2**:
> Pogledaj prethodni prompt i što je napravio Gemini, napravi review i dodaj/predloži promjene.

**Akcija**: Review — identificirani problemi: neažuran backlog, duplikacija `dual_arm_robot/work/`, nedostajuće template datoteke.

**Prompt #3**:
> Implementiraj sve potrebne promjene.

**Akcija**: Sve promjene implementirane — vidi `action_history.md` za detalje.
