# esp32-firmware — Claude Context

**Updated:** 2026-04-21
**Owner:** rajat19981

ESP32 firmware for the sEMG compression garment in the BioLiminal movement screening loop. Pairs with the Flutter app in `bioliminal-mobile-application` and the server in `ML_RandD_Server`.

**Documentation conventions:** see https://gitlab.com/bioliminal/bioliminal/-/blob/main/CONVENTIONS.md — header schema (`Status` / `Created` / `Updated` / `Owner`), git-author → gitlab-handle mapping, and the delete-over-archive rule. Follow the schema for every new doc.

## Cross-repo centralization

- **Strategy, plans, decisions, session progress, internal comms:** private `bioliminal-ops` repo.
- **Literature, paper notes, synthesis (sensing, EMG circuits, biomech):** private `research` repo.
- **Hardware specs + BOM:** `ML_RandD_Server/hardware/`.
- **Server source and API (for reference if the firmware ever talks to it directly):** `ML_RandD_Server`.

Keep firmware-specific engineering notes (pinouts, BLE protocol, power budgets, ADC calibration) in this repo. Move cross-team strategy/ops to `bioliminal-ops`; literature findings to `research`. When referencing prior work, point at the other repo rather than copying content here.

## Scope changes & interface updates

If an MR in this repo shifts what the system does, moves work between components (firmware ↔ mobile ↔ server), or touches a locked interface contract, follow the convention at `bioliminal-ops/operations/conventions-scope-changes.md`.

Before merging, check whether your change invalidates any of:

- `bioliminal-ops/operations/interface-contracts.md` — IC-1 … IC-4 current state (BLE packet, FF04 opcodes, SessionPayload, IC-4 events).
- `bioliminal-ops/operations/2026-04-19-demo-stack-diagram.md` — cross-system linkage.
- `bioliminal-ops/operations/2026-04-19-technical-brief-v1.md` — public-facing Technical Brief.

Update them in the same wave (companion MR in `bioliminal-ops`, linked from your MR description) + append a row to `bioliminal-ops/decisions/decisions.md`. Bug fixes, lint, tuning, dep bumps do not trigger this — see the convention doc for the list.
