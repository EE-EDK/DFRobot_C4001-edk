# DFRobot C4001 — Project Mandates

Extends the root `ENGINEERING-PROJECTS/GEMINI.md`. This file defines project-specific mandates for the DFRobot C4001 mmWave presence detector firmware.

## 1. Project Identity
- **Name:** DFRobot C4001 mmWave Presence Detector (Enhanced)
- **Type:** Arduino library / firmware for Raspberry Pi Pico
- **Framework:** Arduino (C++ with Arduino abstractions)
- **Sensor:** DFRobot C4001 24GHz mmWave Radar (SEN0609/SEN0610)
- **Status:** Complete, production-validated (v2.6.2)

## 2. Project-Specific Mandates

### Code Quality
- **Arduino conventions:** Single `.ino` file per example with `setup()` and `loop()`.
- **Configuration at top:** All tunable parameters (`#define`) must be grouped at the top of the `.ino` file with descriptive comments.
- **Doxygen documentation:** All functions must have `@brief`, `@param`, `@return` tags.

### Sensor Handling
- **Energy corruption filtering is mandatory:** ~20% of C4001 readings return corrupted energy values (>10M). Any firmware modification must preserve this filter.
- **Use Speed Mode only:** Presence Mode has known firmware bugs in the C4001 sensor. Do not switch to Presence Mode without hardware re-validation.
- **Saleae-optimized range (0.3-2.0m):** Default detection range is validated against 103 real deployment samples. Changes require re-validation with logic analyzer data.

### Validation
- **Serial output verification:** After any change, verify startup sequence completes and detection events appear on Serial Monitor at 115200 baud.
- **Data quality reports:** Enable `ENABLE_DATA_QUALITY` and verify corruption rate stays ~20% and valid detection percentage is acceptable.

## 3. Key Files
| File | Purpose |
|------|---------|
| `examples/DFRobot-C4001-24GHz-UART-Enhanced-PiPico/*.ino` | Main firmware |
| `library.properties` | Arduino library metadata |
| `Doxyfile` | Documentation generation config |
| `README.md` | Full documentation with wiring, config, troubleshooting |

## TODO
- [ ] None identified

## Conversation History Archive

Past AI conversations (217 total) are archived at the workspace root: `.claude/conversation-history/`. Search `index.json` by keyword or browse `index.md` for topic-grouped context on prior decisions, approaches, and project history.


## Auto-Commit & Push Mandate

After completing each task, automatically commit all relevant changes with a descriptive message and push to `origin main`. Report what was committed. This is standing authorization — no confirmation needed.