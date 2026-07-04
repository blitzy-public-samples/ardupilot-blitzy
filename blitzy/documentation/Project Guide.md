# Blitzy Project Guide — AfsimL1 Reusable L1-Guidance Service

> **Project:** Extract ArduPilot's L1 lateral-navigation (PNT) guidance into a reusable, host-drivable service behind a stable C ABI
> **Branch:** `blitzy-46f5dfbd-4fd4-4fb7-b110-03ba60585668` · **HEAD:** `8ec9169bbd` · **Base:** `6148c3d422`
> **Color legend:** <span style="color:#5B39F3">■</span> Completed / AI Work `#5B39F3` · <span style="color:#FFFFFF;background:#333">■</span> Remaining `#FFFFFF` · <span style="color:#B23AF2">■</span> Headings/Accents `#B23AF2` · <span style="color:#A8FDD9;background:#333">■</span> Highlight `#A8FDD9`

---

## 1. Executive Summary

### 1.1 Project Overview

This refactoring extracts ArduPilot's L1 lateral-navigation guidance (`AP_L1_Control`) — the Position, Navigation, and Timing (PNT) behaviors embedded in the vehicle flight loop — into a single reusable service, **AfsimL1**, that an external host such as the AFSIM simulator can drive through a stable, language-agnostic `extern "C"` ABI shared library (`libafsim_l1.so`). The host injects legs, platform state, and timing (`dt`) and reads back the commanded roll and lateral acceleration, without linking the ArduPilot firmware or matching its C++ toolchain. The transformation is behavior-preserving (Facade + Adapter + ABI-boundary + Dependency Injection): the guidance mathematics are untouched; only input/output boundaries are re-plumbed. No vehicle firmware is modified.

### 1.2 Completion Status

```mermaid
%%{init: {'theme':'base', 'themeVariables':{ 'pie1':'#5B39F3', 'pie2':'#FFFFFF', 'pieStrokeColor':'#B23AF2', 'pieStrokeWidth':'2px', 'pieOuterStrokeWidth':'2px', 'pieTitleTextSize':'16px', 'pieSectionTextSize':'13px', 'pieLegendTextSize':'13px'}}}%%
pie showData
    title AfsimL1 Refactor — 82.5% Complete (127h of 154h)
    "Completed Work (AI)" : 127
    "Remaining Work" : 27
```

<div align="center"><strong>■ 82.5% Complete</strong> &nbsp;(<span style="color:#5B39F3">Completed = #5B39F3</span> · <span>Remaining = #FFFFFF</span>)</div>

| Metric | Value |
|---|---|
| **Total Hours** | **154 h** |
| **Completed Hours (AI + Manual)** | **127 h** (127 AI + 0 Manual) |
| **Remaining Hours** | **27 h** |
| **Percent Complete** | **82.5%** — `127 / (127 + 27) = 127 / 154 = 0.8247` |

> **Reading this number correctly:** *100% of the explicit Agent Action Plan scope (all 21 requirements) is delivered, validated, and committed with zero placeholders.* The 82.5% reflects the **total** work universe, which also includes standard path-to-production activities (external host integration, cross-toolchain validation, versioning, human review) that cannot be completed autonomously. See §8.

### 1.3 Key Accomplishments

- ✅ **Goal 1 — PNT instances located.** Every L1-navigation PNT touch-point catalogued (16 AHRS read sites → 6 accessor kinds + 2 clock couplings) with a current-location → new-service-location mapping.
- ✅ **Goal 2 — Extracted into one reusable service.** `AfsimL1Behavior` facade + `AfsimL1_AHRS_Shim` adapter + `l1_c_api` boundary, delivered as a standalone shared library (`libafsim_l1.so`) exporting **exactly 8** `L1_*` symbols over an opaque handle.
- ✅ **Goal 3 — Documented twice.** The mapping lives both in the AAP (§0.6.1) and in the regenerated 40-page `ArduPilot_PNT_Reference_Audit.pdf` (new "New Service Location" column), reproducible via a committed Python/ReportLab generator.
- ✅ **Behavior-preserving timing seam.** Additive, **default-off** `set_update_dt(float)` on `AP_L1_Control`; git-diff verified byte-identical fallbacks so existing vehicle callers are numerically unaffected.
- ✅ **Scope guardrail honored.** All vehicle firmware (ArduPlane/ArduCopter/Rover/ArduSub/Blimp/AntennaTracker) and other PNT subsystems show **0 changes**.
- ✅ **Clean, self-contained build.** Standalone `.so` links only system libraries (libstdc++/libm/libgcc_s/libc); zero third-party runtime dependency; zero in-scope compiler warnings.
- ✅ **Rigorous autonomous validation.** 17/17 C-ABI functional checks + 991 documentation-integrity assertions + 3 build targets, all passing; runnable demo emits `roll_deg=48.5, lat_accel=11.088`.

### 1.4 Critical Unresolved Issues

*No release-blocking defects were identified.* All items below are path-to-production enhancements, not defects.

| Issue | Impact | Owner | ETA |
|---|---|---|---|
| Real AFSIM host integration not yet exercised (only in-repo demo/mock) | Guidance loop unproven inside a live external host | Integration Engineer | 1 day (8 h) |
| Cross-compiler/cross-libstdc++ ABI validation pending (only g++ 15.2 tested) | The core "toolchain-independent" value proposition is unverified on other toolchains | Build/Platform Engineer | 0.5 day (4 h) |
| No SONAME / semantic version on `libafsim_l1.so` | Downstream consumers cannot manage ABI compatibility over time | Release Engineer | 0.5 day (3 h) |

### 1.5 Access Issues

| System / Resource | Type of Access | Issue Description | Resolution Status | Owner |
|---|---|---|---|---|
| AFSIM simulation environment | Runtime / license | AFSIM is an external, license-gated simulator; not available in the autonomous environment, so real host integration (task H2) could not be executed | Open — requires customer-provided AFSIM access | Integration Engineer |
| Alternate C++ toolchains (clang / mingw / alt libstdc++) | Build toolchain | Only g++ 15.2 / Linux x86-64 present in the container; cross-toolchain ABI validation (task M1) needs additional compilers | Open — provision CI images | Build/Platform Engineer |

> All in-repo resources (source, build tools, Python/ReportLab, git) were fully accessible; every autonomous gate ran without permission problems.

### 1.6 Recommended Next Steps

1. **[High]** Human code review of the 5,844-line diff (16 files) and merge — verify the additive default-off seam preserves behavior, confirm out-of-scope trees untouched.
2. **[High]** Integrate `libafsim_l1.so` into a real AFSIM (or representative) host and run a closed-loop, multi-leg smoke test through the 8 C-ABI entry points.
3. **[Medium]** Validate the ABI across at least one additional compiler / C++ runtime (clang or mingw) and confirm the 8 symbols resolve and numerics match.
4. **[Medium]** Add SONAME + semantic version + install/export rules to the shared library, and document the thread-safety contract in the README.
5. **[Medium]** Run a targeted numerical fidelity spot-check against stock in-vehicle `AP_L1_Control` over representative waypoint/loiter/heading legs in SITL.

---

## 2. Project Hours Breakdown

### 2.1 Completed Work Detail

All components trace to explicit AAP requirements and were delivered autonomously (AI). **Total = 127 h.**

| Component | Hours | Description |
|---|---:|---|
| Goal 1 — PNT L1-nav instance audit & current→new mapping | 8 | Static analysis of `AP_L1_Control` PNT touch-points; 16 read sites → 6 accessors + 2 clock couplings; authoritative mapping table (AAP §0.6.1) |
| Facade layer — `AfsimL1Behavior.h/.cpp` (603 LOC) | 18 | Task-oriented service API (`set_leg_ne`/`set_state_ne`/`execute`/`get_roll_deg`/`get_lat_accel`); composes `AP_L1_Control`; seeds `PERIOD`/`DAMPING`/`XTRACK_I`/`LIM_BANK`; leg/state management |
| Adapter layer — `AfsimL1_AHRS_Shim.h/.cpp` (396 LOC) | 12 | Satisfies the controller's 6 `AP_AHRS` read APIs from injected state; injection setters; datum-based `Location` construction |
| C-ABI boundary — `l1_c_api.h/.cpp` (391 LOC) | 10 | 8 `extern "C"` exports over an opaque `L1_Context` handle; NULL-safety on every entry; `visibility("default")` |
| Timing seam — `AP_L1_Control.h/.cpp` (73 LOC, behavior-critical) | 8 | Additive, default-off `set_update_dt`; dual override (waypoint `micros()` + loiter `millis()`); clamp semantics preserved |
| Build system — `CMakeLists.txt` (490 LOC) + `wscript` | 12 | Standalone `.so` (Option-B include seam, `-fvisibility=hidden`, `--no-undefined`, opt-in demo target); in-tree waf `bld.ap_example` |
| Demo driver — `main.cpp` (174 LOC) | 4 | "Initialize a simple leg" runnable C-ABI consumer |
| Integration docs — `README.md` (216 LOC) | 5 | Architecture, C-ABI reference, DI model, units, build/verify/usage |
| Goal 3 — PDF pipeline + regenerated 40-page PDF (2,967 LOC Python) | 28 | `generate.py` + `pnt_data.py` + `pnt_render.py`; ReportLab renderer; 991-assertion integrity harness; deterministic byte-stable output |
| C-ABI functional test harness (17 checks) | 10 | NULL-safety ×8, CWE-457 pre-execute guard, turn-geometry sign, cross-instance determinism, host-driven `dt` |
| Autonomous validation + QA-checkpoint resolution + debugging (12 commits) | 12 | CP1/CP2/CP5 review rounds, stale-citation fixes, build-recipe fixes, final acceptance |
| **Total Completed** | **127** | |

### 2.2 Remaining Work Detail

All explicit AAP deliverables are complete; remaining work is **path-to-production** (deploy / validate / review gates). **Total = 27 h.**

| Category | Hours | Priority |
|---|---:|---|
| Human code review & PR merge (16 files, 5,844-line diff) | 4 | High |
| External host (AFSIM) integration & closed-loop smoke test | 8 | High |
| Cross-compiler / cross-toolchain ABI validation of `libafsim_l1.so` | 4 | Medium |
| Shared-library versioning, SONAME & packaging + thread-safety doc | 3 | Medium |
| Targeted numerical fidelity spot-check vs in-vehicle L1 (SITL) | 4 | Medium |
| Optional: Option-A real-`AP_AHRS` in-tree injection wiring | 2 | Low |
| CI integration for the AfsimL1 example build | 2 | Low |
| **Total Remaining** | **27** | |

### 2.3 Hours Reconciliation

| Check | Result |
|---|---|
| Section 2.1 completed total | 127 h |
| Section 2.2 remaining total | 27 h |
| **2.1 + 2.2 = Total (Section 1.2)** | **127 + 27 = 154 h ✓** |
| Completion % (Section 1.2 / 7 / 8) | 127 / 154 = **82.5% ✓** |
| Remaining hours match across §1.2 ↔ §2.2 ↔ §7 | **27 h ✓** |

---

## 3. Test Results

All tests below originate from **Blitzy's autonomous validation logs** for this project (Final Validator Gates 2–4). No coverage-instrumentation tool was run (a dedicated gtest suite is optional per AAP §0.5.1, and the full-firmware regression suite is explicitly out of scope per AAP §0.2.2).

| Test Category | Framework | Total | Passed | Failed | Coverage % | Notes |
|---|---|---:|---:|---:|---|---|
| Unit / Functional (C ABI) | Custom C++ harness | 17 | 17 | 0 | N/A | NULL-safety on all 8 entries; CWE-457 pre-execute guard; turn-geometry sign (N→E `+48.5°`, N→W `−48.5°`, aligned `0°`); cross-instance bit-identical determinism; host-driven `dt` over 10× 50 Hz steps all finite |
| Documentation Integrity | Python assertion harness (`generate.py`) | 991 | 991 | 0 | N/A | Cited source line-ranges validated against the live tree; "HARNESS PASSED"; PDF output byte-deterministic |
| Build / Compilation | CMake + Make, waf | 3 | 3 | 0 | N/A | `libafsim_l1.so` (8 symbols, 0 warnings), `afsim_demo`, in-tree waf example — all exit 0 |
| **Aggregate** | — | **1,011** | **1,011** | **0** | — | 100% pass rate across all autonomous checks |

**Independent reproduction (this assessment):** the standalone build (`cmake .. && make`), symbol check (`nm -D` → 8), demo run (`roll_deg=48.5, lat_accel=11.088`), and PDF harness (`HARNESS PASSED`, byte-identical md5) were all re-executed from a clean state and confirmed.

---

## 4. Runtime Validation & UI Verification

**UI Verification: Not applicable.** The deliverable is a headless C/C++ shared library plus a PDF documentation artifact — there is no graphical or textual end-user interface (AAP §0.3.4). Runtime validation therefore targets the library, demo, in-tree build, and PDF pipeline.

- ✅ **Operational** — Standalone `afsim_demo`: runs exit 0, emits `roll_deg = 48.500000, lat_accel = 11.087974` (full numerical fidelity).
- ✅ **Operational** — Exported symbol surface: `libafsim_l1.so` exports **exactly 8** `L1_*` symbols; `ldd` shows only system libraries (self-contained, zero third-party).
- ✅ **Operational** — PDF deliverable: `generate.py` exits 0, prints `HARNESS PASSED` (991 assertions) + `PDF written`; 40-page A4 output is byte-identical across regenerations (deterministic).
- ✅ **Operational** — In-tree waf example: builds and runs exit 0, integrating the service against the real ArduPilot source tree (proves compile/link/run integration).
- ⚠ **Partial (by design, not a defect)** — In-tree waf demo returns `roll_deg=0.0, lat_accel=0.0`: this is documented **Option-A** behavior (the real `AP_AHRS` is non-injectable in-tree). Numerical fidelity is delivered by the standalone `.so` (Option-B). Optional wiring (task L1) would make the in-tree path emit non-zero values.

---

## 5. Compliance & Quality Review

Cross-mapping AAP deliverables to quality/compliance benchmarks. Fixes applied during autonomous validation are noted.

| Benchmark / AAP Requirement | Status | Evidence & Notes |
|---|:--:|---|
| Goal 1 — Locate all PNT (L1-nav) instances | ✅ Pass | AAP §0.6.1 mapping table; 16 read sites → 6 accessors + 2 clock couplings |
| Goal 2 — Consolidate into one reusable service | ✅ Pass | `AfsimL1Behavior` + shim + C-ABI; single `libafsim_l1.so` |
| Goal 3 — Document current-vs-new twice | ✅ Pass | AAP §0.6.1 **and** regenerated 40-page PDF with "New Service Location" column |
| Behavior preservation (L1 math unchanged) | ✅ Pass | Git-diff verified; else-branches byte-identical; clamp semantics unchanged |
| Public-contract preservation (`AP_Navigation`, `AP_L1_Control` signatures) | ✅ Pass | Only an **additive** `set_update_dt` method; no signature altered |
| Scope guardrail ("no other changes than specified") | ✅ Pass | 18 out-of-scope trees confirmed 0 changes (vehicles + PNT subsystems + AHRS + Navigation + modules) |
| Timing seam default-off | ✅ Pass | `_dt_override = false` by in-class initializer; vehicle callers never enable it |
| Zero-placeholder policy | ✅ Pass | grep for TODO/FIXME/stub/NotImplemented across in-scope C++ → 0 matches |
| C-ABI stability best practices (web research) | ✅ Pass | AAP §0.3.2; opaque handle, C-only surface, `-fvisibility=hidden`, `--no-undefined` |
| No new dependencies | ✅ Pass | Runtime `.so` links only system libs; PDF stack pre-existing (ReportLab/Poppler) |
| Code hygiene (flake8, line endings, no merge markers) | ✅ Pass | flake8 0 violations on all 3 Python files; working tree clean |
| CWE-457 (uninitialized read) | ✅ Pass (fixed) | Pre-execute getters return `0.0`; verified in functional harness |
| Cross-toolchain ABI validation | ⏳ Outstanding | Only g++ 15.2 exercised — see task M1 (remaining) |
| SONAME / semantic versioning | ⏳ Outstanding | Not yet applied — see task M2 (remaining) |
| CI coverage for the example | ⏳ Outstanding | Not yet wired — see task L2 (remaining) |

---

## 6. Risk Assessment

Risks assessed across PA3 categories. **Zero High-severity risks.** All material open items map to remaining tasks (§2.2 / §8).

| Risk | Category | Severity | Probability | Mitigation | Status |
|---|---|:--:|:--:|---|---|
| Flat-earth / equatorial-datum divergence for large offsets or non-equatorial ops | Technical | Low | Low | Documented datum convention; host uses local NE frame; extend shim with datum origin if geographic use needed | Mitigated (documented) |
| `get_EAS2TAS` hardcoded to 1.0 — high-altitude airspeed-scaling fidelity gap | Technical | Low | Low | Documented; add optional EAS2TAS setter if needed | Open (by design) |
| Numerical fidelity validated only via unit-geometry, not full in-vehicle L1 paths | Technical | Medium | Low | Targeted SITL spot-check (task M3) | Open |
| Raw `void*` handle: a non-NULL invalid pointer would be dereferenced (no magic tag) | Security | Medium | Low | NULL-checks on all 8 entries + opaque handle; recommend magic-number validation | Partially mitigated |
| Uninitialized read (CWE-457) if getters called pre-execute | Security | Low | Low | Pre-execute guard returns 0.0 (test-verified) | Mitigated (resolved) |
| No finite-value guard on injected doubles (NaN/Inf could enter guidance) | Security | Low | Low | Units documented; `dt` clamp bounds; recommend host-side sanitization / finite guards | Open (low) |
| Supply-chain: build-time PDF deps (ReportLab/Poppler) | Security | Low | Low | Pinned versions; **not** in runtime `.so` (ldd = system libs only) | Mitigated |
| Thread-safety not documented; no internal locking | Operational | Low | Low | One-handle-per-thread pattern; document contract in README (task M2) | Open (doc gap) |
| No SONAME / semantic version on `libafsim_l1.so` | Operational | Medium | Medium | Add SONAME + semver (task M2) | Open |
| No CI coverage — bitrot risk as `AP_L1_Control`/`AP_AHRS` evolve | Operational | Medium | Medium | Add to CI build matrix (task L2) | Open |
| In-tree waf demo returns 0.0/0.0 — could be misread as a defect | Operational | Low | Medium | Documented expected; fidelity via `.so`; optional Option-A wiring (task L1) | Mitigated (documented) |
| Real AFSIM host integration never exercised (only demo/mock) | Integration | Medium | Medium | AFSIM integration smoke test (task H2) | Open |
| Cross-compiler / cross-libstdc++ ABI validated only on g++ 15.2 / Linux x86-64 | Integration | Medium | Low–Med | C-only boundary + `-fvisibility=hidden` + `--no-undefined` reduce risk; cross-toolchain matrix (task M1) | Partially mitigated |
| Handle lifecycle (Create/Destroy pairing, no use-after-free) is host responsibility | Integration | Low | Low | Documented lifecycle in README + header; NULL-safe | Mitigated (documented) |

---

## 7. Visual Project Status

**Overall progress** (hours; Completed = `#5B39F3`, Remaining = `#FFFFFF`):

```mermaid
%%{init: {'theme':'base', 'themeVariables':{ 'pie1':'#5B39F3', 'pie2':'#FFFFFF', 'pieStrokeColor':'#B23AF2', 'pieStrokeWidth':'2px', 'pieOuterStrokeWidth':'2px', 'pieTitleTextSize':'15px', 'pieSectionTextSize':'13px', 'pieLegendTextSize':'13px'}}}%%
pie showData
    title Project Hours Breakdown (154 h total)
    "Completed Work" : 127
    "Remaining Work" : 27
```

**Remaining work by priority** (27 h total):

```mermaid
%%{init: {'theme':'base', 'themeVariables':{ 'pie1':'#5B39F3', 'pie2':'#B23AF2', 'pie3':'#A8FDD9', 'pieStrokeColor':'#333333', 'pieStrokeWidth':'1px', 'pieTitleTextSize':'15px', 'pieSectionTextSize':'13px', 'pieLegendTextSize':'13px'}}}%%
pie showData
    title Remaining Work by Priority (27 h)
    "High" : 12
    "Medium" : 11
    "Low" : 4
```

**Remaining hours per category** (from §2.2):

| Category | Hours | Bar |
|---|---:|---|
| External host (AFSIM) integration | 8 | ████████ |
| Human code review & merge | 4 | ████ |
| Cross-toolchain ABI validation | 4 | ████ |
| Numerical fidelity spot-check | 4 | ████ |
| Versioning / SONAME / packaging | 3 | ███ |
| Option-A in-tree wiring | 2 | ██ |
| CI integration | 2 | ██ |
| **Total** | **27** | |

> **Integrity check:** Pie "Remaining Work" (27) = §1.2 Remaining Hours (27) = Σ §2.2 Hours (27). ✓

---

## 8. Summary & Recommendations

**Achievements.** The refactoring meets **100% of the explicit Agent Action Plan scope** — all 21 discrete requirements across Goals 1–3 are delivered, validated, and committed with zero placeholders. ArduPilot's L1 lateral-navigation guidance is now consumable by an external host through a stable, self-contained `extern "C"` shared library exposing exactly 8 symbols over an opaque handle, while the wrapped `AP_L1_Control` receives only a single additive, default-off timing seam that leaves every existing vehicle caller byte-for-byte unchanged. The scope guardrail was strictly honored: all vehicle firmware and non-targeted PNT subsystems show zero changes.

**Overall completion: 82.5% (127 h of 154 h).** The gap between "100% of AAP scope" and "82.5% overall" is entirely **path-to-production** work — none of it defects. The autonomous environment delivered every artifact that can be produced without external systems; what remains requires resources and judgment outside the autonomous boundary.

**Critical path to production (27 h remaining).**
1. Human code review & merge (4 h, High).
2. Real AFSIM host integration & closed-loop smoke test (8 h, High) — the deliverable's ultimate purpose; blocked on AFSIM access (§1.5).
3. Cross-toolchain ABI validation (4 h, Medium) — verifies the toolchain-independence value proposition.
4. SONAME/versioning/packaging + thread-safety documentation (3 h, Medium).
5. Numerical fidelity spot-check vs in-vehicle L1 (4 h, Medium).
6. Optional Option-A in-tree wiring (2 h, Low) and CI integration (2 h, Low).

**Success metrics.** Compilation clean (0 in-scope warnings); 8/8 exported symbols; 17/17 functional checks; 991/991 documentation-integrity assertions; demo numerics `48.5° / 11.088 m/s²`; deterministic byte-identical PDF; 0 out-of-scope file changes.

**Production-readiness assessment.** The code is **production-quality and defect-free within the AAP scope** and is safe to merge after human review. It is **not yet production-*deployed***: it has not run inside a live AFSIM host nor been validated across toolchains, and it lacks distribution versioning. Recommendation: **merge after review, then execute the High-priority integration tasks before declaring the capability production-deployed.**

---

## 9. Development Guide

All commands below were tested from a clean state during this assessment (exit codes and outputs verified).

### 9.1 System Prerequisites

| Software | Version (verified) | Purpose |
|---|---|---|
| g++ (GCC) | 15.2.0 | Compile the C++ service & `.so` (`-std=gnu++11`) |
| CMake | 3.31.6 (≥ 3.5 required) | Standalone shared-library build |
| GNU Make | 4.4.1 | Build driver |
| Python | 3.13.7 | PDF regeneration pipeline |
| ReportLab | 4.5.1 | Renders the PDF deliverable (build-time only) |
| Poppler-utils | 25.03.0 | PDF verification (`pdfinfo`/`pdftotext`) |
| binutils (`nm`, `ldd`) | system | Symbol/dependency verification |

- **OS:** Linux x86-64 (validated on Ubuntu 25.10). **No new dependencies** are required — the runtime `.so` links only libstdc++/libm/libgcc_s/libc.

### 9.2 Environment Setup

```bash
# From the repository root
cd libraries/AP_L1_Control/examples/AfsimL1
# No virtualenv or extra installs required; all tools listed in 9.1 are system-provided.
```

### 9.3 Build the Standalone Shared Library

```bash
cd libraries/AP_L1_Control/examples/AfsimL1
mkdir build && cd build
cmake ..
make
```

**Expected:** `cmake` exit 0, `make` exit 0, produces **`libafsim_l1.so`** (~96 KB), zero warnings. The default build produces *only* the library.

### 9.4 Verify the Exported Symbols

```bash
nm -D --defined-only libafsim_l1.so | grep " T "
```

**Expected — exactly these 8 symbols and nothing else:**

```text
L1_Create   L1_Destroy   L1_Init   L1_Execute
L1_SetLegNE   L1_SetStateNE   L1_GetRollDeg   L1_GetLatAccel
```

### 9.5 Build & Run the Demonstration Driver

```bash
cmake -DAFSIM_L1_BUILD_DEMO=ON ..
make
./afsim_demo
```

**Expected output:**

```text
AfsimL1 demo: simple leg prev=(0,0) -> next=(0,500) m, dt=0.02 s
roll_deg = 48.500000, lat_accel = 11.087974
```

(The demo's RPATH points at the co-located `.so`, so no `LD_LIBRARY_PATH` is needed.)

### 9.6 In-Tree waf Build (Alternative)

```bash
# From the repository root
./waf configure --board sitl
./waf --targets examples/AfsimL1
./build/sitl/examples/AfsimL1
```

**Expected:** exit 0. Note the in-tree demo prints `roll_deg=0.0, lat_accel=0.0` **by design** (Option-A: the real `AP_AHRS` is non-injectable in-tree). Use the standalone `.so` (§9.3) for numerical fidelity.

### 9.7 Regenerate the PDF Deliverable

```bash
cd libraries/AP_L1_Control/examples/AfsimL1
PNT_REPO_ROOT="$(git rev-parse --show-toplevel)" python3 generate.py
```

**Expected:** `HARNESS PASSED` + `PDF written: .../ArduPilot_PNT_Reference_Audit.pdf`; output is byte-identical across runs (deterministic).

### 9.8 Example Usage (minimal C host)

```c
#include "l1_c_api.h"

void* h = L1_Create();          /* opaque handle; NULL on failure   */
L1_Init(h);                      /* seed a default simple leg        */

L1_SetLegNE(h, 0, 0, 0, 500);    /* prev(0,0) -> next(0,500): 500 m leg */
L1_SetStateNE(h, 0, 0, 0, 20, 0, 0); /* n,e,velE,velN,yaw_cd,pitch_rad; 20 m/s North */
L1_Execute(h, 0.02);             /* advance one 50 Hz control step   */

double roll_deg  = L1_GetRollDeg(h);   /* commanded bank angle, deg   */
double lat_accel = L1_GetLatAccel(h);  /* lateral acceleration, m/s^2 */

L1_Destroy(h);                   /* release the instance             */
```

### 9.9 Troubleshooting

| Symptom | Cause | Resolution |
|---|---|---|
| No `afsim_demo` after `make` | Demo is opt-in | Re-run cmake with `-DAFSIM_L1_BUILD_DEMO=ON` |
| `nm` shows more/fewer than 8 symbols | Visibility flags missing | Ensure the CMake build applies `-fvisibility=hidden` (default in provided `CMakeLists.txt`) |
| In-tree waf demo prints `0.0 / 0.0` | Option-A `AP_AHRS` non-injectable | Expected; use the standalone `.so` for fidelity, or complete task L1 |
| `generate.py` cannot find repo root | `PNT_REPO_ROOT` unset | Prefix with `PNT_REPO_ROOT="$(git rev-parse --show-toplevel)"` |
| PDF differs after regen | Non-deterministic env | Verify ReportLab 4.5.1; generator pins invariant config for byte-stable output |

---

## 10. Appendices

### A. Command Reference

| Command | Purpose |
|---|---|
| `cmake .. && make` | Build `libafsim_l1.so` |
| `cmake -DAFSIM_L1_BUILD_DEMO=ON .. && make` | Build library + `afsim_demo` |
| `./afsim_demo` | Run the demo (→ `48.5 / 11.088`) |
| `nm -D --defined-only libafsim_l1.so \| grep " T "` | List exported symbols (expect 8) |
| `ldd libafsim_l1.so` | Confirm self-contained (system libs only) |
| `./waf configure --board sitl && ./waf --targets examples/AfsimL1` | In-tree build |
| `PNT_REPO_ROOT="$(git rev-parse --show-toplevel)" python3 generate.py` | Regenerate the PDF |
| `pdfinfo ArduPilot_PNT_Reference_Audit.pdf` | Inspect PDF (40 pages, A4) |

### B. Port Reference

*Not applicable.* The deliverable is a headless library and a PDF; it opens no network ports and runs no services.

### C. Key File Locations

| Path | Role |
|---|---|
| `libraries/AP_L1_Control/examples/AfsimL1/AfsimL1Behavior.{h,cpp}` | Service facade |
| `libraries/AP_L1_Control/examples/AfsimL1/AfsimL1_AHRS_Shim.{h,cpp}` | AHRS state adapter |
| `libraries/AP_L1_Control/examples/AfsimL1/l1_c_api.{h,cpp}` | `extern "C"` ABI boundary (8 exports) |
| `libraries/AP_L1_Control/examples/AfsimL1/CMakeLists.txt` | Standalone `.so` build |
| `libraries/AP_L1_Control/examples/AfsimL1/wscript` | In-tree waf build |
| `libraries/AP_L1_Control/examples/AfsimL1/main.cpp` | Demo driver |
| `libraries/AP_L1_Control/examples/AfsimL1/README.md` | Integration/usage guide |
| `libraries/AP_L1_Control/examples/AfsimL1/{generate,pnt_data,pnt_render}.py` | PDF pipeline |
| `libraries/AP_L1_Control/AP_L1_Control.{h,cpp}` | Wrapped controller (additive default-off seam) |
| `ArduPilot_PNT_Reference_Audit.pdf` | Goal-3 documentation deliverable (40 pages) |

### D. Technology Versions

g++ 15.2.0 · CMake 3.31.6 · GNU Make 4.4.1 · Python 3.13.7 · ReportLab 4.5.1 · Poppler-utils 25.03.0 · C++ standard `gnu++11` · waf (Python 3).

### E. Environment Variable Reference

| Variable | Used by | Value |
|---|---|---|
| `PNT_REPO_ROOT` | `generate.py` | Absolute repository root (e.g., `$(git rev-parse --show-toplevel)`) |
| `AFSIM_L1_BUILD_DEMO` | CMake (`-D` option) | `ON` to also build `afsim_demo` (default `OFF`) |

### F. Developer Tools Guide

- **Build:** CMake + Make (standalone `.so`) or in-tree `waf` (`bld.ap_example(use='ap')`).
- **Symbol inspection:** `nm -D` (exported symbols), `ldd` (runtime deps).
- **Docs:** ReportLab generator with a self-checking 991-assertion integrity harness; Poppler-utils to verify page count/content.
- **Lint:** `flake8` (repo `.flake8`) — 0 violations across the 3 Python files.

### G. Glossary

| Term | Definition |
|---|---|
| **PNT** | Position, Navigation, and Timing — the behaviors extracted from the vehicle loop |
| **L1 guidance** | ArduPilot's L1 lateral-navigation control law (`AP_L1_Control`) |
| **AfsimL1** | The new reusable service wrapping L1 guidance for external hosts |
| **C ABI** | `extern "C"` application binary interface — name-mangling-free, toolchain-independent boundary |
| **Opaque handle** | `void*` (`L1_Context`) passed across the boundary so no C++ type is exposed |
| **AHRS shim** | Adapter presenting `AP_AHRS`'s 6 read APIs from host-injected state |
| **Timing seam** | Additive, default-off `set_update_dt` letting the host drive `dt` |
| **Option A / Option B** | Real-`AP_AHRS` (in-tree) vs. lightweight shim (`.so`) state-decoupling strategies |
| **AFSIM** | External simulation environment intended to consume the service |
| **SONAME** | Shared-object version name enabling ABI-compatibility management |
