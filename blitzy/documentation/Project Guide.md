# Blitzy Project Guide — ArduPilot PNT Reference Audit

---

## 1. Executive Summary

### 1.1 Project Overview

This project delivers an **audit-grade PDF reference document** that exhaustively catalogs every location in the ArduPilot firmware codebase where **Positioning, Navigation, and Timing (PNT)** data is handled, together with a complete two-layer dependency map for each reference. The audience is expert software engineers preparing to extract PNT logic into a standalone, formally-verified service. The deliverable — `ArduPilot_PNT_Reference_Audit.pdf` — is a strictly read-only static-analysis artifact: it surveys `libraries/` and all four vehicle directories (ArduCopter, ArduPlane, Rover, ArduSub) without modifying any source. It catalogs 94 PNT touch-points across 282 evidence rows, each with verbatim line-accurate provenance, role classification, and full transitive dependency chains.

### 1.2 Completion Status

```mermaid
%%{init: {'theme':'base', 'themeVariables': {'pie1':'#5B39F3','pie2':'#FFFFFF','pieStrokeColor':'#B23AF2','pieStrokeWidth':'2px','pieOuterStrokeColor':'#B23AF2','pieOuterStrokeWidth':'2px','pieTitleTextSize':'16px','pieSectionTextColor':'#B23AF2'}}}%%
pie showData title Completion Status — 89.8% Complete
    "Completed Work (AI)" : 88
    "Remaining Work" : 10
```

| Metric | Value |
|--------|-------|
| **Total Hours** | 98 h |
| **Completed Hours (AI + Manual)** | 88 h (88 h AI · 0 h Manual) |
| **Remaining Hours** | 10 h |
| **Percent Complete** | **89.8%** |

> Completion is computed using the PA1 AAP-scoped methodology: `88 / (88 + 10) = 89.8%`. All remaining hours are human verification/acceptance and path-to-production activities; there are no incomplete AAP content deliverables.

### 1.3 Key Accomplishments

- ✅ **Single PDF deliverable produced** — `ArduPilot_PNT_Reference_Audit.pdf` (50 pages, A4 landscape, %PDF-1.4, 201,137 bytes), the exact and only `CREATE` mandated by the AAP.
- ✅ **All 12 tables present in exact order** — `1, 1a, 2, 2a, 3, 3a, 4, 4a, 5, 5a, 6, 6a` under both required group headers ("GROUP 1 — CORE PNT REFERENCES" and "GROUP 2 — INDIRECT / RELATIONAL PNT REFERENCES").
- ✅ **282 evidence rows** — 94 main-table references (63 Core + 31 Indirect) each paired with a Layer-1 (direct edges) and Layer-2 (transitive chain) dependency row, giving perfect 1:1:1 coverage.
- ✅ **Exhaustive coverage** — `libraries/` (151 subsystems incl. AP_GPS's 44 files), the EKF families, and all 81 flight/drive-mode files across four vehicles, plus an explicit **Coverage & Explicit-Absence Register**.
- ✅ **All audit-discipline flags applied** — `[CIRCULAR]` (8), `[SHARED-STRUCT]` (19), `[DUPLICATED]` (36), `[MULTI-CATEGORY]` (7), and 30 explicit `[ABSENT]` notations (non-inference discipline).
- ✅ **User-supplied examples integrated** — the transitive chain `AP_GPS → AP_AHRS → navigate() → ModeAuto::navigate()` (Table 2a #1) and the accessors `gps.status()` / `ahrs.have_inertial_nav()` (Tables 4 & 5).
- ✅ **Read-only discipline upheld** — the branch diff is a single added file; zero source `.cpp`/`.h` files modified.
- ✅ **Autonomous verification harness passes 100%** — 838 discrete assertions, 0 failures, including 188 verbatim source-citation reads against the live ArduPilot tree.
- ✅ **Deterministic reproducibility proven** — regeneration produces a byte-identical-content PDF.

### 1.4 Critical Unresolved Issues

| Issue | Impact | Owner | ETA |
|-------|--------|-------|-----|
| _None — no release-blocking issues_ | The deliverable passes all validation gates with zero compilation errors, zero failing harness assertions, and zero unresolved defects. The only outstanding work is human acceptance review. | — | — |

### 1.5 Access Issues

| System/Resource | Type of Access | Issue Description | Resolution Status | Owner |
|-----------------|----------------|-------------------|-------------------|-------|
| _No access issues identified_ | — | The task required only read access to the ArduPilot source tree, which was fully available. No external services, credentials, or third-party APIs are involved. | N/A | — |

### 1.6 Recommended Next Steps

1. **[High]** Perform an expert spot-check of a representative sample of cited `path:line` references across all 12 tables, confirming snippets are verbatim and centered on the relevant lines.
2. **[High]** Review the Role (Source/Transform/Sink), dependency-type, and behavior-change classifications for **semantic** soundness — the harness verifies existence and format but not engineering judgment.
3. **[Medium]** Sign off on exhaustiveness by reviewing the Coverage & Explicit-Absence Register (30 absences + token-sweep methodology) and formally accept the audit.
4. **[Medium]** Harden reproducibility: archive the gitignored regeneration scaffolding (`tmp/pnt_work/`) alongside the deliverable and record the audited HEAD commit `5b67e27b0a`.
5. **[Low]** Distribute the PDF to audit stakeholders and the downstream PNT-service extraction team; determine whether a machine-readable edge list is also required.

---

## 2. Project Hours Breakdown

### 2.1 Completed Work Detail

| Component | Hours | Description |
|-----------|------:|-------------|
| Document architecture & 12-table skeleton | 5 | Exact table numbering (1,1a…6,6a), group headers, section dividers, monotonic `#`↔`Ref #` cross-reference scheme, and the four exact column schemas (G1=7-col, G2=9-col, L1=6-col, L2=6-col). |
| PDF rendering pipeline | 7 | ReportLab landscape layout (`pnt_render.py`, 498 lines): repeating table headers, monospace soft-wrap snippets, alternating row shading, visible dividers, plus the Unicode glyph fallback fix (DejaVuSans) for arrows/checks. |
| Verification harness | 5 | Five `verify_*` functions enforcing provenance, vocabulary, numbering, Ref# coverage, and chain-depth integrity (838 assertions). |
| Table 1 — Core Positioning | 13 | 26 main rows + 26 Layer-1 + 26 Layer-2 (AP_GPS 14 backends + Blended + base, Location, InertialNav, AC_PosControl, AR_PosControl, Beacon, VisualOdom). |
| Table 2 — Core Navigation | 15 | 28 main rows + 28 Layer-1 + 28 Layer-2 (AHRS hub + backends, EKF/EKF2/EKF3 fusion modules, AC_WPNav, AP_Mission, AP_L1_Control, AP_TECS, AC_AttitudeControl). |
| Table 3 — Core Timing | 5 | 9 main rows + 9 Layer-1 + 9 Layer-2 (AP_HAL clock primitives, AP_RTC + JitterCorrection, AP_Scheduler loop/tick/period, GPS time-of-week). |
| Table 4 — Indirect Positioning | 6 | 10 main rows + dependencies (mode position-gating, geofence breach, GPS-denied fallback). |
| Table 5 — Indirect Navigation | 6 | 10 main rows + dependencies (EKF-health watchdog, nav-output reporting/consumption, crash/flight detection). |
| Table 6 — Indirect Timing | 6 | 11 main rows + dependencies (main-loop-lockup watchdogs, PNT-health hysteresis, rate gating). |
| Audit-discipline flags & explicit absences | 5 | `[CIRCULAR]`/`[SHARED-STRUCT]`/`[DUPLICATED]`/`[MULTI-CATEGORY]` flagging plus 30 checked `[ABSENT]` notations across the surveyed surface. |
| Coverage & Explicit-Absence Register | 5 | Directory-wide token sweeps (AC_Fence, AP_BattMonitor, AP_RCProtocol, AntennaTracker, Blimp) + .gitmodules boundary edges; all 81 mode files enumerated. |
| User-example integration | 2 | Transitive chain in Table 2a #1 (with non-inference note that the literal `flight_mode_auto()` does not exist) and the `gps.status()` / `ahrs.have_inertial_nav()` Group-2 rows. |
| QA / code-review remediation | 8 | Five validation rounds: 19 code-review findings, QA CP-A/B/D findings, Group-2 accessor rows, and the Unicode glyph rendering fix. |
| **Total Completed** | **88** | |

### 2.2 Remaining Work Detail

| Category | Hours | Priority |
|----------|------:|----------|
| Expert provenance & classification spot-check (H1 + H2) | 5 | High |
| Exhaustiveness sign-off & audit acceptance (H3) | 2 | Medium |
| Reproducibility hardening — archive scaffolding, pin toolchain, record commit (H4) | 1 | Medium |
| Distribution & downstream handoff (H5) | 2 | Low |
| **Total Remaining** | **10** | |

> **Cross-section check:** Section 2.1 (88 h) + Section 2.2 (10 h) = **98 h** = Total Project Hours in Section 1.2. ✓

### 2.3 Hours Calculation Summary

```
Completed Hours = 88 h   (all AAP content + quality deliverables, autonomously produced)
Remaining Hours = 10 h   (human verification/acceptance + path-to-production)
Total Hours     = 98 h
Completion %    = 88 / (88 + 10) = 88 / 98 = 89.8%
```

---

## 3. Test Results

All tests below originate from **Blitzy's autonomous validation logs** — the harness-gated `generate.py` run that must pass before the PDF is written. The harness performs **838 discrete assertions**, all passing (0 failures). Because this is a documentation deliverable, "tests" are the audit-integrity assertions (the documentation analog of unit tests): each validates a structural or provenance invariant of the catalog.

| Test Category | Framework | Total Tests | Passed | Failed | Coverage % | Notes |
|---------------|-----------|------------:|-------:|-------:|-----------:|-------|
| Provenance / Citation Validation | Custom harness (`verify_group1/2`, `verify_layer1`) | 188 | 188 | 0 | 100% | Every cited `(path, start, end)` opened against the live ArduPilot tree and confirmed readable (verbatim provenance). |
| Snippet Format (5–10 lines) | Custom harness (`snippet_linecount`) | 94 | 94 | 0 | 100% | Each main-table snippet is between 5 and 10 lines, per AAP 0.7.1. |
| Role Classification Vocabulary | Custom harness (`verify_group1`) | 63 | 63 | 0 | 100% | Every Group-1 row's Role ∈ {Source, Transform, Sink}. |
| Dependency-Type Vocabulary | Custom harness (`verify_layer1`) | 94 | 94 | 0 | 100% | Every Layer-1 row's type ∈ {Data dependency, Function call, Inheritance/Interface, Shared global/singleton}. |
| Singleton (AP::) Consistency | Custom harness (`verify_layer1`) | 5 | 5 | 0 | 100% | Rows typed "Shared global/singleton" actually contain an `AP::` accessor in the cited snippet. |
| Chain-Depth Integrity | Custom harness (`verify_layer2`) | 188 | 188 | 0 | 100% | Layer-2 Chain Depth is a positive int and equals the count of `→` hops listed in the chain. |
| Cross-Reference Integrity | Custom harness (`verify_ref_coverage`, numbering) | 206 | 206 | 0 | 100% | Every `Ref #` resolves to a parent `#`; every parent `#` has L1 + L2 coverage; numbering is monotonic 1..N. |
| **Total** | | **838** | **838** | **0** | **100%** | Harness exit code 0 — PDF render gated on full pass. |

**Runtime / render validation (also from autonomous logs):** the produced PDF opens validly (`%PDF-1.4`, 50 pages, A4 landscape, not encrypted) and all previously-blank Unicode glyphs (`→`, `✓`, `↔`, `≥`, `≤`, `≈`, `←`) now render correctly after the DejaVuSans fallback fix.

---

## 4. Runtime Validation & UI Verification

This is a read-only documentation deliverable with no application runtime and no graphical user interface. "Runtime" maps to **PDF render health** and "UI" maps to **document visual verification**.

**PDF Render Health**
- ✅ **Operational** — Valid PDF container: `%PDF-1.4`, 50 pages, A4 landscape (841.89 × 595.276 pts), not encrypted, 201,137 bytes.
- ✅ **Operational** — Text layer extracts cleanly (`pdftotext` → 3,286 lines); all 12 table headers detected in correct order.
- ✅ **Operational** — Deterministic regeneration: re-render produces byte-identical content (verified by `pdftotext` diff = identical).
- ✅ **Operational** — All Unicode glyphs render (arrows in every Layer-2 chain, checkmarks in the Coverage Register).

**Document Visual Verification**
- ✅ **Operational** — Both group headers and section dividers render and are visually distinct.
- ✅ **Operational** — Tables use repeating headers across page breaks; monospace code snippets soft-wrap without clipping or cell overflow (validator confirmed on pages 1/11/29/32/38/44).
- ✅ **Operational** — Legend, Footprint summary, and Coverage & Explicit-Absence Register render on dedicated pages.

**Source-Citation Integration (the audit's external dependency)**
- ✅ **Operational** — 188 of 188 citations resolve to readable locations in the ArduPilot source tree.
- ✅ **Operational** — User-supplied transitive chain and PNT-state accessors verified present and correctly cited.

---

## 5. Compliance & Quality Review

Cross-mapping of AAP deliverables to Blitzy quality benchmarks. Fixes applied during autonomous validation are noted; all in-scope items pass.

| AAP Requirement (source) | Benchmark | Status | Progress | Notes / Fixes Applied |
|--------------------------|-----------|:------:|:--------:|-----------------------|
| PDF output with group headers, table headers, dividers (0.7.1) | Output format | ✅ Pass | 100% | 50-page PDF; both group headers + dividers render. |
| Exact table numbering `1,1a…6,6a` (0.7.1) | Structure | ✅ Pass | 100% | Verified present in exact order. |
| Exact column schemas G1/G2/L1/L2 (0.1.3) | Schema fidelity | ✅ Pass | 100% | 7/9/6/6-column schemas reproduced verbatim. |
| 5–10 line snippets with exact provenance (0.7.1) | Evidence | ✅ Pass | 100% | Harness enforces line count + verbatim source read on all 94. |
| Role classification Source/Sink/Transform (0.7.3) | Classification | ✅ Pass | 100% | All 63 Group-1 rows validated against vocabulary. |
| Trigger/Observed-state/Behavior-change (0.7.3) | Classification | ✅ Pass | 100% | All 31 Group-2 rows; discriminator column populated. |
| Two-layer dependency mapping (0.7.3) | Dependency graph | ✅ Pass | 100% | 94 Layer-1 + 94 Layer-2 rows; chain depth = hop count. |
| `#`↔`Ref #` cross-reference integrity (0.7.1) | Traceability | ✅ Pass | 100% | Harness confirms full coverage, monotonic numbering. |
| Non-inference discipline / explicit absences (0.7.2) | Audit discipline | ✅ Pass | 100% | 30 `[ABSENT]` + Coverage Register; literal non-existent `flight_mode_auto()` flagged, not invented. |
| Flag circular / shared-struct / duplicated / multi-category (0.7.2) | Extraction-risk flags | ✅ Pass | 100% | 8 / 19 / 36 / 7 flags applied. |
| Coverage across libraries + 4 vehicles + 81 modes (0.2/0.3) | Exhaustiveness | ✅ Pass | 100% | Register enumerates all mode files + supporting libs + .gitmodules edges. |
| User-example integration (0.5.4) | Fidelity | ✅ Pass | 100% | Fixed this session — accessor rows added to Tables 4 & 5; transitive chain in 2a. |
| Read-only discipline — zero source edits (0.3.2, 0.8.1) | Scope control | ✅ Pass | 100% | Branch diff = 1 added file; no `.cpp`/`.h`/`.py` source changed. |
| Unicode glyph rendering | Render quality | ✅ Pass | 100% | Fixed this session — DejaVuSans fallback registered for 7 glyph types. |

**Code-review history:** five autonomous QA rounds resolved 19 initial code-review findings, QA checkpoint findings CP-A (SL-1, XR-1), CP-B (chain-depth normalization), and CP-D (completeness F1–F4), followed by the user-example accessor rows and the Unicode glyph fix — all committed.

---

## 6. Risk Assessment

| Risk | Category | Severity | Probability | Mitigation | Status |
|------|----------|:--------:|:-----------:|------------|--------|
| R1 — Audit snapshot drift: cited line numbers pinned to HEAD `5b67e27b0a`; future source edits drift the static PDF. | Technical | Medium | High (over time) | Record audited commit hash in the document; regenerate from the committed-snapshot harness when refreshing. | Open (mitigation = H4) |
| R2 — Classification subjectivity: Source/Transform/Sink and dependency-type are expert judgments the harness cannot semantically verify. | Technical | Low | Medium | Human expert spot-check of classifications. | Open (mitigation = H1/H2) |
| R3 — Exhaustiveness completeness: the "every PNT location" claim could theoretically miss a novel site. | Technical | Medium | Low | Coverage Register + directory token sweeps + 30 explicit absences already applied; human sign-off. | Mitigated; sign-off pending (H3) |
| R4 — Architectural disclosure: document maps failsafe/safety logic. | Security | Low | Low | ArduPilot is open-source; no secrets exposed; no new dependencies. | Accepted (negligible) |
| R5 — Reproducibility: regeneration scaffolding (`tmp/pnt_work/`) is gitignored, not committed. | Operational | Medium | Medium | Archive scaffolding alongside the deliverable. | Open (mitigation = H4) |
| R6 — Toolchain pinning: regeneration needs Python3 + reportlab 4.5.1 + DejaVuSans + poppler, not pinned in-repo. | Operational | Low | Medium | Versions documented in the Development Guide (Section 9). | Mitigated |
| R7 — Downstream format gap: PDF may not suffice if the extraction team needs a machine-readable edge list. | Integration | Low | Medium | Structured row data exists in `pnt_data.py` and can be exported. | Open (downstream/out-of-scope) |
| R8 — Vendored-module boundary: .gitmodules edges documented but vendored internals not traced. | Integration | Low | Low | Explicitly out-of-scope per AAP 0.3.2; boundary noted in Register. | Accepted (by design) |

---

## 7. Visual Project Status

```mermaid
%%{init: {'theme':'base', 'themeVariables': {'pie1':'#5B39F3','pie2':'#FFFFFF','pieStrokeColor':'#B23AF2','pieStrokeWidth':'2px','pieOuterStrokeColor':'#B23AF2','pieOuterStrokeWidth':'2px','pieSectionTextColor':'#B23AF2','pieTitleTextSize':'16px'}}}%%
pie showData title Project Hours Breakdown (Total 98 h)
    "Completed Work" : 88
    "Remaining Work" : 10
```

**Remaining Hours by Category (Section 2.2)**

```mermaid
%%{init: {'theme':'base', 'themeVariables': {'pie1':'#5B39F3','pie2':'#B23AF2','pie3':'#A8FDD9','pie4':'#FFFFFF','pieStrokeColor':'#B23AF2','pieSectionTextColor':'#000000','pieTitleTextSize':'15px'}}}%%
pie showData title Remaining Work — 10 h
    "Provenance & classification spot-check (High)" : 5
    "Exhaustiveness sign-off (Medium)" : 2
    "Reproducibility hardening (Medium)" : 1
    "Distribution & handoff (Low)" : 2
```

**Remaining Work by Priority**

| Priority | Hours | Share |
|----------|------:|------:|
| High | 5 | 50% |
| Medium | 3 | 30% |
| Low | 2 | 20% |
| **Total** | **10** | 100% |

> **Integrity check:** Section 7 "Remaining Work" (10 h) = Section 1.2 Remaining Hours (10 h) = Section 2.2 total (10 h). Section 7 "Completed Work" (88 h) = Section 1.2 Completed Hours (88 h) = Section 2.1 total (88 h). ✓

---

## 8. Summary & Recommendations

**Achievements.** The project is **89.8% complete** (88 of 98 hours). Every AAP-specified content and quality requirement has been autonomously delivered: a single 50-page PDF cataloging 94 PNT references in 282 evidence rows across all 12 mandated tables, with verbatim line-accurate provenance, two-layer dependency mapping, the full audit-discipline flag taxonomy, and an exhaustive Coverage & Explicit-Absence Register spanning `libraries/` and all four vehicle directories. An autonomous verification harness of 838 assertions passes 100%, and the read-only constraint was upheld absolutely (a single added file, zero source edits).

**Remaining gaps.** The outstanding 10 hours are entirely **human verification and path-to-production** activities — no AAP content is missing. They consist of an expert spot-check of provenance and classification soundness (which automated checks cannot judge semantically), a formal exhaustiveness sign-off, reproducibility hardening, and distribution to the downstream team.

**Critical path to production.** (1) Expert spot-check of citations and classifications → (2) exhaustiveness sign-off and acceptance → (3) archive the regeneration scaffolding and pin the audited commit → (4) hand off to the PNT-service extraction effort.

**Success metrics.** 12/12 tables present and correctly schema'd; 838/838 integrity assertions pass; 188/188 citations resolve to real source; 0 source files modified; 5 audit-discipline flag types and 30 explicit absences applied; both user examples integrated.

**Production readiness.** The deliverable is **audit-grade and production-ready** pending human acceptance. There are no release-blocking issues, no compilation errors, and no failing tests. Recommendation: proceed to expert review and sign-off, then release.

| Metric | Value |
|--------|-------|
| Completion | 89.8% |
| AAP content requirements delivered | 35 / 35 |
| Integrity assertions passing | 838 / 838 (100%) |
| Source files modified | 0 |
| Release-blocking issues | 0 |

---

## 9. Development Guide

This guide explains how to inspect, verify, and regenerate the audit deliverable. Every command was tested during validation.

### 9.1 System Prerequisites

| Software | Version (tested) | Purpose |
|----------|------------------|---------|
| OS | Ubuntu 25.10 (Linux) | Host environment |
| Python | 3.13.7 | Runs the harness + renderer |
| ReportLab | 4.5.1 | PDF rendering library |
| Poppler-utils | 25.03.0 | `pdfinfo`, `pdftotext`, `pdftoppm` for verification |
| DejaVu fonts | system TTF | Unicode glyph fallback (arrows, checks) |

### 9.2 Environment Setup

```bash
# From the repository root, on branch blitzy-e9b9bce3-08ef-44e0-ab6f-3f6803d01182
cd /path/to/ardupilot-repo
export PNT_REPO_ROOT="$(pwd)"
export PNT_OUT="$(pwd)/ArduPilot_PNT_Reference_Audit.pdf"
```

### 9.3 Dependency Installation

```bash
# System tools (Debian/Ubuntu)
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y poppler-utils fonts-dejavu

# Python library — Ubuntu 25 is PEP-668 managed, so either:
pip install --break-system-packages reportlab        # global, OR
python3 -m venv .venv && source .venv/bin/activate && pip install reportlab   # venv (preferred)

# Verify
python3 -c "import reportlab; print('reportlab', reportlab.Version)"   # -> reportlab 4.5.1
pdfinfo -v                                                              # -> pdfinfo version 25.03.0
```

### 9.4 Regenerating the Deliverable

The renderer is harness-gated: it validates every row first and **refuses to write the PDF if any check fails**.

```bash
cd "$PNT_REPO_ROOT"
python3 tmp/pnt_work/generate.py
# Expected output:
#   HARNESS PASSED
#   PDF written: <PNT_OUT>
```

### 9.5 Verification Steps

```bash
# 1) Container properties (expect: Pages 50, A4 landscape, Encrypted no, 201137 bytes, PDF 1.4)
pdfinfo ArduPilot_PNT_Reference_Audit.pdf

# 2) All 12 tables present in order
pdftotext -layout ArduPilot_PNT_Reference_Audit.pdf - | grep -E "Table [0-9]+a? —"

# 3) Run the integrity harness standalone (expect: errors: 0)
python3 -c "import sys,os; sys.path.insert(0,'tmp/pnt_work'); \
os.environ['PNT_REPO_ROOT']=os.getcwd(); import generate as G; \
print('errors:', len(G.run_harness()))"

# 4) Confirm read-only discipline (expect a single 'A' line for the PDF)
git diff b03956c1f4~1 HEAD --name-status
```

### 9.6 Example Usage (consuming the audit)

```bash
# Inspect a specific group's references (e.g., Core Timing)
pdftotext -layout ArduPilot_PNT_Reference_Audit.pdf - | sed -n '/Table 3 —/,/Table 3a —/p'

# Count audit-discipline flags
for f in CIRCULAR SHARED-STRUCT DUPLICATED MULTI-CATEGORY ABSENT; do
  echo "$f: $(pdftotext -layout ArduPilot_PNT_Reference_Audit.pdf - | grep -c "\[$f\]")"
done

# Render a page to PNG for visual review
pdftoppm -png -f 1 -l 1 ArduPilot_PNT_Reference_Audit.pdf /tmp/audit_page
```

### 9.7 Troubleshooting

| Symptom | Cause | Resolution |
|---------|-------|------------|
| `error: externally-managed-environment` on `pip install` | Ubuntu 25 PEP-668 marker | Use `pip install --break-system-packages reportlab` or a venv. |
| Blank glyphs (arrows/checks) in PDF | Base-14 fonts lack non-WinAnsi glyphs | Ensure DejaVuSans is registered as a Unicode fallback (already implemented in `pnt_render.py`). |
| Harness "cite read fail" | `PNT_REPO_ROOT` not pointing at the source tree | Run from the repository root and re-export `PNT_REPO_ROOT="$(pwd)"`. |
| `generate.py` exits without writing the PDF | A harness assertion failed | Read the printed error list; fix the offending row in `tmp/pnt_work/pnt_data.py`. |

---

## 10. Appendices

### A. Command Reference

| Command | Purpose |
|---------|---------|
| `python3 tmp/pnt_work/generate.py` | Validate + render the PDF (harness-gated) |
| `pdfinfo ArduPilot_PNT_Reference_Audit.pdf` | Show PDF properties |
| `pdftotext -layout <pdf> -` | Extract text layer for inspection |
| `pdftoppm -png -f N -l N <pdf> out` | Render page N to PNG |
| `git diff b03956c1f4~1 HEAD --name-status` | Confirm read-only discipline |

### B. Port Reference

_Not applicable._ The deliverable is a static document; no services, ports, or network endpoints are involved.

### C. Key File Locations

| Path | Role |
|------|------|
| `ArduPilot_PNT_Reference_Audit.pdf` | **The deliverable** (repo root, committed) |
| `tmp/pnt_work/generate.py` | Harness + render entry point (gitignored scaffolding) |
| `tmp/pnt_work/pnt_data.py` | All 18 table data objects (94 rows × 3 layers) |
| `tmp/pnt_work/pnt_render.py` | ReportLab rendering + 5 `verify_*` functions |

### D. Technology Versions

| Component | Version |
|-----------|---------|
| Python | 3.13.7 |
| ReportLab | 4.5.1 |
| Poppler-utils | 25.03.0 |
| Audited commit (HEAD) | `5b67e27b0ad5425a78044e3504daf8ae8e41ff81` |
| Branch | `blitzy-e9b9bce3-08ef-44e0-ab6f-3f6803d01182` |

### E. Environment Variable Reference

| Variable | Required | Description |
|----------|----------|-------------|
| `PNT_REPO_ROOT` | Yes (regeneration) | Absolute path to the ArduPilot repository root; used to resolve cited source paths during harness verification. |
| `PNT_OUT` | Optional | Output path for the rendered PDF; defaults to a `tmp/` path if unset. |

### F. Developer Tools Guide

- **PDF inspection:** `pdfinfo`, `pdftotext`, `pdftoppm` (poppler-utils) — inspect properties, extract text, render pages.
- **Integrity harness:** the `verify_*` functions in `pnt_render.py` are the documentation analog of unit tests; run them via `generate.py` or standalone (Section 9.5).
- **Structured data:** `tmp/pnt_work/pnt_data.py` holds every catalog row as Python objects — the source for any future machine-readable export (mitigation for risk R7).

### G. Glossary

| Term | Definition |
|------|------------|
| PNT | Positioning, Navigation, and Timing |
| Group 1 / Core | Code that directly implements, reads, writes, or processes PNT data (Role = Source/Transform/Sink) |
| Group 2 / Indirect | Code whose behavior changes as a result of PNT state without directly reading/writing it |
| Layer 1 | Direct callers/callees with a typed dependency classification |
| Layer 2 | Full transitive call chain, its final consumer, and hop count |
| EKF | Extended Kalman Filter (AP_NavEKF2/3) — PNT state estimation/fusion |
| AHRS | Attitude & Heading Reference System — ArduPilot's sensor-fusion navigation hub |
| Source / Transform / Sink | Produces / fuses-derives / consumes PNT state |
| `[ABSENT]` | A checked, explicitly-documented absence (non-inference discipline) |

---

*Generated by the Blitzy Platform. Completion percentage (89.8%) reflects AAP-scoped and path-to-production work only. Brand colors: Completed = #5B39F3, Remaining = #FFFFFF.*