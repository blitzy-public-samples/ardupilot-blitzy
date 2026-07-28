"""ReportLab renderer + ``verify_*`` integrity harness for the ArduPilot PNT
Reference Audit PDF.

This module is the **rendering + verification layer** of the recreated PNT
(Positioning, Navigation, Timing) Reference Audit generator.  It consumes the
structured catalog objects declared in :mod:`pnt_data` and provides two
independent capabilities:

1.  **Integrity harness** — six ``verify_*`` functions plus the
    :func:`run_all_verifications` aggregator.  Together they assert the
    structural, vocabulary, provenance, numbering, cross-reference and
    chain-depth invariants of the catalog (~838 discrete checks).  The
    downstream ``generate.py`` driver runs these *before* emitting the PDF and
    refuses to render if any check fails — the audit is therefore
    harness-gated.

2.  **Renderer** — :func:`render_pdf` builds a landscape A4 PDF that reproduces
    every feature of the prior audit (all 12 tables in the exact order
    ``1, 1a, 2, 2a, 3, 3a, 4, 4a, 5, 5a, 6, 6a`` under both group headers, the
    Legend, the Footprint summary, the Coverage & Explicit-Absence Register and
    the full audit-discipline flag taxonomy) **and** additively appends the new
    ``New Service Location`` column to the Core Navigation table (Table 2),
    mapping each catalogued L1-navigation instance onto the reusable
    ``AfsimL1Behavior`` service surface.

Design contract
---------------
* The module has **no import-time side effects**: fonts are registered and the
  deterministic-output flag is set only when :func:`render_pdf` runs.
* :func:`render_pdf` performs **no verification** of its own — validation is the
  aggregator's responsibility — but it *may* assume the data is valid.
* Output is **deterministic**: fixed column widths, no timestamps, and
  ReportLab's ``invariant`` mode are used so re-rendering is byte-stable.
* The only third-party runtime dependency is ReportLab (4.5.1); Unicode glyphs
  (``\u2192 \u2713 \u2194 \u2265 \u2264 \u2248 \u2190 \u2026 \u2014``) render via the system-provided DejaVu TrueType
  fonts registered as a fallback face.

Target toolchain: Python 3.13 + ReportLab 4.5.1 (AAP 0.5.1).  Build-time only —
this file is never linked into firmware.
"""

from __future__ import annotations

import os
import re

import reportlab.rl_config
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    LongTable,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

from pnt_data import (
    AUDITED_HEAD,
    COVERAGE_ABSENT_COUNT,
    COVERAGE_ENTRY_COUNT,
    COVERAGE_REGISTER,
    COVERAGE_REGISTER_COLUMNS,
    COVERAGE_REGISTER_TITLE,
    COVERAGE_SUBREGISTER_COUNT,
    CORE_MAIN_ROW_COUNT,
    DEPENDENCY_TYPE_VOCABULARY,
    DOCUMENT_SUBTITLE,
    DOCUMENT_TITLE,
    EVIDENCE_ROW_COUNT,
    EXECUTIVE_SUMMARY_INTRO,
    EXECUTIVE_SUMMARY_KEY_FINDINGS,
    EXECUTIVE_SUMMARY_KEY_FINDINGS_COLUMNS,
    EXECUTIVE_SUMMARY_PROVENANCE,
    EXECUTIVE_SUMMARY_ROADMAP,
    EXECUTIVE_SUMMARY_SECTIONS,
    EXECUTIVE_SUMMARY_TITLE,
    FLAG_COUNTS,
    FLAG_VOCABULARY,
    FOOTPRINT_TEXT,
    G1_MAIN_COLUMNS,
    G2_MAIN_COLUMNS,
    GROUP1_TABLES,
    GROUP2_TABLES,
    INDIRECT_MAIN_ROW_COUNT,
    LAYER1_COLUMNS,
    LAYER1_TABLES,
    LAYER2_COLUMNS,
    LAYER2_TABLES,
    LAYER_COUNT,
    LEGEND,
    MAIN_ROW_COUNT,
    MAIN_TABLES,
    NEW_SERVICE_LOCATION_COLUMN,
    NEW_SERVICE_LOCATION_MAP,
    NEW_SERVICE_LOCATION_ROWS,
    ROLE_VOCABULARY,
    SCOPE_TEXT,
    all_layer1_rows,
    all_layer2_rows,
    all_main_rows,
)

# ---------------------------------------------------------------------------
# Public API surface.
# ---------------------------------------------------------------------------
__all__ = [
    "verify_group1",
    "verify_group2",
    "verify_layer1",
    "verify_layer2",
    "verify_ref_coverage",
    "verify_new_service_mapping",
    "run_all_verifications",
    "render_pdf",
    "register_fonts",
    "BASE_FONT",
    "BASE_FONT_BOLD",
    "MONO_FONT",
    "MONO_FONT_BOLD",
]

# ---------------------------------------------------------------------------
# Font configuration.
#
# The ReportLab base-14 fonts (Helvetica/Courier) only cover WinAnsi, so the
# audit's arrows/checks/relational glyphs render as blank boxes.  DejaVu ships
# with the OS (AAP 0.5.1) and provides full coverage, so we register it as the
# document's primary face and its monospace sibling for code snippets.
# ---------------------------------------------------------------------------
BASE_FONT = "DejaVuSans"
BASE_FONT_BOLD = "DejaVuSans-Bold"
MONO_FONT = "DejaVuSansMono"
MONO_FONT_BOLD = "DejaVuSansMono-Bold"

# Ordered list of directories in which DejaVu TrueType files are commonly
# installed on Linux hosts.  The first match wins; a recursive fallback search
# under a small set of font roots covers less-common layouts.
_DEJAVU_DIR_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu",
    "/usr/share/fonts/dejavu",
    "/usr/share/fonts/TTF",
    "/usr/local/share/fonts/dejavu",
    "/usr/local/share/fonts",
    "/Library/Fonts",
    os.path.expanduser("~/.fonts"),
)

_FONT_FILES = {
    BASE_FONT: "DejaVuSans.ttf",
    BASE_FONT_BOLD: "DejaVuSans-Bold.ttf",
    MONO_FONT: "DejaVuSansMono.ttf",
    MONO_FONT_BOLD: "DejaVuSansMono-Bold.ttf",
}

# Module-level guard so repeated ``render_pdf`` calls register the fonts once.
_FONTS_REGISTERED = False


def _locate_font_file(file_name):
    """Return an absolute path to *file_name* by scanning known font roots.

    The common ``/usr/share/fonts/truetype/dejavu`` location is probed first;
    if the file is not found there a bounded recursive walk of a handful of
    font roots is performed.  Returns ``None`` when the face cannot be located
    so the caller can fail with an actionable message.
    """
    for directory in _DEJAVU_DIR_CANDIDATES:
        candidate = os.path.join(directory, file_name)
        if os.path.isfile(candidate):
            return candidate
    # Bounded fallback search: DejaVu is small and shallow in these roots.
    search_roots = ("/usr/share/fonts", "/usr/local/share/fonts",
                    os.path.expanduser("~/.fonts"))
    for root in search_roots:
        if not os.path.isdir(root):
            continue
        for dirpath, _dirnames, filenames in os.walk(root):
            if file_name in filenames:
                return os.path.join(dirpath, file_name)
    return None


def register_fonts():
    """Register the DejaVu sans + monospace faces as the Unicode fallback.

    Idempotent: the first invocation registers all four faces (regular/bold ×
    sans/mono) plus a ReportLab font *family* mapping so ``<b>`` markup in
    paragraphs resolves to the bold TrueType face.  Subsequent invocations are
    no-ops.  Raises :class:`RuntimeError` if a required DejaVu file is missing
    so PDF generation fails loudly rather than emitting blank glyphs.
    """
    global _FONTS_REGISTERED
    if _FONTS_REGISTERED:
        return
    for logical_name, file_name in _FONT_FILES.items():
        # Avoid double-registration if a caller pre-registered the face.
        if logical_name in pdfmetrics.getRegisteredFontNames():
            continue
        path = _locate_font_file(file_name)
        if path is None:
            raise RuntimeError(
                "Required DejaVu font %r not found; install the system "
                "'fonts-dejavu' package (searched %s)."
                % (file_name, ", ".join(_DEJAVU_DIR_CANDIDATES))
            )
        pdfmetrics.registerFont(TTFont(logical_name, path))
    # Map bold/italic variants so inline <b> markup uses the TrueType bold.
    pdfmetrics.registerFontFamily(
        BASE_FONT,
        normal=BASE_FONT,
        bold=BASE_FONT_BOLD,
        italic=BASE_FONT,
        boldItalic=BASE_FONT_BOLD,
    )
    pdfmetrics.registerFontFamily(
        MONO_FONT,
        normal=MONO_FONT,
        bold=MONO_FONT_BOLD,
        italic=MONO_FONT,
        boldItalic=MONO_FONT_BOLD,
    )
    _FONTS_REGISTERED = True


# ===========================================================================
# Integrity harness — the six ``verify_*`` functions + aggregator.
#
# Convention (consistent across all six): each function *returns* a list of
# human-readable error strings.  An empty list means every assertion for that
# aspect held.  ``generate.py`` collects and concatenates the results via
# :func:`run_all_verifications`; a non-empty aggregate gates (blocks) PDF
# emission.  Functions never raise for a *data* violation — they only raise for
# genuinely exceptional conditions (e.g. an unreadable repository root passed
# for citation checks is reported as an error string, not an exception).
#
# Collectively the harness performs ~838 discrete checks that reconcile with
# the committed ``pnt_data.py`` (94 main rows / 282 evidence rows):
#   verify_group1      role + snippet + citation over 63 Core rows      (189)
#   verify_group2      3 discriminators + snippet + citation over 31    (155)
#   verify_layer1      dependency-type over 94 + 5 AP:: singleton         (99)
#   verify_layer2      positive int + hop-count over 94                  (188)
#   verify_ref_coverage bidirectional Ref#<->#, monotonic numbering      (206)
# ===========================================================================

# Line references in the audit are cited as e.g. "1510-1518", "79-88, 92" or a
# single line "607".  This matches one or more integers, optionally in dashed
# ranges, separated by commas — used only to sanity-check the citation format.
_LINE_SPEC_RE = re.compile(r"^\s*\d+(?:\s*-\s*\d+)?(?:\s*,\s*\d+(?:\s*-\s*\d+)?)*\s*$")


def _snippet_line_count(snippet):
    """Return the number of physical lines in a code snippet block."""
    return len(snippet.split("\n"))


def _citation_readable(repo_root, file_path):
    """Confirm a cited source path resolves to a readable file under repo_root.

    Returns ``None`` on success or an error string describing the failure.  The
    check is deliberately lenient — it confirms the cited file *exists and is
    readable* (the provenance target is present in the tree), not that specific
    line numbers or verbatim text still match.  This mirrors the prior audit's
    "opened against the live ArduPilot tree and confirmed readable" gate while
    remaining robust to benign line drift in the surrounding source.
    """
    if not file_path:
        return "empty file path"
    # Reject absolute/parent-escaping paths: citations are repo-relative.
    if os.path.isabs(file_path) or ".." in file_path.split("/"):
        return "non repo-relative citation path %r" % (file_path,)
    resolved = os.path.join(repo_root, file_path)
    if not os.path.isfile(resolved):
        return "cited source not found: %s" % (file_path,)
    if not os.access(resolved, os.R_OK):
        return "cited source not readable: %s" % (file_path,)
    return None


def verify_group1(repo_root=None):
    """Validate every Group-1 (Core) main row.

    For each of the 63 Core rows this asserts:

    * ``role`` is one of :data:`ROLE_VOCABULARY` (Source/Transform/Sink);
    * ``code_snippet`` is a non-empty block of 5-10 physical lines (AAP 0.7.1);
    * when *repo_root* is supplied, the cited ``file_path`` resolves to a
      readable file (verbatim-provenance readability gate).
    """
    errors = []
    seen = 0
    for table in GROUP1_TABLES:
        for row in table["rows"]:
            seen += 1
            tag = "Table %s #%s (%s)" % (table["id"], row["num"], row["file_path"])
            role = row.get("role")
            if role not in ROLE_VOCABULARY:
                errors.append(
                    "[group1] %s: role %r not in %r"
                    % (tag, role, ROLE_VOCABULARY)
                )
            snippet = row.get("code_snippet", "")
            if not snippet.strip():
                errors.append("[group1] %s: empty code snippet" % (tag,))
            else:
                n = _snippet_line_count(snippet)
                if not (5 <= n <= 10):
                    errors.append(
                        "[group1] %s: snippet has %d lines (want 5-10)"
                        % (tag, n)
                    )
            if repo_root is not None:
                citation_error = _citation_readable(repo_root, row["file_path"])
                if citation_error:
                    errors.append("[group1] %s: %s" % (tag, citation_error))
    if seen != CORE_MAIN_ROW_COUNT:
        errors.append(
            "[group1] expected %d Core rows, iterated %d"
            % (CORE_MAIN_ROW_COUNT, seen)
        )
    return errors


def verify_group2(repo_root=None):
    """Validate every Group-2 (Indirect/Relational) main row.

    For each of the 31 Indirect rows this asserts the three discriminator
    columns — ``trigger_condition``, ``pnt_state_observed`` and
    ``behavior_change`` — are all present and non-empty (an Indirect reference
    is only meaningful when it records what invokes it, what PNT state it
    observes and how behavior changes), that the snippet is a 5-10 line block,
    and (when *repo_root* is supplied) that the cited source is readable.
    """
    errors = []
    seen = 0
    required_fields = (
        "trigger_condition",
        "pnt_state_observed",
        "behavior_change",
    )
    for table in GROUP2_TABLES:
        for row in table["rows"]:
            seen += 1
            tag = "Table %s #%s (%s)" % (table["id"], row["num"], row["file_path"])
            for field in required_fields:
                value = row.get(field, "")
                if not str(value).strip():
                    errors.append(
                        "[group2] %s: empty %s" % (tag, field)
                    )
            snippet = row.get("code_snippet", "")
            if not snippet.strip():
                errors.append("[group2] %s: empty code snippet" % (tag,))
            else:
                n = _snippet_line_count(snippet)
                if not (5 <= n <= 10):
                    errors.append(
                        "[group2] %s: snippet has %d lines (want 5-10)"
                        % (tag, n)
                    )
            if repo_root is not None:
                citation_error = _citation_readable(repo_root, row["file_path"])
                if citation_error:
                    errors.append("[group2] %s: %s" % (tag, citation_error))
    if seen != INDIRECT_MAIN_ROW_COUNT:
        errors.append(
            "[group2] expected %d Indirect rows, iterated %d"
            % (INDIRECT_MAIN_ROW_COUNT, seen)
        )
    return errors


def verify_layer1(repo_root=None):
    """Validate every Layer-1 (direct-edge) dependency row.

    For each of the 94 Layer-1 rows this asserts ``dependency_type`` is one of
    :data:`DEPENDENCY_TYPE_VOCABULARY`.  Rows classified
    ``Shared global/singleton`` additionally must contain an ``AP::`` accessor
    in their cited snippet — the audit's singleton-consistency invariant that a
    shared-singleton edge is evidenced by an ``AP::`` namespace access.

    *repo_root* is accepted for signature symmetry with the other verifiers;
    Layer-1 rows carry inline snippets rather than a resolvable file path, so no
    filesystem citation read is performed here.
    """
    errors = []
    seen = 0
    for table in LAYER1_TABLES:
        for row in table["rows"]:
            seen += 1
            tag = "Table %sa L1 ref#%s (%s)" % (
                table["main_table_id"], row["ref"], row["component_function"],
            )
            dep_type = row.get("dependency_type")
            if dep_type not in DEPENDENCY_TYPE_VOCABULARY:
                errors.append(
                    "[layer1] %s: dependency type %r not in %r"
                    % (tag, dep_type, DEPENDENCY_TYPE_VOCABULARY)
                )
            if dep_type == "Shared global/singleton":
                if "AP::" not in row.get("code_snippet", ""):
                    errors.append(
                        "[layer1] %s: 'Shared global/singleton' row lacks an "
                        "AP:: accessor in its snippet" % (tag,)
                    )
    if seen != MAIN_ROW_COUNT:
        errors.append(
            "[layer1] expected %d Layer-1 rows, iterated %d"
            % (MAIN_ROW_COUNT, seen)
        )
    return errors


def verify_layer2(repo_root=None):
    """Validate every Layer-2 (transitive-chain) dependency row.

    For each of the 94 Layer-2 rows this asserts ``chain_depth`` is a positive
    integer and equals the number of ``\u2192`` hops in ``full_dependency_chain`` —
    i.e. the declared depth is consistent with the rendered arrow chain.

    *repo_root* is accepted for signature symmetry; no filesystem read is
    required for chain-depth integrity.
    """
    errors = []
    seen = 0
    for table in LAYER2_TABLES:
        for row in table["rows"]:
            seen += 1
            tag = "Table %sa L2 ref#%s (%s)" % (
                table["main_table_id"], row["ref"], row["pnt_origin"],
            )
            depth = row.get("chain_depth")
            if not isinstance(depth, int) or isinstance(depth, bool) or depth < 1:
                errors.append(
                    "[layer2] %s: chain_depth %r is not a positive integer"
                    % (tag, depth)
                )
                continue
            hops = row.get("full_dependency_chain", "").count("\u2192")
            if depth != hops:
                errors.append(
                    "[layer2] %s: chain_depth %d != %d arrow hops"
                    % (tag, depth, hops)
                )
    if seen != MAIN_ROW_COUNT:
        errors.append(
            "[layer2] expected %d Layer-2 rows, iterated %d"
            % (MAIN_ROW_COUNT, seen)
        )
    return errors


def verify_ref_coverage(repo_root=None):
    """Validate ``#`` <-> ``Ref #`` cross-reference integrity and numbering.

    For each of the six (main, Layer-1, Layer-2) table triples this asserts:

    * the main rows are numbered monotonically ``1..row_count`` (no gaps or
      duplicates), matching the declared ``row_count``;
    * every Layer-1 ``ref`` resolves to a real main-row ``#`` and the Layer-1
      reference set is exactly the main-row number set (bidirectional
      coverage);
    * the same bidirectional coverage holds for Layer-2.

    This is the traceability gate ensuring each catalogued reference owns
    exactly one Layer-1 and one Layer-2 evidence row.
    """
    errors = []
    triples = list(zip(MAIN_TABLES, LAYER1_TABLES, LAYER2_TABLES))
    if len(triples) != len(MAIN_TABLES):
        errors.append(
            "[ref] table triple count mismatch: main=%d l1=%d l2=%d"
            % (len(MAIN_TABLES), len(LAYER1_TABLES), len(LAYER2_TABLES))
        )
    for main_t, l1_t, l2_t in triples:
        tid = main_t["id"]
        # Alignment: the Xa sub-tables must reference this main table.
        if l1_t.get("main_table_id") != tid or l2_t.get("main_table_id") != tid:
            errors.append(
                "[ref] Table %s: sub-table main_table_id misaligned "
                "(l1=%r l2=%r)"
                % (tid, l1_t.get("main_table_id"), l2_t.get("main_table_id"))
            )
        nums = sorted(r["num"] for r in main_t["rows"])
        expected = list(range(1, main_t["row_count"] + 1))
        if nums != expected:
            errors.append(
                "[ref] Table %s: main numbering not monotonic 1..%d (got %r)"
                % (tid, main_t["row_count"], nums)
            )
        num_set = set(nums)
        l1_refs = sorted(r["ref"] for r in l1_t["rows"])
        if set(l1_refs) != num_set:
            errors.append(
                "[ref] Table %sa: Layer-1 refs %r do not cover main #s %r"
                % (tid, l1_refs, nums)
            )
        l2_refs = sorted(r["ref"] for r in l2_t["rows"])
        if set(l2_refs) != num_set:
            errors.append(
                "[ref] Table %sa: Layer-2 refs %r do not cover main #s %r"
                % (tid, l2_refs, nums)
            )
        # Per-ref resolution (each L1/L2 ref maps to a real main #).
        for ref in l1_refs:
            if ref not in num_set:
                errors.append(
                    "[ref] Table %sa: Layer-1 ref#%s has no parent main #"
                    % (tid, ref)
                )
        for ref in l2_refs:
            if ref not in num_set:
                errors.append(
                    "[ref] Table %sa: Layer-2 ref#%s has no parent main #"
                    % (tid, ref)
                )
    # Global totals reconcile with the declared aggregates.
    if len(all_main_rows()) != MAIN_ROW_COUNT:
        errors.append(
            "[ref] total main rows %d != %d"
            % (len(all_main_rows()), MAIN_ROW_COUNT)
        )
    if len(all_layer1_rows()) != MAIN_ROW_COUNT:
        errors.append(
            "[ref] total Layer-1 rows %d != %d"
            % (len(all_layer1_rows()), MAIN_ROW_COUNT)
        )
    if len(all_layer2_rows()) != MAIN_ROW_COUNT:
        errors.append(
            "[ref] total Layer-2 rows %d != %d"
            % (len(all_layer2_rows()), MAIN_ROW_COUNT)
        )
    return errors


def verify_new_service_mapping(repo_root=None):
    """Validate the CP2 current -> new service mapping deliverable.

    This is the safety gate for the checkpoint's central artifact: the 12-row
    PNT-instance mapping (AAP 0.6.1) and its rendered ``New Service Location``
    column.  Without it the harness would let the driver emit a PDF even if the
    entire mapping were deleted or silently corrupted (the gap this closes).
    It asserts:

    * exactly 12 mapping rows, each carrying all five required, non-empty
      fields (``pnt_pillar``, ``behavior``, ``current_locations``,
      ``current_accessor``, ``new_service_location``);
    * exactly 12 :data:`NEW_SERVICE_LOCATION_MAP` entries, each a non-empty
      value keyed by a non-empty provenance string;
    * ROWS <-> MAP parity: every row's ``current_locations`` is a map key whose
      value equals that row's ``new_service_location``, and every map key
      corresponds to exactly one row (bijective);
    * rendered-column coverage: every main-table row whose data
      ``new_service_location`` is a real mapping (not the em-dash placeholder)
      renders a non-em-dash value through :func:`_service_location_for_row`, so
      a populated data mapping can never silently render blank;
    * default-off documentation (Rule R5): the Timing rows describe the
      ``set_update_dt`` seam as additive / default-off, so the audit text can
      never regress to omitting it.

    *repo_root* is accepted for signature symmetry with the other verifiers; it
    is unused (this check is purely structural over the in-module catalog).
    """
    del repo_root  # structural check; no source-tree access required
    errors = []
    emdash = "\u2014"
    required = ("pnt_pillar", "behavior", "current_locations",
                "current_accessor", "new_service_location")

    rows = NEW_SERVICE_LOCATION_ROWS
    if len(rows) != 12:
        errors.append(
            "[mapping] expected 12 mapping rows, found %d" % (len(rows),))
    for i, row in enumerate(rows, start=1):
        for field in required:
            val = row.get(field)
            if not (isinstance(val, str) and val.strip()):
                errors.append(
                    "[mapping] row %d: field %r missing or empty" % (i, field))

    if len(NEW_SERVICE_LOCATION_MAP) != 12:
        errors.append(
            "[mapping] expected 12 NEW_SERVICE_LOCATION_MAP entries, found %d"
            % (len(NEW_SERVICE_LOCATION_MAP),))
    for key, val in NEW_SERVICE_LOCATION_MAP.items():
        if not (isinstance(key, str) and key.strip()):
            errors.append("[mapping] NEW_SERVICE_LOCATION_MAP has an empty key")
        if not (isinstance(val, str) and val.strip()):
            errors.append(
                "[mapping] map key %r has a missing or empty value" % (key,))

    # ROWS <-> MAP bijective parity.
    row_keys = set()
    for i, row in enumerate(rows, start=1):
        key = row.get("current_locations", "")
        row_keys.add(key)
        if key not in NEW_SERVICE_LOCATION_MAP:
            errors.append(
                "[mapping] row %d current_locations %r absent from "
                "NEW_SERVICE_LOCATION_MAP" % (i, key))
        elif NEW_SERVICE_LOCATION_MAP[key] != row.get("new_service_location"):
            errors.append(
                "[mapping] row %d: map value for %r does not match the row's "
                "new_service_location" % (i, key))
    for key in NEW_SERVICE_LOCATION_MAP:
        if key not in row_keys:
            errors.append(
                "[mapping] map key %r has no corresponding mapping row"
                % (key,))

    # Rendered-column coverage: a populated data mapping must never render as
    # the em-dash placeholder in the main-table New Service Location column.
    nsl_tables = [t for t in MAIN_TABLES if t.get("has_new_service_location")]
    if not nsl_tables:
        errors.append(
            "[mapping] no main table carries the New Service Location column")
    for table in nsl_tables:
        for row in table["rows"]:
            data_val = row.get("new_service_location")
            if (isinstance(data_val, str) and data_val.strip()
                    and data_val.strip() != emdash):
                rendered = _service_location_for_row(row)
                if not (isinstance(rendered, str) and rendered.strip()
                        and rendered.strip() != emdash):
                    errors.append(
                        "[mapping] Table %s #%s: data maps to %r but the "
                        "rendered column value is %r (em-dash/blank)"
                        % (table.get("id"), row.get("num"),
                           data_val[:48], rendered))

    # Rule R5: the additive/default-off timing seam must stay documented.
    timing_text = " ".join(
        row.get("new_service_location", "") for row in rows
        if row.get("pnt_pillar") == "Timing")
    if "default-off" not in timing_text:
        errors.append(
            "[mapping] Timing rows must document the set_update_dt seam as "
            "additive/default-off (Rule R5); 'default-off' not found")

    return errors


def run_all_verifications(repo_root):
    """Run all six ``verify_*`` functions and return the concatenated errors.

    *repo_root* is the absolute path to the ArduPilot repository root; it is
    forwarded to the verifiers so citation/readability checks resolve cited
    source paths.  Returns a (possibly empty) ``list[str]``; an empty list
    means the catalog passed every integrity gate and PDF emission may proceed.
    """
    errors = []
    errors.extend(verify_group1(repo_root))
    errors.extend(verify_group2(repo_root))
    errors.extend(verify_layer1(repo_root))
    errors.extend(verify_layer2(repo_root))
    errors.extend(verify_ref_coverage(repo_root))
    errors.extend(verify_new_service_mapping(repo_root))
    return errors


# ===========================================================================
# Rendering — page geometry, palette, styles and reusable cell helpers.
# ===========================================================================

# Landscape A4 matches the committed deliverable exactly (841.89 x 595.276 pt).
PAGE_SIZE = landscape(A4)
_MARGIN = 8 * mm  # symmetric page margin
# Usable content width in points (fixed, so column widths are deterministic).
CONTENT_WIDTH = PAGE_SIZE[0] - 2 * _MARGIN

# Palette — kept as fixed hex literals so the rendered bytes are stable.
_HEADER_BG = colors.HexColor("#243b53")      # dark slate for header rows
_HEADER_FG = colors.white
_GROUP_BG = colors.HexColor("#102a43")        # deeper slate for group banners
_SECTION_BG = colors.HexColor("#334e68")      # section / table headings
_ROW_BG_A = colors.HexColor("#ffffff")        # alternating body shading
_ROW_BG_B = colors.HexColor("#eef2f7")
_GRID = colors.HexColor("#9fb3c8")            # visible cell dividers
_NSL_BG = colors.HexColor("#fff4e6")          # tint for the added NSL column


def _build_styles():
    """Return the ParagraphStyle set used across the document.

    Built lazily (after :func:`register_fonts`) so every style binds to the
    DejaVu faces and no ReportLab work happens at import time.
    """
    base = getSampleStyleSheet()["Normal"]
    styles = {}
    styles["title"] = ParagraphStyle(
        "PntTitle", parent=base, fontName=BASE_FONT_BOLD, fontSize=19,
        leading=23, spaceAfter=6, textColor=_GROUP_BG,
    )
    styles["subtitle"] = ParagraphStyle(
        "PntSubtitle", parent=base, fontName=BASE_FONT, fontSize=10,
        leading=13, spaceAfter=4, textColor=colors.HexColor("#486581"),
    )
    styles["meta"] = ParagraphStyle(
        "PntMeta", parent=base, fontName=BASE_FONT, fontSize=8, leading=11,
        textColor=colors.HexColor("#627d98"),
    )
    styles["group"] = ParagraphStyle(
        "PntGroup", parent=base, fontName=BASE_FONT_BOLD, fontSize=15,
        leading=19, textColor=_HEADER_FG,
    )
    styles["section"] = ParagraphStyle(
        "PntSection", parent=base, fontName=BASE_FONT_BOLD, fontSize=12,
        leading=15, spaceBefore=6, spaceAfter=4, textColor=_GROUP_BG,
    )
    styles["subsection"] = ParagraphStyle(
        "PntSubsection", parent=base, fontName=BASE_FONT_BOLD, fontSize=9.5,
        leading=12, spaceBefore=4, spaceAfter=2,
        textColor=colors.HexColor("#334e68"),
    )
    styles["body"] = ParagraphStyle(
        "PntBody", parent=base, fontName=BASE_FONT, fontSize=8.5, leading=12,
        alignment=TA_LEFT, spaceAfter=3,
    )
    styles["cell"] = ParagraphStyle(
        "PntCell", parent=base, fontName=BASE_FONT, fontSize=6.3, leading=7.9,
        alignment=TA_LEFT,
    )
    styles["cell_center"] = ParagraphStyle(
        "PntCellCenter", parent=styles["cell"], alignment=1,
    )
    styles["cell_header"] = ParagraphStyle(
        "PntCellHeader", parent=base, fontName=BASE_FONT_BOLD, fontSize=6.7,
        leading=8.2, textColor=_HEADER_FG, alignment=TA_LEFT,
    )
    styles["mono"] = ParagraphStyle(
        "PntMono", parent=base, fontName=MONO_FONT, fontSize=5.5, leading=6.7,
        alignment=TA_LEFT,
    )
    return styles


def _xml_escape(text):
    """Escape the five characters that are significant to ReportLab markup."""
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def _plain_markup(text):
    """Escape *text* and turn newlines into ``<br/>`` soft breaks."""
    if text is None:
        return "\u2014"  # em dash for an absent value
    return _xml_escape(text).replace("\n", "<br/>")


def _mono_markup(text):
    """Escape *text* for a monospace cell, preserving per-line indentation.

    Leading spaces are converted to non-breaking spaces so code indentation is
    retained, while interior spaces remain ordinary spaces so the block can
    still soft-wrap inside a fixed-width cell.  Newlines become ``<br/>``.
    """
    if text is None:
        return "\u2014"
    out_lines = []
    for line in str(text).split("\n"):
        stripped = line.lstrip(" ")
        indent = len(line) - len(stripped)
        out_lines.append("&nbsp;" * indent + _xml_escape(stripped))
    return "<br/>".join(out_lines)


def _cell(text, styles, mono=False, center=False):
    """Wrap a value in a Paragraph so it wraps inside its table cell."""
    if mono:
        return Paragraph(_mono_markup(text), styles["mono"])
    style = styles["cell_center"] if center else styles["cell"]
    return Paragraph(_plain_markup(text), style)


def _header_cells(columns, styles):
    """Build the bold, light-on-dark header row for a data table."""
    return [Paragraph(_plain_markup(c), styles["cell_header"]) for c in columns]


def _scaled_widths(weights):
    """Scale a list of relative column weights to fill ``CONTENT_WIDTH`` exactly.

    Using proportional weights (rather than absolute points) keeps every table
    flush to the same content width regardless of column count and remains
    fully deterministic.
    """
    total = float(sum(weights))
    return [CONTENT_WIDTH * (w / total) for w in weights]


# Relative column-weight schemas.  Each honours the exact column count of its
# schema: 7-col Core (8 with the appended New Service Location), 9-col Indirect,
# 6-col Layer-1, 6-col Layer-2, plus the 5-col mapping and 4-col register.
_G1_WEIGHTS = (18, 118, 42, 108, 46, 250, 202)
_G1_NSL_WEIGHTS = (16, 96, 36, 90, 40, 196, 150, 156)
_G2_WEIGHTS = (15, 84, 32, 82, 96, 96, 112, 150, 125)
_L1_WEIGHTS = (30, 128, 150, 150, 100, 238)
_L2_WEIGHTS = (30, 118, 300, 120, 44, 184)
_NSL_MAP_WEIGHTS = (86, 128, 158, 178, 246)
_COVERAGE_WEIGHTS = (196, 92, 300, 208)


def _data_table_style(ncols, header_span_cols=None):
    """Return a :class:`TableStyle` for a data table.

    Applies: a dark header band with light text, a fine visible grid on every
    cell (dividers), alternating body-row shading, top vertical alignment and
    tight padding.  ``header_span_cols`` (used by the NSL-annotated Table 2)
    tints the trailing column so the additive column reads as a distinct band.
    """
    commands = [
        # Default cell font — ensures any non-Paragraph content still uses the
        # embedded DejaVu face rather than falling back to base-14 Helvetica.
        ("FONTNAME", (0, 0), (-1, -1), BASE_FONT),
        ("FONTNAME", (0, 0), (-1, 0), BASE_FONT_BOLD),
        ("FONTSIZE", (0, 0), (-1, -1), 6.3),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 2.5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2.5),
        ("TOPPADDING", (0, 0), (-1, -1), 2.3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 2.3),
        # Header band.
        ("BACKGROUND", (0, 0), (-1, 0), _HEADER_BG),
        ("TEXTCOLOR", (0, 0), (-1, 0), _HEADER_FG),
        ("LINEBELOW", (0, 0), (-1, 0), 0.6, _GROUP_BG),
        # Visible dividers on every cell + a box around the table.
        ("GRID", (0, 0), (-1, -1), 0.25, _GRID),
        ("BOX", (0, 0), (-1, -1), 0.6, _HEADER_BG),
        # Alternating body-row shading (rows after the header).
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [_ROW_BG_A, _ROW_BG_B]),
    ]
    if header_span_cols is not None:
        # Tint the appended New Service Location column (last column).
        last = ncols - 1
        commands.append(("BACKGROUND", (last, 1), (last, -1), _NSL_BG))
    return TableStyle(commands)


# ===========================================================================
# New Service Location column — map catalogued L1-navigation provenance onto
# the reusable AfsimL1Behavior service surface.
# ===========================================================================

def _parse_line_set(spec):
    """Return the set of integer line numbers described by a line spec.

    Handles dashed ranges (``226-234``), comma lists (``L230, L369``) and any
    surrounding decoration (the ``L`` prefixes and ``/`` separators used in the
    audit's provenance keys are simply ignored — only the integers matter).
    """
    lines = set()
    for match in re.finditer(r"(\d+)\s*-\s*(\d+)|(\d+)", str(spec)):
        if match.group(1) is not None:
            low, high = int(match.group(1)), int(match.group(2))
            if low > high:
                low, high = high, low
            lines.update(range(low, high + 1))
        else:
            lines.add(int(match.group(3)))
    return lines


def _service_location_for_row(row):
    """Resolve a main-row's ``New Service Location`` value, or ``\u2014`` if none.

    Resolution is three-stage and deterministic:

    0.  **Explicit per-row mapping** — the data layer records an authoritative
        ``new_service_location`` on every row carrying this column (the em-dash
        placeholder ``\u2014`` where a row has no service mapping).  When
        present and non-empty it is returned verbatim, so a populated data
        mapping (for example the L1 output row ``nav_roll_cd`` ->
        ``get_roll_deg`` / ``get_lat_accel``) renders exactly as authored and
        can never silently collapse to a blank via the provenance fallback.
    1.  **Verbatim key** — try a direct lookup of ``<basename>:L<lines>`` in
        :data:`NEW_SERVICE_LOCATION_MAP` (for rows without an explicit field).
    2.  **Provenance overlap** — for rows in the wrapped controller
        (``AP_L1_Control.cpp`` / ``.h``) match against each map key whose file
        basename equals the row's and whose cited lines intersect the row's
        line range, joining the mapped service members.

    Rows with no mapping (the overwhelming majority of Table 2, which spans the
    whole navigation hub) resolve to the em-dash placeholder so the additive
    column is present without asserting a mapping that does not exist.
    """
    # Stage 0: an explicit, authoritative per-row mapping wins verbatim.  This
    # is the value the data layer intends for the column (including the em-dash
    # placeholder for rows with no service mapping); preferring it guarantees a
    # populated data mapping is never dropped by the provenance heuristics.
    explicit = row.get("new_service_location")
    if isinstance(explicit, str) and explicit.strip():
        return explicit

    file_path = row.get("file_path", "")
    lines = row.get("lines", "")
    basename = os.path.basename(file_path)

    # Stage 1: verbatim provenance key.
    direct_key = "%s:L%s" % (basename, lines)
    if direct_key in NEW_SERVICE_LOCATION_MAP:
        return NEW_SERVICE_LOCATION_MAP[direct_key]

    # Stage 2: provenance-overlap for the wrapped controller only.
    if basename not in ("AP_L1_Control.cpp", "AP_L1_Control.h"):
        return "\u2014"
    row_lines = _parse_line_set(lines)
    if not row_lines:
        return "\u2014"
    matches = []
    for key, value in NEW_SERVICE_LOCATION_MAP.items():
        key_file, _, key_lines = key.partition(":")
        if key_file != basename:
            continue
        if _parse_line_set(key_lines) & row_lines:
            if value not in matches:
                matches.append(value)
    if matches:
        return "; ".join(matches)
    return "\u2014"


# ===========================================================================
# Block renderers — each returns a list of platypus flowables.
# ===========================================================================

def _main_table_flowables(table, styles):
    """Render a Group-1 or Group-2 main table (with optional NSL column)."""
    flowables = []
    heading = "%s \u2014 %s" % (table["table_label"], table["title"])
    flowables.append(Paragraph(_plain_markup(heading), styles["section"]))

    has_nsl = table.get("has_new_service_location", False)
    if table["kind"] == "G1":
        columns = list(G1_MAIN_COLUMNS)
        weights = list(_G1_NSL_WEIGHTS if has_nsl else _G1_WEIGHTS)
        if has_nsl:
            columns.append(NEW_SERVICE_LOCATION_COLUMN)
        data = [_header_cells(columns, styles)]
        for row in table["rows"]:
            cells = [
                _cell(row["num"], styles, center=True),
                _cell(row["file_path"], styles),
                _cell(row["lines"], styles, center=True),
                _cell(row["function_class"], styles),
                _cell(row["role"], styles, center=True),
                _cell(row["code_snippet"], styles, mono=True),
                _cell(row["notes"], styles),
            ]
            if has_nsl:
                cells.append(_cell(_service_location_for_row(row), styles))
            data.append(cells)
    else:  # Group 2 (Indirect / Relational) — 9-column schema.
        columns = list(G2_MAIN_COLUMNS)
        weights = list(_G2_WEIGHTS)
        data = [_header_cells(columns, styles)]
        for row in table["rows"]:
            data.append([
                _cell(row["num"], styles, center=True),
                _cell(row["file_path"], styles),
                _cell(row["lines"], styles, center=True),
                _cell(row["function_class"], styles),
                _cell(row["trigger_condition"], styles),
                _cell(row["pnt_state_observed"], styles),
                _cell(row["behavior_change"], styles),
                _cell(row["code_snippet"], styles, mono=True),
                _cell(row["notes"], styles),
            ])

    ncols = len(columns)
    tbl = LongTable(data, colWidths=_scaled_widths(weights), repeatRows=1,
                    hAlign="LEFT")
    tbl.setStyle(_data_table_style(
        ncols, header_span_cols=ncols if has_nsl else None))
    flowables.append(tbl)
    flowables.append(Spacer(1, 6))
    return flowables


def _sub_tables_flowables(l1_table, l2_table, styles):
    """Render the paired ``Xa`` Layer-1 and Layer-2 dependency sub-tables."""
    flowables = []
    heading = "%s \u2014 Dependency Map (Layer 1 direct edges + Layer 2 " \
              "transitive chains)" % (l1_table["table_label"],)
    flowables.append(Paragraph(_plain_markup(heading), styles["section"]))

    # Layer 1 block.
    flowables.append(Paragraph(
        "%s: Layer 1 \u2014 direct callers / callees with typed dependency"
        % (l1_table["table_label"],), styles["subsection"]))
    l1_data = [_header_cells(LAYER1_COLUMNS, styles)]
    for row in l1_table["rows"]:
        l1_data.append([
            _cell(row["ref"], styles, center=True),
            _cell(row["component_function"], styles),
            _cell(row["directly_calls"], styles),
            _cell(row["directly_called_by"], styles),
            _cell(row["dependency_type"], styles),
            _cell(row["code_snippet"], styles, mono=True),
        ])
    l1_tbl = LongTable(l1_data, colWidths=_scaled_widths(_L1_WEIGHTS),
                       repeatRows=1, hAlign="LEFT")
    l1_tbl.setStyle(_data_table_style(len(LAYER1_COLUMNS)))
    flowables.append(l1_tbl)
    flowables.append(Spacer(1, 4))

    # Layer 2 block.
    flowables.append(Paragraph(
        "%s: Layer 2 \u2014 full transitive chain, final consumer & hop depth"
        % (l2_table["table_label"],), styles["subsection"]))
    l2_data = [_header_cells(LAYER2_COLUMNS, styles)]
    for row in l2_table["rows"]:
        l2_data.append([
            _cell(row["ref"], styles, center=True),
            _cell(row["pnt_origin"], styles),
            _cell(row["full_dependency_chain"], styles, mono=True),
            _cell(row["final_consumer"], styles),
            _cell(row["chain_depth"], styles, center=True),
            _cell(row["notes"], styles),
        ])
    l2_tbl = LongTable(l2_data, colWidths=_scaled_widths(_L2_WEIGHTS),
                       repeatRows=1, hAlign="LEFT")
    l2_tbl.setStyle(_data_table_style(len(LAYER2_COLUMNS)))
    flowables.append(l2_tbl)
    flowables.append(Spacer(1, 6))
    return flowables


def _nsl_mapping_flowables(styles):
    """Render the standalone PNT-instance current->new service mapping table.

    This is the AAP 0.6.1 instance audit reproduced as a dedicated section: for
    each catalogued L1-navigation touch-point it shows the PNT pillar, the
    behavior, its current location(s) and accessor, and the service member it is
    refactored into (the substantive current-vs-new mapping the audit adds).
    """
    flowables = [PageBreak()]
    flowables.append(Paragraph(
        "PNT Instance Audit \u2014 Current Location \u2192 New Service Location",
        styles["section"]))
    flowables.append(Paragraph(
        "Every Position / Navigation / Timing touch-point inside "
        "AP_L1_Control mapped onto the reusable AfsimL1Behavior service (the "
        "AHRS state shim, the injected-<b>dt</b> timing seam and the "
        "<b>extern \"C\"</b> command surface).", styles["body"]))
    flowables.append(Paragraph(
        "Line-number basis. The <b>Current Location(s)</b> line numbers below "
        "are cited as of the audited HEAD %s (the pre-refactor baseline), "
        "which preserves parity with the AAP 0.6.1 instance table; they are "
        "not post-seam current-source line numbers. The extraction's single "
        "additive, <b>default-off</b> set_update_dt timing seam was applied to "
        "AP_L1_Control.cpp / .h after that commit and shifts only those two "
        "files' line numbers (it relocates no behavior). Because the seam is "
        "<b>default-off</b>, when set_update_dt is never called the controller "
        "keeps its internal AP_HAL::micros() / millis() timing path unchanged, "
        "so existing vehicle callers are unaffected." % AUDITED_HEAD,
        styles["body"]))
    columns = ("PNT Pillar", "Behavior", "Current Location(s)",
               "Current Accessor", NEW_SERVICE_LOCATION_COLUMN)
    data = [_header_cells(columns, styles)]
    for row in NEW_SERVICE_LOCATION_ROWS:
        data.append([
            _cell(row["pnt_pillar"], styles),
            _cell(row["behavior"], styles),
            _cell(row["current_locations"], styles, mono=True),
            _cell(row["current_accessor"], styles, mono=True),
            _cell(row["new_service_location"], styles),
        ])
    tbl = LongTable(data, colWidths=_scaled_widths(_NSL_MAP_WEIGHTS),
                    repeatRows=1, hAlign="LEFT")
    style = _data_table_style(len(columns))
    # Tint the New Service Location column so the mapping target stands out.
    style.add("BACKGROUND", (len(columns) - 1, 1), (len(columns) - 1, -1),
              _NSL_BG)
    tbl.setStyle(style)
    flowables.append(tbl)
    flowables.append(Spacer(1, 6))
    return flowables


def _legend_flowables(styles):
    """Render the Legend explaining every classification vocabulary + flag."""
    flowables = [Paragraph("Legend", styles["section"])]
    data = []
    for label, text in LEGEND:
        data.append([
            Paragraph("<b>%s</b>" % _plain_markup(label), styles["cell"]),
            Paragraph(_plain_markup(text), styles["cell"]),
        ])
    tbl = Table(data, colWidths=_scaled_widths((150, 650)), hAlign="LEFT")
    tbl.setStyle(TableStyle([
        ("FONTNAME", (0, 0), (-1, -1), BASE_FONT),
        ("FONTSIZE", (0, 0), (-1, -1), 6),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 3),
        ("RIGHTPADDING", (0, 0), (-1, -1), 3),
        ("TOPPADDING", (0, 0), (-1, -1), 2.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
        ("GRID", (0, 0), (-1, -1), 0.25, _GRID),
        ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#d9e2ec")),
        ("ROWBACKGROUNDS", (1, 0), (-1, -1), [_ROW_BG_A, _ROW_BG_B]),
    ]))
    flowables.append(tbl)
    flowables.append(Spacer(1, 6))
    return flowables


def _footprint_flowables(styles):
    """Render the surveyed-footprint summary paragraph."""
    flowables = [Paragraph("Footprint Summary", styles["section"])]
    flowables.append(Paragraph(_plain_markup(FOOTPRINT_TEXT), styles["body"]))
    flowables.append(Spacer(1, 6))
    return flowables


def _flags_summary_flowables(styles):
    """Render the audit-discipline flag taxonomy and occurrence counts."""
    flowables = [Paragraph("Audit-Discipline Flags", styles["section"])]
    flowables.append(Paragraph(
        "Extraction-risk and non-inference discipline markers applied inline "
        "in the Notes columns throughout the catalog.  Occurrence counts "
        "reconcile with the verified corpus totals.", styles["body"]))
    meanings = {
        "[CIRCULAR]": "Circular dependency (e.g. AHRS \u2194 EKF).",
        "[SHARED-STRUCT]": "PNT fields packed with non-PNT fields "
                           "(extraction risk).",
        "[DUPLICATED]": "Same PNT value read/derived in multiple places.",
        "[MULTI-CATEGORY]": "Touch-point spans more than one PNT pillar.",
        "[ABSENT]": "A checked, explicitly-documented absence "
                    "(non-inference discipline).",
    }
    data = [_header_cells(("Flag", "Occurrences", "Meaning"), styles)]
    for flag in FLAG_VOCABULARY:
        data.append([
            _cell(flag, styles, mono=True),
            _cell(FLAG_COUNTS.get(flag, 0), styles, center=True),
            _cell(meanings.get(flag, ""), styles),
        ])
    tbl = Table(data, colWidths=_scaled_widths((120, 70, 610)), hAlign="LEFT")
    tbl.setStyle(_data_table_style(3))
    flowables.append(tbl)
    flowables.append(Paragraph(
        "Explicitly-documented absences catalogued in the Coverage register: "
        "%d." % COVERAGE_ABSENT_COUNT, styles["meta"]))
    flowables.append(Spacer(1, 6))
    return flowables


def _coverage_register_flowables(styles):
    """Render the Coverage & Explicit-Absence Register (10 sub-registers)."""
    flowables = [PageBreak(),
                 Paragraph(_plain_markup(COVERAGE_REGISTER_TITLE),
                           styles["section"])]
    flowables.append(Paragraph(
        "Directory-wide token sweeps and explicit absence notations "
        "establishing exhaustiveness across libraries/, the four vehicle "
        "directories and the .gitmodules boundary edges "
        "(%d sub-registers, %d entries, %d checked absences)."
        % (COVERAGE_SUBREGISTER_COUNT, COVERAGE_ENTRY_COUNT,
           COVERAGE_ABSENT_COUNT),
        styles["body"]))
    for sub in COVERAGE_REGISTER:
        flowables.append(Paragraph(_plain_markup(sub["title"]),
                                   styles["subsection"]))
        data = [_header_cells(COVERAGE_REGISTER_COLUMNS, styles)]
        for entry in sub["entries"]:
            data.append([
                _cell(entry["surface"], styles),
                _cell(entry["intersection"], styles),
                _cell(entry["detail"], styles),
                _cell(entry["evidence"], styles, mono=True),
            ])
        tbl = LongTable(data, colWidths=_scaled_widths(_COVERAGE_WEIGHTS),
                        repeatRows=1, hAlign="LEFT")
        tbl.setStyle(_data_table_style(len(COVERAGE_REGISTER_COLUMNS)))
        flowables.append(tbl)
        flowables.append(Spacer(1, 5))
    return flowables


def _front_matter_flowables(styles):
    """Render the title block, audit metrics line and scope statement."""
    flowables = [
        Paragraph(_plain_markup(DOCUMENT_TITLE), styles["title"]),
        Paragraph(_plain_markup(DOCUMENT_SUBTITLE), styles["subtitle"]),
    ]
    metrics = (
        "Audited HEAD %s  \u00b7  Core references: %d  \u00b7  "
        "Indirect references: %d  \u00b7  Total references: %d  \u00b7  "
        "Evidence rows: %d (main + Layer 1 + Layer 2)"
        % (AUDITED_HEAD, CORE_MAIN_ROW_COUNT, INDIRECT_MAIN_ROW_COUNT,
           MAIN_ROW_COUNT, EVIDENCE_ROW_COUNT)
    )
    flowables.append(Paragraph(_plain_markup(metrics), styles["meta"]))
    flowables.append(Paragraph(_plain_markup(
        "Line-number basis: every cited path:line range in this document is "
        "stated as of the audited HEAD %s (the pre-refactor baseline). The "
        "reusable-service extraction adds one additive, default-off "
        "set_update_dt timing seam to AP_L1_Control.cpp / .h after that "
        "commit; that seam relocates no behavior and only shifts those two "
        "files' line numbers, so the numbers here are intentionally not "
        "post-seam current-source lines." % AUDITED_HEAD), styles["meta"]))
    flowables.append(Spacer(1, 8))
    flowables.append(Paragraph(_plain_markup(SCOPE_TEXT), styles["body"]))
    flowables.append(Spacer(1, 6))
    return flowables


def _executive_summary_flowables(styles):
    """Render the Executive Summary that opens the document.

    This is the platform-standard technical-specification summary adapted to
    an audit deliverable: a short lead-in followed by the Project Overview,
    Core Problem Addressed, Key Stakeholders and Users, Key Findings at a
    Glance, Expected Impact and Value Proposition and Document Roadmap
    subsections, closed by the provenance note.

    Every heading, paragraph and table cell is authored in :mod:`pnt_data`.
    The only work done here is presentation plus resolving the summary's
    named figure placeholders from this module's imported catalog constants
    (``CORE_MAIN_ROW_COUNT``, ``INDIRECT_MAIN_ROW_COUNT``,
    ``MAIN_ROW_COUNT``, ``LAYER_COUNT``, ``EVIDENCE_ROW_COUNT``,
    ``FLAG_COUNTS``, the ``COVERAGE_*`` totals and the length of
    ``NEW_SERVICE_LOCATION_ROWS``) -- the same way the front matter, the
    flag taxonomy and the coverage register compose their own metric lines.
    No figure is written as a literal and nothing is read from the source
    tree, so the summary cannot drift from the catalog it summarises.

    The section closes with a page break so the pre-existing Legend and
    Footprint Summary start on a fresh page rather than being interleaved
    with the summary.
    """
    # Named figures, each read from a catalog constant.  The flag-occurrence
    # line iterates the FLAG_VOCABULARY tuple, so its order -- and therefore
    # the rendered bytes -- stay deterministic.
    figures = {
        "audited_head": AUDITED_HEAD,
        "core_refs": CORE_MAIN_ROW_COUNT,
        "indirect_refs": INDIRECT_MAIN_ROW_COUNT,
        "total_refs": MAIN_ROW_COUNT,
        "evidence_rows": EVIDENCE_ROW_COUNT,
        "layers": LAYER_COUNT,
        "tables": 2 * len(MAIN_TABLES),
        "table_groups": len(MAIN_TABLES),
        "flag_kinds": len(FLAG_VOCABULARY),
        "flag_counts": " \u00b7 ".join(
            "%s %d" % (flag, FLAG_COUNTS[flag]) for flag in FLAG_VOCABULARY
        ),
        "coverage_subregisters": COVERAGE_SUBREGISTER_COUNT,
        "coverage_entries": COVERAGE_ENTRY_COUNT,
        "coverage_absences": COVERAGE_ABSENT_COUNT,
        "mapping_rows": len(NEW_SERVICE_LOCATION_ROWS),
    }

    def _para(template, style_key):
        """Resolve *template*'s named figures and wrap it in a Paragraph."""
        return Paragraph(_plain_markup(template % figures), styles[style_key])

    flowables = [
        _para(EXECUTIVE_SUMMARY_TITLE, "section"),
        _para(EXECUTIVE_SUMMARY_INTRO, "body"),
    ]

    for section in EXECUTIVE_SUMMARY_SECTIONS:
        flowables.append(_para(section["heading"], "subsection"))
        for paragraph in section["body"]:
            flowables.append(_para(paragraph, "body"))
        if section["kind"] == "key_findings":
            # At-a-glance aggregates: relative column weights are scaled to
            # CONTENT_WIDTH so the table stays flush with every other table.
            data = [_header_cells(
                EXECUTIVE_SUMMARY_KEY_FINDINGS_COLUMNS, styles)]
            for row in EXECUTIVE_SUMMARY_KEY_FINDINGS:
                data.append([
                    _cell(row["dimension"] % figures, styles),
                    _cell(row["value"] % figures, styles, center=True),
                    _cell(row["meaning"] % figures, styles),
                    _cell(row["source"] % figures, styles),
                ])
            tbl = Table(data, colWidths=_scaled_widths((150, 50, 320, 280)),
                        hAlign="LEFT")
            tbl.setStyle(_data_table_style(
                len(EXECUTIVE_SUMMARY_KEY_FINDINGS_COLUMNS)))
            flowables.append(tbl)
        elif section["kind"] == "roadmap":
            # Reading order, naming each downstream section exactly as it is
            # titled in the document (nothing renamed or renumbered).
            for name, holds in EXECUTIVE_SUMMARY_ROADMAP:
                flowables.append(
                    _para("%s \u2014 %s" % (name, holds), "body"))

    flowables.append(_para(EXECUTIVE_SUMMARY_PROVENANCE, "meta"))
    flowables.append(Spacer(1, 6))
    flowables.append(PageBreak())
    return flowables


def _group_banner_flowables(text, styles):
    """Render a full-width group-header banner (a visible section divider)."""
    banner = Table(
        [[Paragraph(_plain_markup(text), styles["group"])]],
        colWidths=[CONTENT_WIDTH], hAlign="LEFT",
    )
    banner.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), _GROUP_BG),
        ("LEFTPADDING", (0, 0), (-1, -1), 8),
        ("RIGHTPADDING", (0, 0), (-1, -1), 8),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ("BOX", (0, 0), (-1, -1), 1, _GROUP_BG),
    ]))
    return [banner, Spacer(1, 6)]


def _make_page_decorator():
    """Return an ``onPage`` callback drawing a deterministic header/footer.

    The footer carries the audited commit, the document short-name and the
    page number — all deterministic values (no timestamps), preserving
    byte-stable output.
    """
    def _decorate(canvas, doc):
        canvas.saveState()
        canvas.setFont(BASE_FONT, 6.5)
        canvas.setFillColor(colors.HexColor("#627d98"))
        # Footer baseline just inside the bottom margin.
        y = _MARGIN * 0.55
        canvas.drawString(
            _MARGIN, y,
            "ArduPilot PNT Reference Audit  \u00b7  HEAD %s" % AUDITED_HEAD)
        canvas.drawCentredString(
            PAGE_SIZE[0] / 2.0, y, "Page %d" % canvas.getPageNumber())
        canvas.drawRightString(
            PAGE_SIZE[0] - _MARGIN, y,
            "Read-only static-analysis catalog")
        # Thin rule above the footer text.
        canvas.setStrokeColor(_GRID)
        canvas.setLineWidth(0.4)
        canvas.line(_MARGIN, y + 9, PAGE_SIZE[0] - _MARGIN, y + 9)
        canvas.restoreState()
    return _decorate


def render_pdf(out_path, repo_root=None):
    """Render the full PNT Reference Audit PDF to *out_path*.

    Assembles the platypus story in the mandated order and writes a landscape
    A4 document:

    * front matter (title, subtitle, audit metrics, scope);
    * the Executive Summary, at the front of the document;
    * Legend and Footprint summary;
    * ``GROUP 1 \u2014 CORE PNT REFERENCES`` then Tables 1/1a, 2/2a (Table 2
      carrying the additive New Service Location column) and 3/3a;
    * ``GROUP 2 \u2014 INDIRECT / RELATIONAL PNT REFERENCES`` then Tables 4/4a,
      5/5a and 6/6a;
    * the standalone Current \u2192 New Service Location mapping table;
    * the audit-discipline flag taxonomy;
    * the Coverage & Explicit-Absence Register.

    This function performs **no verification** — that is the aggregator's role
    (see :func:`run_all_verifications`) — but may assume the data is valid.
    *repo_root* is accepted for signature symmetry with the harness (the
    renderer does not read source files); it is otherwise unused.

    Output is deterministic: ReportLab's ``invariant`` mode is enabled and all
    geometry is fixed, so re-rendering the same data yields byte-identical
    bytes.
    """
    del repo_root  # renderer does not read the source tree

    # Deterministic output: fixed document ID + timestamp (no wall-clock leak).
    reportlab.rl_config.invariant = 1
    register_fonts()
    styles = _build_styles()

    story = []
    story.extend(_front_matter_flowables(styles))
    story.extend(_executive_summary_flowables(styles))
    story.extend(_legend_flowables(styles))
    story.extend(_footprint_flowables(styles))

    # GROUP 1 — Core references (Tables 1, 1a, 2, 2a, 3, 3a).
    story.append(PageBreak())
    story.extend(_group_banner_flowables(
        "GROUP 1 \u2014 CORE PNT REFERENCES", styles))
    for main_t, l1_t, l2_t in zip(MAIN_TABLES, LAYER1_TABLES, LAYER2_TABLES):
        if main_t["group"] != 1:
            continue
        story.extend(_main_table_flowables(main_t, styles))
        story.extend(_sub_tables_flowables(l1_t, l2_t, styles))

    # GROUP 2 — Indirect / relational references (Tables 4, 4a, 5, 5a, 6, 6a).
    story.append(PageBreak())
    story.extend(_group_banner_flowables(
        "GROUP 2 \u2014 INDIRECT / RELATIONAL PNT REFERENCES", styles))
    for main_t, l1_t, l2_t in zip(MAIN_TABLES, LAYER1_TABLES, LAYER2_TABLES):
        if main_t["group"] != 2:
            continue
        story.extend(_main_table_flowables(main_t, styles))
        story.extend(_sub_tables_flowables(l1_t, l2_t, styles))

    # Standalone current->new mapping, flag taxonomy and coverage register.
    story.extend(_nsl_mapping_flowables(styles))
    story.extend(_flags_summary_flowables(styles))
    story.extend(_coverage_register_flowables(styles))

    doc = SimpleDocTemplate(
        out_path,
        pagesize=PAGE_SIZE,
        leftMargin=_MARGIN,
        rightMargin=_MARGIN,
        topMargin=_MARGIN,
        bottomMargin=_MARGIN,
        title=DOCUMENT_TITLE,
        author="Blitzy PNT Reference Audit",
        subject=DOCUMENT_SUBTITLE,
        creator="pnt_render.py (ReportLab)",
        invariant=1,
    )
    decorate = _make_page_decorator()
    doc.build(story, onFirstPage=decorate, onLaterPages=decorate)
    return out_path
