# -*- coding: utf-8 -*-
# flake8: noqa: E501
"""ReportLab renderer + integrity-verification harness for the ArduPilot PNT Reference Audit PDF.

This module is the *rendering and verification* layer of the recreated PNT
Reference Audit generator (the data layer lives in the sibling module
``pnt_data.py``).  It provides two clearly separated responsibilities:

1.  **Verification harness** — nine ``verify_*`` functions plus a
    :func:`run_all_verifications` aggregator.  Together they assert the full
    991-check integrity contract described in the project handbook
    (``blitzy/documentation/Project Guide.md`` §3):

        * ``verify_group1``      — 189 checks (role vocabulary, snippet format,
                                    verbatim citation provenance for the 63 Core rows)
        * ``verify_group2``      —  62 checks (Group-2 field completeness and
                                    verbatim citation provenance for the 31 Indirect rows)
        * ``verify_layer1``      — 193 checks (dependency-type vocabulary,
                                    verbatim citation provenance, ``AP::`` singleton
                                    consistency for the 94 Layer-1 edges)
        * ``verify_layer2``      — 188 checks (chain-depth is a positive integer
                                    and equals the number of ``->`` hops for the
                                    94 Layer-2 transitive chains)
        * ``verify_ref_coverage`` — 206 checks (bidirectional ``#`` <-> ``Ref #``
                                    coverage and monotonic ``1..N`` numbering)
        * ``verify_global_counts`` — 47 checks (frozen Core=63 / Indirect=31 /
                                    Main=94 / Evidence=282 constants, the
                                    63+31=94 and 94x3=282 reconciliations, and
                                    the 3/3/6/6 table-structure invariants)
        * ``verify_new_service_location_map`` — 53 checks (the 12-row
                                    ``NEW_SERVICE_LOCATION_MAP`` matches the frozen
                                    AAP §0.6.1 pillar/behavior/accessor/new-service
                                    text verbatim, in order, incl. the Position
                                    row's ``(update_waypoint)``/``(update_loiter)``
                                    annotations, plus a per-row exact match of the
                                    frozen ``current_location`` source:line refs)
        * ``verify_line_reference_provenance`` — 32 checks (every cited
                                    ``AP_L1_Control`` source line in the mapping
                                    still resolves to its accessor token in the
                                    live source — fails closed on line drift)
        * ``verify_no_extraction_artifacts`` — 21 checks (PDF-extraction
                                    artifacts — filename bleed, ``( )`` splits,
                                    ``:: `` scope splits, and known split-identifier
                                    fragments — never reappear in display fields)

    Each function returns a ``list[str]`` of human-readable error messages; an
    empty list means every check for that function passed.  The functions do
    **not** raise on a failed data check — they collect and return the failure
    messages so that the caller (``generate.py``) can aggregate and report all
    problems at once.  (An unexpected structural problem — e.g. a missing key —
    still surfaces as an ordinary exception, which is the correct behaviour for
    a programming error rather than a data-integrity failure.)

2.  **PDF renderer** — :func:`render_pdf` builds the landscape-A4 platypus story
    (all twelve tables in the exact order ``1, 1a, 2, 2a, 3, 3a, 4, 4a, 5, 5a,
    6, 6a`` under both group headers, plus the Legend, Footprint summary,
    audit-discipline flag summary, and the Coverage & Explicit-Absence
    Register) and writes it to disk.  The renderer additively extends the prior
    audit by appending a **"New Service Location"** column to Table 2 (the Core
    Navigation table), pulling each mapped value from the data layer.  The
    renderer does **not** run verification — that gating is ``generate.py``'s
    responsibility — but it may assume the data is valid.

Determinism: :func:`render_pdf` sets ``reportlab.rl_config.invariant = 1`` so
that the emitted PDF carries a fixed document ``/ID`` and fixed
creation/modification timestamps.  Combined with fixed column widths and the
absence of any wall-clock content, re-rendering the same tree is byte-stable.

Unicode: the glyphs used throughout the audit (``-> checkmark <-> >= <= ~= <- ...``
rendered as U+2192, U+2713, U+2194, U+2265, U+2264, U+2248, U+2190, U+2026) are
not present in the ReportLab base-14 fonts, so the DejaVu Sans / DejaVu Sans
Mono TrueType faces are registered as the Unicode-capable glyph fallback.

The module has no import-time side effects beyond making ``pnt_data`` importable
(a ``sys.path`` shim so the sibling module resolves regardless of the current
working directory) and defining functions/constants.  Font registration and the
``rl_config.invariant`` toggle happen lazily inside :func:`render_pdf`.

Build-time only (Python 3.13 + ReportLab 4.5.1); it is never linked into or
executed by ArduPilot firmware.
"""

import os
import re
import sys
import glob

# ---------------------------------------------------------------------------
# Import-resolution shim
# ---------------------------------------------------------------------------
# ``pnt_data`` is the sibling data module in this same directory.  Prepend this
# file's directory to ``sys.path`` so ``import pnt_data`` resolves whether the
# caller runs the generator from the repository root, from this directory, or
# from anywhere else.  This is pure import resolution, not a rendering side
# effect.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from pnt_data import (  # noqa: E402  (import after sys.path shim, by design)
    # Table collections (rendered, in order)
    GROUP1_TABLES,
    GROUP2_TABLES,
    LAYER1_TABLES,
    LAYER2_TABLES,
    ALL_MAIN_TABLES,
    COVERAGE_REGISTER,
    # Additive extension: the L1 current -> new service-location mapping
    NEW_SERVICE_LOCATION_COLUMN,
    NEW_SERVICE_LOCATION_MAP,
    NEW_SERVICE_LOCATION_BY_PROVENANCE,
    L1_PROVENANCE_CHECKS,
    # Global reconciliation counts (CP2 invariants) + live count helpers
    CORE_ROW_COUNT,
    INDIRECT_ROW_COUNT,
    MAIN_ROW_COUNT,
    EVIDENCE_ROW_COUNT,
    main_row_count,
    evidence_row_count,
    # Controlled vocabularies (verification)
    ROLE_VOCABULARY,
    DEPENDENCY_TYPE_VOCABULARY,
    FLAG_TOKENS,
    all_flag_counts,
    # Front-matter text blocks
    DOCUMENT_TITLE,
    OUTPUT_PDF_FILENAME,
    SCOPE_TEXT,
    LEGEND_HEADING,
    LEGEND_BLOCKS,
    FOOTPRINT_TEXT,
    # Group / layer headings
    GROUP1_HEADER,
    GROUP1_SUBTITLE,
    GROUP2_HEADER,
    GROUP2_SUBTITLE,
)

# ---------------------------------------------------------------------------
# ReportLab imports
# ---------------------------------------------------------------------------
import reportlab  # noqa: E402  (needed for rl_config.invariant)
from reportlab.lib import colors  # noqa: E402
from reportlab.lib.pagesizes import A4, landscape  # noqa: E402
from reportlab.lib.styles import ParagraphStyle  # noqa: E402
from reportlab.platypus import (  # noqa: E402
    SimpleDocTemplate,
    LongTable,
    TableStyle,
    Paragraph,
    Spacer,
)
from reportlab.pdfbase import pdfmetrics  # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont  # noqa: E402

# ---------------------------------------------------------------------------
# Module constants
# ---------------------------------------------------------------------------

#: The single Unicode arrow used to express dependency hops in Layer-2 chains
#: (U+2192 RIGHTWARDS ARROW).  ``verify_layer2`` counts these to validate the
#: recorded ``chain_depth``.
ARROW = u'\u2192'

#: Registered font-family names used throughout the renderer.
FONT_SANS = 'DejaVuSans'
FONT_SANS_BOLD = 'DejaVuSans-Bold'
FONT_MONO = 'DejaVuSansMono'
FONT_MONO_BOLD = 'DejaVuSansMono-Bold'

#: TrueType file names for each face, in registration order.
_FONT_FILES = (
    (FONT_SANS, 'DejaVuSans.ttf'),
    (FONT_SANS_BOLD, 'DejaVuSans-Bold.ttf'),
    (FONT_MONO, 'DejaVuSansMono.ttf'),
    (FONT_MONO_BOLD, 'DejaVuSansMono-Bold.ttf'),
)

#: Preferred directories to probe for the DejaVu TrueType faces (the first is
#: the canonical Debian/Ubuntu location); a recursive fallback search under
#: ``/usr/share/fonts`` covers non-standard layouts.
_FONT_DIRS = (
    '/usr/share/fonts/truetype/dejavu',
    '/usr/share/fonts/dejavu',
    '/usr/share/fonts/TTF',
    '/usr/local/share/fonts',
)

#: Page geometry.  ``landscape(A4)`` == (841.8897.., 595.2755..) points.
PAGE_SIZE = landscape(A4)
MARGIN = 20.0
USABLE_WIDTH = PAGE_SIZE[0] - (2.0 * MARGIN)

# ---------------------------------------------------------------------------
# Deterministic palette (all fixed hex values -> byte-stable output)
# ---------------------------------------------------------------------------
_HEADER_BG = colors.HexColor('#2C3E50')      # dark slate — table header row
_HEADER_FG = colors.white
_GROUP_BG = colors.HexColor('#1A2530')       # group-title band
_SUBHEADER_BG = colors.HexColor('#D5DBDB')   # register sub-matrix bands
_ALT_BG = colors.HexColor('#F2F4F4')         # zebra shading (odd body rows)
_GRID = colors.HexColor('#9AA5AB')           # inner grid lines
_BOX = colors.HexColor('#33404A')            # outer table box

#: Guard so fonts are registered exactly once per process.
_FONTS_REGISTERED = False


def _find_font_file(filename):
    """Locate a font file by name, preferring the canonical DejaVu directory.

    Returns the absolute path to *filename* if found, else ``None``.  A recursive
    walk under ``/usr/share/fonts`` is used as a last-resort fallback so the
    renderer still works on distributions that install the fonts elsewhere.
    """
    for directory in _FONT_DIRS:
        candidate = os.path.join(directory, filename)
        if os.path.isfile(candidate):
            return candidate
    # Recursive fallback search (glob is deterministic once sorted).
    for base in ('/usr/share/fonts', '/usr/local/share/fonts'):
        if not os.path.isdir(base):
            continue
        matches = sorted(glob.glob(os.path.join(base, '**', filename), recursive=True))
        if matches:
            return matches[0]
    return None


def register_fonts():
    """Register the DejaVu Sans / Sans-Mono faces as the Unicode glyph fallback.

    Registration is idempotent (guarded by a module flag) and safe to call
    repeatedly.  After the four faces are registered, two font *families* are
    declared so that ``<b>`` markup inside paragraphs resolves to the matching
    bold face.

    Raises:
        RuntimeError: if the primary ``DejaVuSans.ttf`` face cannot be located,
            because without it the required Unicode glyphs cannot render and the
            deliverable would be visually wrong.
    """
    global _FONTS_REGISTERED
    if _FONTS_REGISTERED:
        return

    registered = set()
    for face_name, filename in _FONT_FILES:
        path = _find_font_file(filename)
        if path is None:
            continue
        pdfmetrics.registerFont(TTFont(face_name, path))
        registered.add(face_name)

    if FONT_SANS not in registered:
        raise RuntimeError(
            'Required font %r not found under %s; cannot render Unicode glyphs.'
            % ('DejaVuSans.ttf', ', '.join(_FONT_DIRS))
        )

    # Fall back gracefully if a bold/mono face is somehow missing: map the
    # missing face onto the regular sans face so rendering still succeeds.
    sans_bold = FONT_SANS_BOLD if FONT_SANS_BOLD in registered else FONT_SANS
    mono = FONT_MONO if FONT_MONO in registered else FONT_SANS
    mono_bold = FONT_MONO_BOLD if FONT_MONO_BOLD in registered else mono

    pdfmetrics.registerFontFamily(
        FONT_SANS, normal=FONT_SANS, bold=sans_bold,
        italic=FONT_SANS, boldItalic=sans_bold,
    )
    pdfmetrics.registerFontFamily(
        FONT_MONO, normal=mono, bold=mono_bold,
        italic=mono, boldItalic=mono_bold,
    )
    _FONTS_REGISTERED = True


# ---------------------------------------------------------------------------
# Paragraph styles
# ---------------------------------------------------------------------------

def _build_styles():
    """Construct and return the dict of :class:`ParagraphStyle` objects.

    Styles are built fresh on each render call (after fonts are registered) so
    the module carries no import-time ReportLab state.  All sizes and colours
    are fixed constants, keeping output deterministic.
    """
    styles = {}

    # Front-matter / section styles ------------------------------------------
    styles['doc_title'] = ParagraphStyle(
        'doc_title', fontName=FONT_SANS_BOLD, fontSize=15, leading=18,
        spaceBefore=0, spaceAfter=8, textColor=colors.HexColor('#1A2530'),
    )
    styles['scope'] = ParagraphStyle(
        'scope', fontName=FONT_SANS, fontSize=7.5, leading=10,
        spaceBefore=0, spaceAfter=6,
    )
    styles['legend_heading'] = ParagraphStyle(
        'legend_heading', fontName=FONT_SANS_BOLD, fontSize=10, leading=12,
        spaceBefore=6, spaceAfter=3, textColor=colors.HexColor('#1A2530'),
    )
    styles['legend'] = ParagraphStyle(
        'legend', fontName=FONT_SANS, fontSize=7, leading=9.5,
        spaceBefore=0, spaceAfter=3,
    )
    styles['footprint'] = ParagraphStyle(
        'footprint', fontName=FONT_SANS, fontSize=7, leading=9.5,
        spaceBefore=2, spaceAfter=4,
    )
    styles['flags'] = ParagraphStyle(
        'flags', fontName=FONT_SANS, fontSize=7, leading=9.5,
        spaceBefore=2, spaceAfter=6,
    )

    # Group / table headings --------------------------------------------------
    styles['group_header'] = ParagraphStyle(
        'group_header', fontName=FONT_SANS_BOLD, fontSize=13, leading=16,
        spaceBefore=12, spaceAfter=3, textColor=colors.white,
        backColor=_GROUP_BG, borderPadding=(4, 4, 4, 4), leftIndent=0,
    )
    styles['group_subtitle'] = ParagraphStyle(
        'group_subtitle', fontName=FONT_SANS, fontSize=8, leading=10.5,
        spaceBefore=2, spaceAfter=8,
    )
    styles['table_title'] = ParagraphStyle(
        'table_title', fontName=FONT_SANS_BOLD, fontSize=9.5, leading=12,
        spaceBefore=10, spaceAfter=3, textColor=colors.HexColor('#1A2530'),
    )
    styles['subheading'] = ParagraphStyle(
        'subheading', fontName=FONT_SANS_BOLD, fontSize=8, leading=10,
        spaceBefore=5, spaceAfter=2, textColor=colors.HexColor('#2C3E50'),
    )

    # Register-specific -------------------------------------------------------
    styles['register_title'] = ParagraphStyle(
        'register_title', fontName=FONT_SANS_BOLD, fontSize=12, leading=15,
        spaceBefore=14, spaceAfter=4, textColor=colors.white,
        backColor=_GROUP_BG, borderPadding=(4, 4, 4, 4),
    )
    styles['register_intro'] = ParagraphStyle(
        'register_intro', fontName=FONT_SANS, fontSize=7.5, leading=10,
        spaceBefore=2, spaceAfter=6,
    )
    styles['register_subheader'] = ParagraphStyle(
        'register_subheader', fontName=FONT_SANS_BOLD, fontSize=7.5, leading=9.5,
        textColor=colors.HexColor('#1A2530'),
    )

    # Table cell styles -------------------------------------------------------
    styles['th'] = ParagraphStyle(
        'th', fontName=FONT_SANS_BOLD, fontSize=6.5, leading=8,
        textColor=_HEADER_FG,
    )
    styles['th_center'] = ParagraphStyle(
        'th_center', parent=styles['th'], alignment=1,  # TA_CENTER
    )
    styles['cell'] = ParagraphStyle(
        'cell', fontName=FONT_SANS, fontSize=6, leading=7.6,
        wordWrap='LTR',
    )
    styles['cell_center'] = ParagraphStyle(
        'cell_center', parent=styles['cell'], alignment=1,  # TA_CENTER
    )
    styles['mono'] = ParagraphStyle(
        'mono', fontName=FONT_MONO, fontSize=5.5, leading=6.8,
        wordWrap='LTR',
    )
    return styles


# ---------------------------------------------------------------------------
# Text / cell helpers
# ---------------------------------------------------------------------------

def _xml_escape(text):
    """Escape the XML metacharacters that ReportLab paragraph markup reserves.

    Order matters: ``&`` must be escaped first so the entities emitted for
    ``<`` and ``>`` are not double-escaped.
    """
    return (
        text.replace('&', '&amp;')
            .replace('<', '&lt;')
            .replace('>', '&gt;')
    )


def _para(value, style):
    """Wrap arbitrary cell text in a soft-wrapping :class:`Paragraph`.

    Newlines become ``<br/>`` line breaks and XML metacharacters are escaped so
    values containing ``&``, ``<`` or ``>`` render literally.  ``None`` renders
    as an empty cell.
    """
    if value is None:
        value = ''
    text = _xml_escape(str(value)).replace('\n', '<br/>')
    return Paragraph(text, style)


def _mono_para(value, style):
    """Wrap a code snippet in a monospace, indentation-preserving Paragraph.

    Leading spaces on each physical line are converted to non-breaking spaces so
    source indentation survives, while interior spaces remain ordinary spaces so
    long lines still soft-wrap inside the fixed-width cell.  Newlines become
    ``<br/>`` breaks; ``splitLongWords`` (ReportLab's default) breaks any single
    token too wide for the column so nothing overflows.
    """
    if value is None:
        value = ''
    out_lines = []
    for physical in str(value).split('\n'):
        stripped = physical.lstrip(' ')
        indent = len(physical) - len(stripped)
        out_lines.append(('&nbsp;' * indent) + _xml_escape(stripped))
    return Paragraph('<br/>'.join(out_lines), style)


# ===========================================================================
# Verification harness (991 assertions across nine functions)
# ===========================================================================
#
# Design contract:
#   * Each verify_* function performs a fixed, deterministic number of checks
#     and returns a ``list[str]`` of failure messages (empty == all passed).
#   * A data-integrity failure is reported (not raised) so ``generate.py`` can
#     collect every problem in one pass; a genuine structural/programming error
#     (missing key, wrong type where a type is assumed) surfaces as an ordinary
#     exception, which is the correct signal for a broken data module.
#   * The exact per-function check counts (verified against the committed tree)
#     are recorded in EXPECTED_CHECK_COUNTS below and total 991.
#
# The first five verifiers guard the audit corpus (row provenance, vocabularies,
# chain depths, cross-references).  The final four -- added to close the F4
# harness-bypass gap -- guard the checkpoint-CP2 invariants that were previously
# unasserted: the 63+31=94 / 282 global reconciliation and table structure
# (verify_global_counts), the 12-row L1 mapping's AAP 0.6.1 parity
# (verify_new_service_location_map), the current-source line-reference provenance
# against the live controller (verify_line_reference_provenance), and the absence
# of PDF-extraction artifacts in rendered text (verify_no_extraction_artifacts).
#
# The optional ``_checker`` parameter lets callers (e.g. the aggregator or a
# test harness) inject a shared :class:`_Checker` to observe the running check
# count; when omitted, each function uses its own private counter.  The public
# signature remains ``verify_xxx(repo_root)``.

#: Authoritative per-function check counts (sum == 991).  Used for
#: documentation and by the test harness to assert the contract is met.
EXPECTED_CHECK_COUNTS = {
    'verify_group1': 189,
    'verify_group2': 62,
    'verify_layer1': 193,
    'verify_layer2': 188,
    'verify_ref_coverage': 206,
    # CP2 invariant verifiers (added to close the F4 harness-bypass gap).
    'verify_global_counts': 47,
    'verify_new_service_location_map': 53,
    'verify_line_reference_provenance': 32,
    'verify_no_extraction_artifacts': 21,
}
EXPECTED_TOTAL_CHECKS = 991


class _Checker(object):
    """Accumulates integrity checks: counts every assertion and records failures.

    Attributes:
        count (int): total number of checks performed.
        errors (list[str]): human-readable messages for each failed check.
    """

    __slots__ = ('count', 'errors')

    def __init__(self):
        self.count = 0
        self.errors = []

    def check(self, ok, message):
        """Record one check. Increment the counter and, if *ok* is falsy, append *message*."""
        self.count += 1
        if not ok:
            self.errors.append(message)
        return bool(ok)


def _read_cited(repo_root, path, line_start, line_end):
    """Return the verbatim text of *path* lines ``[line_start, line_end]`` (1-based, inclusive).

    Lines are obtained by splitting the whole file on ``'\\n'`` (matching the way
    ``pnt_data`` captured its snippets), so the returned block joins the selected
    lines with ``'\\n'`` and contains no trailing newline.

    Returns:
        tuple[str|None, str|None]: ``(block, None)`` on success, or
        ``(None, reason)`` where *reason* is ``'missing'`` (file absent) or
        ``'short'`` (fewer lines than *line_end*).
    """
    full = os.path.join(repo_root, path)
    if not os.path.isfile(full):
        return None, 'missing'
    with open(full, 'r', errors='replace') as handle:
        lines = handle.read().split('\n')
    if line_end > len(lines):
        return None, 'short'
    return '\n'.join(lines[line_start - 1:line_end]), None


def _check_citation(chk, repo_root, row, label):
    """One provenance check: the row's ``code_snippet`` must match its cited lines verbatim."""
    block, reason = _read_cited(
        repo_root, row['file_path'], row['line_start'], row['line_end'],
    )
    if reason is not None:
        chk.check(False, '%s: citation unreadable (%s) for %s:%s-%s'
                  % (label, reason, row['file_path'], row['line_start'], row['line_end']))
    else:
        chk.check(block == row['code_snippet'],
                  '%s: citation mismatch for %s:%s-%s'
                  % (label, row['file_path'], row['line_start'], row['line_end']))


def verify_group1(repo_root, _checker=None):
    """Verify the 63 Core (Group-1) main rows — 189 checks.

    Per row (x3): (a) ``Role`` is one of Source/Transform/Sink; (b) the code
    snippet is a non-empty 5-to-10-line block; (c) the snippet matches its cited
    file/line range verbatim.

    Returns:
        list[str]: failure messages (empty when all 189 checks pass).
    """
    chk = _checker if _checker is not None else _Checker()
    for table in GROUP1_TABLES:
        for row in table['rows']:
            # (a) Role vocabulary
            chk.check(row['role'] in ROLE_VOCABULARY,
                      'group1 #%s: role %r not in %s'
                      % (row.get('num'), row.get('role'), tuple(ROLE_VOCABULARY)))
            # (b) Snippet format: non-empty, 5..10 physical lines
            snippet = row.get('code_snippet', '') or ''
            line_count = len(snippet.split('\n'))
            chk.check(bool(snippet.strip()) and 5 <= line_count <= 10,
                      'group1 #%s: snippet has %d lines (want 5-10, non-empty)'
                      % (row.get('num'), line_count))
            # (c) Verbatim citation provenance
            _check_citation(chk, repo_root, row, 'group1 #%s' % row.get('num'))
    return chk.errors


def verify_group2(repo_root, _checker=None):
    """Verify the 31 Indirect (Group-2) main rows — 62 checks.

    Per row (x2): (a) the three discriminator fields (Trigger Condition, PNT
    State Observed, Behavior Change Description) are all present and non-empty;
    (b) the code snippet matches its cited file/line range verbatim.

    Returns:
        list[str]: failure messages (empty when all 62 checks pass).
    """
    chk = _checker if _checker is not None else _Checker()
    for table in GROUP2_TABLES:
        for row in table['rows']:
            # (a) Field completeness
            complete = all((row.get(key) or '').strip()
                           for key in ('trigger', 'observed', 'behavior'))
            chk.check(complete,
                      'group2 #%s: empty trigger/observed/behavior field'
                      % row.get('num'))
            # (b) Verbatim citation provenance
            _check_citation(chk, repo_root, row, 'group2 #%s' % row.get('num'))
    return chk.errors


def verify_layer1(repo_root, _checker=None):
    """Verify the 94 Layer-1 dependency edges — 193 checks.

    (a) 94 checks: each edge's ``Dependency Type`` is in the controlled
    vocabulary {Data dependency, Function call, Inheritance / Interface,
    Shared global / singleton}.  (b) 94 checks: each edge's code snippet matches
    its cited file/line range verbatim.  (c) 5 checks: every edge typed
    ``Shared global / singleton`` mentions an ``AP::`` accessor (the singleton
    consistency invariant) — there are exactly five such edges.

    Returns:
        list[str]: failure messages (empty when all 193 checks pass).
    """
    chk = _checker if _checker is not None else _Checker()
    # (a) dependency-type vocabulary + (b) verbatim citation
    for table in LAYER1_TABLES:
        for row in table['rows']:
            chk.check(row['dependency_type'] in DEPENDENCY_TYPE_VOCABULARY,
                      'layer1 %s ref %s: dependency type %r not in vocabulary'
                      % (table.get('number'), row.get('ref'), row.get('dependency_type')))
            _check_citation(chk, repo_root, row,
                            'layer1 %s ref %s' % (table.get('number'), row.get('ref')))
    # (c) AP:: singleton consistency (exactly the 'Shared global / singleton' edges)
    for table in LAYER1_TABLES:
        for row in table['rows']:
            if row['dependency_type'] == 'Shared global / singleton':
                blob = '%s %s %s' % (
                    row.get('directly_calls', ''),
                    row.get('directly_called_by', ''),
                    row.get('code_snippet', ''),
                )
                chk.check('AP::' in blob,
                          'layer1 %s ref %s: singleton edge lacks AP:: accessor'
                          % (table.get('number'), row.get('ref')))
    return chk.errors


def verify_layer2(repo_root, _checker=None):
    """Verify the 94 Layer-2 transitive chains — 188 checks.

    Per row (x2): (a) ``Chain Depth`` is a positive integer; (b) it equals the
    number of ``->`` (U+2192) hops in the ``Full Dependency Chain``.

    Returns:
        list[str]: failure messages (empty when all 188 checks pass).
    """
    chk = _checker if _checker is not None else _Checker()
    for table in LAYER2_TABLES:
        for row in table['rows']:
            depth = row.get('chain_depth')
            chk.check(isinstance(depth, int) and not isinstance(depth, bool) and depth > 0,
                      'layer2 %s ref %s: chain_depth %r is not a positive int'
                      % (table.get('number'), row.get('ref'), depth))
            arrows = str(row.get('full_dependency_chain', '')).count(ARROW)
            chk.check(arrows == depth,
                      'layer2 %s ref %s: %d arrows but chain_depth=%r'
                      % (table.get('number'), row.get('ref'), arrows, depth))
    return chk.errors


def verify_ref_coverage(repo_root, _checker=None):
    """Verify bidirectional cross-reference integrity and numbering — 206 checks.

    (a) 94 checks: every Layer-1 ``Ref #`` resolves to a real main-row ``#`` in
    its parent table.  (b) 94 checks: every Layer-2 ``Ref #`` resolves likewise.
    (c) 18 checks: within each of the 6 main + 6 Layer-1 + 6 Layer-2 tables, the
    ``#`` / ``Ref #`` sequence is monotonic ``1..N`` with no gaps or duplicates.

    Because each parent main table's row numbers restart at 1 and every Layer-1
    and Layer-2 table has exactly one row per parent main row (checked here),
    the resolution checks establish full 1:1:1 coverage in both directions.

    Returns:
        list[str]: failure messages (empty when all 206 checks pass).
    """
    chk = _checker if _checker is not None else _Checker()
    # Map each main table number -> set of its row numbers.
    main_nums = {}
    for table in ALL_MAIN_TABLES:
        main_nums[table['number']] = set(row['num'] for row in table['rows'])

    # (a) Layer-1 ref -> parent main row
    for table in LAYER1_TABLES:
        parent_set = main_nums.get(table['parent'], set())
        for row in table['rows']:
            chk.check(row['ref'] in parent_set,
                      'layer1 %s: Ref # %s has no matching main #%s row'
                      % (table.get('number'), row.get('ref'), table.get('parent')))

    # (b) Layer-2 ref -> parent main row
    for table in LAYER2_TABLES:
        parent_set = main_nums.get(table['parent'], set())
        for row in table['rows']:
            chk.check(row['ref'] in parent_set,
                      'layer2 %s: Ref # %s has no matching main #%s row'
                      % (table.get('number'), row.get('ref'), table.get('parent')))

    # (c) Monotonic 1..N numbering in all 18 tables
    for table in ALL_MAIN_TABLES:
        nums = [row['num'] for row in table['rows']]
        chk.check(nums == list(range(1, len(nums) + 1)),
                  'main table %s: numbering not monotonic 1..%d'
                  % (table.get('number'), len(nums)))
    for table in list(LAYER1_TABLES) + list(LAYER2_TABLES):
        refs = [row['ref'] for row in table['rows']]
        chk.check(refs == list(range(1, len(refs) + 1)),
                  'layer table %s: Ref # not monotonic 1..%d'
                  % (table.get('number'), len(refs)))
    return chk.errors


# ===========================================================================
# CP2 invariant references (frozen contracts) + helpers
# ===========================================================================
# The following four verifiers assert the checkpoint-CP2 invariants that were
# previously unguarded: the 63+31=94 / 282 global reconciliation, the rendered
# table structure, the 12-row L1 mapping content (AAP 0.6.1 parity + row-1
# annotations), the line-reference provenance against the live controller, and
# the absence of PDF-extraction artifacts.  Each is wired into
# ``run_all_verifications`` and ``count_all_checks`` so that any violation makes
# ``generate.py`` refuse to (over)write the PDF.

#: The 12 canonical L1-navigation mapping rows, frozen verbatim from AAP 0.6.1.
#: Each entry pins the AAP text fields that must match byte-for-byte:
#: ``pnt_pillar`` / ``behavior`` / ``current_accessor`` / ``new_service_location``
#: AND the full ``current_locations`` citation -- INCLUDING its line numbers,
#: which final acceptance requires the rendered PDF to reproduce verbatim from
#: AAP 0.6.1.  :func:`verify_new_service_location_map` fails closed on any drift
#: of a rendered row from this frozen contract.  (The line numbers are the
#: AAP 0.6.1 snapshot, NOT the live controller layout -- the additive, default-off
#: ``set_update_dt`` seams on the waypoint and loiter paths shift the live layout;
#: :func:`verify_line_reference_provenance` separately confirms every cited
#: accessor still exists in the live controller.)
#:
#: Tuple layout: (pnt_pillar, behavior, current_accessor, new_service_location,
#:                loc_file, current_locations).
_AAP_061_MAP = (
    ('Position', 'Read vehicle position',
     '_ahrs.get_location(_current_loc)',
     'AHRS shim get_location(), fed by set_state_ne(n, e, \u2026)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L230 (update_waypoint), L369 (update_loiter)'),
    ('Navigation (velocity)', 'Read ground velocity vector',
     '_ahrs.groundspeed_vector()',
     'AHRS shim groundspeed_vector(), fed by set_velocity_EN(velE, velN)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L236, L375, L507'),
    ('Navigation (attitude)', 'Read yaw (radians)',
     '_ahrs.get_yaw_rad()',
     'AHRS shim get_yaw_rad(), fed by set_yaw_cd(yaw_cd)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L59, L61, L402, L536'),
    ('Navigation (attitude)', 'Read yaw (centideg sensor)',
     '_ahrs.yaw_sensor',
     'AHRS shim yaw_sensor, fed by set_yaw_cd(yaw_cd)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L70, L72, L503, L535'),
    ('Navigation (attitude)', 'Read pitch (radians)',
     '_ahrs.get_pitch_rad()',
     'AHRS shim get_pitch_rad(), fed by set_pitch_rad(pitch_rad)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L91'),
    ('Navigation (airspeed)', 'Airspeed scaling factor',
     '_ahrs.get_EAS2TAS()',
     'AHRS shim get_EAS2TAS() (injected or unit default)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L126, L159'),
    ('Timing', 'Control-step clock',
     'AP_HAL::micros()',
     'set_update_dt(dt) injected timebase', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L214'),
    ('Timing', 'Step delta + state store',
     '_last_update_waypoint_us',
     'Injected-dt path (clamp preserved)', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L215, L224'),
    ('Timing', 'Loiter/heading clock',
     'AP_HAL::millis()',
     'Injected time for the loiter path', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L448'),
    ('Navigation (math)', 'Bearing / NE distance',
     'Location::get_bearing_to / get_distance_NE',
     'Preserved unchanged inside AP_L1_Control', 'AP_L1_Control.cpp',
     'AP_L1_Control.cpp:L239, L382 / L260, L266, L274, L295, L391'),
    ('Navigation (output)', 'Roll command',
     'nav_roll_cd()',
     'get_roll_deg() = nav_roll_cd()/100 \u2192 L1_GetRollDeg', 'AP_L1_Control.h',
     'AP_L1_Control.h:L36'),
    ('Navigation (output)', 'Lateral acceleration',
     'lateral_acceleration()',
     'get_lat_accel() \u2192 L1_GetLatAccel', 'AP_L1_Control.h',
     'AP_L1_Control.h:L37'),
)

#: Row-1 (Position) ``current_locations`` must retain these AAP 0.6.1 function
#: annotations attached to the two ``get_location`` call sites.
_AAP_061_ROW1_ANNOTATIONS = ('(update_waypoint)', '(update_loiter)')

#: High-signal PDF-extraction artifact fragments that must never appear in any
#: rendered descriptive field.  Presence of any of these indicates a regression
#: of the F3 cleanup (pdftotext column-wrap corruption bleeding back in).
_ARTIFACT_FRAGMENTS = (
    'SelectVelPosFusi on', 'U pdateFilter', 'get_location(lo c)',
    'calcGpsGoodToAli gn', 'get_relative_position_N ED', 'handle_pose_est imate',
    'nav_ controller', 'attitude _control', 'send_nav_ controller_output',
    'Ardu Plane', 'Pla ne.cpp', 'A P_AHRS', 'v el_ned', 'update_G PS',
    'require s_GPS', 'ekf_ch eck', 'dtfiltering', 'now_us - \u2192',
)


def _loc_refs_from_map():
    """Return the set of ``(filename, line_int)`` cited across the map's
    ``current_locations`` column.

    The ``AP_L1_Control`` filename token is stripped before extracting ``L<n>``
    references so the ``L1`` inside the product name is never mistaken for a
    line number.
    """
    out = set()
    for entry in NEW_SERVICE_LOCATION_MAP:
        cl = entry.get('current_locations', '') or ''
        fn = 'AP_L1_Control.cpp' if 'AP_L1_Control.cpp' in cl else (
            'AP_L1_Control.h' if 'AP_L1_Control.h' in cl else None)
        stripped = cl.replace('AP_L1_Control.cpp', '').replace('AP_L1_Control.h', '')
        for num in re.findall(r'L(\d+)', stripped):
            if fn is not None:
                out.add((fn, int(num)))
    return out


def _loc_refs_from_provenance_keys():
    """Return the set of ``(filename, line_int)`` from the provenance dict keys
    (e.g. ``'AP_L1_Control.cpp:L248'`` -> ``('AP_L1_Control.cpp', 248)``)."""
    out = set()
    for key in NEW_SERVICE_LOCATION_BY_PROVENANCE:
        fn, _, lpart = key.rpartition(':')
        if fn and lpart.startswith('L') and lpart[1:].isdigit():
            out.add((fn, int(lpart[1:])))
    return out


def _loc_refs_from_provenance_checks():
    """Return the set of ``(filename, line_int)`` from :data:`L1_PROVENANCE_CHECKS`
    (relpath basename is used so it aligns with the map/provenance filenames)."""
    out = set()
    for relpath, line, _token in L1_PROVENANCE_CHECKS:
        out.add((os.path.basename(relpath), int(line)))
    return out


def _iter_display_text():
    """Yield ``(label, value)`` for every rendered *descriptive* string field.

    Descriptive fields are all rendered strings EXCEPT the verbatim
    ``code_snippet`` / ``file_path`` fields (which are provenance-checked against
    live source and legitimately contain ``::`` and parenthesised code).  This
    is the surface scanned by :func:`verify_no_extraction_artifacts`.
    """
    skip = ('code_snippet', 'file_path')
    for table in list(GROUP1_TABLES) + list(GROUP2_TABLES) + \
            list(LAYER1_TABLES) + list(LAYER2_TABLES):
        for row in table['rows']:
            for key, value in row.items():
                if key not in skip and isinstance(value, str):
                    yield ('%s#%s.%s' % (table.get('number'), row.get('num', row.get('ref')), key), value)
    for item in COVERAGE_REGISTER.get('items', []):
        for key, value in item.items():
            if isinstance(value, str):
                yield ('coverage.%s' % key, value)
    for i, entry in enumerate(NEW_SERVICE_LOCATION_MAP):
        for key, value in entry.items():
            if isinstance(value, str):
                yield ('nsl_map[%d].%s' % (i, key), value)


def verify_global_counts(repo_root, _checker=None):
    """Verify the document-wide count reconciliation and table structure.

    Asserts the checkpoint invariants that 63 Core + 31 Indirect = 94 main rows,
    that the corpus totals 282 evidence rows (94 x 3 layers), that the rendered
    structure is 6 main + 6 Layer-1 + 6 Layer-2 tables, and that each dependency
    table mirrors its parent main table's row count.  Both the frozen constants
    and the live table contents are checked so a drift in either is caught.

    Returns:
        list[str]: failure messages (empty when every count reconciles).
    """
    chk = _checker if _checker is not None else _Checker()
    # (1) Frozen reconciliation constants
    chk.check(CORE_ROW_COUNT == 63, 'global: CORE_ROW_COUNT %r != 63' % CORE_ROW_COUNT)
    chk.check(INDIRECT_ROW_COUNT == 31, 'global: INDIRECT_ROW_COUNT %r != 31' % INDIRECT_ROW_COUNT)
    chk.check(MAIN_ROW_COUNT == 94, 'global: MAIN_ROW_COUNT %r != 94' % MAIN_ROW_COUNT)
    chk.check(EVIDENCE_ROW_COUNT == 282, 'global: EVIDENCE_ROW_COUNT %r != 282' % EVIDENCE_ROW_COUNT)
    chk.check(CORE_ROW_COUNT + INDIRECT_ROW_COUNT == MAIN_ROW_COUNT,
              'global: 63+31 reconciliation != MAIN_ROW_COUNT (%r+%r != %r)'
              % (CORE_ROW_COUNT, INDIRECT_ROW_COUNT, MAIN_ROW_COUNT))
    chk.check(MAIN_ROW_COUNT * 3 == EVIDENCE_ROW_COUNT,
              'global: 94x3 != EVIDENCE_ROW_COUNT (%r != %r)'
              % (MAIN_ROW_COUNT * 3, EVIDENCE_ROW_COUNT))
    # (2) Live row totals reconcile to the frozen constants
    g1 = sum(len(t['rows']) for t in GROUP1_TABLES)
    g2 = sum(len(t['rows']) for t in GROUP2_TABLES)
    l1 = sum(len(t['rows']) for t in LAYER1_TABLES)
    l2 = sum(len(t['rows']) for t in LAYER2_TABLES)
    chk.check(g1 == CORE_ROW_COUNT, 'global: live Group-1 rows %d != %d' % (g1, CORE_ROW_COUNT))
    chk.check(g2 == INDIRECT_ROW_COUNT, 'global: live Group-2 rows %d != %d' % (g2, INDIRECT_ROW_COUNT))
    chk.check(main_row_count() == MAIN_ROW_COUNT,
              'global: main_row_count() %d != %d' % (main_row_count(), MAIN_ROW_COUNT))
    chk.check(evidence_row_count() == EVIDENCE_ROW_COUNT,
              'global: evidence_row_count() %d != %d' % (evidence_row_count(), EVIDENCE_ROW_COUNT))
    chk.check(l1 == MAIN_ROW_COUNT, 'global: live Layer-1 rows %d != %d' % (l1, MAIN_ROW_COUNT))
    chk.check(l2 == MAIN_ROW_COUNT, 'global: live Layer-2 rows %d != %d' % (l2, MAIN_ROW_COUNT))
    # (3) Rendered table structure: 6 main + 6 Layer-1 + 6 Layer-2 = 18
    chk.check(len(GROUP1_TABLES) == 3, 'global: GROUP1_TABLES %d != 3' % len(GROUP1_TABLES))
    chk.check(len(GROUP2_TABLES) == 3, 'global: GROUP2_TABLES %d != 3' % len(GROUP2_TABLES))
    chk.check(len(ALL_MAIN_TABLES) == 6, 'global: ALL_MAIN_TABLES %d != 6' % len(ALL_MAIN_TABLES))
    chk.check(len(LAYER1_TABLES) == 6, 'global: LAYER1_TABLES %d != 6' % len(LAYER1_TABLES))
    chk.check(len(LAYER2_TABLES) == 6, 'global: LAYER2_TABLES %d != 6' % len(LAYER2_TABLES))
    # (4) Each main table populated + non-empty title; each dependency table
    #     mirrors its parent main table's row count and uses the 'Na' number.
    for idx, main in enumerate(ALL_MAIN_TABLES):
        chk.check(len(main['rows']) > 0 and bool((main.get('title') or '').strip()),
                  'global: main table %s empty rows/title' % main.get('number'))
        l1t = LAYER1_TABLES[idx]
        l2t = LAYER2_TABLES[idx]
        chk.check(len(l1t['rows']) == len(main['rows']),
                  'global: Layer-1 %s rows %d != parent %s rows %d'
                  % (l1t.get('number'), len(l1t['rows']), main.get('number'), len(main['rows'])))
        chk.check(len(l2t['rows']) == len(main['rows']),
                  'global: Layer-2 %s rows %d != parent %s rows %d'
                  % (l2t.get('number'), len(l2t['rows']), main.get('number'), len(main['rows'])))
        chk.check(str(l1t.get('number')) == str(main.get('number')) + 'a',
                  'global: Layer-1 number %r != %ra' % (l1t.get('number'), main.get('number')))
        chk.check(str(l2t.get('number')) == str(main.get('number')) + 'a',
                  'global: Layer-2 number %r != %ra' % (l2t.get('number'), main.get('number')))
    return chk.errors


def verify_new_service_location_map(repo_root, _checker=None):
    """Verify the 12-row L1 mapping matches AAP 0.6.1 verbatim (F5 parity).

    Fails closed on ANY drift of :data:`NEW_SERVICE_LOCATION_MAP` from the frozen
    :data:`_AAP_061_MAP` contract: the map must have exactly the 12 AAP 0.6.1
    entries in order, and each row's ``pnt_pillar`` / ``behavior`` /
    ``current_accessor`` / ``new_service_location`` AND its full
    ``current_locations`` citation -- INCLUDING the AAP 0.6.1 line numbers -- must
    be byte-identical to the frozen text.  Row 1 additionally retains the
    ``(update_waypoint)`` / ``(update_loiter)`` annotations (subsumed by the exact
    match, kept as an explicit diagnostic).  That every cited accessor still
    exists in the live controller is confirmed separately by
    :func:`verify_line_reference_provenance`.

    Returns:
        list[str]: failure messages (empty when the mapping is faithful).
    """
    chk = _checker if _checker is not None else _Checker()
    chk.check(len(NEW_SERVICE_LOCATION_MAP) == 12,
              'nsl_map: expected 12 rows, found %d' % len(NEW_SERVICE_LOCATION_MAP))
    chk.check(len(_AAP_061_MAP) == 12,
              'nsl_map: frozen AAP reference corrupt (%d != 12)' % len(_AAP_061_MAP))
    # Ordered (pillar, behavior) identity vs the frozen AAP order.
    data_keys = [(e.get('pnt_pillar'), e.get('behavior')) for e in NEW_SERVICE_LOCATION_MAP]
    frozen_keys = [(p, b) for (p, b, _a, _n, _f, _cl) in _AAP_061_MAP]
    chk.check(data_keys == frozen_keys,
              'nsl_map: (pillar, behavior) sequence diverged from AAP 0.6.1')
    lookup = {(e.get('pnt_pillar'), e.get('behavior')): e for e in NEW_SERVICE_LOCATION_MAP}
    for (pillar, behavior, accessor, nsl, loc_file, current_locations) in _AAP_061_MAP:
        row = lookup.get((pillar, behavior))
        chk.check(row is not None and row.get('current_accessor') == accessor,
                  'nsl_map: current_accessor drift for %r/%r' % (pillar, behavior))
        chk.check(row is not None and row.get('new_service_location') == nsl,
                  'nsl_map: new_service_location drift for %r/%r' % (pillar, behavior))
        cl = (row or {}).get('current_locations', '') or ''
        # Fail closed on ANY current_locations drift -- INCLUDING line numbers --
        # from the frozen AAP 0.6.1 citation (the F5 documentation-parity guard).
        chk.check(cl == current_locations,
                  'nsl_map: current_locations for %r/%r must match AAP 0.6.1 verbatim '
                  '(expected %r, got %r)' % (pillar, behavior, current_locations, cl))
        chk.check(cl.startswith(loc_file + ':'),
                  'nsl_map: current_locations for %r/%r must reference %s (got %r)'
                  % (pillar, behavior, loc_file, cl))
    # Row-1 (Position) function annotations preserved.
    row1 = lookup.get(('Position', 'Read vehicle position'))
    row1_cl = (row1 or {}).get('current_locations', '') or ''
    for annot in _AAP_061_ROW1_ANNOTATIONS:
        chk.check(annot in row1_cl,
                  'nsl_map: row-1 current_locations missing %r annotation' % annot)
    return chk.errors


def verify_line_reference_provenance(repo_root, _checker=None):
    """Verify the frozen AAP 0.6.1 provenance is self-consistent and live (F5).

    Enforces two invariants:

    1. Lock-step: the ``(filename, line)`` set cited by the map's
       ``current_locations`` column, the :data:`NEW_SERVICE_LOCATION_BY_PROVENANCE`
       keys, and the :data:`L1_PROVENANCE_CHECKS` table are identical, so the
       three surfaces can never drift from the single frozen AAP 0.6.1 line set.
    2. Live existence: every cited accessor ``token`` still appears somewhere in
       the live ArduPilot controller source, so the audit never documents a
       behaviour that has been removed.

    It deliberately does NOT require the frozen AAP 0.6.1 line to equal the
    accessor's *live* line: those numbers are the frozen documentation snapshot
    that :func:`verify_new_service_location_map` pins the PDF to, whereas the
    project's own additive, default-off ``set_update_dt`` seams shift the live
    layout.

    Returns:
        list[str]: failure messages (empty when provenance is consistent + live).
    """
    chk = _checker if _checker is not None else _Checker()
    chk.check(len(L1_PROVENANCE_CHECKS) > 0, 'provenance: L1_PROVENANCE_CHECKS is empty')
    # Lock-step of the three line-reference surfaces.
    map_locs = _loc_refs_from_map()
    key_locs = _loc_refs_from_provenance_keys()
    chk_locs = _loc_refs_from_provenance_checks()
    chk.check(map_locs == chk_locs,
              'provenance: map current_locations lines != L1_PROVENANCE_CHECKS lines '
              '(only-map=%s only-checks=%s)' % (sorted(map_locs - chk_locs), sorted(chk_locs - map_locs)))
    chk.check(map_locs == key_locs,
              'provenance: map current_locations lines != NEW_SERVICE_LOCATION_BY_PROVENANCE keys '
              '(only-map=%s only-keys=%s)' % (sorted(map_locs - key_locs), sorted(key_locs - map_locs)))
    # Per-citation: every documented accessor token must still EXIST in the live
    # controller source.  Existence (not exact-line) is checked on purpose: the
    # frozen AAP 0.6.1 line numbers predate the project's own additive, default-off
    # set_update_dt seams, which shift the live layout.  A token that has vanished
    # entirely means the documented behaviour was genuinely removed -- a real
    # parity defect -- so the check fails closed on that.
    cache = {}
    for relpath, line, token in L1_PROVENANCE_CHECKS:
        if relpath not in cache:
            full = os.path.join(repo_root, relpath)
            if os.path.isfile(full):
                with open(full, 'r', errors='replace') as handle:
                    cache[relpath] = handle.read()
            else:
                cache[relpath] = None
        text = cache[relpath]
        if text is None:
            chk.check(False, 'provenance: source file missing: %s' % relpath)
        else:
            chk.check(token in text,
                      'provenance: accessor %r cited for %s (AAP 0.6.1 L%d) no longer '
                      'exists anywhere in the live controller' % (token, relpath, line))
    return chk.errors


def verify_no_extraction_artifacts(repo_root, _checker=None):
    """Verify no PDF-extraction artifacts remain in rendered descriptive text (F3).

    Scans every rendered descriptive field (all rendered strings except the
    verbatim ``code_snippet`` / ``file_path`` provenance fields) and fails on:
    (a) the output filename bleeding into a data field; (b) an empty-argument
    ``'( )'`` paren; (c) a spaced ``'::'`` scope-resolution split; and (d) any of
    the known high-signal split-identifier fragments.  This makes a regression of
    the F3 cleanup fail the harness before the PDF is (re)written.

    Returns:
        list[str]: failure messages (empty when the corpus is artifact-free).
    """
    chk = _checker if _checker is not None else _Checker()
    items = list(_iter_display_text())
    # (a) filename bleed
    fn_hit = next(((lbl, v) for (lbl, v) in items if OUTPUT_PDF_FILENAME in v), None)
    chk.check(fn_hit is None,
              'artifacts: output filename bled into descriptive field %s: %r'
              % (fn_hit[0], fn_hit[1][:80]) if fn_hit else 'artifacts: filename bleed')
    # (b) empty-argument parens
    par_hit = next(((lbl, v) for (lbl, v) in items if '( )' in v), None)
    chk.check(par_hit is None,
              "artifacts: '( )' in descriptive field %s: %r"
              % (par_hit[0], par_hit[1][:80]) if par_hit else "artifacts: '( )'")
    # (c) spaced scope-resolution split
    scope_re = re.compile(r'[A-Za-z0-9_]+:: [A-Za-z0-9_]+')
    scope_hit = next(((lbl, v) for (lbl, v) in items if scope_re.search(v)), None)
    chk.check(scope_hit is None,
              "artifacts: ':: ' scope split in %s: %r"
              % (scope_hit[0], scope_hit[1][:80]) if scope_hit else "artifacts: ':: ' split")
    # (d) known split-identifier fragments
    for frag in _ARTIFACT_FRAGMENTS:
        hit = next(((lbl, v) for (lbl, v) in items if frag in v), None)
        chk.check(hit is None,
                  'artifacts: fragment %r in %s: %r'
                  % (frag, hit[0], hit[1][:80]) if hit else 'artifacts: fragment %r' % frag)
    return chk.errors


def run_all_verifications(repo_root):
    """Run all nine ``verify_*`` functions and return the concatenated errors.

    Args:
        repo_root: repository root used to resolve cited file paths.

    Returns:
        list[str]: every failure message from every verifier, in a stable order.
        An empty list means all 991 checks passed.
    """
    errors = []
    errors.extend(verify_group1(repo_root))
    errors.extend(verify_group2(repo_root))
    errors.extend(verify_layer1(repo_root))
    errors.extend(verify_layer2(repo_root))
    errors.extend(verify_ref_coverage(repo_root))
    # CP2 invariants (global reconciliation, mapping parity, line-reference
    # provenance, artifact-free rendered text).
    errors.extend(verify_global_counts(repo_root))
    errors.extend(verify_new_service_location_map(repo_root))
    errors.extend(verify_line_reference_provenance(repo_root))
    errors.extend(verify_no_extraction_artifacts(repo_root))
    return errors


def count_all_checks(repo_root):
    """Return the total number of integrity checks performed across all verifiers.

    Provided so callers/tests can confirm the 991-assertion contract is met on
    the current tree.  Uses a shared :class:`_Checker` per function to observe
    the running count without altering the public return contract.
    """
    total = 0
    for func in (verify_group1, verify_group2, verify_layer1,
                 verify_layer2, verify_ref_coverage,
                 verify_global_counts, verify_new_service_location_map,
                 verify_line_reference_provenance, verify_no_extraction_artifacts):
        chk = _Checker()
        func(repo_root, _checker=chk)
        total += chk.count
    return total


# ===========================================================================
# Renderer
# ===========================================================================

# Column-width fractions (each tuple sums to 1.0); scaled by USABLE_WIDTH so the
# widths are fixed points -> deterministic layout.  Keys are chosen per schema.
_FRACTIONS = {
    # Group-1 Core (7 columns): #, File Path, Line(s), Function/Class, Role,
    # Code Snippet, Notes
    'g1': (0.028, 0.119, 0.052, 0.112, 0.060, 0.337, 0.292),
    # Group-1 Core + additive New Service Location (8 columns)
    'g1_nsl': (0.025, 0.106, 0.047, 0.097, 0.052, 0.262, 0.212, 0.199),
    # Group-2 Indirect (9 columns): #, File Path, Line(s), Function/Class,
    # Trigger, Observed, Behavior, Code Snippet, Notes
    'g2': (0.022, 0.090, 0.042, 0.085, 0.119, 0.119, 0.137, 0.219, 0.167),
    # Layer-1 (6 columns): Ref #, Component, Directly Calls, Directly Called By,
    # Dependency Type, Code Snippet
    'l1': (0.042, 0.162, 0.187, 0.187, 0.112, 0.310),
    # Layer-2 (6 columns): Ref #, PNT Origin, Full Dependency Chain,
    # Final Consumer, Chain Depth, Notes
    'l2': (0.042, 0.162, 0.375, 0.150, 0.057, 0.214),
    # Coverage & Explicit-Absence Register (4 columns)
    'reg': (0.237, 0.150, 0.362, 0.251),
    # Additive L1 extraction map (5 columns): PNT Pillar, Behavior,
    # Current Location(s), Current Accessor, New Service Location
    'nsl_map': (0.130, 0.180, 0.190, 0.230, 0.270),
}

#: Column headers for the additive L1 extraction-map section (mirrors AAP 0.6.1).
_NSL_MAP_COLUMNS = (
    ('PNT Pillar', 'pnt_pillar'),
    ('Behavior', 'behavior'),
    ('Current Location(s)', 'current_locations'),
    ('Current Accessor', 'current_accessor'),
    ('New Service Location', 'new_service_location'),
)


def _widths(kind):
    """Return absolute column widths (points) for a schema *kind*, summing to USABLE_WIDTH."""
    return [fraction * USABLE_WIDTH for fraction in _FRACTIONS[kind]]


def _row_table_style(n_rows, header_bg=_HEADER_BG, first_col_center=True):
    """Build the shared :class:`TableStyle` for a data table.

    Applies: header-row background, top vertical alignment, a thin inner grid,
    a heavier outer box, tight cell padding, and alternating (zebra) body-row
    shading beginning at the first body row.  ``repeatRows=1`` on the table
    itself re-draws the header after every page break.
    """
    commands = [
        ('BACKGROUND', (0, 0), (-1, 0), header_bg),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('GRID', (0, 0), (-1, -1), 0.25, _GRID),
        ('BOX', (0, 0), (-1, -1), 0.6, _BOX),
        ('LINEBELOW', (0, 0), (-1, 0), 0.8, _BOX),
        ('LEFTPADDING', (0, 0), (-1, -1), 3.0),
        ('RIGHTPADDING', (0, 0), (-1, -1), 3.0),
        ('TOPPADDING', (0, 0), (-1, -1), 2.6),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 2.6),
    ]
    if first_col_center:
        commands.append(('ALIGN', (0, 1), (0, -1), 'CENTER'))
    # Alternating shading over the body rows (row 0 is the header).
    if n_rows > 1:
        commands.append(('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, _ALT_BG]))
    return TableStyle(commands)


def _header_cells(headers, styles, center_first=True):
    """Return the header row: white-bold paragraphs, first cell optionally centred."""
    cells = []
    for index, header in enumerate(headers):
        style = styles['th_center'] if (center_first and index == 0) else styles['th']
        cells.append(Paragraph(_xml_escape(str(header)).replace('\n', '<br/>'), style))
    return cells


def _build_main_table(table, styles):
    """Build a Group-1 or Group-2 main-reference LongTable.

    Honours the table's own ``columns`` schema (7-col Core or 9-col Indirect) and,
    when ``has_new_service_location`` is set (Table 2), appends the additive
    "New Service Location" column, sourcing each row's value from the row data
    with a provenance-map fallback and an em-dash default.
    """
    schema = list(table['columns'])
    headers = [head for (head, _key) in schema]
    keys = [key for (_head, key) in schema]

    add_nsl = bool(table.get('has_new_service_location'))
    if add_nsl:
        headers.append(NEW_SERVICE_LOCATION_COLUMN[0])
        keys.append(NEW_SERVICE_LOCATION_COLUMN[1])

    # Choose the width profile: NSL variant, else Group-2 (9 cols), else Group-1.
    if add_nsl:
        width_kind = 'g1_nsl'
    elif len(schema) == 9:
        width_kind = 'g2'
    else:
        width_kind = 'g1'

    data = [_header_cells(headers, styles)]
    for row in table['rows']:
        cells = []
        for key in keys:
            if key == 'code_snippet':
                cells.append(_mono_para(row.get(key, ''), styles['mono']))
            elif key == 'num':
                cells.append(_para(row.get(key, ''), styles['cell_center']))
            elif key == 'new_service_location':
                # The data layer pre-materialises each Core Navigation row's
                # service-location value (a real mapping for the L1 rows, an
                # em-dash for rows with no L1 mapping).  Render it verbatim,
                # defaulting to an em-dash if a row omits the field entirely.
                value = row.get(key) or u'\u2014'
                cells.append(_para(value, styles['cell']))
            else:
                cells.append(_para(row.get(key, ''), styles['cell']))
        data.append(cells)

    tbl = LongTable(data, colWidths=_widths(width_kind), repeatRows=1)
    tbl.setStyle(_row_table_style(len(data)))
    return tbl


def _build_layer1_table(table, styles):
    """Build a Layer-1 dependency LongTable (6 columns; code snippet is monospace)."""
    schema = list(table['columns'])
    headers = [head for (head, _key) in schema]
    keys = [key for (_head, key) in schema]

    data = [_header_cells(headers, styles)]
    for row in table['rows']:
        cells = []
        for key in keys:
            if key == 'code_snippet':
                cells.append(_mono_para(row.get(key, ''), styles['mono']))
            elif key == 'ref':
                cells.append(_para(row.get(key, ''), styles['cell_center']))
            else:
                cells.append(_para(row.get(key, ''), styles['cell']))
        data.append(cells)

    tbl = LongTable(data, colWidths=_widths('l1'), repeatRows=1)
    tbl.setStyle(_row_table_style(len(data)))
    return tbl


def _build_layer2_table(table, styles):
    """Build a Layer-2 transitive-chain LongTable (6 columns; arrows via DejaVu)."""
    schema = list(table['columns'])
    headers = [head for (head, _key) in schema]
    keys = [key for (_head, key) in schema]

    data = [_header_cells(headers, styles)]
    for row in table['rows']:
        cells = []
        for key in keys:
            if key in ('ref', 'chain_depth'):
                cells.append(_para(row.get(key, ''), styles['cell_center']))
            else:
                cells.append(_para(row.get(key, ''), styles['cell']))
        data.append(cells)

    tbl = LongTable(data, colWidths=_widths('l2'), repeatRows=1)
    tbl.setStyle(_row_table_style(len(data)))
    return tbl


def _build_register_table(register, styles):
    """Build the Coverage & Explicit-Absence Register LongTable (4 columns).

    Entry rows populate the four schema columns; sub-header rows (the per-vehicle
    flight-mode gating matrices) span all four columns and carry a distinct band
    background.  ``repeatRows=1`` keeps the column header visible across pages.
    """
    schema = list(register['columns'])
    headers = [head for (head, _key) in schema]
    keys = [key for (_head, key) in schema]
    n_cols = len(keys)

    data = [_header_cells(headers, styles, center_first=False)]
    span_rows = []
    for item in register['items']:
        if item.get('type') == 'subheader':
            row_index = len(data)
            cells = [_para(item.get('text', ''), styles['register_subheader'])]
            cells.extend([''] * (n_cols - 1))
            data.append(cells)
            span_rows.append(row_index)
        else:
            data.append([_para(item.get(key, ''), styles['cell']) for key in keys])

    tbl = LongTable(data, colWidths=_widths('reg'), repeatRows=1)
    style = _row_table_style(len(data), first_col_center=False)
    # Sub-header rows: span all columns and override the zebra shading.
    for row_index in span_rows:
        style.add('SPAN', (0, row_index), (-1, row_index))
        style.add('BACKGROUND', (0, row_index), (-1, row_index), _SUBHEADER_BG)
        style.add('LINEABOVE', (0, row_index), (-1, row_index), 0.6, _BOX)
    tbl.setStyle(style)
    return tbl


def _build_extraction_map_table(styles):
    """Build the additive L1-navigation extraction map (current -> new service).

    Renders :data:`NEW_SERVICE_LOCATION_MAP` (the 12 canonical Position /
    Navigation / Timing behaviours wrapped by the AfsimL1 service) as a
    five-column table mirroring AAP 0.6.1, so the PDF deliverable shows both
    where each behaviour currently lives (file/line + accessor) and the new
    injectable service member it is refactored into.

    The two code-valued columns -- ``Current Location(s)`` and
    ``Current Accessor`` -- are rendered in the monospace style so the PDF shows
    them as code, mirroring the AAP 0.6.1 code-span (backtick) formatting.
    """
    headers = [head for (head, _key) in _NSL_MAP_COLUMNS]
    keys = [key for (_head, key) in _NSL_MAP_COLUMNS]
    mono_keys = {'current_locations', 'current_accessor'}
    data = [_header_cells(headers, styles, center_first=False)]
    for entry in NEW_SERVICE_LOCATION_MAP:
        cells = []
        for key in keys:
            if key in mono_keys:
                cells.append(_mono_para(entry.get(key, ''), styles['mono']))
            else:
                cells.append(_para(entry.get(key, ''), styles['cell']))
        data.append(cells)
    tbl = LongTable(data, colWidths=_widths('nsl_map'), repeatRows=1)
    tbl.setStyle(_row_table_style(len(data), first_col_center=False))
    return tbl


# ---------------------------------------------------------------------------
# Page furniture + story assembly
# ---------------------------------------------------------------------------

def _on_page(canvas, doc):
    """Running header/footer drawn on every page (deterministic content only).

    Left header: the document title.  Right header: the canonical output file
    name.  Footer: the page number (an integer derived from content, so it stays
    byte-stable across renders).
    """
    canvas.saveState()
    page_w, page_h = PAGE_SIZE
    header_y = page_h - 13.0
    canvas.setFont(FONT_SANS, 7)
    canvas.setFillColor(colors.HexColor('#555555'))
    canvas.drawString(MARGIN, header_y, DOCUMENT_TITLE)
    canvas.drawRightString(page_w - MARGIN, header_y, OUTPUT_PDF_FILENAME)
    canvas.setStrokeColor(colors.HexColor('#999999'))
    canvas.setLineWidth(0.4)
    canvas.line(MARGIN, header_y - 2.5, page_w - MARGIN, header_y - 2.5)
    canvas.drawRightString(page_w - MARGIN, 8.0, 'Page %d' % canvas.getPageNumber())
    canvas.restoreState()


def _flags_summary_text():
    """One-line audit-discipline flag legend with live occurrence counts."""
    counts = all_flag_counts()
    parts = ['%s \u00d7%d' % (token, counts.get(token, 0)) for token in FLAG_TOKENS]
    return ('Audit-discipline flags (occurrences across the catalog): '
            + '   |   '.join(parts))


def _layer_for(tables, parent_number):
    """Return the layer table whose ``parent`` matches *parent_number*, or ``None``."""
    for table in tables:
        if table.get('parent') == parent_number:
            return table
    return None


def _append_main_and_layers(story, main, styles):
    """Append a main table followed by its matching Layer-1 and Layer-2 blocks.

    The main table's own ``title`` is rendered first, then the shared ``Table Xa``
    title (once), then each layer's sub-heading and table in Layer-1, Layer-2
    order — reproducing the audit's ``N`` then ``Na`` sequence.
    """
    story.append(_para(main['title'], styles['table_title']))
    story.append(_build_main_table(main, styles))

    parent = main['number']
    layer1 = _layer_for(LAYER1_TABLES, parent)
    layer2 = _layer_for(LAYER2_TABLES, parent)

    if layer1 is not None or layer2 is not None:
        shared_title = (layer1 or layer2)['title']
        story.append(_para(shared_title, styles['table_title']))
    if layer1 is not None:
        story.append(_para(layer1.get('subheading', ''), styles['subheading']))
        story.append(_build_layer1_table(layer1, styles))
    if layer2 is not None:
        story.append(_para(layer2.get('subheading', ''), styles['subheading']))
        story.append(_build_layer2_table(layer2, styles))
    story.append(Spacer(1, 10))


def _build_story(styles):
    """Assemble the full platypus story in the exact audit order.

    Order: front matter (title, scope, legend, footprint, flags) ->
    GROUP 1 header + tables 1/1a/2/2a/3/3a -> GROUP 2 header + tables
    4/4a/5/5a/6/6a -> Coverage & Explicit-Absence Register.
    """
    story = []

    # --- Front matter -------------------------------------------------------
    story.append(_para(DOCUMENT_TITLE, styles['doc_title']))
    story.append(_para(SCOPE_TEXT, styles['scope']))
    story.append(_para(LEGEND_HEADING, styles['legend_heading']))
    for block in LEGEND_BLOCKS:
        story.append(_para(block, styles['legend']))
    story.append(_para(FOOTPRINT_TEXT, styles['footprint']))
    story.append(_para(_flags_summary_text(), styles['flags']))

    # --- Group 1: Core references (tables 1/1a, 2/2a, 3/3a) -----------------
    story.append(_para(GROUP1_HEADER, styles['group_header']))
    story.append(_para(GROUP1_SUBTITLE, styles['group_subtitle']))
    for main in GROUP1_TABLES:
        _append_main_and_layers(story, main, styles)

    # --- Group 2: Indirect references (tables 4/4a, 5/5a, 6/6a) -------------
    story.append(_para(GROUP2_HEADER, styles['group_header']))
    story.append(_para(GROUP2_SUBTITLE, styles['group_subtitle']))
    for main in GROUP2_TABLES:
        _append_main_and_layers(story, main, styles)

    # --- Coverage & Explicit-Absence Register -------------------------------
    story.append(_para(COVERAGE_REGISTER['title'], styles['register_title']))
    story.append(_para(COVERAGE_REGISTER['intro'], styles['register_intro']))
    story.append(_build_register_table(COVERAGE_REGISTER, styles))

    # --- Additive: L1 navigation current -> new service extraction map ------
    # Reproduces AAP 0.6.1 in the PDF deliverable (the mapping documented
    # "twice": in the AAP and in this output document).
    story.append(_para(
        u'L1 NAVIGATION PNT EXTRACTION \u2014 CURRENT LOCATION \u2192 '
        u'NEW SERVICE LOCATION',
        styles['register_title']))
    story.append(_para(
        'Additive extension deliverable: each Position / Navigation / Timing '
        'behaviour wrapped by the AfsimL1 reusable service, showing its current '
        'in-controller location and accessor alongside the new injectable '
        'service member it is refactored into (see also the New Service '
        'Location column on Table 2).',
        styles['register_intro']))
    story.append(_build_extraction_map_table(styles))

    return story


def render_pdf(out_path, repo_root=None):
    """Render the PNT Reference Audit PDF to *out_path*.

    Builds the complete platypus story (all twelve tables in order under both
    group headers, plus the Legend, Footprint summary, audit-discipline flag
    summary, and the Coverage & Explicit-Absence Register, with the additive
    "New Service Location" column on Table 2) and writes it to *out_path*.

    This function performs **no** verification — that gating belongs to
    ``generate.py`` — and assumes the data in ``pnt_data`` is valid.  Fonts are
    registered here (lazily, idempotently) and ``reportlab.rl_config.invariant``
    is enabled so the output is byte-stable across renders.

    Args:
        out_path: filesystem path of the PDF to write.
        repo_root: accepted for signature symmetry with the verifiers and future
            citation-driven rendering; the current renderer sources everything
            from the in-memory ``pnt_data`` structures, so it is unused today.

    Returns:
        str: the *out_path* that was written.
    """
    del repo_root  # currently unused; see docstring.

    register_fonts()
    # Deterministic output: fixed /ID and fixed creation/mod timestamps.
    reportlab.rl_config.invariant = 1

    styles = _build_styles()
    story = _build_story(styles)

    doc = SimpleDocTemplate(
        out_path,
        pagesize=PAGE_SIZE,
        leftMargin=MARGIN,
        rightMargin=MARGIN,
        topMargin=MARGIN + 8.0,
        bottomMargin=MARGIN,
        title=DOCUMENT_TITLE,
        author='ArduPilot PNT Reference Audit',
        subject='Positioning, Navigation, Timing reference audit',
        creator='pnt_render.py (ReportLab)',
    )
    doc.build(story, onFirstPage=_on_page, onLaterPages=_on_page)
    return out_path
