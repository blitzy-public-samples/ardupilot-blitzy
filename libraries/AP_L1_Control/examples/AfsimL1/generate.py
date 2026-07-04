"""Harness-gated entry point for the ArduPilot PNT Reference Audit PDF.

This is the **driver** of the recreated PNT (Positioning, Navigation, Timing)
Reference Audit generator.  It ties together the two sibling modules that do
the real work:

* :mod:`pnt_data`   -- the structured, verbatim catalog of every PNT touch-point
  (94 main rows / 282 evidence rows) plus the additive
  ``New Service Location`` mapping onto the ``AfsimL1Behavior`` service surface.
* :mod:`pnt_render` -- the ReportLab renderer (:func:`render_pdf`) *and* the five
  ``verify_*`` integrity checks aggregated by :func:`run_all_verifications`
  (~838 discrete assertions).

Generation contract (validate-first, refuse-on-failure)
-------------------------------------------------------
The audit is **harness-gated**: this driver runs the full verification harness
*before* rendering and **refuses to write the PDF if any check fails**.  On a
clean run it emits, in order and on stdout, exactly two success markers::

    HARNESS PASSED
    PDF written: <absolute output path>

If any verifier reports an error (or raises), the driver prints the errors and a
clear failure banner to stderr, writes **nothing**, and exits non-zero -- so a
stale or inconsistent catalog can never overwrite the committed deliverable.

Configuration (environment variables are the primary contract)
--------------------------------------------------------------
``PNT_REPO_ROOT``
    Absolute path to the ArduPilot repository root.  Used to resolve the cited
    source paths during harness verification.  When unset, the repository root
    is derived relative to this file (it lives four directory levels below the
    root: ``libraries/AP_L1_Control/examples/AfsimL1/generate.py``).

``PNT_OUT``
    Output path for the rendered PDF.  When unset it defaults to the repo-root
    deliverable ``<repo_root>/ArduPilot_PNT_Reference_Audit.pdf`` -- the single
    UPDATE target this generator owns.

Usage::

    # From the repository root (recommended -- makes citation checks resolve):
    PNT_REPO_ROOT="$(pwd)" python3 libraries/AP_L1_Control/examples/AfsimL1/generate.py

    # Or simply, relying on the relative-path fallback for the repo root:
    python3 libraries/AP_L1_Control/examples/AfsimL1/generate.py

Determinism & footprint
------------------------
Output is deterministic -- this driver injects no timestamps, and
:func:`pnt_render.render_pdf` enables ReportLab's ``invariant`` mode, so
re-running yields a byte-stable PDF.  The generator is **build-time only**: it
is never linked into firmware and adds no runtime dependency to any vehicle.

Target toolchain (AAP 0.5.1): Python 3.13.7, ReportLab 4.5.1, Poppler-utils
25.03.0 (downstream verification) and the system DejaVu TrueType fonts.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Sibling-module path bootstrap.
#
# ``pnt_render`` and ``pnt_data`` live in this same directory.  When invoked as
# ``python3 .../generate.py`` Python already places the script's directory on
# ``sys.path``, but callers may also import this module from a different working
# directory (e.g. the standalone-harness usage documented in the Development
# Guide).  Inserting this file's directory defensively guarantees the sibling
# imports resolve in every invocation.  This is the sole pre-import statement;
# it necessitates the ``# noqa: E402`` markers on the two sibling imports below.
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

import pnt_data  # noqa: E402  (import follows the sys.path bootstrap above)
from pnt_render import render_pdf, run_all_verifications  # noqa: E402

# ---------------------------------------------------------------------------
# Configuration constants.
# ---------------------------------------------------------------------------
#: Environment variable naming the ArduPilot repository root.
REPO_ROOT_ENV = "PNT_REPO_ROOT"
#: Environment variable naming the rendered-PDF output path.
OUT_ENV = "PNT_OUT"
#: Default deliverable file name, written at the repository root.
DEFAULT_PDF_NAME = "ArduPilot_PNT_Reference_Audit.pdf"
#: Repository-root sentinels used to sanity-check a resolved root.
REPO_MARKERS = ("libraries", "wscript")
#: Number of directory levels between this file and the repository root
#: (``libraries/AP_L1_Control/examples/AfsimL1/generate.py`` -> root).
REPO_ROOT_DEPTH = 4
#: Success marker printed to stdout once the harness passes.
HARNESS_PASSED_MARKER = "HARNESS PASSED"
#: Prefix of the second success marker (followed by the absolute PDF path).
PDF_WRITTEN_PREFIX = "PDF written: "

_USAGE = (
    "usage: python3 generate.py\n"
    "\n"
    "Validate the PNT Reference Audit catalog and, only if every check "
    "passes, render the PDF.\n"
    "\n"
    "Environment variables:\n"
    "  PNT_REPO_ROOT  Absolute path to the ArduPilot repo root (default: "
    "derived relative to this file).\n"
    "  PNT_OUT        Output PDF path (default: "
    "<repo_root>/" + DEFAULT_PDF_NAME + ").\n"
)


# ---------------------------------------------------------------------------
# Repository-root and output-path resolution.
# ---------------------------------------------------------------------------
def _repo_root_from_file():
    """Return the repository root computed relative to this file.

    ``generate.py`` lives at ``libraries/AP_L1_Control/examples/AfsimL1/`` --
    four levels below the repository root -- so ``parents[REPO_ROOT_DEPTH]``
    yields the root regardless of the current working directory.
    """
    return str(Path(__file__).resolve().parents[REPO_ROOT_DEPTH])


def resolve_repo_root():
    """Resolve the ArduPilot repository root.

    Prefers the ``PNT_REPO_ROOT`` environment variable (required so the
    harness can read cited source files against the live tree); otherwise
    falls back to the path computed relative to this file.  The returned path
    is always absolute and user-expanded.
    """
    env = os.environ.get(REPO_ROOT_ENV)
    if env and env.strip():
        return os.path.abspath(os.path.expanduser(env.strip()))
    return _repo_root_from_file()


def has_repo_marker(repo_root):
    """Return ``True`` if *repo_root* looks like the ArduPilot repository root.

    The check confirms at least one well-known root sentinel
    (:data:`REPO_MARKERS`) exists.  It is advisory only -- the verification
    harness remains the authoritative gate -- but a missing marker is a strong
    early signal that the resolved root is wrong (which would in turn make
    citation checks fail).
    """
    return any(
        os.path.exists(os.path.join(repo_root, marker))
        for marker in REPO_MARKERS
    )


def resolve_out_path(repo_root):
    """Resolve the output PDF path.

    Prefers the ``PNT_OUT`` environment variable; otherwise defaults to the
    repo-root deliverable ``<repo_root>/ArduPilot_PNT_Reference_Audit.pdf``.
    The returned path is always absolute and user-expanded.
    """
    env = os.environ.get(OUT_ENV)
    if env and env.strip():
        return os.path.abspath(os.path.expanduser(env.strip()))
    return os.path.abspath(os.path.join(repo_root, DEFAULT_PDF_NAME))


# ---------------------------------------------------------------------------
# Verification harness.
# ---------------------------------------------------------------------------
def run_harness(repo_root=None):
    """Run the full ``verify_*`` harness and return its error list.

    Delegates to :func:`pnt_render.run_all_verifications`, returning a
    (possibly empty) ``list[str]`` -- an empty list means every integrity gate
    passed and PDF emission may proceed.  When *repo_root* is omitted it is
    resolved via :func:`resolve_repo_root`.

    The aggregator reports data violations as error strings rather than
    exceptions, but *may* raise for genuinely exceptional conditions (e.g. an
    unreadable tree).  Any such exception is converted into a single error
    entry so callers always receive a uniform ``list[str]`` and the
    refuse-on-failure gate treats it as a failure.

    Exposed as a standalone callable so the harness can be exercised without
    rendering (see the Development Guide's verification steps).
    """
    if repo_root is None:
        repo_root = resolve_repo_root()
    try:
        return list(run_all_verifications(repo_root))
    except Exception as exc:
        # Intentional broad catch: any exceptional verifier failure is folded
        # into the returned error list so the refuse-on-failure gate treats it
        # as a failure rather than letting the driver crash with a traceback.
        return [
            "[harness] verifier raised %s: %s"
            % (type(exc).__name__, exc)
        ]


# ---------------------------------------------------------------------------
# CLI driver.
# ---------------------------------------------------------------------------
def _emit_failure(errors, out_path):
    """Print the failure banner and every harness error to stderr."""
    bar = "=" * 72
    print(bar, file=sys.stderr)
    print(
        "HARNESS FAILED: %d integrity check(s) did not pass."
        % len(errors),
        file=sys.stderr,
    )
    print(
        "Refusing to write the PDF: %s" % out_path,
        file=sys.stderr,
    )
    print(bar, file=sys.stderr)
    for err in errors:
        print("  - %s" % err, file=sys.stderr)


def main(argv=None):
    """Validate the catalog and render the PDF; return a process exit code.

    Returns ``0`` on success (harness passed and PDF written), ``1`` when the
    harness fails or rendering raises (no PDF written / a partial write is
    surfaced as an error), and ``2`` for a command-line usage error.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] in ("-h", "--help"):
        sys.stdout.write(_USAGE)
        return 0
    if argv:
        print(
            "error: unexpected argument(s): %s" % " ".join(argv),
            file=sys.stderr,
        )
        sys.stderr.write(_USAGE)
        return 2

    repo_root = resolve_repo_root()
    out_path = resolve_out_path(repo_root)

    if not has_repo_marker(repo_root):
        print(
            "warning: resolved repo root %r lacks expected markers %r; "
            "citation checks may fail -- set PNT_REPO_ROOT to the ArduPilot "
            "repository root." % (repo_root, REPO_MARKERS),
            file=sys.stderr,
        )

    # Progress diagnostic on stderr (keeps stdout to exactly the two success
    # markers).  Sourcing the expected counts from ``pnt_data`` also ties the
    # driver's report to the single source of truth for the catalog size.
    print(
        "harness: validating %d main rows / %d evidence rows against %s"
        % (pnt_data.MAIN_ROW_COUNT, pnt_data.EVIDENCE_ROW_COUNT, repo_root),
        file=sys.stderr,
    )

    # --- Gate: validate FIRST; refuse to write on any failure. -------------
    errors = run_harness(repo_root)
    if errors:
        _emit_failure(errors, out_path)
        return 1

    print(HARNESS_PASSED_MARKER)
    sys.stdout.flush()

    # --- Render + write (only reached on a fully-passing harness). ----------
    try:
        render_pdf(out_path, repo_root)
    except Exception as exc:
        # Report a rendering failure cleanly (no traceback) and exit non-zero;
        # HARNESS PASSED has already been emitted, so the absence of the second
        # marker unambiguously signals the write did not complete.
        print(
            "error: PDF rendering failed: %s: %s"
            % (type(exc).__name__, exc),
            file=sys.stderr,
        )
        return 1

    print("%s%s" % (PDF_WRITTEN_PREFIX, os.path.abspath(out_path)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
