#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Harness-gated entry point for the ArduPilot PNT Reference Audit PDF generator.

This is the command-line driver of the recreated *PNT Reference Audit* pipeline.
Three sibling modules make up the pipeline, all in this directory:

* ``pnt_data.py``   — the pure-data layer (all audit rows + the additive
  "New Service Location" mapping); no I/O, no import-time side effects.
* ``pnt_render.py`` — the verification harness (nine ``verify_*`` functions
  aggregated by :func:`run_all_verifications`, 979 discrete assertions) *and*
  the deterministic ReportLab renderer (:func:`render_pdf`).
* ``generate.py``   — *this file*: the harness-gated entry point that the
  repository-root ``ArduPilot_PNT_Reference_Audit.pdf`` regeneration step
  invokes.

Contract (see ``blitzy/documentation/Project Guide.md`` §9.4/§9.5)
------------------------------------------------------------------
The renderer is **harness-gated**: this script runs the *entire* verification
harness **first** and **refuses to write (or overwrite) the PDF if any check
fails**.  Only when all 979 integrity assertions pass does it render and write
the deliverable.  On success it prints exactly two markers, in order, to
``stdout``::

    HARNESS PASSED
    PDF written: <absolute-output-path>

On failure it prints every failing check and a clear failure banner to
``stderr``, writes nothing, and exits non-zero.

Environment variables (the primary configuration contract)
----------------------------------------------------------
``PNT_REPO_ROOT``
    Absolute path to the ArduPilot repository root.  Required for the
    harness to resolve the cited ``(path, line_start, line_end)`` source
    snippets against the live tree.  When unset, the repository root is
    derived relative to this file (``parents[4]`` of
    ``libraries/AP_L1_Control/examples/AfsimL1/generate.py``).

``PNT_OUT``
    Output path for the rendered PDF.  When unset, defaults to
    ``<repo_root>/ArduPilot_PNT_Reference_Audit.pdf`` — the repository-root
    deliverable and the *only* location this generator writes.

Exit codes
----------
``0``  success — harness passed and the PDF was written.
``1``  harness failure — one or more integrity checks failed (or the harness
       raised); the PDF was **not** written.
``2``  render failure — the harness passed but the PDF renderer raised.

Determinism
-----------
The renderer enables ``reportlab.rl_config.invariant`` (fixed document ``/ID``
and fixed creation/modification timestamps) and this script emits no wall-clock
content, so re-running against an unchanged tree yields a byte-stable PDF.

Build-time only
---------------
This module is part of the documentation-deliverable tooling (Python 3.13 +
ReportLab 4.5.1).  It is never linked into or executed by ArduPilot firmware
and introduces no firmware runtime dependency.
"""

import argparse
import os
import sys
import traceback

# ---------------------------------------------------------------------------
# Import-resolution shim (pure import resolution — NOT a runtime side effect)
# ---------------------------------------------------------------------------
# The verifier/renderer (``pnt_render``) and the data layer (``pnt_data``) are
# sibling modules in this same directory.  Prepend this file's directory to
# ``sys.path`` so ``import pnt_render`` / ``import pnt_data`` resolve regardless
# of the current working directory or how this module is launched (``python3
# generate.py`` from the repo root, from this directory, or imported as
# ``generate`` by an external verification snippet — see Project Guide §9.5).
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from pnt_render import run_all_verifications, render_pdf  # noqa: E402  (after path shim, by design)
import pnt_data  # noqa: E402  (after path shim; used for the default output filename)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
#: Environment variable naming the repository root (harness citation base).
PNT_REPO_ROOT_ENV = 'PNT_REPO_ROOT'

#: Environment variable naming the output PDF path.
PNT_OUT_ENV = 'PNT_OUT'

#: Default output filename, sourced from the data layer so the two agree.
OUTPUT_PDF_FILENAME = pnt_data.OUTPUT_PDF_FILENAME

#: Number of directory levels from this file up to the repository root.
#: ``libraries/AP_L1_Control/examples/AfsimL1/generate.py`` -> repo root is
#: four ``dirname`` hops above the containing directory (``parents[4]``).
_REPO_ROOT_LEVELS = 4

#: Filesystem markers used to sanity-check a resolved repository root.
_REPO_ROOT_MARKERS = ('wscript', 'libraries')

#: The two success markers, emitted in this exact order on ``stdout``.
HARNESS_PASSED_MARKER = 'HARNESS PASSED'
PDF_WRITTEN_PREFIX = 'PDF written: '


# ---------------------------------------------------------------------------
# Path resolution (pure — no printing, no I/O beyond ``os.path`` checks)
# ---------------------------------------------------------------------------
def _looks_like_repo_root(path):
    """Return ``True`` if *path* contains every expected ArduPilot root marker.

    The markers (``wscript`` and ``libraries/``) both exist at a genuine
    ArduPilot repository root.  This is a diagnostic aid only: the authoritative
    gate is the verification harness, which fails loudly if citations cannot be
    resolved against *path*.
    """
    try:
        return all(os.path.exists(os.path.join(path, marker))
                   for marker in _REPO_ROOT_MARKERS)
    except (OSError, TypeError):
        return False


def resolve_repo_root():
    """Resolve the repository root used to verify cited source snippets.

    Resolution order:

    1. The ``PNT_REPO_ROOT`` environment variable, when set (expanded and made
       absolute).  This is the primary contract and is required when the live
       tree does not sit at the expected relative location.
    2. Otherwise, the path derived relative to this file — four levels above the
       containing directory (``parents[4]``), i.e. the ArduPilot repository root
       that contains ``libraries/AP_L1_Control/examples/AfsimL1/``.

    Returns:
        str: an absolute filesystem path to the resolved repository root.
    """
    env_value = os.environ.get(PNT_REPO_ROOT_ENV)
    if env_value:
        return os.path.abspath(os.path.expanduser(env_value))

    root = _THIS_DIR
    for _ in range(_REPO_ROOT_LEVELS):
        root = os.path.dirname(root)
    return os.path.abspath(root)


def resolve_out_path(repo_root):
    """Resolve the output PDF path.

    Resolution order:

    1. The ``PNT_OUT`` environment variable, when set (expanded and made
       absolute).
    2. Otherwise, ``<repo_root>/ArduPilot_PNT_Reference_Audit.pdf`` — the
       repository-root deliverable, which is the only location this generator
       writes.

    Args:
        repo_root: repository root used to build the default output path.

    Returns:
        str: an absolute filesystem path for the PDF to be written.
    """
    env_value = os.environ.get(PNT_OUT_ENV)
    if env_value:
        return os.path.abspath(os.path.expanduser(env_value))
    return os.path.join(repo_root, OUTPUT_PDF_FILENAME)


# ---------------------------------------------------------------------------
# Harness (importable, side-effect free — see Project Guide §9.5)
# ---------------------------------------------------------------------------
def run_harness(repo_root=None):
    """Run the full ``verify_*`` integrity harness and return its errors.

    This is the programmatic entry point used by the standalone verification
    snippet in the project handbook (``print('errors:', len(run_harness()))``).
    It performs no printing and writes nothing, so it is safe to call from a
    test or an interactive session.

    Args:
        repo_root: repository root used to resolve cited source paths.  When
            ``None``, it is resolved via :func:`resolve_repo_root` (honouring
            ``PNT_REPO_ROOT``).

    Returns:
        list[str]: every failure message from every verifier, in a stable
        order.  An empty list means all 979 integrity checks passed.
    """
    if repo_root is None:
        repo_root = resolve_repo_root()
    return run_all_verifications(repo_root)


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------
def _emit_failure_report(errors, out_path, stream):
    """Print each harness error and a clear refusal banner to *stream*."""
    print('%d integrity check(s) FAILED:' % len(errors), file=stream)
    for message in errors:
        print('  - %s' % message, file=stream)
    _emit_failure_banner(out_path, stream)


def _emit_failure_banner(out_path, stream):
    """Print the 'refusing to write' banner to *stream*."""
    bar = '=' * 72
    print(bar, file=stream)
    print('HARNESS FAILED - refusing to write %s'
          % os.path.basename(out_path), file=stream)
    print('No PDF was written or modified. Fix the reported row(s) in '
          'pnt_data.py and re-run.', file=stream)
    print('(See blitzy/documentation/Project Guide.md 9.4 for the '
          'regeneration contract.)', file=stream)
    print(bar, file=stream)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
def _build_arg_parser():
    """Build the (deliberately minimal) argument parser.

    The configuration contract is environment-variable driven
    (``PNT_REPO_ROOT`` / ``PNT_OUT``); the parser exists so that ``--help``
    documents that contract and unexpected arguments are rejected cleanly.
    """
    parser = argparse.ArgumentParser(
        prog='generate.py',
        description='Harness-gated generator for the ArduPilot PNT Reference '
                    'Audit PDF. Runs the full integrity harness first and '
                    'refuses to write the PDF if any check fails.',
        epilog='Environment variables:\n'
               '  PNT_REPO_ROOT  Repository root for citation verification '
               '(default: derived relative to this file).\n'
               '  PNT_OUT        Output PDF path '
               '(default: <repo_root>/%s).' % OUTPUT_PDF_FILENAME,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    return parser


def main(argv=None):
    """Validate, then (only on success) render and write the deliverable PDF.

    Args:
        argv: optional argument vector (defaults to ``sys.argv[1:]``).

    Returns:
        int: the process exit code (``0`` success, ``1`` harness failure,
        ``2`` render failure).
    """
    _build_arg_parser().parse_args(argv)

    repo_root = resolve_repo_root()
    out_path = resolve_out_path(repo_root)

    # Diagnostics go to stderr so stdout carries only the success markers.
    print('[generate] repo root : %s' % repo_root, file=sys.stderr)
    print('[generate] output    : %s' % os.path.abspath(out_path), file=sys.stderr)
    if not _looks_like_repo_root(repo_root):
        print('[generate] WARNING: %r does not look like an ArduPilot repo '
              'root (missing %s); citation checks will likely fail. Set %s to '
              'the repository root.'
              % (repo_root, ' / '.join(_REPO_ROOT_MARKERS), PNT_REPO_ROOT_ENV),
              file=sys.stderr)

    # -- Gate: run the full harness FIRST; refuse to write on any failure. ----
    try:
        errors = run_harness(repo_root)
    except Exception:  # noqa: BLE001 - a raised verifier is a hard gate failure
        print('[generate] the verification harness raised an unexpected '
              'exception:', file=sys.stderr)
        traceback.print_exc()
        _emit_failure_banner(out_path, sys.stderr)
        return 1

    if errors:
        _emit_failure_report(errors, out_path, sys.stderr)
        return 1

    # Success marker #1 (stdout).
    print(HARNESS_PASSED_MARKER)
    sys.stdout.flush()

    # -- Render + write (only reached when the harness fully passed). ---------
    try:
        written = render_pdf(out_path, repo_root) or out_path
    except Exception:  # noqa: BLE001 - surface a clean failure, distinct code
        print('[generate] PDF rendering failed after the harness passed:',
              file=sys.stderr)
        traceback.print_exc()
        return 2

    # Success marker #2 (stdout).
    print('%s%s' % (PDF_WRITTEN_PREFIX, os.path.abspath(written)))
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
