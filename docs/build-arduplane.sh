#!/usr/bin/env bash

#==============================================================================
# ArduPlane Documentation Build Script
#==============================================================================
# Purpose: Generates ArduPlane-specific Doxygen HTML API documentation
#
# This script builds comprehensive API documentation for ArduPlane (fixed-wing
# vehicle code) including flight modes, TECS energy management, L1 navigation,
# and plane-specific systems. It uses the Doxygen configuration tailored for
# ArduPlane in docs/config/arduplane.
#
# Prerequisites:
#   - Library documentation must be built first (creates cross-reference tags)
#   - Run docs/build-libs.sh before this script
#   - Doxygen 1.9.8+ and Graphviz must be installed
#
# Output Location:
#   - $DOCS_OUTPUT_BASE/arduplane/html/ (typically ~/build/ArduPilot-docs/)
#
# Usage:
#   ./docs/build-arduplane.sh
#
# Dependencies:
#   - docs/setup.sh (sets DOCS_OUTPUT_BASE environment variable)
#   - docs/config/arduplane (Doxygen configuration for ArduPlane)
#   - $DOCS_OUTPUT_BASE/tags/libraries (cross-reference tags from library docs)
#
# When to Rerun:
#   - After changes to ArduPlane/ source files
#   - After changes to docs/config/arduplane Doxygen configuration
#   - After library documentation updates (for cross-reference accuracy)
#==============================================================================

# Determine the absolute path to the docs/ directory where this script resides.
# This normalization pattern ensures the script works correctly regardless of
# where it's invoked from (e.g., ./docs/build-arduplane.sh from root, or
# ./build-arduplane.sh from docs/, or via absolute path).
# - BASH_SOURCE[0] is the path to this script
# - dirname extracts the directory component
# - cd + pwd converts to absolute path, resolving symlinks
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to repository root directory (parent of docs/).
# All subsequent paths in this script are relative to the repository root.
# This ensures consistent behavior with other build scripts and allows
# Doxygen to correctly locate source files specified in docs/config/arduplane.
cd $DIR/..

# Source the documentation setup script to initialize environment variables.
# Key variable set by docs/setup.sh:
#   - DOCS_OUTPUT_BASE: Base directory for all generated documentation
#     (default: $HOME/build/ArduPilot-docs/)
# This centralized setup ensures all documentation build scripts use
# consistent output locations and can find each other's generated files
# (e.g., cross-reference tag files).
. docs/setup.sh

# Guard check: Ensure library documentation has been built before proceeding.
# ArduPlane code extensively uses libraries (AP_AHRS, AP_NavEKF3, AP_TECS,
# AC_AttitudeControl, etc.), and the generated documentation must cross-reference
# these library APIs.
#
# The library documentation build (docs/build-libs.sh) generates a Doxygen
# "tag file" at $DOCS_OUTPUT_BASE/tags/libraries containing symbol definitions
# for all library classes, functions, and types. This tag file enables:
#   - Hyperlinked cross-references from ArduPlane code to library documentation
#   - Proper resolution of inherited methods and base classes
#   - Complete class hierarchy diagrams spanning vehicle and library code
#
# If the tag file doesn't exist, library documentation must be generated first:
#   ./docs/build-libs.sh
#
# Exit code 0 (success) is used rather than error exit to allow graceful
# handling in automated build pipelines that may run vehicle builds conditionally.
if [ ! -f $DOCS_OUTPUT_BASE/tags/libraries ]; 
then
	echo "Must build libraries first"
	exit 0
fi

# Invoke Doxygen to generate ArduPlane API documentation.
#
# Configuration File: docs/config/arduplane
#   - Inherits base settings from docs/config/default
#   - Specifies INPUT paths: ArduPlane/ directory for vehicle-specific code
#   - Sets PROJECT_NAME to "ArduPlane" for documentation title
#   - Configures TAGFILES to link to library documentation via tag file
#   - Sets OUTPUT_DIRECTORY to $DOCS_OUTPUT_BASE/arduplane/
#
# Generated Output:
#   - HTML documentation in $DOCS_OUTPUT_BASE/arduplane/html/
#     * index.html - Main entry point
#     * Searchable API reference for all ArduPlane classes and functions
#     * Class hierarchy diagrams (requires Graphviz)
#     * Cross-referenced links to library documentation
#   - XML output for potential further processing
#   - Tag file for this vehicle (if configured for cross-referencing)
#
# Documentation Content Includes:
#   - ArduPlane main class and scheduler tasks
#   - Flight mode implementations (mode*.cpp files)
#   - TECS energy management integration
#   - L1 navigation controller usage
#   - Quadplane-specific logic
#   - Fixed-wing specific failsafes and arming checks
#   - MAVLink message handlers (GCS_Plane)
#   - Parameter definitions and groups
#
# Build Time:
#   - Typically 30-60 seconds depending on system performance
#   - Parses ~100 ArduPlane source files plus referenced library headers
#
# Warnings:
#   - Doxygen warnings about undocumented APIs are printed to stderr
#   - Goal: Zero warnings (indicates complete documentation coverage)
doxygen docs/config/arduplane

