#!/usr/bin/env bash
#
# build-apmrover2.sh - Build Rover-specific Doxygen documentation
#
# Purpose:
#   Generates HTML documentation for ArduPilot Rover (ground vehicle) code
#   using Doxygen with the Rover-specific configuration file.
#
# Usage:
#   ./docs/build-apmrover2.sh
#
# Prerequisites:
#   - Doxygen must be installed
#   - Library documentation must be built first (./docs/build-libs.sh)
#   - This creates cross-reference tags between Rover and library docs
#
# Output:
#   Generated documentation is written to $DOCS_OUTPUT_BASE/apmrover2/html/
#   (typically $HOME/build/ArduPilot-docs/apmrover2/html/)
#
# Dependencies:
#   - docs/setup.sh - Defines DOCS_OUTPUT_BASE environment variable
#   - docs/config/apmrover2 - Rover-specific Doxygen configuration
#   - $DOCS_OUTPUT_BASE/tags/libraries - Tag file for cross-referencing library docs
#

# Get the absolute path to the docs directory, resolving symlinks
# This ensures the script works regardless of where it's called from
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the ArduPilot repository root directory
# All Doxygen paths are relative to repository root, so we must run from there
cd $DIR/..

# Source the documentation setup script to configure environment variables
# This sets DOCS_OUTPUT_BASE which defines where generated documentation is written
# Default: $HOME/build/ArduPilot-docs/ (can be overridden by user)
. docs/setup.sh

# Guard check: Verify that library documentation has been built first
# The libraries tag file is created by docs/build-libs.sh and contains
# cross-reference information that allows Rover documentation to link to
# library classes, functions, and types (e.g., AP_HAL, AP_Motors, etc.)
# Without this tag file, Rover docs would have broken links to library APIs
if [ ! -f $DOCS_OUTPUT_BASE/tags/libraries ]; 
then
	echo "Must build libraries first"
	exit 0
fi

# Generate Rover documentation using Doxygen
# Configuration file: docs/config/apmrover2 contains Rover-specific settings:
#   - INPUT paths: Rover/ directory plus relevant shared libraries
#   - PROJECT_NAME: "APMrover2" (ground vehicle documentation)
#   - OUTPUT_DIRECTORY: $DOCS_OUTPUT_BASE/apmrover2/
#   - TAGFILES: References library tag file for cross-linking
# Output: HTML documentation written to $DOCS_OUTPUT_BASE/apmrover2/html/index.html
doxygen docs/config/apmrover2

#
# Operational Notes:
# ------------------
# - Run this script after building library documentation (./docs/build-libs.sh)
# - Full build orchestration available via ./Tools/scripts/build_docs.sh
# - Rebuild when Rover source code or comments are modified
# - Generated HTML can be viewed locally or deployed to dev.ardupilot.org
# - Build time: typically 1-3 minutes depending on system performance
# - Documentation includes: Rover modes, steering control, motor output,
#   waypoint navigation, arming checks, failsafe behavior, parameter reference

