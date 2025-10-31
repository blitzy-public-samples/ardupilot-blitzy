#!/usr/bin/env bash

################################################################################
# ArduPilot Library Documentation Build Script
################################################################################
#
# Purpose: Builds comprehensive Doxygen documentation for all shared libraries
#          in the libraries/ directory and generates a tag file for cross-
#          referencing with vehicle-specific documentation.
#
# Build Sequence: This script MUST be run BEFORE any vehicle-specific 
#                 documentation builds (build-arducopter.sh, build-arduplane.sh,
#                 etc.) because those scripts check for the presence of the
#                 libraries tag file as a guard condition.
#
# Output Structure:
#   - HTML documentation: $DOCS_OUTPUT_BASE/libraries/html/
#   - Tag file: $DOCS_OUTPUT_BASE/tags/libraries
#   - Build log: $DOCS_OUTPUT_BASE/build_docs.log (when run via build_docs.sh)
#
# Tag File Purpose: The generated tag file enables Doxygen cross-references
#                   between library documentation and vehicle-specific docs,
#                   allowing vehicle code documentation to link to library APIs.
#
# When to Regenerate:
#   - After adding new libraries
#   - After changing library public APIs
#   - After modifying library documentation comments
#   - As part of the full documentation build (Tools/scripts/build_docs.sh)
#
# Usage: ./docs/build-libs.sh
#
# Requirements:
#   - Doxygen 1.9.8 or later
#   - Graphviz (for class diagrams)
#   - docs/setup.sh must define DOCS_OUTPUT_BASE environment variable
#
################################################################################

# Directory normalization: Get the absolute path to the docs/ directory
# This pattern handles symlinks and relative paths correctly by:
# 1. ${BASH_SOURCE[0]} - Gets the path to this script
# 2. dirname - Extracts the directory containing this script
# 3. cd + pwd - Changes to that directory and gets its absolute path
# Result: Reliable absolute path regardless of how script is invoked
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to repository root directory (one level up from docs/)
# All subsequent paths are relative to repository root
cd $DIR/..

# Source the documentation setup script to initialize environment variables
# Key variable defined: DOCS_OUTPUT_BASE - Root directory for all generated docs
# Default value: $HOME/build/ArduPilot-docs/
# This can be overridden by setting DOCS_OUTPUT_BASE before running this script
. docs/setup.sh

# Build library documentation using Doxygen
# Configuration file: docs/config/libraries
# This configuration:
#   - Scans all libraries in libraries/ directory
#   - Generates HTML documentation with class diagrams
#   - Creates tag file at $DOCS_OUTPUT_BASE/tags/libraries for cross-referencing
#   - Excludes external modules and generated code
# Output: $DOCS_OUTPUT_BASE/libraries/ directory with complete library API docs
doxygen docs/config/libraries
