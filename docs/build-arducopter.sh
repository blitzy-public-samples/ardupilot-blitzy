#!/usr/bin/env bash

# ArduCopter Documentation Build Script
# ======================================
# Purpose: Generates ArduCopter-specific Doxygen API documentation
#
# This script builds HTML documentation for ArduCopter vehicle code, including:
# - Flight mode implementations and state machines
# - Scheduler and main loop structure
# - Vehicle-specific MAVLink message handlers
# - Multicopter-specific control algorithms
# - Arming checks and failsafe mechanisms
#
# Output: $DOCS_OUTPUT_BASE/arducopter/html/index.html
# Dependencies: Must run docs/build-libs.sh first to generate library tag file
# Configuration: Uses docs/config/arducopter for ArduCopter-specific Doxygen settings

# Normalize script directory path to absolute location
# This ensures the script works correctly regardless of where it's invoked from
# Pattern: Get directory of script -> convert to absolute path -> store in DIR
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to repository root directory (parent of docs/)
# All documentation builds execute from repository root for consistent paths
cd $DIR/..

# Source documentation build environment configuration
# This loads DOCS_OUTPUT_BASE variable (default: $HOME/build/ArduPilot-docs/)
# which defines where generated documentation files will be written
. docs/setup.sh

# Guard check: Ensure library documentation has been built first
# The libraries tag file enables cross-referencing between ArduCopter code and library APIs
# Doxygen uses this tag file to create hyperlinks from ArduCopter documentation to library docs
# Exit gracefully if libraries not built yet - caller should run docs/build-libs.sh first
if [ ! -f $DOCS_OUTPUT_BASE/tags/libraries ]; 
then
	echo "Must build libraries first"
	exit 0
fi

# Generate ArduCopter documentation using Doxygen
# Configuration: docs/config/arducopter specifies:
#   - Input paths: ArduCopter/ directory and related sources
#   - Output directory: $DOCS_OUTPUT_BASE/arducopter/
#   - Tag file references: Links to libraries tag for cross-referencing
#   - ArduCopter-specific settings: Project name, version, branding
# Output: HTML documentation at $DOCS_OUTPUT_BASE/arducopter/html/index.html
doxygen docs/config/arducopter

# Operational Notes:
# - Run this script after docs/build-libs.sh to ensure tag file exists
# - Rerun when ArduCopter source code changes to update documentation
# - Part of full documentation build: Tools/scripts/build_docs.sh
# - Generated docs include flight modes, scheduler tasks, GCS handlers, and vehicle logic
