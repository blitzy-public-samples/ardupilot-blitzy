# ArduPilot API Documentation

## Overview

This directory contains **auto-generated HTML documentation** produced by Doxygen from the ArduPilot source code. The documentation provides comprehensive API reference for all ArduPilot libraries, modules, classes, functions, and data structures.

## ⚠️ Important Notice

**DO NOT EDIT FILES IN THIS DIRECTORY MANUALLY**

All files in this directory are automatically generated from source code comments and should never be manually modified. Any changes you make will be overwritten the next time the documentation is regenerated.

## Contents

When generated, this directory contains:

- **index.html** - Main entry point for browsing the API documentation
- **classes.html** - Alphabetical list of all classes and structures
- **files.html** - List of documented source files
- **namespaces.html** - C++ namespace documentation
- **annotated.html** - Class hierarchy and relationships
- **functions.html** - Cross-reference of all functions
- **search/** - Search functionality and indices
- **html/** - Supporting HTML, CSS, and JavaScript files

## Generating the Documentation

The ArduPilot API documentation is generated using Doxygen with build scripts located in the parent `docs/` directory.

### Prerequisites

Ensure you have Doxygen installed on your system:

```bash
# Ubuntu/Debian
sudo apt-get install doxygen graphviz

# macOS
brew install doxygen graphviz

# Verify installation
doxygen --version
```

### Build Process

#### Step 1: Generate Library Documentation (Required First)

The library documentation must be built first as other components depend on it:

```bash
# From repository root
./docs/build-libs.sh
```

This creates the foundation library documentation that other vehicle-specific builds reference.

#### Step 2: Generate Vehicle-Specific Documentation

After building the libraries, generate documentation for specific vehicle types:

```bash
# ArduCopter (multicopter) documentation
./docs/build-arducopter.sh

# ArduPlane (fixed-wing) documentation
./docs/build-arduplane.sh

# ArduSub (underwater) documentation
./docs/build-ardusub.sh

# APM Rover2 (ground vehicle) documentation
./docs/build-apmrover2.sh
```

#### Step 3: Generate Complete Documentation

To generate all documentation in one command:

```bash
# From repository root
./docs/build-libs.sh && \
./docs/build-arducopter.sh && \
./docs/build-arduplane.sh && \
./docs/build-ardusub.sh && \
./docs/build-apmrover2.sh
```

### Build Configuration

The documentation build process uses:

- **Environment Setup**: `docs/setup.sh` - Sets `DOCS_OUTPUT_BASE` environment variable and creates output directories
- **Doxygen Configs**: `docs/config/` - Contains Doxygen configuration files for each component
- **Output Location**: Controlled by `DOCS_OUTPUT_BASE` environment variable (defaults to build output directory)

You can customize the output location by setting the environment variable before running build scripts:

```bash
export DOCS_OUTPUT_BASE=/path/to/custom/output
./docs/build-libs.sh
```

## Viewing the Documentation

### Option 1: Local Web Browser

After generating the documentation, open the main index file in your web browser:

```bash
# Open in default browser (Linux)
xdg-open docs/api/index.html

# Open in default browser (macOS)
open docs/api/index.html

# Open in default browser (Windows)
start docs/api/index.html

# Or manually navigate to:
file:///path/to/ardupilot/docs/api/index.html
```

### Option 2: Local Web Server

For better performance and full search functionality, serve the documentation via HTTP:

```bash
# Using Python 3
cd docs/api
python3 -m http.server 8080

# Then browse to http://localhost:8080
```

```bash
# Using Node.js http-server
cd docs/api
npx http-server -p 8080

# Then browse to http://localhost:8080
```

## Documentation Structure

The generated API documentation is organized as follows:

### Class Documentation
- **Class Hierarchy**: Shows inheritance relationships between classes
- **Class List**: Alphabetical listing of all documented classes
- **Class Members**: Detailed member documentation with parameters and return types

### Module Documentation
- **Modules**: Logical groupings of related functionality
- **Libraries**: Core ArduPilot libraries (AP_HAL, AP_AHRS, AP_Motors, etc.)
- **Vehicle Code**: ArduCopter, ArduPlane, Rover, ArduSub specific implementations

### File Documentation
- **File List**: All source files with documentation
- **File Members**: Global functions, variables, and macros per file
- **Includes**: Dependency graphs showing header inclusion relationships

### Cross-References
- **Function Index**: All functions across the codebase
- **Variable Index**: Global and member variables
- **Type Index**: Typedefs, enums, and structures
- **Search**: Full-text search across all documentation

## Version Control

### .gitignore Configuration

Generated documentation files are typically **excluded from version control** to:
- Reduce repository size
- Avoid merge conflicts on auto-generated files
- Ensure documentation stays in sync with code

The repository's `.gitignore` should include:

```gitignore
# Doxygen generated documentation
docs/api/*.html
docs/api/*.css
docs/api/*.js
docs/api/*.png
docs/api/search/
docs/api/html/

# Keep only the README
!docs/api/README.md
!docs/api/.gitkeep
```

### Continuous Integration

In CI/CD pipelines, documentation should be:
1. Generated fresh on each build
2. Validated for warnings and errors
3. Optionally published to documentation hosting service

```bash
# CI documentation build example
./docs/build-libs.sh
doxygen_warnings=$(cat doxygen.log | grep -i warning | wc -l)
if [ $doxygen_warnings -gt 0 ]; then
    echo "Documentation has $doxygen_warnings warnings"
    exit 1
fi
```

## Troubleshooting

### "Must build libraries first" Error

If you see this message when building vehicle-specific documentation:

```bash
Must build libraries first
```

**Solution**: Run `./docs/build-libs.sh` before building vehicle documentation.

### Missing Dependencies

If Doxygen fails to generate diagrams:

```bash
sudo apt-get install graphviz
```

### Empty or Incomplete Documentation

If documentation is missing or incomplete:

1. **Check Doxygen version**: Ensure you have Doxygen 1.8.17 or later
2. **Verify source comments**: Ensure source files have proper Doxygen comment blocks
3. **Review Doxygen config**: Check `docs/config/` for configuration issues
4. **Check build logs**: Look for Doxygen warnings during generation

### Configuration Issues

If the build scripts fail:

1. **Ensure you're in repository root**: Scripts assume execution from repository root
2. **Check file permissions**: Build scripts must be executable (`chmod +x docs/build-*.sh`)
3. **Verify Doxygen installation**: Run `doxygen --version`
4. **Check DOCS_OUTPUT_BASE**: Ensure the output directory is writable

## Documentation Standards

The API documentation is generated from specially formatted comments in source code. To contribute to the API documentation:

### Doxygen Comment Syntax

ArduPilot uses the following Doxygen comment styles:

```cpp
/**
 * @brief Brief description of the function
 * 
 * @details Detailed description with implementation notes,
 *          algorithm details, and usage considerations.
 * 
 * @param[in]  param_name  Description with units and valid range
 * @param[out] result      Description of output parameter
 * 
 * @return Description of return value
 * 
 * @note Important implementation notes
 * @warning Safety warnings and side effects
 */
void example_function(int param_name, float& result);
```

### Documentation Guidelines

For information on writing effective API documentation:
- See the **Doxygen Manual**: https://www.doxygen.nl/manual/docblocks.html
- Review **ArduPilot Coding Standards**: https://ardupilot.org/dev/docs/coding-style.html
- Check existing well-documented code for examples

## Related Documentation

This API documentation complements other ArduPilot documentation resources:

- **Developer Wiki**: https://ardupilot.org/dev/ - Development guides and tutorials
- **User Wiki**: https://ardupilot.org/copter/ - End-user documentation
- **Architecture Docs**: `docs/ARCHITECTURE.md` - System architecture overview
- **Build System**: `docs/../BUILD_SYSTEM.md` - Build system documentation
- **Coding Standards**: Online developer wiki - Code style guidelines

## Support and Contributing

### Reporting Documentation Issues

If you find issues with the API documentation:

1. **Missing Documentation**: Add Doxygen comments to source files
2. **Incorrect Information**: Fix the source comments and regenerate
3. **Build Issues**: Report to the ArduPilot developer forum or Discord
4. **Configuration Problems**: Submit issues to the ArduPilot GitHub repository

### Contributing to Documentation

To improve the API documentation:

1. Add comprehensive Doxygen comments to under-documented code
2. Include parameter descriptions with units and valid ranges
3. Document return values and error conditions
4. Add usage examples for complex APIs
5. Follow the documentation standards in the Agent Action Plan

For detailed documentation standards, see Section 0.4 of the Agent Action Plan in the project documentation.

---

**Note**: This README is maintained in version control. The HTML documentation files are auto-generated and should not be committed to the repository.

**Last Updated**: 2024 (ArduPilot Documentation Initiative)
