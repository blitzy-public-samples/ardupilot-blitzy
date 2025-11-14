# How to contribute to the ArduPilot project?

If you are reading this page, you are possibly interested in contributing to our project.  We have a very active (and friendly) developer group and would love to have the help!  Possible ways you can help:

* Testing the code
* Filing issues on github, when you see a problem (or adding detail to existing issues that effect you)
* Fixing issues
* Adding new features
* Reviewing existing pull requests, and notifying the maintainer if it passes your code review.

# How to make a good bug report...

* Make sure your bug is not a support issue. Support issues should go to [the support forums](http://discuss.ardupilot.org) and include a .bin log file if possible. If you're not sure you have a bug, you should seek support first.
* Search for your bug, make sure it is not already reported. If it is already reported, make a comment on that issue.
* Only report one bug per issue report.
* Write a clear and concise summary. Be specific about what component of the software you are writing about, and follow the convention: "Copter: blah blah blah"
* Write a clear and concise description, with **particularly clear steps** to reproduce the problem. Include logs that display the bug. **Try to report only facts in your issue report, keeping your assumptions out of it.** 
* The majority of issues open now are good or acceptable by these guidelines. Please refer to them for examples.

# Submitting patches

Please see our [wiki article](https://ardupilot.org/dev/docs/submitting-patches-back-to-master.html).

# Documentation Requirements

Comprehensive documentation is essential for ArduPilot's safety-critical nature and effective developer onboarding. All contributions must include appropriate documentation to ensure code maintainability, safety validation, and knowledge transfer to the community.

## Doxygen Documentation for Code

All new public APIs must have Doxygen-style headers including:

* **@brief**: Concise one-line description (mandatory for all public methods, classes, and functions)
* **@details**: Extended explanation for non-trivial functions (required for complex logic or algorithms)
* **@param**: All parameters must be documented with:
  * Direction specifier: `[in]`, `[out]`, or `[in,out]`
  * Units (e.g., meters, degrees, seconds, centidegrees)
  * Valid ranges where applicable
* **@return**: Return value meaning, units, and error conditions
* **@warning**: Safety-critical considerations, flight stability effects, and timing constraints
* **@note**: Usage notes, performance characteristics, and frequency of execution (e.g., "called at 400Hz")

Example:
```cpp
/**
 * @brief Calculate desired attitude from pilot input
 * 
 * @details Converts pilot stick inputs to desired attitude quaternions
 *          while respecting configured limits and expo curves.
 * 
 * @param[in]  pilot_roll_cd   Pilot roll input in centidegrees (-4500 to 4500)
 * @param[in]  pilot_pitch_cd  Pilot pitch input in centidegrees (-4500 to 4500)
 * @param[out] target_attitude Desired vehicle attitude quaternion (NED frame)
 * 
 * @return true if calculation successful, false if limits exceeded
 * 
 * @note Called at main loop rate (typically 400Hz)
 * @warning Modifying rate limits can affect vehicle stability
 */
```

## Inline Comments for Complex Logic

Complex algorithms and safety-critical code paths require explanatory inline comments:

* **Algorithm explanations**: Document non-obvious logic, mathematical operations, and control flow
* **Safety implications**: Clearly mark code affecting flight safety, failsafe behavior, or arming checks
* **Coordinate frame specifications**: Always specify coordinate frames (NED, body frame, earth frame) for position/attitude operations
* **Unit conventions**: Explicitly state units to avoid ambiguity (degrees vs radians, meters vs centimeters, seconds vs milliseconds)

## README Files for Architectural Changes

If adding new features or modifying system architecture:

* **Update or create README.md** in affected module directories (e.g., `libraries/AP_NewModule/README.md`)
* **Include architecture diagrams** using Mermaid syntax for visualization
* **Document usage patterns** with code examples showing typical integration approaches
* **Document integration approaches** explaining how the module interacts with other components
* **Provide configuration parameter references** listing relevant parameters and their effects

## Code Examples Must Compile

All documentation code examples must be:

* **Tested and verified** to compile without errors against the current codebase
* **Representative of actual usage** showing realistic integration patterns
* **Properly commented** to explain the example's purpose and key steps

## Safety-Critical Code Documentation

Functions affecting flight safety require enhanced documentation:

* **@warning tags** explicitly identifying safety implications (e.g., "modifying this affects altitude hold stability")
* **Failure mode descriptions** documenting what happens if the function fails or receives invalid input
* **Fallback behavior** explaining redundancy mechanisms or safe defaults
* **Testing procedures** describing how to safely validate changes without risking vehicles (e.g., SITL testing procedures)

## Pull Request Documentation Checklist

Before submitting your pull request, verify:

* [ ] All new public APIs have Doxygen headers (`@brief`, `@param`, `@return`)
* [ ] Complex algorithms have inline explanatory comments
* [ ] Safety-critical code has `@warning` tags
* [ ] README files updated if architecture changed
* [ ] Code examples compile and have been tested
* [ ] Coordinate frames explicitly specified where relevant (never assume NED vs body frame)
* [ ] Units explicitly stated (never assume meters/centimeters or degrees/radians)
* [ ] Documentation build runs without warnings: `./Tools/scripts/build_docs.sh`

## Documentation Standards

For consistent terminology and style, please reference:

* **Documentation generation guide**: `docs/README.md` for building and previewing documentation
* **ArduPilot glossary**: `docs/glossary.md` for consistent terminology (EKF, AHRS, NED, DCM, etc.)
* **Existing module documentation**: Review similar modules' README files for style consistency

# Development Team

The ArduPilot project is open source and [maintained](https://github.com/ArduPilot/ardupilot#maintainers) by a team of volunteers.

To contribute, you can send a pull request on GitHub.

New developers are recommended to join the `#general` channel on
[Discord](https://ardupilot.org/discord).

You can also join the
[development discussion on Discourse](https://discuss.ardupilot.org/c/development-team),
or [Gitter](https://gitter.im/ArduPilot/ardupilot).

Note that these are NOT for user tech support, and are moderated
for new users to prevent off-topic discussion.
