# Coding Guidelines

**Source:** ANSCER Robotics C++ Coding Guidelines + NASA Power of 10 (Gerard Holzmann, NASA JPL)  
**Applies to:** All C++ modules in the AGV software stack  
**Language:** C++11 or later  

---

## File header template

Every `.cpp` and `.h` file must begin with:

```cpp
/*
 * Name:        FileName.cpp
 * Author:      <name>
 * Date:        YYYY-MM-DD
 * Version:     1.0
 * Description: One-paragraph summary of what this file does
 */
```

---

## Project structure

```
module_name/
├── include/
│   └── ModuleName.h       # All declarations go here
├── src/
│   └── ModuleName.cpp     # All definitions go here
└── config/
    └── params.yaml        # All tunable parameters
```

Rules:
- All `.h` files go in `/include`, all `.cpp` files in `/src`
- Package names use `snake_case` (e.g. `pid_controller`, `diff_drive_sim`)
- One class per `.h`/`.cpp` pair unless classes are trivially small

---

## Naming conventions

| Entity | Convention | Example |
|---|---|---|
| Package / directory | `snake_case` | `agv_sim` |
| `.h` and `.cpp` files | `PascalCase` | `PidController.h` |
| Class | `PascalCase` | `PidController` |
| Member function | `camelCase` | `computeSteering()` |
| Member variable | `m_` prefix + `camelCase` | `m_integralError` |
| Pointer | `p_` prefix + `camelCase` | `p_sensorModel` |
| Thread | `t_` prefix + `camelCase` | `t_controlLoop` |
| Non-member variable | `camelCase` | `lateralError` |
| Constants / enums | `UPPER_SNAKE_CASE` | `MAX_RPM` |

Vague names (`aa`, `temp`, `data`, `test()`, `class1`) are **not allowed**. Every name must communicate its purpose without reading the implementation.

---

## Class design

### OOP is mandatory — no free-function modules

Every module, regardless of size, must be implemented as one or more classes. Use multiple classes when a module has clearly separable responsibilities.

### Interface class pattern

For larger modules, create an interface (mediator) class that owns and coordinates all sub-classes. Sub-classes do not communicate directly with each other — all data flows through the interface.

```
AgvControllerInterface
    ├── receives data from: SensorModel, TapeMap
    ├── forwards to:        PidController, VelocityMapper
    └── returns result to:  DiffDriveModel
```

### Access control

All member variables, member functions, pointers, and instances of other classes must be `private` by default. Only expose as `public` or `protected` when another class explicitly requires access, and document why.

### Abstract base classes

Use abstract base classes whenever two or more modules share the same interface contract. Any class with a `virtual` function must also declare a `virtual` destructor — even if it does nothing.

```cpp
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual double computeOutput(double error) = 0;
    virtual void reset() = 0;
};
```

---

## Declarations vs. definitions

| What | Where |
|---|---|
| Class declaration | `.h` file |
| Member variable declarations | `.h` file |
| Member function declarations | `.h` file |
| Pointer / instance declarations | `.h` file |
| Member function definitions (bodies) | `.cpp` file |
| Inline functions | `.h` file — only for short, trivial logic |

---

## Function rules

### Comment template (above every definition)

```cpp
/**
 * @brief  Computes PID steering correction from lateral error.
 * @param  error  Signed lateral distance from tape center, in meters.
 * @return Steering angular velocity correction in rad/s.
 */
double PidController::computeOutput(double error) { ... }
```

### Function length (NASA Rule 4)

No function body should exceed one printed page (~60 lines). If it does, extract sub-functions. Long functions are a sign of missing class decomposition.

### Return value checking (NASA Rule 7)

Every call to a non-void function must check its return value. Every function must validate its inputs before using them.

```cpp
// Required
bool ok = sensor.initialize();
if (!ok) {
    // handle error
}

// Forbidden
sensor.initialize(); // return value discarded
```

### Assertions (NASA Rule 5)

Every non-trivial function must contain at least two assertions — typically a precondition and a postcondition. Use `assert()` in debug builds. For production, also add explicit error-state handling.

```cpp
double PidController::computeOutput(double error) {
    assert(std::isfinite(error));           // precondition
    // ... computation ...
    assert(std::abs(output) <= m_maxOutput); // postcondition
    return output;
}
```

### Pass by reference for large types

Pass vectors, structs, classes, and other large types by `const&` rather than by value.

```cpp
// Preferred
void updateTapeMap(const std::vector<Waypoint>& waypoints);

// Avoid
void updateTapeMap(std::vector<Waypoint> waypoints);
```

---

## Memory management

### Use smart pointers (NASA Rule 3 + ANSCER)

Prefer `std::unique_ptr` and `std::shared_ptr` over raw `new`/`delete`. Reserve `std::weak_ptr` for breaking ownership cycles.

```cpp
// Preferred
std::unique_ptr<PidController> p_pid = std::make_unique<PidController>(kp, ki, kd);

// Avoid
PidController* p_pid = new PidController(kp, ki, kd);
```

### No dynamic allocation after initialization (NASA Rule 3)

All heap allocations must occur at startup (construction / `initialize()`). No `new` or `malloc` calls inside the control loop or any function called at runtime.

### Memory leak verification

After completing a module, run it under `htop` (or `valgrind`) for a minimum of 20–25 minutes. Memory usage must remain flat. Any monotonic growth is a defect to fix before review.

---

## Control flow (NASA Rules 1 & 2)

### No goto, setjmp, longjmp, or recursion

```cpp
// Forbidden
goto error_label;
setjmp(env);
int factorial(int n) { return n * factorial(n-1); } // recursion
```

Use state machines, loops, or iterative algorithms instead.

### All loops must have a fixed upper bound

Every `for`, `while`, or `do-while` loop must have a statically verifiable termination condition. If the bound is a parameter, assert it is positive before the loop.

```cpp
// Good — fixed bound
for (int i = 0; i < MAX_SENSOR_COUNT; ++i) { ... }

// Requires bound check
assert(waypointCount > 0 && waypointCount <= MAX_WAYPOINTS);
for (int i = 0; i < waypointCount; ++i) { ... }
```

---

## Scope and preprocessor (NASA Rules 6 & 8)

### Declare variables at the narrowest scope

Declare variables as close to their first use as possible, not at the top of a function.

```cpp
// Preferred
for (int i = 0; i < n; ++i) {
    double error = computeError(i);
    ...
}

// Avoid
int i;
double error;
for (i = 0; i < n; ++i) { error = computeError(i); ... }
```

### Preprocessor: headers and simple macros only

The preprocessor (`#define`) must only be used for include guards and simple named constants. No function-like macros. Use `constexpr` and `inline` functions instead.

```cpp
// Forbidden
#define SQUARE(x) ((x)*(x))

// Preferred
constexpr double square(double x) { return x * x; }
```

---

## Pointer discipline (NASA Rule 9)

Limit pointer dereferences to a single level. Avoid `**ptr` and multi-level pointer arithmetic. Do not use function pointers — use virtual functions or `std::function` instead.

```cpp
// Avoid
void process(int** matrix, int rows);

// Preferred
void process(std::vector<std::vector<int>>& matrix);
```

---

## Logging

Use throttled logging to avoid flooding output. Never log inside the inner control loop without throttling.

```cpp
// Preferred (ROS-style; adapt to project logger)
ROS_INFO_THROTTLE(1.0, "Lateral error: %.3f m", error);

// Avoid in hot path
ROS_INFO("Lateral error: %.3f m", error); // floods at 100Hz
```

For non-ROS projects, implement equivalent throttling:

```cpp
// Simple throttle pattern
if (m_logCounter++ % 100 == 0) {  // log every 100 steps = 1s at 100Hz
    LOG_INFO("error=%.3f  output=%.3f  state=%s", error, output, stateName());
}
```

---

## Configuration / parameters

All tunable values (PID gains, speed limits, thresholds, radii) must be read from a config file, not hard-coded. Re-tuning must not require recompilation.

```yaml
# config/params.yaml
pid_controller:
  kp: 2.0
  ki: 0.1
  kd: 0.05
  windup_limit: 1.0
  max_output: 2.0

diff_drive:
  wheel_radius: 0.05
  wheel_base: 0.30
  max_rpm: 150
```

Constants that are truly physical invariants (wheel circumference formula, π) may be `constexpr` in code.

---

## Compile-time warnings (NASA Rule 10)

Compile with maximum warnings enabled. All warnings must be resolved before a module is submitted for review. No warning suppressions without a documented comment explaining why.

Recommended flags:
```
-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror
```

---

## Git and code review

**Branch naming:**
```
feature_<featureName>        e.g. feature_pidController
```

**Commit message format:**
```
git commit -m "agv_sim:: <concise description of change>"
```

**Review process:** submit a pull request within 3–5 days of completing a module. No direct merge to `main` — all merges are done by the reviewer after passing review.

**Repository hygiene:**
- No zip files, build artifacts, or IDE project files committed
- `CMakeLists.txt` kept clean — no commented-out blocks
- All unused includes removed

---

## Quick-reference checklist

Before submitting any module for review, verify:

- [ ] File header present on every `.cpp` and `.h`
- [ ] All declarations in `.h`, all definitions in `.cpp`
- [ ] Every member variable prefixed `m_`, every pointer prefixed `p_`
- [ ] All members `private` unless justified
- [ ] Every non-void function call has its return value checked
- [ ] Every function has ≥ 2 assertions
- [ ] No function body exceeds ~60 lines
- [ ] No `goto`, recursion, or unbounded loops
- [ ] No dynamic memory allocation outside constructors
- [ ] Smart pointers used; no raw `new`/`delete`
- [ ] All loop bounds are fixed or asserted positive
- [ ] All parameters loaded from config file, not hard-coded
- [ ] Logging is throttled in any loop running faster than 1 Hz
- [ ] Compiles with zero warnings at `-Wall -Wextra -Wpedantic`
- [ ] Memory profile flat after 20–25 min runtime
- [ ] Doxygen comment block above every function definition
- [ ] Branch named `feature_<name>` and PR raised
