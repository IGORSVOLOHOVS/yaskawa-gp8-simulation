# Study Module Contract

The study layer turns every topic of the MSc semester into a live, inspectable,
adjustable module that runs **on this GP8 robot model**. The rule that keeps it
honest: **the user interface is generated from the code's own self-description**,
so a panel cannot drift away from the formula it claims to show.

```
browser  ──HTTP/JSON──>  study_server.py  ──stdin/stdout JSON lines──>  study_api (C++23)
   ^                                                                        |
   └──────────────────── describe() drives every control ────────────────────┘
```

Python only relays. All numerics live in C++ so a panel can be dragged at
interactive rates and still be benchmarked.

---

## 1. Wire protocol

`study_api` reads **one JSON object per line** on stdin and writes **one JSON
object per line** on stdout. No framing, no length prefix, no pretty printing.

Request:

```json
{"id": 17, "module": "spatial_math", "op": "compose", "args": {"rpy": [0.1, 0.2, 0.3], "p": [0.4, 0, 0.35]}}
```

Success response:

```json
{"id": 17, "ok": true, "module": "spatial_math", "op": "compose", "us": 3, "result": { }}
```

Failure response:

```json
{"id": 17, "ok": false, "module": "spatial_math", "op": "compose", "error": "rpy must have 3 elements"}
```

Rules:

- `id` is echoed back unchanged. It is an opaque integer owned by the caller.
- `us` is the wall-clock microseconds the op itself took, measured with
  `std::chrono::steady_clock` around the call only — not around parsing.
- An unknown `module` or `op`, a missing argument, or an out-of-range value is an
  `ok: false` response. It is **never** a crash, an exception escaping `main`, or
  a non-zero exit. The process stays alive and serves the next line.
- Two reserved ops exist on every build, handled by the registry itself:
  - `{"op": "describe"}` with no `module` → the whole catalogue (all modules).
  - `{"module": "<name>", "op": "describe"}` → that one module's description.

## 2. Self-description

Every module returns a `ModuleDescription`. This is the contract the UI reads;
it is what makes the system transparent rather than a black box with sliders.

```json
{
  "name": "spatial_math",
  "title": "Position, Orientation and Rotation Matrices",
  "course": { "id": 3883, "code": "M-407-01", "name": "Robotics Modelling" },
  "topics": ["Block 1 · Spatial Descriptions and Transformations"],
  "source": "cpp_solver/include/study/spatial_math.hpp",
  "summary": "One sentence on what the module computes.",
  "ops": [
    {
      "name": "compose",
      "title": "Compose homogeneous transforms",
      "formula": "T = T_1 T_2 \\cdots T_n,\\quad T_i = \\begin{bmatrix} R_i & p_i \\\\ 0 & 1 \\end{bmatrix}",
      "explain": "Two or three sentences a student can read instead of the code.",
      "params": [
        {"name": "rpy", "type": "vec3", "unit": "rad", "min": -3.14159, "max": 3.14159, "default": [0,0,0], "label": "Roll-pitch-yaw"}
      ],
      "outputs": [
        {"name": "T", "type": "mat4", "label": "Homogeneous transform"},
        {"name": "det_R", "type": "scalar", "label": "det(R), must stay 1"}
      ]
    }
  ]
}
```

Field rules:

- `course.id` is the Moodle course id, so a topic in the UI links back to the
  course it is assessed in. The six ids in use are listed in
  [COURSE_TOPIC_MAP.md](COURSE_TOPIC_MAP.md).
- `source` is a repository-relative path. The UI shows it next to the panel so
  the student can open the code that produced the number.
- `formula` is LaTeX **without** surrounding `$`. It is rendered by the browser.
- `explain` is for a person, not for a compiler. Write it for someone revising
  the night before the exam.
- `params[].type` and `outputs[].type` are from the type vocabulary below.
  Every numeric param carries `min`, `max`, `default` and a `unit`, because the
  UI builds its slider from those four values alone.
- `unit` is **always emitted**, including for dimensionless quantities, where it
  is the empty string. A sample count, a joint index and a loop gain have no
  unit; saying so explicitly is what lets the UI lay every parameter out the
  same way instead of special-casing a missing key.

## 3. Type vocabulary

| `type` | JSON shape | Used for |
| :--- | :--- | :--- |
| `scalar` | `number` | a single value |
| `bool` | `true`/`false` | a toggle |
| `int` | `number` (integral) | counts, orders, indices |
| `enum` | `string` | one of `options: [...]`, which the param must then carry |
| `vec3` | `[x, y, z]` | position, RPY, angular velocity |
| `vec6` | `[…6…]` | joint vector, twist, wrench |
| `mat3` | row-major `[[…3…], …]` | rotation matrix |
| `mat4` | row-major `[[…4…], …]` | homogeneous transform |
| `matrix` | row-major `[[…], …]` | any other matrix, e.g. the 6×6 Jacobian |
| `series` | `{"x": [...], "y": [...], "label": "…"}` | one curve (step response, Bode magnitude) |
| `series_set` | `[series, …]` | several curves on one axis |
| `points` | `[[x,y,z], …]` | a point cloud, e.g. workspace samples |
| `complex_set` | `[[re, im], …]` | poles, zeros, a Nyquist contour |
| `table` | `{"columns": [...], "rows": [[...], …]}` | a DH table, an error-constant table |
| `text` | `string` | a verdict, a derivation step |

## 4. Writing a module

```cpp
// cpp_solver/include/study/<name>.hpp
#ifndef YASKAWA_STUDY_<NAME>_HPP
#define YASKAWA_STUDY_<NAME>_HPP

#include "study/study_module.hpp"

namespace yaskawa::study {

class <Name>Module final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "<name>"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study
#endif
```

Hard requirements:

1. **No allocation in a hot loop.** Fixed-size `Eigen::Matrix` and
   `std::array` for anything per-sample. A `std::vector` reserved once before a
   sweep is fine; a `std::vector` grown inside the sample loop is not.
2. **`const` and `noexcept` where they are true**, `[[nodiscard]]` on every
   function returning a computed value.
3. **Validation before computation.** Use the `require_*` helpers from
   `study_module.hpp`; they throw `StudyError`, which the registry converts into
   an `ok: false` response.
4. **Reuse what exists.** `yaskawa_kinematics.hpp` already holds the GP8 joint
   limits, FK and the Levenberg-Marquardt IK; `yaskawa_trajectory.hpp` holds the
   coroutine trajectory generator. Extend them, do not fork them.
5. **One physical robot.** Every module describes the *same* GP8: the DH
   parameters, link masses, inertias, gear ratios and joint limits come from
   `study/gp8_model.hpp` and from nowhere else.
6. **Every op is covered by a test** in `cpp_solver/src/test_study_modules.cpp`
   that asserts a property, not a printed string: a rotation matrix is
   orthonormal, an IK solution round-trips through FK to within 1e-9, a stable
   loop's step response settles, a designed filter's magnitude hits its
   specification within tolerance.

## 5. Verification

The same commands CI runs, in the same way:

```bash
./robot build release
./robot test
./robot benchmark
```

`./robot test` must print `ALL TESTS PASSED` and exit 0. A module that cannot be
tested without a GPU, a network call or a camera is in the wrong layer: put the
model in C++ with an injectable data source, and keep the device in the browser.
