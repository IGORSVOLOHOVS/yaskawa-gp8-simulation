# Study console

Generic web UI for the study layer. Every panel, control, plot and readout is
built from the `describe()` output of the C++ modules; nothing here names a
module.

## Run it

```bash
./robot build release                       # builds cpp_solver/build/study_api
python cpp_solver/web/study_server.py       # http://localhost:8090/  (--port to change)
STUDY_API=/path/to/study_api python cpp_solver/web/study_server.py   # explicit binary
```

The server keeps one `study_api` child alive and relays JSON lines to it. The
old dashboard on port 8080 is untouched. If the binary is missing the console
still loads and shows the build command instead of a blank page.

## How a new module appears

Add the module in C++ and register it; nothing in this folder changes. On the
next reload `GET /api/describe` returns it and the UI:

- groups it under its `course` (name, code, Moodle link from `course.id`);
- builds one control per `params[]` entry from `type`, `min`, `max`, `default`,
  `unit` and `options`;
- renders each `outputs[]` entry by `type` - `scalar`/`int`/`bool`/`text` as
  readouts, `vec3`/`vec6` as component rows, `mat3`/`mat4`/`matrix` as aligned
  grids, `table` as a table, `series`/`series_set` and `complex_set` as canvas
  plots, `points` and `mat4_set` in the 3D scene;
- shows the op title, rendered formula, `explain`, course badge, measured `us`
  and `source` path in the panel header.

## Keyboard and permalinks

`/` focuses the search box, left/right step through ops, and the hash
`#<module>/<op>?args=<json>` restores both the selection and the parameter
values.
