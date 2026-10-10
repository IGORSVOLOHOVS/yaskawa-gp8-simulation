# External Assets

The real Yaskawa documents and the upstream ROS-Industrial GP8 robot model are
**not in this repository**. They are fetched from their own publishers into
`assets/downloads/`, which is listed in `.gitignore`.

## Why they are not in git

- **Size.** The 20 assets are 28.3 MiB (29,661,503 bytes). A clone should not
  carry that, and a binary PDF re-committed on every vendor revision would never
  be diffable anyway.
- **Rights.** The datasheets and manuals are Yaskawa's. This repository
  redistributes nothing: each file is downloaded at `init` time from the URL its
  own publisher serves. The upstream meshes and xacro are BSD-3-Clause from
  `ros-industrial/motoman`, and are likewise fetched, not vendored, here.
- **Provenance.** A recorded SHA-256 per asset is a stronger statement than a
  committed copy: it proves the exact bytes the specification numbers were read
  out of, and it makes a silent vendor republication visible.

## Commands

```bash
./robot init                                        # fetches everything, never fails on a network error
python tools/fetch_external_assets.py               # fetch all; a second run is a no-op
python tools/fetch_external_assets.py --list        # table of ids, kinds, sizes, local paths
python tools/fetch_external_assets.py --check       # verify what is on disk, download nothing
python tools/fetch_external_assets.py --only doc_gp7_gp8_datasheet_logic_control
python tools/fetch_external_assets.py --offline-ok  # failures become warnings, exit code stays 0
python tools/fetch_external_assets.py --update-digests   # re-measure sha256/bytes into the manifest
```

The fetcher is standard library only (`urllib`, `hashlib`, `json`, `argparse`) -
no `pip install` step, because `./robot init` must work on a bare CI runner.
Downloads go to a `.part` file and are renamed only after the whole body has
arrived, so an interrupted run never leaves a half-written asset. A digest
mismatch deletes the bad file and prints both the expected and the served digest.

`./robot init` calls the fetcher with `--offline-ok`: the repository's CI runs
`./robot init` on every push, and a network hiccup at a vendor's CDN must not
turn the build red. With that flag every per-asset problem - unreachable host or
unexpected digest - is printed as a `WARNING` and the exit code stays 0. Without
it, the same problem is an `ERROR` and the exit code is 1, which is the mode to
use when you actually care that the assets are present and correct.

## What each asset is for

The authoritative list, with a `why` and a `licence` note per file, is
`assets/external_assets_manifest.json`. In summary:

### Documents (4 files, 24.2 MiB)

| Asset | Content |
| --- | --- |
| `doc_gp7_gp8_datasheet_logic_control` | GP7/GP8 datasheet. The specification table the solver is built on: payload 8 kg, reach 727 mm, repeatability +/-0.02 mm, robot mass 32 kg in the GP8 column (the 34 kg on the same row is the GP7), per-axis ranges and maximum speeds, allowable wrist moments R/B/T 17/17/10 N m, allowable wrist moments of inertia R/B/T 0.5/0.5/0.2 kg m^2. |
| `doc_gp7_gp8_datasheet_motoman` | The publisher's own copy of the same datasheet, a **different revision**. Both are kept so a changed specification number can be seen rather than assumed. |
| `doc_hw1484385_gp8_gp7_supplemental_instructions` | Yaskawa manual HW1484385, MOTOMAN-GP8/-GP7 Supplemental Instructions: the model-specific addendum to the general instruction manual. |
| `doc_hw1483944_gp8_ar700_gp7_ar900_instructions` | Yaskawa manual HW1483944, MOTOMAN-GP8/AR700 and -GP7/AR900 Instructions: the full manipulator manual, including the dimensional and working-envelope drawings. |

### Upstream robot model (16 files, 4.0 MiB)

`ros-industrial/motoman`, branch `noetic-devel`, package `motoman_gp8_support`:
7 visual meshes, 7 collision meshes and both xacro files (`gp8.xacro`,
`gp8_macro.xacro`). These are the **originals** of the meshes already vendored
in `src/yaskawa_workcell_description/meshes`. Fetching them is what lets the
vendored copies be checked against their source instead of trusted:

```bash
python tools/fetch_external_assets.py
diff <(sha256sum < assets/downloads/motoman_gp8_support/meshes/visual/gp8_base_link.stl) \
     <(sha256sum < src/yaskawa_workcell_description/meshes/visual/gp8_base_link.stl)
```

## Refreshing a digest

When a publisher republishes a document, the next plain fetch fails with a
digest mismatch. That is the intended alarm, not a bug. To accept the new
revision deliberately:

```bash
python tools/fetch_external_assets.py --update-digests --only <asset_id>
git diff assets/external_assets_manifest.json      # review what changed
```

`--update-digests` re-downloads the selected assets, writes the served `sha256`
and `bytes` back into the manifest, and prints `(digest CHANGED)` next to every
asset whose digest moved. Commit the manifest change on its own, with the reason
in the commit message - the digest is the record of which bytes the numbers in
this repository were read from.

## Known limitation: part-level CAD cannot be scripted

Yaskawa's part-level CAD for the GP8 - STEP, IGES and the native solid models -
is **not fetchable by this script**. It is published behind a TraceParts account
or a Yaskawa / Motoman customer login, both of which require an interactive
sign-in and accepting a per-download licence click-through. There is no
anonymous URL to record in the manifest, so there is no entry for it.

What this means in practice: the geometry available here is the ROS-Industrial
STL tessellation (good enough for visualisation, collision checking and
simulation) plus the dimensional drawings in manual HW1483944. Exact machined
part geometry and mass properties per part are not available without a manual,
logged-in download. If someone does download the STEP files by hand, they belong
in `assets/downloads/` as well - never in git.
