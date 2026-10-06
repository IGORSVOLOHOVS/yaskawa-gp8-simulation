# Contributing

1. Open an issue on GitHub: what is wanted, why, and numbered acceptance criteria.
2. Branch from `test`: `wmp-<issue>/<type>/<slug>`,
   e.g. `wmp-7/feat/kinematics-solver`.
3. Work. Run `./robot test` until it is green; commit what the
   formatter leaves behind.
4. Commit as `<type>(<scope>): <what changed>`, with `git commit -s`.
5. Push and open a pull request on GitHub with base `test`, first line
   `Issue: #<issue>`. The gate runs by itself; merge it once it is green.
6. `test` into `release` is the lab's move, not yours: manual tests on the
   cell, a confirmation, and the release makes itself.

The whole of it: [BEST_REQUIREMENTS-v4.0.0.md](file:///home/igors/Downloads/BEST_REQUIREMENTS-v4.0.0.md).
