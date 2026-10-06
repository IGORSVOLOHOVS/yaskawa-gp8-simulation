# Contributing to Yaskawa GP8 Solver (`waam-manipulator`)

All contributions follow revision 4.0.0 of **BEST_REQUIREMENTS-v4.0.0.md**.

## Development Workflow

1. Open an issue on Gitea for every piece of work.
2. Cut a feature branch using the format: `wmp-<issue>/<type>/<description>`.
   Supported types: `feat`, `fix`, `docs`, `refactor`, `test`, `build`, `ci`, `chore`, `hotfix`, `experiment`.
3. Write conventional commits: `type(scope): description`.
4. Run `./robot test` before pushing to ensure all C++26 unit tests pass.
5. Create a Pull Request into branch `test`.
