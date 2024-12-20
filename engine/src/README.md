[![OpenSSF Scorecard](https://api.securityscorecards.dev/projects/github.com/flutter/buildroot/badge)](https://api.securityscorecards.dev/projects/github.com/flutter/buildroot)

# buildroot

Build environment for the Flutter engine

This repository is used by the [flutter/engine](https://github.com/flutter/engine) repository.
For instructions on how to use it, see that repository's [CONTRIBUTING.md](https://github.com/flutter/engine/blob/main/CONTRIBUTING.md) file.

To update your checkout to use the latest buildroot, run `gclient sync`.

To submit patches to this buildroot repository, create a branch, push to that branch, then submit a PR on GitHub for that branch.

To point the engine to a new version of buildroot after your patch is merged, update the buildroot hash in the engine's [DEPS file](https://github.com/flutter/engine/blob/main/DEPS).
