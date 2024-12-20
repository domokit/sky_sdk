# Flutter SDK dependency versions

The files in this directory specifies pinned versions of various
dependencies of the flutter SDK.

The `bin/internal/engine.version` file controls where to find compiled artifacts
of the engine. These artifacts are compiled in the Merge Queue for every commit
in the flutter repository.

The `bin/internal/flutter_packages.version` file specifies the version
of the `flutter/packages` repository to be used for testing. The
`flutter/packages` repository isn't an upstream dependency of
`flutter/flutter`; it is only used as part of the test suite for
verification, and the pinned version here makes sure that tests are
deterministic at each `flutter/flutter` commit.
