# `platform/embedder/fixtures`

The files in this directory are golden-file outputss of [`tests`](../tests),
but lack a simple way to be re-generated.

For example, here is what a failure might look like on CI:

```txt
[0;32m[ RUN      ] [mEmbedderTest.VerifyB143464703WithSoftwareBackend
[ERROR:flutter/shell/platform/embedder/tests/embedder_unittests_util.cc(199)] Image did not match expectation.
Expected:/b/s/w/ir/cache/builder/src/out/host_debug_unopt/gen/flutter/shell/platform/embedder/assets/expectation_verifyb143464703_soft_noxform.png
Got:/b/s/w/ir/cache/builder/src/out/host_debug_unopt/gen/flutter/shell/platform/embedder/assets/actual_verifyb143464703_soft_noxform.png

../../flutter/shell/platform/embedder/tests/embedder_unittests.cc:1335: Failure
Value of: ImageMatchesFixture("verifyb143464703_soft_noxform.png", rendered_scene)
  Actual: false
Expected: true
[0;31m[  FAILED  ] [mEmbedderTest.VerifyB143464703WithSoftwareBackend (8077 ms)
[0;32m[----------] [m1 test from EmbedderTest (8080 ms total)

[0;32m[----------] [mGlobal test environment tear-down
[0;32m[==========] [m1 test from 1 test suite ran. (8080 ms total)
[0;32m[  PASSED  ] [m0 tests.
[0;31m[  FAILED  ] [m1 test, listed below:
[0;31m[  FAILED  ] [mEmbedderTest.VerifyB143464703WithSoftwareBackend

 1 FAILED TEST
[13/296] EmbedderTest.VerifyB143464703WithSoftwareBackend returned/aborted with exit code 1 (8226 ms)
[14/296] EmbedderTest.VerifyB143464703WithSoftwareBackend (8484 ms)
[INFO:test_timeout_listener.cc(76)] Test timeout of 300 seconds per test case will be enforced.
[0;33mNote: Google Test filter = EmbedderTest.VerifyB143464703WithSoftwareBackend
[m[0;32m[==========] [mRunning 1 test from 1 test suite.
[0;32m[----------] [mGlobal test environment set-up.
[0;32m[----------] [m1 test from EmbedderTest
[0;33m[ DISABLED ] [mEmbedderTest.DISABLED_CanLaunchAndShutdownMultipleTimes
[0;32m[ RUN      ] [mEmbedderTest.VerifyB143464703WithSoftwareBackend
[ERROR:flutter/shell/platform/embedder/tests/embedder_unittests_util.cc(199)] Image did not match expectation.
Expected:/b/s/w/ir/cache/builder/src/out/host_debug_unopt/gen/flutter/shell/platform/embedder/assets/expectation_verifyb143464703_soft_noxform.png
Got:/b/s/w/ir/cache/builder/src/out/host_debug_unopt/gen/flutter/shell/platform/embedder/assets/actual_verifyb143464703_soft_noxform.png

../../flutter/shell/platform/embedder/tests/embedder_unittests.cc:1335: Failure
Value of: ImageMatchesFixture("verifyb143464703_soft_noxform.png", rendered_scene)
  Actual: false
Expected: true
[0;31m[  FAILED  ] [mEmbedderTest.VerifyB143464703WithSoftwareBackend (8348 ms)
[0;32m[----------] [m1 test from EmbedderTest (8350 ms total)

[0;32m[----------] [mGlobal test environment tear-down
[0;32m[==========] [m1 test from 1 test suite ran. (8350 ms total)
[0;32m[  PASSED  ] [m0 tests.
[0;31m[  FAILED  ] [m1 test, listed below:
[0;31m[  FAILED  ] [mEmbedderTest.VerifyB143464703WithSoftwareBackend
```

In order to update `verifyb143464703_soft_noxform.png`:

```shell
# The examples below assume:
#   $ENGINE = /path/to/engine/src
#   $TARGET = /path/to/engine/src/out/{{host_you_want_to_build}}

# 1. Make sure you have built the engine:
$ ninja -j1000 -C $ENGINE/out/$TARGET

# 2. Run the test locally (assuming you have built the engine).
$ $ENGINE/out/$TARGET/embedder_unittests*

# Or, to run just a single test:
$ $ENGINE/out/$TARGET/embedder_unittests --gtest_filter="EmbedderTest.VerifyB143464703WithSoftwareBackend"

# Or, a suite of tests:
$ $ENGINE/out/$TARGET/embedder_unittests --gtest_filter="EmbedderTest.*"

# 3. Now, copy the output to the golden file (notice the "actual_" prefix):
$ cp \
  $ENGINE/out/$TARGET/gen/flutter/shell/platform/embedder/assets/actual_verifyb143464703_soft_noxform.png \
  $ENGINE/flutter/shell/platform/embedder/fixtures/verifyb143464703_soft_noxform.png
```

⚠️ **WARNING**: Some of the golden tests do not run on non-Linux OSes, which
means its not currently possible to re-generate them on non-Linux OSes
(<https://github.com/flutter/flutter/issues/53784>). So uh, setup a Linux VM
or find a friend with a Linux machine.
