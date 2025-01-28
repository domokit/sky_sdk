# customer_testing

This tool checks out <https://github.com/flutter/tests> at the commit SHA
specified in [`tests.version`](tests.version), and runs the tests registered to
verify that end-user apps and libraries are working at the current tip-of-tree
of Flutter.

To (locally) test a specific SHA, use `ci.dart`:

```sh
cd dev/customer_testing
dart ci.dart [sha]
```

Or, to update the SHA for our CI, edit and send a PR for
[`tests.version`](tests.version).
