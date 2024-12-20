# Git Hooks

The behavior of `git` commands can be customized through the use of "hooks".
These hooks are described in detail in git's
[documentation](https://git-scm.com/book/en/v2/Customizing-Git-Git-Hooks).

`git` looks for an executables by name in the directory specified by
the `core.hooksPath` `git config` setting. The script `setup.py` here points
`core.hooksPath` at this directory. It runs during a `gclient sync` or a
`gclient runhooks`.

The hooks here are implemented in Dart by the program with
entrypoint `bin/main.dart` in this directory. The commands of the program
are the implementation of the different hooks, for example
`bin/main.dart pre-push ...`. Since the Dart program itself isn't an executable,
these commands are invoked by small Python wrapper scripts. These wrapper
scripts have the names that `git` will look for.

## pre-push

This hooks runs when pushing commits to a remote branch, for example to
create or update a pull request: `git push origin my-local-branch`.

The `pre-push` hook runs `ci/clang_tidy.sh`, `ci/pylint.sh` and `ci/format.sh`.
`ci/analyze.sh` and `ci/licenses.sh` are more expensive and are not run.

### Adding new pre-push checks

Since the pre-push checks run on every `git push`, they should run quickly.
New checks can be added by modifying the `run()` method of the `PrePushCommand`
class in `lib/src/pre_push_command.dart`.

## Creating a new hook

1. Check the `git` documentation, and copy `pre-push` into a script with
the right name.
1. Make sure the script has the executable bit set
(`chmod +x <script>`).
1. Add a new `Command` implementation under `lib/src`. Give the new
`Command` the same name as the new hook.
1. Add the new `Command` to the `CommandRunner` in `lib/githooks.dart`.
1. Make sure the script from step (1) is passing the new command to the Dart
program.
