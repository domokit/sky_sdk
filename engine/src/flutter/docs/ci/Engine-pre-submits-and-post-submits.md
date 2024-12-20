The Flutter engine repo runs both pre-submit (before merging) and post-submit (after merging) suites of tests and checks, defined in [`.ci.yaml`](https://github.com/flutter/engine/blob/main/.ci.yaml).

> [!TIP]
> See [Cocoon Scheduler](https://github.com/flutter/cocoon/blob/main/CI_YAML.md) for details.

Failure to run appropriate tests for changes can (and do) result in the engine tree turning red, which in turn causes an expensive cascade of developers being blocked, investigative work, reverts, and roll-forwards. Where possible, all attempts should be made to run any/all tests _before_ merging a PR. See nuances (below) for exceptional cases.

<!-- Github Wikis do not support an automatic index, sorry -->

* [Pre-submit](#pre-submit)
* [Post-submit](#post-submit)
  * [Running post-submits eagerly](#running-post-submits-eagerly)

## Pre-submit

Presubmits run (and are required to be passing) to merge a PR:

<img width="839" alt="Checks" src="https://github.com/flutter/flutter/assets/168174/dff8e2b4-2a71-4929-b163-2ac437675380">

<p>

For example, the `linux_host_engine` target above runs based on the configuration in [`ci/builders/linux_host_engine.json`](https://github.com/flutter/engine/blob/458956228dad9837956aeb78b2988879e764a0b2/ci/builders/linux_host_engine.json).

### Nuances

Typically, pre-submits _always_ run on every PR, and don't need any special attention (other than keeping them green). There are two exceptions:

1. Targets that provide a `runIf: ...` configuration
2. Changes that impact Clang Tidy

> [!WARNING]
>
> `runIf: ...` is a powerful (but dangerous) feature that trades predictability for speed.
>
> `runIf` will skip certain targets if a particular file (or commonly, sets of files) are not changed in a given PR.
>
> For example, the [`linux_clang_tidy_presubmit`](https://github.com/flutter/engine/blob/991676f3bc9482eaaeb3764b6b835f0e3ff8b3c5/.ci.yaml#L219-L235) target will not run if only markdown (`*.md`) files are changed.

Clang Tidy, on the other hand, is only run on _files that have changed in a given PR_. For example, if you have:

```h
// impeller/a.h
struct A {}
```

... files that import that header, such as `impeller/foo/bar/baz.cc`, will _not_ be run in pre-submit. This means that changing (or updating the `DEPS` of libraries that provide) headers is _not_ a safe change, and will _not_ be detected in pre-submit. As an example, [#48705](https://github.com/flutter/engine/pull/48705) had to be reverted (despite passing all pre-submit checks), because the Clang Tidy _post-submit_ caught a failure.

See [post-submit](#post-submit) below for options to run post-submits eagerly (i.e. as a pre-submit).

## Post-submit

Some (albeit fewer) targets are configured with the property `presubmit: false`:

```yaml
  - name: Mac mac_clang_tidy
    recipe: engine_v2/engine_v2
    presubmit: false
```

These targets will _not_ show up during a PR, and will not be executed, but can (and do) turn the tree red.

### Running post-submits eagerly

We've intentionally chosen to make it _easier_ to land PRs, at the cost of turning the tree red periodically because post-submit checks catch something that the developer did not intend (or even know about). As a code author (or reviewer), you can optionally turn on post-submits to run eagerly (during pre-submit) by adding the label `test: all` (available only in the `flutter/engine` repo).

Add the label, and push the PR (or a new commit, **the scheduler will not understand the label being added without a commit**):

<img width="311" alt="Screenshot 2023-12-07 at 1 55 48 PM" src="https://github.com/flutter/flutter/assets/168174/9a4d88b8-8e67-4e96-805a-adb21f06a4c2">

<p>

For example, [#48158](https://github.com/flutter/engine/pull/48158) ran _all_ of the checks, including what is typically post-submits:

<img width="319" alt="Screenshot 2023-12-07 at 1 55 39 PM" src="https://github.com/flutter/flutter/assets/168174/67abd263-c516-4240-b82b-a9b691543951">

<p>

> [!WARNING]
> This increases the use of workers/capacity, and should be discouraged to be used on _all_ PRs.
