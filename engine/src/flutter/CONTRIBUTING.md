# Contributing to the Flutter engine

_See also: [Flutter's code of conduct][code_of_conduct]_

## Welcome

For an introduction to contributing to Flutter, see [our contributor
guide][contrib_guide].

For specific instructions regarding building Flutter's engine, see [Setting up
the Engine development environment][engine_dev_setup] on our wiki. Those
instructions are part of the broader onboarding instructions described in the
contributing guide.

## Style

The Flutter engine _generally_ follows Google style for the languages it uses,
with some exceptions.

### C/C++

Follows the [Google C++ Style Guide][google_cpp_style] and is automatically
formatted with `clang-format`.

Some additional considerations that are in compliance with the style guide, but
are worth noting:

#### Judiciously use shared_ptr

The engine currently (as of 2024-05-15) uses `shared_ptr` liberally, which can
be expensive to copy, and is not always necessary.

The C++ style guide has a
[section on ownership and smart pointers][cpp_ownership] worth reading:

> Do not design your code to use shared ownership without a very good reason.
> One such reason is to avoid expensive copy operations, but you should only do
> this if the performance benefits are significant, and the underlying object is
> immutable.

Prefer using `std::unique_ptr` when possible.

#### Judiciously use `auto`

The C++ style guide has a [section on type deduction][cpp_auto] that is worth
reading:

> The fundamental rule is: use type deduction only to make the code clearer or
> safer, and do not use it merely to avoid the inconvenience of writing an
> explicit type. When judging whether the code is clearer, keep in mind that
> your readers are not necessarily on your team, or familiar with your project,
> so types that you and your reviewer experience as unnecessary clutter will
> very often provide useful information to others. For example, you can assume
> that the return type of `make_unique<Foo>()` is obvious, but the return type
> of `MyWidgetFactory()` probably isn't.

Due to our codebase's extensive use of `shared_ptr`, `auto` can have surprising
performance implications. See [#49801][pr_49801] for an example.

#### Linux Embedding

> [!NOTE]  
> The Linux embedding instead follows idiomatic GObject-based C style.

Use of C++ in the [Linux embedding][] is discouraged in that embedding to avoid
creating hybrid code that feels unfamiliar to either developers used to working
with `GObject` or C++ developers.

For example, _do not_ use STL collections or `std::string`, but _do_:

- Use C++ casts (C-style casts are forbidden).
- Use `nullptr` rather than `NULL`.
- Avoid `#define`; for internal constants use `static constexpr` instead.

### Dart

The Flutter engine _intends_ to follow the [Dart style guide][dart_style] but
currently follows the [Flutter style guide][flutter_style], with the following
exceptions:

#### Use of type inference is allowed

The [Dart style guide][dart_inference] only requires explicit types when type
inference is not possible, but the Flutter style guide always requires explicit
types. The engine is moving towards the Dart style guide, but this is a gradual
process. In the meantime, follow these guidelines:

- **Always** annotate when inference is not possible.
- **Prefer** annotating when inference is possible but the type is not
  obvious.

Some cases when using `var`/`final`/`const` is appropriate:

- When the type is obvious from the right-hand side of the assignment:

  ```dart
  // Capitalized constructor name always returns a Foo.
  var foo = Foo();

  // Similar with factory constructors.
  var bar = Bar.create();

  // Literals (strings, numbers, lists, maps, etc) always return the same type.
  var name = 'John Doe';
  var flag = true;
  var numbers = [1, 2, 3];
  var map = {'one': 1, 'two': 2, 'three': 3};
  ```

- When the type is obvious from the method name:

  ```dart
  // toString() always returns a String.
  var string = foo().toString();

  // It's reasonable to assume that length returns an int.
  var length = string.length;
  ```

- When the type is obvious from the context:

  ```dart
  // When variables are in the same scope, reduce() clearly returns an int.
  var list = [1, 2, 3];
  var sum = list.reduce((a, b) => a + b);
  ```

Some cases where an explicit type should be considered:

- When the type is not obvious from the right-hand side of the assignment:

  ```dart
  // What does 'fetchLatest()' return?
  ImageBuffer buffer = fetchLatest();

  // What does this large chain of method calls return?
  Iterable<int> numbers = foo().bar().map((b) => b.baz());
  ```

- When there are semantic implications to the type:

  ```dart
  // Without 'num', the map would be inferred as 'Map<String, int>'.
  const map = <String, num>{'one': 1, 'two': 2, 'three': 3};
  ```

- Or, **when a reviewer requests it**!

  Remember that the goal is to make the code more readable and maintainable, and
  explicit types _can_ help with that. Code can be changed, so it's always
  possible to add or remove type annotations later as the code evolves, so avoid
  bikeshedding over this.

### Java

Follows the [Google Java Style Guide][java_style] and is automatically formatted
with `google-java-format`.

### Objective-C

Follows the [Google Objective-C Style Guide][objc_style], including for
Objective-C++ and is automatically formatted with `clang-format`.

### Python

Follows the [Google Python Style Guide][google_python_style] and is
automatically formatted with `yapf`.

> [!WARNING]
> Historically, the engine grew a number of one-off Python scripts, often as
> part of the testing or build infrastructure (i.e. command-line tools). We are
> instead moving towards using Dart for these tasks, so new Python scripts
> should be avoided whenever possible.

### GN

Automatically formatted with `gn format`.

[cpp_auto]: https://google.github.io/styleguide/cppguide.html#Type_deduction
[cpp_ownership]: https://google.github.io/styleguide/cppguide.html#Ownership_and_Smart_Pointers
[dart_inference]: https://dart.dev/effective-dart/design#types
[dart_style]: https://dart.dev/effective-dart/style
[linux embedding]: shell/platform/linux
[google_cpp_style]: https://google.github.io/styleguide/cppguide.html
[pr_49801]: https://github.com/flutter/engine/pull/49801
[code_of_conduct]: https://github.com/flutter/flutter/blob/master/CODE_OF_CONDUCT.md
[contrib_guide]: https://github.com/flutter/flutter/blob/master/CONTRIBUTING.md
[engine_dev_setup]: docs/contributing/Setting-up-the-Engine-development-environment.md
[flutter_style]: https://github.com/flutter/flutter/wiki/Style-guide-for-Flutter-repo
[objc_style]: https://google.github.io/styleguide/objcguide.html
[java_style]: https://google.github.io/styleguide/javaguide.html
[google_python_style]: https://google.github.io/styleguide/pyguide.html

## Testing

The testing policy for contributing to the flutter engine can be found at the
[Tree Hygiene Wiki][tree_hygiene_wiki]. The summary is that all PR's to the
engine should be tested or have an explicit test exemption.

Because the engine targets multiple platforms the testing infrastructure is
fairly complicated. Here are some more resources to help guide writing tests:

- [Testing the engine wiki][testing_the_engine_wiki] - A guide on writing tests
  for the engine including an overview of the different tests and the different
  technologies the engine uses.
- [`//testing`](./testing) - This is where the `run_tests.py` script is located.
  All tests will have the ability to be executed with `run_tests.py`.
- [`//ci/builders`](./ci/builders) - The JSON files that determine how tests are
  executed on CI.

Tests will be executed on CI, but some tests will be executed before PR's can be
merged (presubmit) and others after they have been merged (postsubmit). Ideally
everything would be presubmit but tests that take up more resources are executed
in postsubmit.

### Skia Gold

The Flutter engine uses [Skia Gold][skia_gold] for image comparison tests which fail if:

- The image is different from an accepted baseline.
- An image is not uploaded but is expected to be (see
  [`dir_contents_diff`][dir_contents_diff]).

[skia_gold]: https://flutter-engine-gold.skia.org/
[dir_contents_diff]: ./tools/dir_contents_diff/

Any untriaged failures will block presubmit and postsubmit tests.

### Example run_tests.py invocation

```sh
# Configure host build for macOS arm64 debug.
$ flutter/tools/gn --runtime-mode=debug --unoptimized --no-lto --mac-cpu=arm64
# Compile default targets (should cover all applicable run_tests.py requirements).
$ ninja -j100 -C out/host_debug_unopt_arm64
# Run all cross-platform C++ tests for the debug build arm64 variant.
$ cd flutter/testing
$ ./run_tests.py --variant=host_debug_unopt_arm64 --type=engine
```

### Directory

| Name                                     | run_tests.py type   | Description                                                     |
| ---------------------------------------- | ------------------- | --------------------------------------------------------------- |
| accessibility_unittests                  | engine              |                                                                 |
| assets_unittests                         | engine              |                                                                 |
| client_wrapper_glfw_unittests            | engine              |                                                                 |
| client_wrapper_unittests                 | engine              |                                                                 |
| common_cpp_core_unittests                | engine              |                                                                 |
| common_cpp_unittests                     | engine              |                                                                 |
| dart_plugin_registrant_unittests         | engine              |                                                                 |
| display_list_rendertests                 | engine              |                                                                 |
| display_list_unittests                   | engine              |                                                                 |
| embedder_a11y_unittests                  | engine              |                                                                 |
| embedder_proctable_unittests             | engine              |                                                                 |
| embedder_unittests                       | engine              |                                                                 |
| felt                                     | n/a                 | The test runner for flutter web. See //lib/web_ui               |
| flow_unittests                           | engine              |                                                                 |
| flutter_tester                           | dart                | Launcher for engine dart tests.                                 |
| fml_unittests                            | engine              | Unit tests for //fml                                            |
| framework_common_unittests               | engine(mac)         |                                                                 |
| gpu_surface_metal_unittests              | engine(mac)         |                                                                 |
| impeller_dart_unittests                  | engine              |                                                                 |
| impeller_golden_tests                    | engine(mac)         | Generates golden images for impeller (vulkan, metal, opengles). |
| impeller_unittests                       | engine              | impeller unit tests and interactive tests                       |
| ios_test_flutter                         | objc                | dynamic library of objc tests to be run with XCTest             |
| jni_unittests                            | engine(not windows) |                                                                 |
| no_dart_plugin_registrant_unittests      | engine              |                                                                 |
| platform_view_android_delegate_unittests | engine(not windows) |                                                                 |
| runtime_unittests                        | engine              |                                                                 |
| shell_unittests                          | engine(not windows) |                                                                 |
| scenario_app                             | android             | Integration and golden tests for Android, iOS                   |
| tonic_unittests                          | engine              | Unit tests for //third_party/tonic                              |
| txt_unittests                            | engine(linux)       |                                                                 |
| ui_unittests                             | engine              |                                                                 |

[tree_hygiene_wiki]: https://github.com/flutter/flutter/wiki/Tree-hygiene#tests
[testing_the_engine_wiki]: https://github.com/flutter/flutter/wiki/Testing-the-engine

## Fuchsia Contributions from Googlers

Googlers contributing to Fuchsia should follow the additional steps at:
go/flutter-fuchsia-pr-policy.
