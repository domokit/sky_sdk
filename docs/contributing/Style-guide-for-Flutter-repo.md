# Style guide for Flutter repo

## Summary

Optimize for readability. Write detailed documentation.
Make error messages useful.
Never use timeouts or timers.
Avoid `is`, `print`, `part of`, `extension` and `_`.

## Introduction

This document contains some high-level philosophy and policy decisions for the Flutter
project, and a description of specific style issues for some parts of the codebase.

The style portion describes the preferred style for code written as part of the Flutter
project (the framework itself and all our sample code). Flutter application developers
are welcome to follow this style as well, but this is by no means required. Flutter
will work regardless of what style is used to author applications that use it.

Code under the `engine` subdirectory uses [other style guides for non-Dart code](../../engine/src/flutter/CONTRIBUTING.md#style).
The language-neutral sections in this document still apply to engine code, however.


## Overview

This document describes our approach to designing and programming Flutter,
from high-level architectural principles all the way to naming rules.
These are our norms, written down so that we can easily convey our shared
understanding with new team members.

The primary goal of these style guidelines is to improve code readability so
that everyone, whether reading the code for the first time or
maintaining it for years, can quickly determine what the code does.
Secondary goals are to design systems that are simple; to increase the
likelihood of catching bugs quickly; and to avoid arguments when there are
disagreements over subjective matters.

For anything not covered by this document, check the
[Dart style guide](https://www.dartlang.org/guides/language/effective-dart/)
for more advice. That document is focused primarily on Dart-specific
conventions, while this document is more about Flutter conventions.

In some cases the Dart style guide differs from the Flutter guide. For Flutter project code,
the Flutter guide governs. The differences are a result of slightly different
priorities. The Flutter guide is designed for making code highly readable
even to people who have never seen the code before and are new to Dart, as
the Flutter framework code will be read millions of times more than it is written.
The Dart guide, on the other hand, is designed to provide a more balanced approach
that assumes that the writing of the code will be a bigger proportion of the
interactions with the code, and that the reader is more experienced with Dart.

### A word on designing APIs

Designing an API is an art. Like all forms of art, one learns by practicing. The best way to get good at designing APIs is to spend a decade or more designing them, while working closely with people who are using your APIs. Ideally, one would first do this in very controlled situations, with small numbers of developers using one's APIs, before graduating to writing APIs that will be used by hundreds of thousands or even millions of developers.

In the absence of one's own experience, one can attempt to rely on the experience of others. The biggest problem with this is that sometimes explaining why an API isn't optimal is a very difficult and subtle task, and sometimes the reasoning doesn't sound convincing unless you already have a lot of experience designing them.

Because of this, and contrary to almost any other situation in engineering, when you are receiving feedback about API design from an experienced API designer, they will sometimes seem unhappy without quite being able to articulate why. When this happens, seriously consider that your API should be scrapped and a new solution found.

This requires a different and equally important skill when designing APIs: not getting attached to one's creations. One should try many wildly different APIs, and then attempt to write code that uses those APIs, to see how they work. Throw away APIs that feel frustrating, that lead to buggy code, or that other people don't like. If it isn't elegant, it's usually better to try again than to forge ahead.

An API is for life, not just for the one PR you are working on.


## Philosophy

### Lazy programming

Write what you need and no more, but when you write it, do it right.

Avoid implementing features you don't need. You can't design a feature
without knowing what the constraints are. Implementing features "for
completeness" results in unused code that is expensive to maintain,
learn about, document, test, etc.

When you do implement a feature, implement it the right way. Avoid
workarounds. Workarounds merely kick the problem further down the
road, but at a higher cost: someone will have to relearn the problem,
figure out the workaround and how to dismantle it (and all the places
that now use it), _and_ implement the feature. It's much better to
take longer to fix a problem properly, than to be the one who fixes
everything quickly but in a way that will require cleaning up later.

You may hear team members say "embrace the [yak shave](http://www.catb.org/jargon/html/Y/yak-shaving.html)!". This is
an encouragement to [take on the larger effort necessary](https://www.youtube.com/watch?v=AbSehcT19u0) to perform a
proper fix for a problem rather than just applying a band-aid.


### Write Test, Find Bug

When you fix a bug, first write a test that fails, then fix the bug
and verify the test passes.

When you implement a new feature, write tests for it. (See also: [Running and writing tests](./testing/Running-and-writing-tests.md), and the section on writing tests below.)

Check the code coverage
to make sure every line of your new code is tested. See also: [Test coverage for package:flutter](./testing/Test-coverage-for-package-flutter.md).

If something isn't tested, it is very likely to regress or to get "optimized away".
If you want your code to remain in the codebase, you should make sure to test it.

Don't submit code with the promise to "write tests later".  Just take the
time to write the tests properly and completely in the first place.


### Avoid duplicating state

There should be no objects that represent live state that reflect
some state from another source, since they are expensive to maintain.
(The Web's `HTMLCollection` object is an example of such an object.)
In other words, **keep only one source of truth**, and **don't replicate
live state**.


### Getters feel faster than methods

Property getters should be efficient (e.g. just returning a cached
value, or an O(1) table lookup). If an operation is inefficient, it
should be a method instead. (Looking at the Web again: we would have
`document.getForms()`, not `document.forms`, since it walks the entire tree).

Similarly, a getter that returns a Future should not kick-off the work
represented by the future, since getters appear idempotent and side-effect free.
Instead, the work should be started from a method or constructor, and the
getter should just return the preexisting Future.


### No synchronous slow work

There should be no APIs that require synchronously completing an
expensive operation (e.g. computing a full app layout outside of the
layout phase). Expensive work should be asynchronous.


### Layers

We use a layered framework design, where each layer addresses a
narrowly scoped problem and is then used by the next layer to solve
a bigger problem. This is true both at a high level (widgets relies
on rendering relies on painting) and at the level of individual
classes and methods (e.g. `Text` uses `RichText` and `DefaultTextStyle`).

Convenience APIs belong at the layer above the one they are simplifying.


### Avoid interleaving multiple concepts together

Each API should be self-contained and should not know about other features.
Interleaving concepts leads to _complexity_.

For example:

- Many Widgets take a `child`. Widgets should be entirely agnostic about the type
of that child. Don't use `is` or similar checks to act differently based on the
type of the child.

- Render objects each solve a single problem. Rather than having a render object
handle both clipping and opacity, we have one render object for clipping, and one
for opacity.

- In general, prefer immutable objects over mutable data. Immutable objects can
be passed around safely without any risk that a downstream consumer will change
the data. (Sometimes, in Flutter, we pretend that some objects are immutable even
when they technically are not: for example, widget child lists are often technically
implemented by mutable `List` instances, but the framework will never modify them
and in fact cannot handle the user modifying them.) Immutable data also turns out
to make animations much simpler through _lerping_.


### Avoid secret (or global) state

A function should operate only on its arguments and, if it is an instance
method, data stored on its object. This makes the code significantly easier
to understand.

For example, when reading this code:

```dart
// ... imports something that defines foo and bar ...

void main() {
  foo(1);
  bar(2);
}
```

...the reader should be confident that nothing in the call to `foo` could affect anything in the
call to `bar`.

This usually means structuring APIs so that they either take all relevant inputs as arguments, or so
that they are based on objects that are created with the relevant input, and can then be called to
operate on those inputs.

This significantly aids in making code testable and in making code understandable and debuggable.
When code operates on secret global state, it's much harder to reason about.


### Prefer general APIs, but use dedicated APIs where there is a reason

For example, having dedicated APIs for performance reasons is fine. If one
specific operation, say clipping a rounded rectangle, is expensive
using the general API but could be implemented more efficiently
using a dedicated API, then that is where we would create a dedicated API.


### Avoid the lowest common denominator

It is common for SDKs that target multiple platforms (or meta-platforms that
themselves run on multiple platforms, like the Web) to provide APIs that
work on all their target platforms. Unfortunately, this usually means that
features that are unique to one platform or another are unavailable.

For Flutter, we want to avoid this by explicitly aiming to be the best way
to develop for each platform individually. Our ability to be used cross-
platform is secondary to our ability to be used on each platform. For example,
[TextInputAction](https://master-api.flutter.dev/flutter/services/TextInputAction-class.html)
has values that only make sense on some platforms. Similarly, our platform
channel mechanism is designed to allow separate extensions to be created on
each platform.


### Avoid APIs that encourage bad practices

For example, don't provide APIs that walk entire trees, or that encourage
O(N^2) algorithms, or that encourage sequential long-lived operations where
the operations could be run concurrently.

In particular:

  - String manipulation to generate data or code that will subsequently
    be interpreted or parsed is a bad practice as it leads to code
    injection vulnerabilities.

  - If an operation is expensive, that expense should be represented
    in the API (e.g. by returning a `Future` or a `Stream`).  Avoid
    providing APIs that hide the expense of tasks.


### Avoid exposing API cliffs

Convenience APIs that wrap some aspect of a service from one environment
for exposure in another environment (for example, exposing an Android API
in Dart), should expose/wrap the complete API, so that there's no cognitive cliff
when interacting with that service (where you are fine using the exposed
API up to a point, but beyond that have to learn all about the underlying
service).


### Avoid exposing API oceans

APIs that wrap underlying services but prevent the underlying API from
being directly accessed (e.g. how `dart:ui` exposes Skia) should carefully
expose only the best parts of the underlying API. This may require refactoring
features so that they are more usable. It may mean avoiding exposing
convenience features that abstract over expensive operations unless there's a
distinct performance gain from doing so. A smaller API surface is easier
to understand.

For example, this is why `dart:ui` doesn't expose `Path.fromSVG()`: we checked,
and it is just as fast to do that work directly in Dart, so there is no benefit
to exposing it. That way, we avoid the costs (bigger API surfaces are more
expensive to maintain, document, and test, and put a compatibility burden on
the underlying API).


### Avoid heuristics and magic

Predictable APIs that the developer feels gives them control are generally preferred
over APIs that mostly do the right thing but don't give the developer any way to adjust
the results.

Predictability is reassuring.


### Solve real problems by literally solving a real problem

Where possible, especially for new features, you should partner with a real
customer who wants that feature and is willing to help you test it. Only by
actually using a feature in the real world can we truly be confident that a
feature is ready for prime time.

Listen to their feedback, too. If your first customer is saying that your
feature doesn't actually solve their use case completely, don't dismiss their
concerns as esoteric. Often, what seems like the problem when you start a
project turns out to be a trivial concern compared to the real issues faced
by real developers.


### Get early feedback when designing new APIs

If you're designing a new API or a new feature, consider [writing a design doc](Design-Documents.md).
Then, get feedback from the relevant people, e.g. post it on the [relevant chat channel](Chat.md#existing-channels).


### Start designing APIs from the closest point to the developer

When we create a new feature that requires a change to the entire stack, it's tempting to design the lowest-level API first, since that's the closest to the "interesting" code (the "business end" of the feature, where we actually do the work). However, that then forces the higher level APIs to be designed against the lower-level API, which may or may not be a good fit, and eventually the top-level API, which developers will primarily be using, may be forced to be a tortured and twisted mess (either in implementation or in terms of the exposed API). It may even be that the final API doesn't fit how people think about the problem or solve their actual issues, but instead merely exposes the lowest-level feature almost verbatim.

Instead, always design the top-level API first. Consider what the most ergonomic API would be at the level that most developers will be interacting with it. Then, once that API is cleanly designed and usability-tested, build the lower levels so that the higher level can be layered atop.

Concretely, this means designing the API at the `material` or `widgets` layer first, then the API in the `rendering`, `scheduler`, or `services` layer, then the relevant binding, then the `dart:ui` API or the message channel protocol, then the internal engine API or the plugin API. (The details may vary from case to case.)


### Only log actionable messages to the console

If the logs contain messages that the user can safely ignore, then they will do so, and eventually their logs
will be so chatty and verbose that they will miss the critical messages. Therefore, only log actual errors and
actionable warnings (warnings that can always be dealt with and fixed).

Never log "informational" messages by default. It is possible that it may be useful to have messages on certain topics while debugging those topics. To deal with that, have debug flags you can enable that enable extra logging for particular topics. For example, setting `debugPrintLayouts` to true enables logging of layouts.

This also applies to our unopt builds. It's annoying for other people on the team to have to wade through messages that aren't directly relevant to their work. Rely on feature flags, not verbosity levels, when deciding
to output messages. The one exception to this is reporting useful milestones; for example, the `flutter` tool in
verbose mode (`-v`) reports meaningful steps that it is executing because those are almost always useful.


### Error messages should be useful

Every time you find the need to report an error (e.g. throwing an exception in the framework, handling some bad state in the engine, reporting a syntax error in the Dart compiler, etc), consider how you can make this the most useful and helpful error message ever.

Put yourself in the shoes of whoever sees that error message. Why did they see it? What can we do to help them? They are at a crossroads, having seen your error message: they can either get frustrated and hate Flutter, or they can feel thankful that the error helped them resolve an actual issue. **Every error message is an opportunity to make someone love our product.**

### Template values should set developers up for success

Template defaults should focus on providing the best developer experience. Templates should help developers understand the code, be easy to run now and support in the future. Help developers by picking dependencies that are broadly used and/or broadly supported and by leaving [comments that are helpful](#leave-breadcrumbs-in-the-comments).

See flutter create's templates for an example.

## Policies

This section defines some policies that we have decided to honor. In the absence of a very specific policy in this section, the general philosophies in the section above are controlling.

### Plugin compatibility

We guarantee that a plugin published with a version equal to or greater than 1.0.0 will require no more recent a version of Flutter than the latest stable release at the time that the plugin was released. (Plugins may support older versions too, but that is not guaranteed.)

### Workarounds

We are willing to implement temporary (one week or less) workarounds (e.g. `//ignore` hacks) if it helps a high profile developer or prolific contributor with a painful transition. Please contact @Hixie (ian@hixie.ch) if you need to make use of this option.

### Avoid abandonware

Code that is no longer maintained should be deleted or archived in some way that clearly indicates
that it is no longer maintained.

For example, we delete rather than commenting out code. Commented-out code will bitrot too fast to be
useful, and will confuse people maintaining the code.

Similarly, all our repositories should have an owner that does regular triage of incoming issues and PRs,
and fixes known issues. Repositories where nobody is doing triage at least monthly, preferably more often,
should be deleted, hidden, or otherwise archived.

### Widget libraries follow the latest OEM behavior

For our material and cupertino libraries, we generally implement the latest behavior unless doing so
would be a seriously disruptive breaking change. For example, we use the latest stylings for iOS
switch controls, but when Material Design introduced a whole new type of button, we created a new
widget for that rather than updating the existing buttons to have the new style.

### Code that is not copyrighted "The Flutter Authors"

All code in all Flutter repositories must be contributed by developers who have signed [the Google CLA](https://cla.developers.google.com/), and must be licensed using our normal BSD license with a copyright referencing "The Flutter Authors", except if it is "third party code".

"Third party code" that is not part of a Dart package must be in a subdirectory of a `third_party` directory located:

* [/third_party](../../third_party) for the Flutter Framework
* [/engine/src/flutter/third_party](../../engine/src/flutter/third_party) for the Flutter Engine.

The subdirectory in question must contain a `LICENSE` file that details the license covering that code and a `README` describing the provenance of that code.

"Third party code" that is part of a Dart package and is not Dart code must be in a subdirectory of a `third_party` directory at the root of the package, and the subdirectory in question must contain a `LICENSE` file that details the license covering that code and a `README` describing the provenance of that code. The license must then also be duplicated into the package's `LICENSE` file using the syntax described in the [LicenseRegistry](https://master-api.flutter.dev/flutter/foundation/LicenseRegistry-class.html) API docs.

"Third party code" that is part of a Dart package and is Dart code must be in a subdirectory of the package's `lib/src/third_party` directory, and the subdirectory in question must contain a `LICENSE` file that details the license covering that code and a `README` describing the provenance of that code. The license must then also be duplicated into the package's `LICENSE` file using the syntax described in the [LicenseRegistry](https://master-api.flutter.dev/flutter/foundation/LicenseRegistry-class.html) API docs.

All licenses included in this manner must have been reviewed and determined to be legally acceptable licenses.

All such "third party code" must either be a fork for which we take full responsibility, or there must be an automated rolling mechanism that keeps the code up to date when the upstream source changes.

In general it is _strongly_ recommended that we avoid any such code unless strictly necessary. In particular, we aim for all _Flutter Framework_ code in the flutter/flutter repository to be single-licensed.

## Documentation (dartdocs, javadocs, etc)

We use "dartdoc" for our Dart documentation, and similar technologies for the documentation
of our APIs in other languages, such as ObjectiveC and Java. All public members in Flutter
libraries should have a documentation.

In general, follow the
[Dart documentation guide](https://www.dartlang.org/effective-dart/documentation/#doc-comments)
except where that would contradict this page.

### Answer your own questions straight away

When working on Flutter, if you find yourself asking a question about
our systems, please place whatever answer you subsequently discover
into the documentation in the same place where you first looked for
the answer. That way, the documentation will consist of answers to real
questions, where people would look to find them. Do this right away;
it's fine if your otherwise-unrelated PR has a bunch of documentation
fixes in it to answer questions you had while you were working on your PR.

We try to avoid reliance on "oral tradition". It should be possible
for anyone to begin contributing without having had to learn all the
secrets from existing team members. To that end, all processes should
be documented (typically on the wiki), code should be self-explanatory
or commented, and conventions should be written down, e.g. in our style
guide.


### Avoid useless documentation

If someone could have written the same documentation without knowing
anything about the class other than its name, then it's useless. Avoid checking
in such documentation.

Example (from [`CircleAvatar`](http://docs.flutter.io/flutter/material/CircleAvatar-class.html)):

```dart
// BAD:

/// The background color.
final Color backgroundColor;

/// Half the diameter of the circle.
final double radius;


// GOOD:

/// The color with which to fill the circle.
///
/// Changing the background color will cause the avatar to animate to the new color.
final Color backgroundColor;

/// The size of the avatar.
///
/// Changing the radius will cause the avatar to animate to the new size.
final double radius;
```

### Writing prompts for good documentation

If you are having trouble coming up with useful documentation, here are some prompts that might help you write more detailed prose:

 * If someone is looking at this documentation, it means that they have a question which they couldn't answer by guesswork or by looking at the code. What could that question be? Try to answer all questions you can come up with.

 * If you were telling someone about this property, what might they want to know that they couldn't guess? For example, are there edge cases that aren't intuitive?

 * Consider the type of the property or arguments. Are there cases that are outside the normal range that should be discussed? e.g. negative numbers, non-integer values, transparent colors, empty arrays, infinities, NaN, null? Discuss any that are non-trivial.

 * Does this member interact with any others? For example, can it only be non-null if another is null? Will this member only have any effect if another has a particular range of values? Will this member affect whether another member has any effect, or what effect another member has?

 * Does this member have a similar name or purpose to another, such that we should point to that one, and from that one to this one? Use the `See also:` pattern.

 * Are there timing considerations? Any potential race conditions?

 * Are there lifecycle considerations? For example, who owns the object that this property is set to? Who should `dispose()` it, if that's relevant?

 * What is the contract for this property/method? Can it be called at any time? Are there limits on what values are valid? If it's a `final` property set from a constructor, does the constructor have any limits on what the property can be set to? If this is a constructor, are any of the arguments not nullable?

 * If there are `Future` values involved, what are the guarantees around those? Consider whether they can complete with an error, whether they can never complete at all, what happens if the underlying operation is canceled, and so forth.


### Introduce terms as if every piece of documentation is the first the reader has ever seen

It's easy to assume that the reader has some basic knowledge of Dart or Flutter when writing API documentation.

Unfortunately, the reality is that everyone starts knowing nothing, and we do not control where they will begin their journey.

For this reason, avoid using terms without first defining them, unless you are linking to more fundamental documentation that defines that term without reference to the API you are documenting.

For example, a fancy widget in the Material library can refer to the `StatefulWidget` documentation and assume that the reader either knows about the `StatefulWidget` class, or can learn about it by following the link and then later returning to the documentation for the fancy widget. However, the documentation for the `StatefulWidget` class should avoid assuming that the reader knows what a `State` class is, and should avoid deferring to it for its definition, because `State` could is likely to defer back to `StatefulWidget` and the reader would be stuck in a loop unable to grasp the basic principles. This is the documentation equivalent of a bootstrapping problem.

Another way to express this is that API documentation should follow a similar layering philosophy as code. The goal of documentation is not just to act as a refresher for experts, but to act as a tutorial for new developers.


### Avoid empty prose

It's easy to use more words than necessary. Avoid doing so
where possible, even if the result is somewhat terse.

```dart
// BAD:

/// Note: It is important to be aware of the fact that in the
/// absence of an explicit value, this property defaults to 2.

// GOOD:

/// Defaults to 2.
```

In particular, avoid saying "Note:", or starting a sentence with "Note that". It adds nothing.


### Leave breadcrumbs in the comments

This is especially important for documentation at the level of classes.

If a class is constructed using a builder of some sort, or can be
obtained via some mechanism other than merely calling the constructor,
then include this information in the documentation for the class.

If a class is typically used by passing it to a particular API, then
include that information in the class documentation also.

If a method is the main mechanism used to obtain a particular object,
or is the main way to consume a particular object, then mention that
in the method's description.

Typedefs should mention at least one place where the signature is used.

These rules result in a chain of breadcrumbs that a reader can follow
to get from any class or method that they might think is relevant to
their task all the way up to the class or method they actually need.

Example:

```dart
// GOOD:

/// An object representing a sequence of recorded graphical operations.
///
/// To create a [Picture], use a [PictureRecorder].
///
/// A [Picture] can be placed in a [Scene] using a [SceneBuilder], via
/// the [SceneBuilder.addPicture] method. A [Picture] can also be
/// drawn into a [Canvas], using the [Canvas.drawPicture] method.
abstract class Picture ...
```

You can also use "See also" links, as in:

```dart
/// See also:
///
/// * [FooBar], which is another way to peel oranges.
/// * [Baz], which quuxes the wibble.
```

Each line should end with a period. Prefer "which..." rather than parentheticals on such lines.
There should be a blank line between "See also:" and the first item in the bulleted list.


### Refactor the code when the documentation would be incomprehensible

If writing the documentation proves to be difficult because the API is
convoluted, then rewrite the API rather than trying to document it.


### Canonical terminology

The documentation should use consistent terminology:

 * _method_ - a member of a class that is a non-anonymous closure
 * _function_ - a callable non-anonymous closure that isn't a member of a class
 * _parameter_ - a variable defined in a closure signature and possibly used in the closure body.
 * _argument_ - the value passed to a closure when calling it.

Prefer the term "call" to the term "invoke" when talking about jumping to a closure.

Prefer the term "member variable" to the term "instance variable" when talking about variables associated with a specific object.

Typedef dartdocs should usually start with the phrase "Signature for...".


### Use correct grammar

Avoid starting a sentence with a lowercase letter.

```dart
// BAD

/// [foo] must not be null.

// GOOD

/// The [foo] argument must not be null.
```

Similarly, end all sentences with a period.


### Use the passive voice; recommend, do not require; never say things are simple

Never use "you" or "we". Avoid the imperative voice. Avoid value judgements.

Rather than telling someone to do something, use "Consider", as in "`To obtain the foo, consider using [bar].`".

In general, you don't know who is reading the documentation or why. Someone could have inherited a terrible codebase and be reading our documentation to find out how to fix it; by saying "you should not do X" or "avoid Y" or "if you want Z", you will put the reader in a defensive state of mind when they find code that contradicts the documentation (after all, they inherited this codebase, who are we to say that they're doing it wrong, it's not their fault).

For similar reasons, never use the word "simply", or say that the reader need "just" do something, or otherwise imply that the task is easy.
By definition, if they are looking at the documentation, they are not finding it easy.


### Provide sample code

Sample code helps developers learn your API quickly. Writing sample code also helps you think through how your API is going to be used by app developers.

Sample code should go in a documentation comment that typically begins with `/// {@tool dartpad}`, and ends with `/// {@end-tool}`, with the example source and corresponding tests placed in a file under [the API examples directory](https://github.com/flutter/flutter/blob/main/examples/api). This will then be checked by automated tools, and formatted for display on the API documentation web site [api.flutter.dev](https://api.flutter.dev). For details on how to write sample code, see [the API example documentation](https://github.com/flutter/flutter/blob/main/examples/api/README.md#authoring).

#### Provide full application samples.

Our UX research has shown that developers prefer to see examples that are in the context of an entire app. So, whenever it makes sense, provide an example that can be presented as part of an entire application instead of just a snippet that uses the `{@tool snippet}` or &#96;&#96;&#96;dart ... &#96;&#96;&#96; indicators.

An application sample can be created using the `{@tool dartpad}` ... `{@end-tool}` or `{@tool sample}` ... `{@end-tool}` dartdoc indicators. See [here](https://github.com/flutter/flutter/blob/main/examples/api/README.md#authoring) for more details about writing these kinds of examples.

Dartpad examples (those using the dartdoc `{@tool dartpad}` indicator) will be presented on the [API documentation website](https://api.flutter.dev) as an in-page executable and editable example. This allows developers to interact with the example right there on the page, and is the preferred form of example. Here is [one such example](https://api.flutter.dev/flutter/widgets/AnimatedSwitcher-class.html#widgets.AnimatedSwitcher.1).

For examples that don't make sense in a web page (for example, code that interacts with a particular platform feature), application examples (using the dartdoc `{@tool sample}` indicator) are preferred, and will be presented on the API documentation website along with information about how to instantiate the example as an application that can be run.

Supported IDEs viewing the Flutter source code using the Flutter plugin also offer the option of creating a new project with either kind of example.

### Provide illustrations, diagrams or screenshots

For any widget that draws pixels on the screen, showing how it looks like in its API doc helps developers decide if the widget is useful and learn how to customize it. All illustrations should be easily reproducible, e.g. by running a Flutter app or a script.

Examples:

* A diagram for the AppBar widget

![AppBar diagram](https://flutter.github.io/assets-for-api-docs/assets/material/app_bar.png)


* A screenshot for the Card widget

![Card screenshot](https://user-images.githubusercontent.com/348942/28338544-2c3681b8-6bbe-11e7-967d-fcd7c830bf53.png)

When creating diagrams, make sure to provide alternative text [as described in the HTML specification](https://html.spec.whatwg.org/multipage/images.html#alt).


### Link to Widget of the Week videos

Link to a widget's Flutter Widget of the Week video if it has one:

```dart
/// {@youtube 560 315 https://www.youtube.com/watch?v=<id>}
```

The first two arguments are the video's width and height. These should be `560` and `315` respectively.


### Clearly mark deprecated APIs

We have conventions around deprecation. See the [Tree Hygiene](Tree-hygiene.md#deprecations) page for more details.


### Use `///` for public-quality private documentation

In general, private code can and should also be documented. If that documentation is of good enough
quality that we could include it verbatim when making the class public (i.e. it satisfies all the
style guidelines above), then you can use `///` for those docs, even though they're private.

Documentation of private APIs that is not of sufficient quality should only use `//`. That way, if
we ever make the corresponding class public, those documentation comments will be flagged as missing,
and we will know to examine them more carefully.

Feel free to be conservative in what you consider "sufficient quality". It's ok to use `//` even if
you have multiple paragraphs of documentation; that's a sign that we should carefully rereview the
documentation when making the code public.

### Dartdoc templates and macros

Dartdoc supports creating templates that can be reused in other parts of the code. They are defined
like so:

```dart
/// {@template <id>}
/// ...
/// {@endtemplate}
```

and used via:

```dart
/// {@macro <id>}
```

The `<id>` should be a unique identifier that is of the form `flutter.library.Class.member[.optionalDescription]`.

For example:

```
// GOOD:
/// {@template flutter.rendering.Layer.findAnnotations.aboutAnnotations}
/// Annotations are great!
/// {@endtemplate

// BAD:
/// {@template the_stuff!}
/// This is some great stuff!
/// {@endtemplate}
```

The `optionalDescription` component of the identifier is only necessary if there is more than one
template defined in one Dartdoc block. If a symbol is not part of a library, or not part of a class, then
just omit those parts from the ID.

### Dartdoc-specific requirements

The first paragraph of any dartdoc section must be a short self-contained sentence that explains the purpose
and meaning of the item being documented. Subsequent paragraphs then must elaborate. Avoid having the first paragraph have multiple sentences. (This is because the first paragraph gets extracted and used in tables of
contents, etc, and so has to be able to stand alone and not take up a lot of room.)

When referencing a parameter, use backticks. However, when referencing a parameter that also corresponds to a property, use square brackets instead. (This contradicts the Dart style guide, which says to use square brackets for both. We do this because of [dartdoc issue 1486](https://github.com/dart-lang/dartdoc/issues/1486). Currently, there's no way to unambiguously reference a parameter. We want to avoid cases where a parameter that happens to be named the same as a property despite having no relationship to that property gets linked to the property.)

```dart
// GOOD

  /// Creates a foobar, which allows a baz to quux the bar.
  ///
  /// The [bar] argument must not be null.
  ///
  /// The `baz` argument must be greater than zero.
  Foo({ this.bar, int baz }) : assert(bar != null), assert(baz > 0);
```

Avoid using terms like "above" or "below" to reference one dartdoc section from another. Dartdoc sections are often shown alone on a Web page, the full context of the class is not present.


## Coding patterns and catching bugs early

### Use asserts liberally to detect contract violations and verify invariants

`assert()` allows us to be diligent about correctness without paying a
performance penalty in release mode, because Dart only evaluates asserts in
debug mode.

It should be used to verify contracts and invariants are being met as we expect.
Asserts do not _enforce_ contracts, since they do not run at all in release builds.
They should be used in cases where it should be impossible for the condition
to be false without there being a bug somewhere in the code.

The following example is from `box.dart`:

```dart
abstract class RenderBox extends RenderObject {
  // ...

  double getDistanceToBaseline(TextBaseline baseline, {bool onlyReal: false}) {
    // simple asserts:
    assert(!needsLayout);
    assert(!_debugDoingBaseline);
    // more complicated asserts:
    assert(() {
      final RenderObject parent = this.parent;
      if (owner.debugDoingLayout) {
        return (RenderObject.debugActiveLayout == parent) && parent.debugDoingThisLayout;
      }
      if (owner.debugDoingPaint) {
        return ((RenderObject.debugActivePaint == parent) && parent.debugDoingThisPaint)
            || ((RenderObject.debugActivePaint == this)   && debugDoingThisPaint);
      }
      assert(parent == this.parent);
      return false;
    });
    // ...
    return 0.0;
  }

  // ...
}
```

### Prefer specialized functions, methods and constructors

Use the most relevant constructor or method, when there are multiple
options.

Example:

```dart
// BAD:
const EdgeInsets.TRBL(0.0, 8.0, 0.0, 8.0);

// GOOD:
const EdgeInsets.symmetric(horizontal: 8.0);
```


### Minimize the visibility scope of constants

Prefer using a local const or a static const in a relevant class than using a
global constant.

As a general rule, when you have a lot of constants, wrap them in a
class. For examples of this, see
[lib/src/material/colors.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/src/material/colors.dart).


### Avoid using `if` chains or `?:` or `==` with enum values

Use `switch` with no `default` case if you are examining an enum, since the analyzer will warn you if you missed any of the values when you use `switch`. The `default` case should be avoided so that the analyzer will complain if a value is missing. Unused values can be grouped together with a single `break` or `return` as appropriate.

Avoid using `if` chains, `? ... : ...`, or, in general, any expressions involving enums.


### Avoid using `var` and `dynamic`

All variables and arguments are typed; avoid `dynamic` or `Object` in
any case where you could figure out the actual type. Always specialize
generic types where possible. Explicitly type all list and map
literals. Give types to all parameters, even in closures and even if you
don't use the parameter.

This achieves two purposes: it verifies that the type that the compiler
would infer matches the type you expect, and it makes the code self-documenting
in the case where the type is not obvious (e.g. when calling anything other
than a constructor).

Always avoid `var` and `dynamic`. If the type is unknown, prefer using
`Object` (or `Object?`) and casting, as using `dynamic` disables all
static checking.


### Avoid using `library` and `part of`.

Prefer that each library be self-contained. Only name a `library` if you are documenting it (see the
documentation section).

We avoid using `part of` because that feature makes it very hard to reason about how private a private
really is, and tends to encourage "spaghetti" code (where distant components refer to each other) rather
than "lasagna" code (where each section of the code is cleanly layered and separable).


### Avoid using `extension`.

Extension methods are confusing to document and discover. To an end developer,
they appear no different than the built-in API of the class, and discovering
the documentation and implementation of an extension is more challenging than
for class members.

Prefer instead adding methods directly to relevant classes. If that is not
possible, create a method that clearly identifies what object(s) it works with
and is part of.

(A rare exception can be made for extensions that provide temporary workarounds
when deprecating features. In those cases, however, the extensions and all their
members must be deprecated in the PR that adds them, and they must be removed
in accordance with our deprecation policy.)


### Avoid using `FutureOr<T>`

The `FutureOr` type is a Dart-internal type used to explain certain aspects of the `Future` API. In public APIs, avoid the temptation to create APIs that are both synchronous and asynchronous by returning this type, as it usually only results in the API being more confusing and less type safe.

In certain extreme cases where the API absolutely needs to be asynchronous but a synchronous "escape hatch" is needed for performance, consider using `SynchronousFuture` (but be aware that this still suffers from many of the same risks of making the API only subtle and complicated). This is used, for example, when loading images in the Flutter framework.

You may use `FutureOr` to accept a callback that may or may not return a `Future`.


### Avoid using `Expando`

Generally speaking, `Expando` objects are a sign of an architectural problem. Carefully consider whether your usage is actually necessary. When your classes have clear documented ownership rules, there is usually a better solution.

Expando objects tend to invite code that is hard to understand because one cannot simply follow references to find all the dependencies.


### Avoid using `@visibleForTesting`

The [`@visibleForTesting`](https://api.flutter.dev/flutter/meta/visibleForTesting-constant.html) annotation marks a public API so that developers that have not disabled the `invalid_use_of_visible_for_testing_member` analyzer error get a warning when they use this API outside of a `test` directory.

This means that the API has to be treated as being public (nothing prevents a developer from using the API even in non-test code), meaning it must be designed to be a public API, it must be documented, it must be tested, etc. At which point, there's really no reason not to just make it a public API. If anything, the use of `@visibleForTesting` becomes merely a crutch to convince ourselves that it's ok that we're making something public that we should really not have made public.

So rather than rely on `@visibleForTesting`, consider designing your APIs so that they are directly testable using the public API, without exposing any sensitive internals.

(One exception is combining `@visibleForTesting` with `@protected`. The `@protected` annotation marks a member as one that is intended for subclasses, so it is already a public API and considered as such. The `@visibleForTesting` annotation in that case merely enables the member to be called directly in tests without having to create a fake subclass and without having to add `//ignore` pragmas.)


### Never check if a port is available before using it, never add timeouts, and other race conditions.

If you look for an available port, then try to open it, it's extremely likely that several times a week some other code will open that port between your check and when you open the port, and that will cause a failure.

> Instead, have the code that opens the port pick an available port and return it, rather than being given a (supposedly) available port.

If you have a timeout, then it's very likely that several times a week some other code will happen to run while your timeout is running, and your "really conservative" timeout will trigger even though it would have worked fine if the timeout was one second longer, and that will cause a failure.

> Instead, have the code that would time out just display a message saying that things are unexpectedly taking a long time, so that someone interactively using the tool can see that something is fishy, but an automated system won't be affected.

Race conditions like this are the primary cause of flaky tests, which waste everyone's time.

Similarly, avoid delays or sleeps that are intended to coincide with how long something takes. You may think that waiting two seconds is fine because it normally takes 10ms, but several times a week your 10ms task will actually take 2045ms and your test will fail because waiting two seconds wasn't long enough.

> Instead, wait for a triggering event.


### Avoid mysterious and magical numbers that lack a clear derivation

Numbers in tests and elsewhere should be clearly understandable. When the provenance of a number is not obvious,
consider either leaving the expression or adding a clear comment (bonus points for leaving a diagram).

```dart
// BAD
expect(rect.left, 4.24264068712);

// GOOD
expect(rect.left, 3.0 * math.sqrt(2));
```


### Have good hygiene when using temporary directories

Give the directory a unique name that starts with `flutter_` and ends with a period (followed by the autogenerated random string).

For consistency, name the `Directory` object that points to the temporary directory `tempDir`, and create it with `createTempSync` unless you need to do it asynchronously (e.g. to show progress while it's being created).

Always clean up the directory when it is no longer needed. In tests, use the `tryToDelete` convenience function to delete the directory. (We use `tryToDelete` because on Windows it's common to get "access denied" errors when deleting temporary directories. We have no idea why; if you can figure it out then that could simplify a lot of code!)


### Perform dirty checks in setters

Dirty checks are processes to determine whether a changed values have been synchronized with the rest of the app.

When defining mutable properties that mark a class dirty when set, use
the following pattern:

```dart
/// Documentation here (don't wait for a later commit).
TheType get theProperty => _theProperty;
TheType _theProperty;
void set theProperty(TheType value) {
  assert(value != null);
  if (_theProperty == value) {
    return;
  }
  _theProperty = value;
  markNeedsWhatever(); // the method to mark the object dirty
}
```

The argument is called 'value' for ease of copy-and-paste reuse of
this pattern. If for some reason you don't want to use 'value', use
'newProperty' (where 'Property' is the property name).

Start the method with any asserts you need to validate the value.

Don't do anything _else_ in setters, other than marking the object as dirty and updating internal state.
Getters and setters should not have significant side-effects. For example, setting a property whose value
is a callback should not result in that callback being invoked. Setting a property whose value is an object
of some sort should not result in any of that object's methods being called.


### Common boilerplates for `operator ==` and `hashCode`

We have many classes that override `operator ==` and `hashCode` ("value classes"). To keep the code consistent,
we use the following style for these methods:

```dart
  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is Foo
        && other.bar == bar
        && other.baz == baz
        && other.quux == quux;
  }

  @override
  int get hashCode => Object.hash(bar, baz, quux);
```

For objects with a lot of properties, consider adding the following at the top of the `operator ==`:

```dart
    if (identical(other, this)) {
      return true;
    }
```

(We don't yet use this _exact_ style everywhere, so feel free to update code you come across that isn't yet using it.)

In general, consider carefully whether overriding `operator ==` is a good idea. It can be expensive, especially
if the properties it compares are themselves comparable with a custom `operator ==`. If you do override equality,
you should use `@immutable` on the class hierarchy in question.


### Override `toString`

Use [`Diagnosticable`](https://api.flutter.dev/flutter/foundation/Diagnosticable-mixin.html) (rather than directly overriding `toString`) on all but the most trivial classes. That allows us to inspect the object from [devtools](https://pub.dartlang.org/packages/devtools) and IDEs.

For trivial classes, override `toString` as follows, to aid in debugging:

```dart
  @override
  String toString() => '${objectRuntimeType(this, 'NameOfObject')}($bar, $baz, $quux)';
```

...but even then, consider using `Diagnosticable` instead.  Avoid using `$runtimeType`, since it adds a non-trivial cost even in release and profile mode. The `objectRuntimeType` method handles this for you, falling back to a supplied constant string when asserts are disabled.


### Be explicit about `dispose()` and the object lifecycle

Even though Dart is garbage collected, having a defined object lifecycle and explicit ownership model (describing in the API documentation who is allowed to mutate the object, for instance) is important to avoid subtle bugs and confusing designs.

If your class has a clear "end of life", for example, provide a `dispose()` method to clean up references such as listeners that would otherwise prevent some objects from getting garbage collected. For example, consider a widget that has a subscription on a global broadcast stream (that might have other listeners). That subscription will keep the widget from getting garbage collected until the stream itself goes away (which, for a global stream, might never happen).

In general, pretending that Dart does not have garbage collection is likely to lead to less confusing and buggy code, because it forces you to think about the implications of object ownership and lifecycles.


### Test APIs belong in the test frameworks

Mechanisms that exist for test purposes do not belong in the core libraries, they belong in test harnesses. This keeps the cost of the main library down in production and avoids the risk that people might abuse test APIs.


### Immutable classes should not have hidden state

Immutable classes (those with `const` constructors) should not have hidden state. For example, they should not use private statics or Expandos. If they are stateful, then they should not be `const`.


### Avoid `sync*`/`async*`

Using generator functions (`sync*`/`async*`) can be a powerful improvement when callers will
actually lazily evaluate the iterable and each iteration is expensive _or_ there are a very
large number of iterations.

It should not be used in place of building and returning a `List`, particularly for trivial methods
that only yield a small number of members or when callers will evaluate the whole collection
anyway. It should also be avoided in very large functions.

It incurs runtime overhead in maintaining and using an iterator, and space overhead for the compiler
to actually desugar the generator into something that uses an iterator class.

## Writing tests

### Make each test entirely self-contained

Embrace code duplication in tests. It makes it easier to make new tests by copying and pasting them and
tweaking a few things.

Avoid using `setUp`, `tearDown`, and similar features, as well as test-global variables or other state
shared between tests. They make writing tests easier but make maintaining them, debugging them, and
refactoring code much harder. (These are commonly used in Flutter's codebase today, but that is almost
always a mistake. When you are editing a file that uses those features, aim to reduce the number of
tests using them while you're there.)

Specifically, we are trying to avoid shared state, which could persist across tests, and non-local
side-effects, which would prevent being able to move a test to another file without breaking the test.
(It's fine to factor out code into functions that are called by tests, so long as the functions don't
have side-effects that might change how other tests run.)


### Prefer more test files, avoid long test files

Avoid adding tests to files that already have more than one or two hundred lines of code. It's easier
to understand a test file when it has only a few related tests, rather than when it has an entire test
suite. (It also makes developing the tests faster because you can run the test file faster.)


### Avoid using `pumpAndSettle`

As per the API docs for [pumpAndSettle](https://main-api.flutter.dev/flutter/flutter_test/WidgetController/pumpAndSettle.html), prefer using explicit [`pump`](https://main-api.flutter.dev/flutter/flutter_test/WidgetController/pump.html) calls rather than `pumpAndSettle`.

Using `pumpAndSettle`, especially without checking its return value, makes it very easy for bugs to sneak in where we trigger animations across multiple frames instead of immediately. It is almost always the case that a call to `pumpAndSettle` is more strictly correctly written as two `pump` calls, one to trigger the animations and one (with a duration) to jump to the point after the animations.


## Naming

### Begin global constant names with prefix "k"

Examples:

```dart
const double kParagraphSpacing = 1.5;
const String kSaveButtonTitle = 'Save';
```

However, where possible avoid global constants. Rather than `kDefaultButtonColor`, consider `Button.defaultColor`. If necessary, consider creating an `abstract final class` to hold relevant constants.


### Avoid abbreviations

Unless the abbreviation is more recognizable than the expansion (e.g. XML, HTTP, JSON), expand abbreviations
when selecting a name for an identifier. In general, avoid one-character names unless one character is idiomatic
(for example, prefer `index` over `i`, but prefer `x` over `horizontalPosition`).


### Avoid anonymous parameter names

Provide full type information and names even for parameters that are otherwise unused. This makes it easier for
people reading the code to tell what is actually going on (e.g. what is being ignored). For example:

```dart
  onTapDown: (TapDownDetails details) { print('hello!'); }, // GOOD
  onTapUp: (_) { print('good bye'); }, // BAD
```


### Naming rules for typedefs and function variables

When naming callbacks, use `FooCallback` for the typedef, `onFoo` for
the callback argument or property, and `handleFoo` for the method
that is called. If `Foo` is a verb, prefer the present tense to the
past tense (e.g. `onTap` instead of `onTapped`).

If you have a callback with arguments but you want to ignore the
arguments, give the type and names of the arguments anyway. That way,
if someone copies and pastes your code, they will not have to look up
what the arguments are.

Never call a method `onFoo`. If a property is called `onFoo` it must be
a function type. (For all values of "Foo".)

Prefer using `typedef`s to declare callbacks. Typedefs benefit from having
documentation on the type itself and make it easier to read and find
common callsites for the signature.

### Spell words in identifiers and comments correctly

Our primary source of truth for spelling is the
[Material Design Specification](https://material.google.com/).
Our secondary source of truth is dictionaries.

Avoid "cute" spellings. For example, 'colors', not 'colorz'.

Prefer US English spellings. For example, 'colorize', not 'colourise', and 'canceled', not 'cancelled'.

Prefer compound words over "cute" spellings to avoid conflicts with reserved words. For example, 'classIdentifier', not 'klass'.


### Capitalize identifiers consistent with their spelling

In general, we use [Dart's recommendations](https://dart.dev/guides/language/effective-dart/style#identifiers) for naming identifiers. Please consider the following additional guidelines:

If a word is correctly spelled (according to our sources of truth as described in the previous section) as a single word, then it should not have any inner capitalization or spaces.

For examples, prefer `toolbar`, `scrollbar`, but `appBar` ('app bar' in documentation), `tabBar` ('tab bar' in documentation).

Similarly, prefer `offstage` rather than `offStage`.

Avoid using class names with `iOS` when possible. The capitalization of `iOS` is supposed to be exactly that, but that doesn't work well with camelCase and even less with UpperCamelCase; use alternatives like "Cupertino" or "UIKit" instead when possible. If you really really must use "iOS" in an identifier, capitalize it to `IOS`. Whether or not [the two-letter exception](https://dart.dev/guides/language/effective-dart/style#do-capitalize-acronyms-and-abbreviations-longer-than-two-letters-like-words) applies to "iOS" is debatable, but `IOS` is consistent with Dart APIs, and the alternatives (`IOs`, `Ios`) are even more jarring. (Previous versions of this guide incorrectly indicated that `Ios` was the correct capitalization when necessary; this form should not be used in new code.)


### Avoid double negatives in APIs

Name your boolean variables in positive ways, such as "enabled" or "visible", even if the default value is true.

This is because, when you have a property or argument named "disabled" or "hidden", it leads to code such as `input.disabled = false` or `widget.hidden = false` when you're trying to enable or show the widget, which is very confusing.


### Prefer naming the argument to a setter `value`

Unless this would cause other problems, use `value` for the name of a setter's argument. This makes it easier to copy/paste the setter later.


### Qualify variables and methods used only for debugging

If you have variables or methods (or even classes!) that are only used in debug mode,
prefix their names with `debug` or `_debug` (or, for classes, `_Debug`).

Do not use debugging variables or methods (or classes) in production code.


### Avoid naming undocumented libraries

In other words, do not use the `library` keyword, unless it is a
documented top-level library intended to be imported by users.

### Avoid "new/old" modifiers in code

The definition of "New" changes as code grows and time passes. If the code
needed a replacement version the odds of needing another replacement in the
future is higher. Instead find a name that represents the idea being used
or replaced.


## Comments

### Avoid checking in comments that ask questions

Find the answers to the questions, or describe the confusion, including
references to where you found answers.

If commenting on a workaround due to a bug, also leave a link to the issue and
a TODO to clean it up when the bug is fixed.

Example:

```dart
// BAD:

// What should this be?

// This is a workaround.


// GOOD:

// According to this specification, this should be 2.0, but according to that
// specification, it should be 3.0. We split the difference and went with
// 2.5, because we didn't know what else to do.

// TODO(username): Converting color to RGB because class Color doesn't support
//                 hex yet. See http://link/to/a/bug/123
```

TODOs should include the string TODO in all caps, followed by the GitHub username of
the person with the best _context_ about the problem referenced by the TODO in
parentheses. A TODO is not a commitment that the person referenced will fix the
problem, it is intended to be the person with enough context to explain the problem.
Thus, when you create a TODO, it is almost always your username that is given.

Including an issue link in a TODO description is required.

_(See also [#37519](https://github.com/flutter/flutter/issues/37519),
which tracks a proposal to change the syntax of TODOs to not include usernames.)_

### Comment all `// ignores`

Sometimes, it is necessary to write code that the analyzer is unhappy with.

If you find yourself in this situation, consider how you got there. Is the analyzer actually correct but you
don't want to admit it? Think about how you could refactor your code so that the analyzer is happy. If such a
refactor would make the code better, do it. (It might be a lot of work... embrace the yak shave.)

If you are really really sure that you have no choice but to silence the analyzer, use `// ignore: `. The ignore
directive should be on the same line as the analyzer warning.

If the ignore is temporary (e.g. a workaround for a bug in the compiler or analyzer, or a workaround for some known problem in Flutter that you cannot fix), then add a link to the relevant bug, as follows:

```dart
  foo(); // ignore: lint_code, https://link.to.bug/goes/here
```

If the ignore directive is permanent, e.g. because one of our lints has some unavoidable false positives and in this case violating the lint is definitely better than all other options, then add a comment explaining why:

```dart
  foo(); // ignore: lint_code, sadly there is no choice but to do
  // this because we need to twiddle the quux and the bar is zorgle.
```

### Comment all test skips

On very rare occasions it may be necessary to skip a test. To do that, use the `skip` argument.
Any time you use the `skip` argument, file an issue describing why it is skipped and
include a link to that issue in the code.


### Comment empty closures to `setState`

Generally the closure passed to `setState` should include all the code that changes the state. Sometimes this is not possible because the state changed elsewhere and the `setState` is called in response. In those cases, include a comment in the `setState` closure that explains what the state is that changed.

```dart
  setState(() {
    // The animation ticked. We use the animation's value in the build method.
  });
```


## Formatting

These guidelines have no technical effect, but they are still important purely
for consistency and readability reasons.

We use `dart format` to auto-format all Dart code. This is enforced by our CI.

Beyond whitespace formatting handled by the formatter, this section discusses
additional guidelines for code structure to ensure consistency and readibility.


### Constructors come first in a class

The default (unnamed) constructor should come first, then the named
constructors. They should come before anything else (including, e.g., constants or static methods).

This helps readers determine whether the class has a default implied constructor or not at a glance. If it was possible for a constructor to be anywhere in the class, then the reader would have to examine every line of the class to determine whether or not there was an implicit constructor or not.


### Order other class members in a way that makes sense

The methods, properties, and other members of a class should be in an order that
will help readers understand how the class works.

If there's a clear lifecycle, then the order in which methods get invoked would be useful, for example an  `initState` method coming before `dispose`. This helps readers because the code is in chronological order, so
they can see variables get initialized before they are used, for instance. Fields should come before the methods that manipulate them, if they are specific to a particular group of methods.

> For example, RenderObject groups all the layout fields and layout
> methods together, then all the paint fields and paint methods, because layout
> happens before paint.

If no particular order is obvious, then the following order is suggested, with blank lines between each one:

1. Constructors, with the default constructor first.
2. Constants of the same type as the class.
3. Static methods that return the same type as the class.
4. Final fields that are set from the constructor.
5. Other static methods.
6. Static properties and constants.
7. Members for mutable properties, without new lines separating the members of a property, each property in the order:
    - getter
    - private field
    - setter
8. Read-only properties (other than `hashCode`).
9. Operators (other than `==`).
10. Methods (other than `toString` and `build`).
11. The `build` method, for `Widget` and `State` classes.
12. `operator ==`, `hashCode`, `toString`, and diagnostics-related methods, in that order.

Be consistent in the order of members. If a constructor lists multiple
fields, then those fields should be declared in the same order, and
any code that operates on all of them should operate on them in the
same order (unless the order matters).


### Prefer a maximum line length of 100 characters for comments and docs

Aim for a maximum line length of roughly 100 characters for comments and docs. Existing docs may use a
line length of 80 characters and should not be reflowed to a maximum line length of 100 to simplify
reviewing docs in PRs.

Line length for code is automatically handled by `dart format`, which is configured to use a maximum
line length of 100.


### Consider using `=>` for short functions and methods

But only use `=>` when everything, including the function declaration, fits
on a single line.

Example:

```dart
// BAD:
String capitalize(String s) =>
  '${s[0].toUpperCase()}${s.substring(1)}';

// GOOD:
String capitalize(String s) => '${s[0].toUpperCase()}${s.substring(1)}';

String capitalize(String s) {
  return '${s[0].toUpperCase()}${s.substring(1)}';
}
```

### Use `=>` for getters and callbacks that just return literals or switch expressions

```dart
// GOOD:
List<Color> get favorites => <Color>[
  const Color(0xFF80FFFF),
  const Color(0xFF00FFF0),
  const Color(0xFF4000FF),
  _mysteryColor(),
];

// GOOD:
bool get isForwardOrCompleted => switch (status) {
  AnimationStatus.forward || AnimationStatus.completed => true,
  AnimationStatus.reverse || AnimationStatus.dismissed => false,
};
```

It's important to use discretion, since there are cases where a function body
is easier to visually parse:

```dart
// OKAY, but the code is more dense than it could be:
String? get validated => switch(input[_inputIndex]?.trim()) {
  final String value when value.isNotEmpty => value,
  _ => null,
}

// BETTER (more verbose, but also more readable):
String? get validated {
  final String? value = input[_inputIndex]?.trim();

  if (value != null && value.isNotEmpty) {
    return value;
  }
  return null;
}
```

If your code is passing an inline closure containing only a `return` statement,
you can instead use the `=>` form.\
When doing this, the closing `]`, `}`, or `)` bracket will have the same
indentation as the line where the callback starts.

For example:

```dart
    // GOOD, but slightly more verbose than necessary since it doesn't use =>
    @override
    Widget build(BuildContext context) {
      return PopupMenuButton<String>(
        onSelected: (String value) { print('Selected: $value'); },
        itemBuilder: (BuildContext context) {
          return <PopupMenuItem<String>>[
            PopupMenuItem<String>(
              value: 'Friends',
              child: MenuItemWithIcon(Icons.people, 'Friends', '5 new'),
            ),
            PopupMenuItem<String>(
              value: 'Events',
              child: MenuItemWithIcon(Icons.event, 'Events', '12 upcoming'),
            ),
          ];
        }
      );
    }

    // GOOD, does use =>, slightly briefer
    @override
    Widget build(BuildContext context) {
      return PopupMenuButton<String>(
        onSelected: (String value) { print('Selected: $value'); },
        itemBuilder: (BuildContext context) => <PopupMenuItem<String>>[
          PopupMenuItem<String>(
            value: 'Friends',
            child: MenuItemWithIcon(Icons.people, 'Friends', '5 new'),
          ),
          PopupMenuItem<String>(
            value: 'Events',
            child: MenuItemWithIcon(Icons.event, 'Events', '12 upcoming'),
          ),
        ]
      );
    }
```

The important part is that the closing punctuation lines up with the start
of the line that has the opening punctuation, so that you can easily determine
what's going on by just scanning the indentation on the left edge.


### Use braces for long functions and methods

Use a block (with braces) when a body would wrap onto more than one line (as opposed to using `=>`; the cases where you can use `=>` are discussed in the previous two guidelines).


### Prefer `+=` over `++`

We generally slightly prefer `+=` over `++`.

In some languages/compilers postfix `++` is an antipattern because of performance reasons, and so it's easier to just avoid it in general.

Because of the former, some people will use the prefix `++`, but this leads to statements that lead with punctuation, which is aesthetically displeasing.

In general, mutating variables as part of larger expressions leads to confusion about the order of operations, and entwines the increment with another calculation.

Using `++` does not make it obvious that the underlying variable is actually being mutated, whereas `+=` more clearly does (it's an assignment with an `=` sign).

Finally, `+=` is more convenient when changing the increment to a number other than 1.

### Use double literals for double constants

To make it clearer when something is a double or an integer, even if the number is a round number, include a decimal point in double literals. For example, if a function `foo` takes a double, write `foo(1.0)` rather than `foo(1)` because the latter makes it look like the function takes an integer.


## Conventions

### Expectations around potential crashes in the engine

The engine should never crash in an uncontrolled fashion.

In unopt mode, the engine C++ code should have asserts that check for contract violations.

In opt debug mode, the `dart:ui` code should have asserts that check for contract violations. These asserts should have messages that are detailed and useful, if they are not self-explanatory.

In opt release mode, the exact behavior can be arbitrary so long as it is defined and non-vulnerable for every input. For example, a contract violation could be checked in Dart, with an exception thrown for invalid data; but equally valid would be for the C++ code to return early when faced with invalid data. The idea is to optimize for speed in the case where the data is valid.

For practical purposes we don't currently check for out-of-memory errors. Ideally we would.


### Features we expect every widget to implement

Now that the Flutter framework is mature, we expect every new widget to implement all of the following:

- full accessibility, so that on both Android and iOS the widget works with the native accessibility tools.
- full localization with default translations for all our default languages.
- full support for both right-to-left and left-to-right layouts, driven by the ambient Directionality.
- full support for text scaling up to at least 3.0x.
- documentation for every member; see the section above for writing prompts to write documentation.
- good performance even when used with large amounts of user data.
- a complete lifecycle contract with no resource leaks (documented, if it differs from usual widgets).
- tests for all the above as well as all the unique functionality of the widget itself.

It's the job of the programmer to provide these before submitting a PR.

It's the job of the reviewer to check that all these are present when reviewing a PR.


### Use of streams in Flutter framework code

In general we avoid the use of `Stream` classes in Flutter framework code (and `dart:ui`). Streams in general are fine and we encourage people to use them. However, they have some disadvantages and we prefer to keep them out of the framework for this reason. For example:

* Streams have a heavy API. For example, they can be synchronous or asynchronous, broadcast or single-client, and they can be paused and resumed. It is non-trivial to determine the right semantics for a particular stream when it will be used in all the ways framework code could be used, and it is non-trivial to fully implement the semantics correctly.

* Streams don't have a "current value" accessor, which makes them difficult to use in `build` methods.

* The APIs for manipulating streams are non-trivial (e.g. transformers).

We generally prefer `Listenable` subclasses (e.g. `ValueNotifier` or `ChangeNotifier`).

In the specific case of exposing a value from `dart:ui` via a callback, we expect the bindings in the framework to register a single listener and then provide a mechanism to fan the notification to multiple listeners. Sometimes this is a rather involved process (e.g. the `SchedulerBinding` exists almost entirely for the purpose of doing this for `onBeginFrame`/`onDrawFrame`, and the `GesturesBinding` exists exclusively for the purpose of doing this for pointer events). Sometimes it's simpler (e.g. propagating changes to life cycle events).


## Packages

### Structure

As per normal Dart conventions, a package should have a single import
that reexports all of its API.

> For example,
> [rendering.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/rendering.dart)
> exports all of lib/src/rendering/*.dart

If a package uses, as part of its exposed API, types that it imports
from a lower layer, it should reexport those types.

> For example,
> [material.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/material.dart)
> reexports everything from
> [widgets.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/widgets.dart).
> Similarly, the latter
> [reexports](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/src/widgets/basic.dart)
> many types from
> [rendering.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/rendering.dart),
> such as `BoxConstraints`, that it uses in its API. On the other
> hand, it does not reexport, say, `RenderProxyBox`, since that is not
> part of the widgets API.

Flutter packages should not have "private" APIs other than those that are
prefixed with underscores. Every file in a Flutter package should be exported.
("Private" files can still be imported so they are still actually public APIs;
by not exporting them explicitly we are tricking ourselves into thinking of
them as private APIs which may lead to poor design.)

When developing new features in Flutter packages, one should follow the philosophy:

> Only expose the APIs that are necessities to the features.

Since the private classes in dart language are file-bound, this may often result in
large file sizes. In Flutter, this is considered to be more preferable than creating
multiple smaller files but exposing intermediate classes that are not needed to use
the features.


### Import conventions

Under `lib/src`, for in-folder import, use relative import. For cross-folder import,
import the entire package with absolute import.

When importing the `rendering.dart` library into higher level libraries,
if you are creating new
`RenderObject` subclasses, import the entire library. If you are only
referencing specific `RenderObject` subclasses, then import the
`rendering.dart` library with a `show` keyword explicitly listing the
types you are importing. This latter approach is generally good for
documenting why exactly you are importing particularly libraries and
can be used more generally when importing large libraries for very
narrow purposes.

By convention, `dart:ui` is imported using `import 'dart:ui' show ...;` for
common APIs (this isn't usually necessary because a lower
level will have done it for you), and as `import 'dart:ui' as ui show ...;`
for low-level APIs, in both cases listing all the identifiers being imported.
See [basic_types.dart](https://github.com/flutter/flutter/blob/main/packages/flutter/lib/src/painting/basic_types.dart)
in the `painting` package for details of which identifiers we import
which way. Other packages are usually imported undecorated unless they
have a convention of their own (e.g. `path` is imported `as path`).

The `dart:math` library is always imported `as math`.

### Deciding where to put code

As a general rule, if a feature is entirely self-contained (not requiring low-level integration into the Flutter framework) and is not something with universal appeal, we would encourage that that feature be provided as a package.

We try to be very conservative with what we put in the core framework, because there's a high cost to having anything there. We have to commit to supporting it for years to come, we have to document it, test it, create samples, we have to consider everyone's varied desires which they may have as they use the feature, we have to fix bugs. If there are design problems, we may not find out for a long time but then once we do we then have to figure out how to fix them without breaking downstream code, or we have to migrate all our existing widgets to the new architecture, etc.

Basically, code is expensive. So before we take it, if possible, we like to see if we can prove the code's value. By creating a package, we can see if people use the feature, how they like it, whether it would be useful for the framework, etc, without having to take on the costs.

We have two main kinds of packages that are maintained by the Flutter team, both of which live in [flutter/packages](https://github.com/flutter/packages):

1. Regular packages, which are pure Dart. Packages can also be written and maintained by people outside the Flutter team.

2. Plugin packages, which provide access to platform features and therefore include native code (such as Java or Objective-C) as well as Dart.

You can also consider making an independent package. Packages are published to [pub](https://pub.dartlang.org/).

Often once we have made a package we find that that is actually sufficient to solve the problem that the code sets out to solve, and there ends up being no need to bring it into the framework at all.
