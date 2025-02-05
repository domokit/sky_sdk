This page covers additional information that is specific to contributing to flutter/packages. If you aren't already familiar with the general guidance on Flutter contribution, start with [Tree hygiene](../../contributing/Tree-hygiene.md).

## Version and CHANGELOG updates

Most changes need version and CHANGELOG changes; see below for details.

In most cases, the easiest way to create them is to use [the `update-release-info` repository command](https://github.com/flutter/packages/blob/main/script/tool/README.md#update-changelog-and-version). If you are adding a feature, use `--version=minor`, otherwise `--version=minimal` will almost always do the right thing.

### Version

Any change that needs to be published in order to take effect must update the version in `pubspec.yaml`. There are very few exceptions:
- PRs that only affect tests.
- PRs that only affect unpublished parts of example apps.
- PRs that only affect local development of the package (e.g., changes to ignored lints).
- Breaking change batching (see below).
- Non-code changes that we make to future-proof development in some way, but don't directly benefit clients. Examples of this include
    - Updating the minimum Dart or Flutter SDK for all packages when we adjust our test matrix.
    - Updating the minimum OS version of plugins to match changes to [Flutter support](https://docs.flutter.dev/reference/supported-platforms) (on `stable`).

  (Unless you are a member of the Flutter team, you are likely not making changes that fall under this exemption.)

This is because the packages in flutter/packages use a continuous release model rather than a set release cadence. This model gets improvements to the community faster, makes regressions easier to pinpoint, and simplifies the release process.

(The `override: no versioning needed` label can be added to skip this check if it fails, but only if the criteria above are met, or team members agree there is a compelling reason for a new exemption. Team members: please leave a comment when adding the `override` label explaining the reason for the override.)

### CHANGELOG

All version changes must have an accompanying CHANGELOG update. Even version-exempt changes should often update CHANGELOG by adding a special `NEXT` entry at the top of `CHANGELOG.md` (unless they only affect development of the package, such as test-only changes):
```
## NEXT

* Description of the change.

## 1.0.2
...
```

This policy exists because some changes (e.g., certain updates to examples) that do not need to be published may still be of interest to clients of a package.

The tooling errs on the side of false positives, so will sometimes flag changes that should be exempt for one of the reasons listed above. In these cases, or if the reviewer feels that the change would not be relevant to package clients, they can add the `override: no changelog needed` label can to skip this check. Team members: please leave a comment when adding the `override` label explaining the reason for the override.

#### CHANGELOG style

For consistency, all CHANGELOG entries should follow a common style:
- Use `##` for the version line. A version line should have a blank line before and after it.
- Use `*` for individual items.
  - Exception: When editing an existing CHANGELOG that uses `-`, use that instead for local consistency.
- Entries should use present tense indicative for verbs, with "this version" as an implied subject. For example, "Adds cool new feature.",
  not "Add", "Added", or "Adding".
- Entries should end with a `.`.
- Breaking changes should be introduced with `**BREAKING CHANGE**:`, or `**BREAKING CHANGES**:`
  if there is a sub-list of changes.
  - Breaking change notifications should include information about how to migrate. If extensive migration is required, this can be a reference to a longer description elsewhere (usually README.md) rather than inline instructions.

Example:
```
## 2.0.0

* Adds the ability to fetch data from the future.
* **BREAKING CHANGES**:
  * Removes the deprecated `neverCallThis` method.
  * URLs parameters are now `Uri`s rather than `String`s.

## 1.0.3

* Fixes a crash when the device teleports during a network operation.
```

#### Updating a CHANGELOG that has a `NEXT`

*Note: If you are using `update-release-info`, this will be handled correctly for you.*

If you are adding a version change to a CHANGELOG that starts with `NEXT`, and your change also doesn't require a version update, just add a description to the existing `NEXT` list:
```
## NEXT

* Description of your new change.
* Existing entry.

## 1.0.2
...
```

If your change does require a version change, do the same, but then replace `NEXT` with the new version. For example:

```
## 1.0.3

* Description of your new change.
* Existing entry.

## 1.0.2
...
```

If you leave `NEXT` when adding a version change, automated tests for your PR will fail.

### FAQ

**Do I need to update the version if I'm just changing the README?** Yes. Most people read the README on pub.dev, not GitHub, so a README change is not very useful unless it is published.

**Do I need to update the version if I'm just changing comments?** If the comment is intended for clients of the package (a `///` comment on anything exported by the package), then yes, since what developers using the package will see in their IDE will come from the published version. If the comment is only useful for someone working on the package (such as an implementation comment within a method, or a comment in a non-exported file), then no.

**What do I do if I there are conflicts to those changes before or during review?** This is common. You can leave the conflicts until you're at the end of the review process to avoid needing to resolve frequently. Including the version changes at the beginning despite the likelihood of conflicts makes it much harder to forget that step, and also means that a reviewer can easily fix it from the GitHub UI just before landing.

### Breaking changes

Because we prefer to minimize breaking changes to packages after 1.0, breaking changes do not always follow the normal one-version-change-per-PR approach. Instead, when making a breaking change consider whether there are other breaking changes that should be made at the same time, and discuss with other regular contributors. If there are multiple changes to make, they can be batched as follows:
1. File an issue tracking all of the breaking changes to include.
1. Prepare PRs for all of the changes, without version changes. Ensure that they have all been reviewed, but do not land them yet. (They should fail CI due to the lack of version change, preventing accidental landing.)
1. Land a PR that temporarily adds `publish_to: none` to the package, with a comment referencing the issue from step 1.
1. Land all of the breaking change PRs. This step should be done in a relatively short window of time (thus the advance preparation above) to avoid having the plugin be unpublishable for longer than necessary. The PRs will pass CI once rebased, since the version check is disabled for unpublishable packages.
1. Once all breaking changes have landed, land a final PR to update the version and remove `publish_to: none`.

## Dependencies

We try to minimize external package dependencies as much as possible, where "external" means packages that are not generally within the control of the Flutter or Dart teams (Examples of non-external packages include `sdk:` dependencies, packages in flutter/packages, and packages published by dart.dev), or by an organization with a track record of strong support for both engineering best practices and Flutter. This is for several reasons:
- Maintainability:
    - We have a policy of always supporting Flutter `master`; if we can't guarantee that fixes for breaking changes can be landed immediately in dependencies, it can block our roller.
    - If a package is abandoned by its authors, we will have to fork or migrate in order to unblock the entire repository.
    - External packages increase our expose to out-of-band breakage (e.g., if breaking change is published without following semver).
- Security: We cannot make any guarantees about the code in an external package. For example, if a package we depend on is later taken over by a malicious actor, our dependency would expose all clients of our own package to potential attack via the transitive dependency.

If you are considering adding an external dependency:
- Consider other options, and discuss with #hackers-ecosystem in Discord.
- If you add a `dev_dependency` on an external package, pin it to a specific version if at all possible.
- If you add a `dependency` on an external package in an `example/`, pin it to a specific version if at all possible.
- If you add a non-dev, non-example dependency on an external package, a formal approval is required, and all of the following must be true:
  - The dependency is necessary for a high-priority product/feature.
  - The code has an OSS license, and the original source repository is available.
  - The dependency has an exact upper bound set to a version that has been reviewed by the Flutter team
  - The ecosystem TL and PM are confident that the code could be forked and maintained by the Flutter team if it were to become necessary (e.g., if the package were later abandoned).

### Native dependencies

The same general principles apply to native dependencies for plugins (e.g., dependencies specified in an Android Gradle file or iOS podspec file): minimize dependencies—and especially non-test dependencies—on libraries that are not created by either the platform vendor or another organization with a track record of strong support for engineering best practices in whom we can have a very high degree of confidence in ongoing support and prompt updates.

### Dev dependencies

Some dependencies should only be linked as dev_dependencies, such as `integration_test`. Known dev-only dependencies are checked by CI, and if you are adding a new dev-only dependency, consider adding it to
[the repository tooling checks](https://github.com/flutter/packages/blob/main/script/tool/lib/src/pubspec_check_command.dart).

### Dependency versions

Our general policy is not to update the minimum version of a dependency beyond what the package requires. For instance:
- We do not regularly make the app-facing package in a federated plugin require the latest bugfix versions of all platform implementation packages just so that people who do not regularly update their transitive dependencies get the latest bugfixes. Our view is that clients who want updates should update their transitive dependencies (`dart pub upgrade`, or `dart pub upgrade --unlock-transitive <package>` for targeted updates), rather than us artifically pushing transitive dependency updates on everyone.
- When a dependency updates its major version, and the breaking changes do not affect our usage, we expand the constrain to *add* the new major version to the range, rather than requiring the new major version. This minimizes version lock in the ecosystem (where clients are unable to use the latest version of two different packages because each requires a specific, different major version).

We do update minimum versions when it is actually required, such as a breaking change that does affect our usage, or when adding a new feature to the app-facing package in a federated plugin that relies on new APIs in the other sub-packages of that plugin. In most cases, the `analyze_downgraded` CI task will catch cases where that is necessary, but in some cases we rely on PR authors and reviewers (for example, when exposing a new feature in a federated plugin it is often possible to compile with only the platform interface package's minimum version updated, but the feature may throw `UnimplementedError`s at runtime if the implementation package versions are not updated as well).

This is a general policy rather than a hard rule. If you believe you have an exceptional case, please discuss with your reviewer and/or with the `#hackers-ecosystem` Discord channel. Examples of cases where we have made exceptions include:
- Fixes for serious security issues in dependencies.
- Changes in dependencies that clients *must* adopt for external reasons, such as policy changes in a platform's app store.

## Platform Support

The goal is to have any plugin feature work on every platform on which that feature makes sense; having a lot of features that are only partially implemented across platforms is often confusing and frustrating for developers trying to use those plugins. However, a single developer will not always have the expertise to implement a feature across all supported platforms.

Given that, we welcome PRs that only implement a feature for a subset of platforms, including just one. To set expectations for how such PRs will be handled:
- They will not be fully reviewed until there's an understanding of what support would look like across other platforms, for several reasons:
  - API for features that will be permanently platform-specific might be structured in ways that make that limitation more clear, so knowing if other platforms can support it will affect the review process.
  - We want to avoid over-fitting the plugin APIs to a single platform's API. It is often the case that several platforms can implement a feature, but the behavior is different enough across platforms that we need to design the API in a way that covers those variations in a cohesive way. This means that knowing at a high level what the implementation on other platform also affects the review process.
  - Features that are missing implementations on some platforms need to be clearly documented as such in the API, and those comments should clearly express whether those platform are temporarily missing implementations, or are not expected to ever have implementations due to platform limitations.

  The PR author isn't necessarily responsible for answering these questions. These cases will be noted in comments during PR triage; if the PR author, or others in the community, can contribute that information, that will certainly help. If not, that investigation will be part of the review process (in which case the review will likely take longer).
- In some cases, a reviewer may wait on approving a PR for landing until there is a plan in place for landing implementations for other platforms. This is not a hard rule, and will be up to reviewer judgement. This could take a number of forms:
  - Waiting for other PRs from the community that implement the feature for other platforms, and then moving forward with all of them at once.
  - Finding resources within the Flutter team for implementing other platforms before moving forward.
  - Landing the platform interface change and the platform implementations that are done, but waiting on one of the options above before adding the API to the app-facing package's API (allowing developers the option of drilling down to the platform implementation to use the feature on some platforms before it's ready everywhere).

  "Other platforms" might not always include all other platforms. E.g., a feature might be something that's much more likely to be useful on mobile than desktop, or the reverse, and so we might only wait for implementations of that subset. Again, this will be up to reviewer judgement.

In the case where a PR is put on hold for the reasons above, it should be clearly noted in the PR and on the associated issue. We encourage anyone interested in contributing more platform implementation PRs to comment in the bug in such cases.

### API support queries

In cases where a plugin offers API in the app-facing package that is not implemented on all platforms (or not implemented on some OS versions of some platforms), there should also be an API to query for support of that API at runtime. That is, plugin clients should always be able to write code like:

```dart
if (somePluginInstance.supportsDoingThing) {
  somePluginInstance.doThing();
}
```

rather than hard-coding a platform check. (See [`image_picker`'s `supportsImageSource`](https://github.com/flutter/packages/blob/9323e33ed9d8345d87514711dcaeb4cf4159ad1c/packages/image_picker/image_picker/lib/image_picker.dart#L309-L315) for an example.) This is for several reasons:
- It is federation-friendly: a new implementation of the plugin (including the addition of 1P support for a previously-unsupported platform) will automatically be covered.
- It allows seamless adoption of new functionality: if we add previously-missing support to an existing implementation, clients will automatically benefit without any code changes on their part.
- It makes our own documentation more evergreen: we've frequently had app-facing APIs that had a comment saying that an API is only available on platform X, and then forgotten to update it when adding support for platform Y.

These APIs can take a variety of forms, including specific methods to check individual methods or parameters, a single method with an enum of API options, etc. Discuss with your reviewer what pattern is best suited to the specific case, and aim for consistency within a plugin when possible.

*We currently have many cases of APIs that do not follow this guidance because they predate it. This is technical debt, rather than something that should be pointed to as a justification for adding new APIs that do not.*

## Languages

On some platforms, there are multiple native languages that can be used to write plugins; repository policy limits the languages that are used in some cases. These are currently the allowed languages for each platform:
- Android: Depends on the plugin; mixing languages within a plugin is currently not allowed:
  - Kotlin if the plugin is already in Kotlin.
    - Currently our policy is to have a limited number of plugins in Kotlin, to ensure that we are finding Kotlin-specific plugin issues in our own development.
  - Java if the plugin is already in Java (which is almost all of them).
    - Allowing Kotlin more broadly is [under consideration](https://docs.google.com/document/d/1Ok_mUPgmw8_l-ynLueEKtXm2Fs48lEVB0JqZd7M7uyA/edit?usp=sharing), but currently most plugin development is limited to Java to avoid adding another language to the set of languages Flutter team members need to interact with regularly.
    - If you are interested in taking on a **long-term ownership role in a plugin**, and would like to migrate it to Kotlin, please reach out in the `hackers-ecosystem` channel [on Discord](../../contributing/Chat.md).
- iOS: Depends on the plugin. The goal is to eventually migrate all plugins to Swift; see [the migration section below](#swift-migration-for-1p-plugins) for details. For non-migration PRs, use:
  - Swift if the plugin is entirely Swift.
  - Swift or Objective-C if the plugin is partially migrated.
  - Objective-C if the plugin is still entirely Objective-C.
- Linux: C. Use of C++ constructs is strongly discouraged; see [repo style notes](../../../CONTRIBUTING.md#style) for details
- macOS: Swift only\*.
  - \* In some cases an existing iOS implementation has been updated to support macOS, so is Objective-C (e.g., `in_app_purchase`). This is the only case where Objective-C is allowed for macOS plugins.
- Windows: C++.
- Web: Dart.

For all platforms, use of Dart for platform-specific features is both allowed and encouraged. While there are no specific rules about this in most plugins, in general we are moving toward having more logic written in Dart rather than a host language, as having more code in the project's primary language eases maintenance.

## Generated code

### Pigeon

Most plugins in flutter/packages use [`pigeon`](https://pub.dev/packages/pigeon) for communication between Dart and host-language code. Unless a package lists specific instructions in its CONTRIBUTING.md file, after changing a Pigeon interface definition file in the `pigeons/` directory run:
   ```sh
   $ dart run pigeon --input pigeons/[changed file]
   ```

If the package's tests have mocks, they likely include mocks of Pigeon-generated classes; see below for instructions to update the mocks to reflect the changes in the Pigeon-generated code.

### Mockito

Many packages use [`mockito`](https://pub.dev/packages/mockito) for unit tests. To regenerate mocks run:
   ```sh
   $ dart run build_runner build --delete-conflicting-outputs
   ```

## Changing federated plugins

Most of the plugins in flutter/packages are [federated](https://flutter.dev/docs/development/packages-and-plugins/developing-packages#federated-plugins). Because a logical plugin consists of multiple packages, and our CI tests using published package dependencies—in order to ensure that every PR can be published without breaking the ecosystem—changes that span multiple packages will need to be done in multiple PRs. This is common when adding new features.

We are investigating ways to streamline this, but currently the process for a multi-package PR is:

1. Create a PR that has all the changes, and update the pubspec.yaml files to have path-based dependency overrides. This can be done with the [plugin repo tool](https://github.com/flutter/packages/blob/main/script/tool/README.md)'s `make-deps-path-based` command, targeting any dependency packages changed in the PR. For instance, for an Android-specific change to `video_player` that required platform interface changes as well:
    ```
    $ dart run script/tool/bin/flutter_plugin_tools.dart make-deps-path-based --target-dependencies=video_player_platform_interface,video_player_android
    ```

1. Upload that PR and get it into a state where the only test failure is the one complaining that you can't publish a package that has dependency overrides, then go through the normal process of getting reviews and approvals.
    * The overall review is completed first to prevent situations where part of the overall change (usually a platform interface change) lands but the rest is never landed, or where the review of the changes in a higher-level part of the change (the app-facing and/or implementation packages) identifies issues in the lower-level parts.

1. **Once the combined PR is approved**, create a new PR that has only the platform interface package changes from the PR above, and ask the reviewers of the main package to review that.
    * This review is generally trivial (unless the CI finds an issue that wasn't noticed in the main PR) since it's a subset of the PR that was already reviewed.

1. Once it has been reviewed, landed, and published, update the initial PR to:
    * remove the changes that are part of the other PR,
    * replace the dependency overrides on the platform interface package with a dependency on the published version, and
    * merge in (or rebase to) the latest version of `master`.

1. If there are any dependency overrides remaining, repeat the previous two steps with those packages. There should never be interdependencies between platform implementation packages, so all implementations should be able to be handled in a single new PR.
    * If new functionality is being added to the API surface of the app-facing package, be sure to update the version constraints of the implementation packages in its `pubspec.yaml`. Even though it's possible to compile with only the interface package's constraint updated, we don't want clients of the package who update without using `pub upgrade` to have surprising failures (e.g., `UnimplementedError`s) when using the new functionality due to not having the updated platform packages that implement the new feature.

1. Once there are no dependency overrides, ask the reviewer to land the main PR.

### Breaking changes to plugin platform interfaces

Breaking changes to platform interfaces (any package ending in `_platform_interface`) are strongly discouraged:
- They require each platform implementation to adopt the new version, and the app-facing package can't pick up any of those changes until all implementations have been updated. This could cause situations where bug fixes for one platform are held up on another platform adopting a feature change.
- They require a series of changes to CI to selectively disable testing the latest versions of all packages, then later re-enable them.
- They temporarily lock out unendorsed implementations, until their developers can update.

Because platform interfaces are not expected to be called by clients of the plugins, we favor backward compatibility over having a clean API at that layer.

In order to avoid accidental breaking changes that are missed in review, CI will by default fail for any breaking change to the platform interface. If you believe you need to make a breaking change, discuss with your reviewer and make sure they agree; once they do, add (or if you are not a project member, ask the reviewer to add) the `override: allow breaking change` label, then re-run the failing check. You should also adding something like the following to the PR description:
```
## Breaking change justification

<Insert good reason for breaking change here.>
```

This makes it easy for someone looking back at the change to find the reason for the breaking change without reading all of the comments.

### Changing platform interface method parameters

Because platform implementations are subclasses of the platform interface and override its methods, almost *any* change to the parameters of a method is a breaking change. In particular, adding an optional parameter to a platform interface method *is* a breaking change even though it doesn't break callers of the method.

The least disruptive way to make a parameter change is:

1. Add a new method to the platform interface with the new parameters, whose default implementation calls the existing method. This is not a breaking change.
    1. Strongly consider replacing some or all of the parameters with a parameter object (see [`AuthenticationOptions`](https://github.com/flutter/packages/blob/0ba896b35b2f33517c38ccbb1654ed646ad07af7/packages/local_auth/local_auth_platform_interface/lib/types/auth_options.dart#L7-L9) for [`authenticate`](https://github.com/flutter/packages/blob/0ba896b35b2f33517c38ccbb1654ed646ad07af7/packages/local_auth/local_auth_platform_interface/lib/local_auth_platform_interface.dart#L60) as an example), as this will allow adding other parameters in the future without breaking changes or new methods.
    2. Mark the old method as `@Deprecated`, and file an issue about updating all uses of that method in the other packages in the plugin.

    Note: The delegation of this method will feel backwards, because the implementing the new method in terms of the old method will discard the information from the new parameters. This is correct though, because the goal is to allow the app-facing package to call the new method regardless of whether implementation packages have implemented it. By forwarding to the existing method by default, implementation packages that haven't been updated will continue to have the behavior they had before (which, by necessity, won't include support for the new parameters). Implementation packages that have been updated will override this implementation with their own that honors the new parameters.

2. Update the implementations to override both methods.

    * Usually this will involve having the old method call the new method (the opposite of the delegation done above in the interface package), or in some cases extracting a private helper method that both versions call.

3. Update the app-facing package to call the new method.

At some later point the deprecated method can be cleaned up by:

1. Making a breaking change in the platform interface package to remove any deprecated methods.

2. Updating the app-facing package to allow either the new major version or the previous version of the platform interface (to minimize version lock issues with implementations).

3. Updating all the implementations to use the new major version, removing the override of the deprecated methods.

This cleanup step is low priority though; deprecated methods in a platform interface should be largely harmless, as it's an API with very few direct customers. It's actually preferable to wait quite some time before doing this, as it gives any unendorsed third-party implementations that may exist more time to adapt to the change without disruption.

## Supported Flutter versions

flutter/packages has a general policy of supporting the latest `stable` version of Flutter, as well as the current `master`. In practice, many packages often support older versions as well, as minimum version requirements are generally only updated when there is a specific need, such as using a new Flutter feature.
- One exception is new packages which require features that aren't yet available on `stable`. In this rare case, discuss with `#hackers-ecosystem` as it requires adding conditional logic to the CI scripts.

Most CI only runs against those two version. There are only minimal tests of versions older that current `stable` (currently analysis only, for the previous two stable versions of Flutter).

### When to update the required version

- If you know your change requires a feature of Flutter that was added recently, you should update the minimum version accordingly. In particular, if your change was originally written against `master` and had to wait until a change reached `stable`, that means you need to update the minimum version. (Adding a constraint update as soon as the PR fails on `stable` tests is a good way to make sure you don't forget.)
    - If your change requires a newer version of Flutter than is available on `stable`, and it can't wait until that feature reaches `stable`, ask in `#hackers-ecosystem` to see if there's a good solution.
- If your change fails a `*_legacy` CI test, update the minimum version accordingly.

The ecosystem team may also mass-update minimum versions from time to time, to reduce the potential of breaking untested versions (see below). In general, this should use a minimum version somewhat older than the current `stable`, but in some special cases may use the current stable version instead.

### Handling breakage in old versions

Sometimes a change to a package accidentally breaks older version of Flutter; because the full CI test suite does not run anything older than the current stable, such breakage will not necessarily be caught by CI, and will only be discovered via issue reports. When that happens there are several options:
- If the breakage is found quickly enough that retraction is still possible:
    - Retract the latest release.
    - Release a new version that is identical except for updating the minimum Flutter version.
  This minimizes disruption to developers using old versions of Flutter, with no effect on people using current versions.
- Otherwise, there are several options; discuss with `#hackers-ecosystem` to decide which makes sense for the specific case:
    1. Release a new update that restores compatibility with the old version. This should generally only be done if it's trivial to do so.
    1. Release a revert, then release a new version that reverts the revert but with a constraint update. Consider this option if the number of people affected is likely to be large (e.g., a popular plugin is broken for the previous stable version of Flutter shortly after a stable release). This has essentially the same outcome as retraction, but can be done at any time.
    1. Document the need to pin an old version of the package in the issue, and close it (along with making a `## NEXT` PR for the package that updates its minimum version, to document the correct reality). This will require all affected users to find the issue to learn how to fix it, so should generally only be done if the number of people affected is likely to be small (e.g., when it only affects versions of Flutter that are several stable releases behind).

## Plugin architecture conventions

All plugins in flutter/packages should follow the following conventions. Note that existing plugins do not currently always follow those conventions because they predate them. PRs that update plugins to follow conventions are welcome.

### Federation

All plugins should be fully [federated](https://docs.flutter.dev/development/packages-and-plugins/developing-packages#federated-plugins). This is to ensure that:
- Unofficial federated implementations can be created for any of our plugins. This allows for alternate implementations, as well as supporting unofficial embeddings.
  - Our development processes for federated plugins also helps ensure that we don't accidentally break any such implementations. For instance, our federated safety checks help ensure that we don't make breaking changes to the platform interface without changing the major version.
- We are eating our own dogfood with federation. Federation adds non-trivial complexity to maintaining a plugin, and best practices for federation aren't always obvious. Using federation ourselves means that we are aware of potential issues, and encourages us to create documentation and tooling to improve the developer experience of using federation.

The only exception to this policy is plugins that are inherently specific to a single platform, such as `espresso`.

### In-package platform channels

All implementations should use in-package platform channels, for the reasons outlined in [the proposal document](https://flutter.dev/go/platform-channels-in-federated-plugins). Most plugins that predate this policy and have a legacy "shared method channel" default implementation in the platform interface package, but it is not be used by any first-party implementations.

### Platform exception handling

Having consistent error handling across platforms is an important part of providing a usable cross-platform API surface. Maintaining consistent errors directly from the native implementations is challenging since there is no easy way to share constants for error code strings across all the different languages, nor any clear reference point for what the possible errors are without reading all of the other implementations. It's also challenging to make changes, as it potentially requires changing every package in the federated plugin.

To ensure that the native errors are a coherent part of the interface, plugins that can throw `PlatformException`s internally should follow these best practices:
- The platform interface package should define a clear, structured error system as part of its interface. Options include:
    - A plugin-specific `Exception` subclass, including constants or enums for known error types (e.g., permission failures).
    - An alternate structured error-returning mechanism, such as return values that include either a successful value or an error state (e.g., result classes, or Dart 3 patterns and records).
- App-facing packages should `export` that definition, and should include it in relevant API documentation.
- Implementation packages should, in general, have Dart code to catch any `PlatformExceptions` that the native implementation is likely to throw, and convert them to the appropriate interface-defined error types.
  - This means that the string constants for error codes returned from native need only be consistent within that platform implementation package, since it won't be passed out of the package (and thus only need to be kept in sync between two languages).

This means that in general, clients of a plugin should not be expected to see raw `PlatformException`s created from error responses in native code. (This is not a strict rule; failure cases that are so obscure that clients would be unlikely to actually have specific handlers for them don't necessarily need to be converted to a common exception type.)

**Note:** Existing `PlatformException`s are a de-facto part of the API, so updating plugins to follow this practice should be done as a breaking change.

### Enum handling

The best practice for doing `switch`es over `enum`s varies depending on the situation:
- If the `enum` is defined in the same package as the `switch`, the `switch` should cover all cases with no fallback, as it can be updated at the same time as any changes to the enum.
- If the `enum` is defined in a different package—this is not uncommon in federated plugins—the code should be robust against new enum values (unless it is just example code), since adding enum values is generally not considered a breaking change:
  - Do not use `default:`; instead put the fallback code outside the enum. This will cause the linter to flag code in other packages that we need to update when an `enum` is changed. (This will often require the use of `ignore: dead_code`.) The fallback is needed because even though we can and should [update the package(s) containing the `switch` as a follow-up](https://github.com/flutter/flutter/issues/89866), we **cannot** guarantee that someone won't use an older version of the package containing the `switch` with a newer version of the package containing the `enum`, so that combination must have some well-defined behavior.
  - When there is a reasonable default, use that so that the package continues to work, even if it's not the ideal behavior.
  - When there is no reasonable default, throw a clear `UnimplementedError` so the client knows they need a new version of the package.

## README code

All new code samples in `README.md` files must `<?code-excerpt?>` pragmas to manage the code. With `<?code-excerpt?>`, the source of truth for the code is an actual Dart file, which is analyzed, compiled, and tested by our CI. This ensures that it stays updated as the package APIs, Flutter, Dart, and repository analyzer settings change.

### Updating `<?code-excerpt?>`-managed examples

If a code block has a `<?code-excerpt ...?>` tag just before it, it is already using `code-excerpt`. To update it:
1. Find the file referenced in that tag (e.g., `<?code-excerpt "lib/main.dart (AppLifecycle)"?>` comes from `lib/main.dart`; if there is a `<?code-excerpt path-base=""?>` directive earlier in the file, paths are resolved relative to the given `path-base` instead of the markdown file's path).
2. Update the code in that file. If you are fixing a bug, add unit tests or widget tests for the change.
3. Run the `update-excerpts` [repository tool command](https://github.com/flutter/packages/blob/main/script/tool/README.md#update-readmemd-from-example-sources), which will update `README.md`. From the root of the packages repository, the command to run this is: `dart run script/tool/bin/flutter_plugin_tools.dart update-excerpts`

### Add or converting a code block

To add a new code block, or fix a legacy code block:

1. Optionally, if this is the first code block to be added, add the following line to the start of the package's `README.md`:

    ```
    <?code-excerpt path-base="example/lib"?>
    ```

    replacing `example/lib` with the directory from which samples will be taken. This sets the base against which other paths are resolved. By default, the base is the path of the markdown file being updated. The base itself is resolved relative to the path of that file as well.

2. Find or write the code that you want to use in the example, then annotate it with `#docregion` and `#enddocregion` comments. Ideally the code should be in one of the examples, but you could also put the code in a test. The key is to put the code somewhere that is analyzed and, ideally, tested in CI.

    For example:

    ~~~
    // #docregion purple
    const Color purple = Color(0xFFE6E6FA);
    // #enddocregion purple
    ~~~

3. Reference the code from the `README.md` (or other markdown file where you want the sample to appear):

    ~~~
    <?code-excerpt "{example/lib/-relative filename} ({excerpt name})"?>
    ```dart
    ```
    ~~~

    So if you added `#docregion Foo` to `example/lib/main.dart` and specified `example/lib` as the path base, you would put the following in `README.md`:

    ~~~
    <?code-excerpt "main.dart (Foo)"?>
    ```dart
    ```
    ~~~

4. Follow the steps in the "Updating" section above to automatically fill the new block with the code excerpt.

### Documentation

Samples can come from any file. In XML and HTML files, use `<!--#docregion sectionname-->` and `<!--#enddocregion sectionname-->`. In CSS files, use `/* #docregion sectionname */` and `/* #enddocregion sectionname */`. In YAML files, use `# #docregion sectionname` and `# #enddocregion sectionname`. In C++, Dart, JS, Kotlin, Swift, and other similar languages, use `// #docregion sectionname` and `// #enddocregion sectionname`.

You can extract multiple segments into one sample by just having multiple docregions with the same name. The regions will be concatenated with a language-appropriate comment such as `// ···`. You can control what this looks like by using the `plaster` control, as in:

    <?code-excerpt "main.dart (Foo)" plaster="more..."?>
    ```dart
    ```

To disable this feature, use `plaster="none"`.

The plaster is indented as much as the `#enddocregion` of the region where the splice occurs.

Regions are maximally unindented before being injected into the markdown files (while preserving relative indentation within the block).

## Swift Migration for 1P Plugins

### Preparation

1. Read the [Swift migration design doc](https://docs.google.com/document/d/1XsaulkJA6_ZSpM7chkQLhQY25sQqhEMiqHMYzw8H85o/edit?resourcekey=0-_cUjF1c0iBvRLKfV-3gK5A), which covers several important topics such as the ["top down approach"](https://docs.google.com/document/d/1XsaulkJA6_ZSpM7chkQLhQY25sQqhEMiqHMYzw8H85o/edit?resourcekey=0-_cUjF1c0iBvRLKfV-3gK5A#heading=h.reauq7bq8vak). Feel free to leave any comments and feedback in the doc.

2. Read Google's [Swift Style Guide](https://google.github.io/swift/) and Apple's [API Design Guidelines](https://www.swift.org/documentation/api-design-guidelines/) (which is endorsed by Google's Swift Style Guide).

3. Pick a plugin that you are interested in from the [umbrella GitHub issue](https://github.com/flutter/flutter/issues/119015). This is for multiple contributors to coordinate with each other, and to avoid duplicate or conflicting work.

### Project Settings

1. Remove custom modulemap ([example](https://github.com/flutter/plugins/pull/6229/files)). This is due to a limitation in Cocoapods ([details](https://github.com/flutter/plugins/pull/6369#issue-1363961940)). This unfortunately means that we cannot use `Test` submodules, and all ObjC headers are exposed publicly during the migration.

2. Add Swift dependency information to the podspec ([example](https://github.com/flutter/packages/blob/617f9d99954fe394dc91258c431c9a2626921e08/packages/quick_actions/quick_actions_ios/ios/quick_actions_ios.podspec#L17-L22)). Note that `.h` and `.m` should be removed from the `spec.source_files` after migration is done.

3. After the plugin class is migrated to Swift, update `pluginClass` in `pubspec.yaml` ([example](https://github.com/flutter/packages/blob/617f9d99954fe394dc91258c431c9a2626921e08/packages/quick_actions/quick_actions_ios/pubspec.yaml#L16)).

### Format your Swift code

1. Install [swift-format](https://github.com/apple/swift-format). If you also contribute to the flutter engine, there is [an issue](https://github.com/flutter/flutter/issues/41129#issuecomment-1400812109) with the swift-format under `depot_tools`. Make sure to set up the export PATH properly to run your own copy of `swift-format`.

2. Run `./script/tool_runner.sh format`

### Testing

To reduce the risk of regression, it is important to backfill unit tests to full coverage before the migration, and maintain full coverage during the migration. If possible (i.e., if adding testing does not require refactoring the plugin to make it testable), adding test coverage should be done in a separate PR before the conversion of the implementation.

We do not report code coverage on the CI, since the number can actually be misleading (for example, a 100% coverage may make PR reviewers think that everything is fine, which is not necessarily true). Instead, you (and your PR reviewer) should evaluate whether the test is complete, by actually reading the test code (rather than just looking at the coverage). You may well want to use a coverage tool as part of your evaluation.

During the migration, especially when backfilling tests, it is possible to discover existing bugs. Migration PRs should just focus on migration and should not contain bug fixes. You can either (1) fix the bug in ObjC first in a separate PR, or (2) if the bug is too hard to fix, just acknowledge the bug in a comment so that someone can fix it in the future.
