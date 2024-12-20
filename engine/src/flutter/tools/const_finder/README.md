# Const Finder

This program uses package:kernel from the Dart SDK in //third_party.

A snapshot is created via the build rules in BUILD.gn. This is then vended
to the Flutter tool, which uses it to find `const` creations of `IconData`
classes. The information from this can then be passed to the `font-subset` tool
to create a smaller icon font file specific to the application.

Once [flutter/flutter#47162](https://github.com/flutter/flutter/issues/47162) is
resolved, this package should be moved to the flutter tool.
