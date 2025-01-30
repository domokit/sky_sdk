// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:code_builder/code_builder.dart';

import '../base/file_system.dart';
import '../project.dart';
import 'preview_detector.dart';

/// Generates the Dart source responsible for importing widget previews from the developer's project
/// into the widget preview scaffold.
class PreviewCodeGenerator {
  PreviewCodeGenerator({required this.widgetPreviewScaffoldProject, required this.fs});

  final FileSystem fs;

  /// The project for the widget preview scaffold found under `.dart_tool/` in the developer's
  /// project.
  final FlutterProject widgetPreviewScaffoldProject;

  static const String generatedPreviewFilePath = 'lib/generated_preview.dart';

  /// Generates code used by the widget preview scaffold based on the preview instances listed in
  /// [previews].
  ///
  /// The generated file will contain a single top level function named `previews()` which returns
  /// a `List<WidgetPreview>` that contains each widget preview defined in [previews].
  ///
  /// An example of a formatted generated file containing previews from two files could be:
  ///
  /// ```dart
  /// import 'package:foo/foo.dart' as _i1;
  /// import 'package:foo/src/bar.dart' as _i2;
  /// import 'package:widget_preview/widget_preview.dart';
  ///
  /// List<WidgetPreview> previews() => [
  ///   _i1.fooPreview(),
  ///   _i2.barPreview1(),
  ///   _i3.barPreview2(),
  /// ];
  /// ```
  void populatePreviewsInGeneratedPreviewScaffold(PreviewMapping previews) {
    final Library lib = Library(
      (LibraryBuilder b) => b.body.addAll(<Spec>[
        Directive.import(
          // TODO(bkonyi): update with actual location in the framework
          'package:widget_preview/widget_preview.dart',
        ),
        Method(
          (MethodBuilder b) =>
              b
                ..body =
                    literalList(<Object?>[
                      for (final MapEntry<String, List<String>>(
                            key: String path,
                            value: List<String> previewMethods,
                          )
                          in previews.entries) ...<Object?>[
                        for (final String method in previewMethods)
                          refer(method, path).call(<Expression>[]),
                      ],
                    ]).code
                ..name = 'previews'
                ..returns = refer('List<WidgetPreview>'),
        ),
      ]),
    );
    final DartEmitter emitter = DartEmitter.scoped(useNullSafetySyntax: true);
    final File generatedPreviewFile = fs.file(
      widgetPreviewScaffoldProject.directory.uri.resolve(generatedPreviewFilePath),
    );
    // TODO(bkonyi): do we want to bother with formatting this?
    generatedPreviewFile.writeAsStringSync(lib.accept(emitter).toString());
  }
}
