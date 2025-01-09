// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async' show Completer;
import 'dart:convert' show base64Decode;
import 'dart:typed_data' show ByteData, Uint8List;
import 'dart:ui' as ui;
import 'package:flutter/material.dart';

/// A 100x100 png in Display P3 colorspace.
const String _displayP3Logo =
    'iVBORw0KGgoAAAANSUhEUgAAAGQAAABkCAYAAABw4pVUAAABdWlDQ1BrQ0dDb2xv'
    'clNwYWNlRGlzcGxheVAzAAAokXWQvUvDUBTFT6tS0DqIDh0cMolD1NIKdnFoKxRF'
    'MFQFq1OafgltfCQpUnETVyn4H1jBWXCwiFRwcXAQRAcR3Zw6KbhoeN6XVNoi3sfl'
    '/Ticc7lcwBtQGSv2AijplpFMxKS11Lrke4OHnlOqZrKooiwK/v276/PR9d5PiFlN'
    'u3YQ2U9cl84ul3aeAlN//V3Vn8maGv3f1EGNGRbgkYmVbYsJ3iUeMWgp4qrgvMvH'
    'gtMunzuelWSc+JZY0gpqhrhJLKc79HwHl4plrbWD2N6f1VeXxRzqUcxhEyYYilBR'
    'gQQF4X/8044/ji1yV2BQLo8CLMpESRETssTz0KFhEjJxCEHqkLhz634PrfvJbW3v'
    'FZhtcM4v2tpCAzidoZPV29p4BBgaAG7qTDVUR+qh9uZywPsJMJgChu8os2HmwiF3'
    'e38M6Hvh/GMM8B0CdpXzryPO7RqFn4Er/QcXKWq8UwZBywAAANplWElmTU0AKgAA'
    'AAgABgESAAMAAAABAAEAAAEaAAUAAAABAAAAVgEbAAUAAAABAAAAXgExAAIAAAAh'
    'AAAAZgEyAAIAAAAUAAAAiIdpAAQAAAABAAAAnAAAAAAAAABIAAAAAQAAAEgAAAAB'
    'QWRvYmUgUGhvdG9zaG9wIDI0LjEgKE1hY2ludG9zaCkAADIwMjM6MDI6MDggMTA6'
    'MTY6NDQAAAOQBAACAAAAFAAAAMagAgAEAAAAAQAAAGSgAwAEAAAAAQAAAGQAAAAA'
    'MjAyMzowMjowOCAxMDoxMzoyMgBgamvuAAAACXBIWXMAAAsTAAALEwEAmpwYAAAL'
    'v2lUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPHg6eG1wbWV0YSB4bWxuczp4PSJh'
    'ZG9iZTpuczptZXRhLyIgeDp4bXB0az0iWE1QIENvcmUgNi4wLjAiPgogICA8cmRm'
    'OlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRm'
    'LXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0i'
    'IgogICAgICAgICAgICB4bWxuczp0aWZmPSJodHRwOi8vbnMuYWRvYmUuY29tL3Rp'
    'ZmYvMS4wLyIKICAgICAgICAgICAgeG1sbnM6ZGM9Imh0dHA6Ly9wdXJsLm9yZy9k'
    'Yy9lbGVtZW50cy8xLjEvIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9u'
    'cy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOnhtcE1NPSJo'
    'dHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvbW0vIgogICAgICAgICAgICB4bWxu'
    'czpzdEV2dD0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL3NUeXBlL1Jlc291'
    'cmNlRXZlbnQjIgogICAgICAgICAgICB4bWxuczpzdFJlZj0iaHR0cDovL25zLmFk'
    'b2JlLmNvbS94YXAvMS4wL3NUeXBlL1Jlc291cmNlUmVmIyIKICAgICAgICAgICAg'
    'eG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8x'
    'LjAvIj4KICAgICAgICAgPHRpZmY6WVJlc29sdXRpb24+NzI8L3RpZmY6WVJlc29s'
    'dXRpb24+CiAgICAgICAgIDx0aWZmOk9yaWVudGF0aW9uPjE8L3RpZmY6T3JpZW50'
    'YXRpb24+CiAgICAgICAgIDx0aWZmOlhSZXNvbHV0aW9uPjcyPC90aWZmOlhSZXNv'
    'bHV0aW9uPgogICAgICAgICA8ZGM6Zm9ybWF0PmltYWdlL3BuZzwvZGM6Zm9ybWF0'
    'PgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAyMy0wMi0wOFQxMDoxNjo0NC0w'
    'ODowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6Q3JlYXRvclRvb2w+'
    'QWRvYmUgUGhvdG9zaG9wIDI0LjEgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9v'
    'bD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMjMtMDItMDhUMTA6MTM6MjIt'
    'MDg6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1ldGFkYXRhRGF0'
    'ZT4yMDIzLTAyLTA4VDEwOjE2OjQ0LTA4OjAwPC94bXA6TWV0YWRhdGFEYXRlPgog'
    'ICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAg'
    'ICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgog'
    'ICAgICAgICAgICAgICAgICA8c3RFdnQ6c29mdHdhcmVBZ2VudD5BZG9iZSBQaG90'
    'b3Nob3AgMjQuMSAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAg'
    'ICAgICAgICAgICAgICAgPHN0RXZ0OmNoYW5nZWQ+Lzwvc3RFdnQ6Y2hhbmdlZD4K'
    'ICAgICAgICAgICAgICAgICAgPHN0RXZ0OndoZW4+MjAyMy0wMi0wOFQxMDoxNjo0'
    'NC0wODowMDwvc3RFdnQ6d2hlbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omlu'
    'c3RhbmNlSUQ+eG1wLmlpZDo3ZmM3YjMyNC0xNDUyLTQ2ZGUtODI2MC0yNGRmMTlh'
    'YjdkNTc8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2'
    'dDphY3Rpb24+c2F2ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgPC9y'
    'ZGY6bGk+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVz'
    'b3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6c29mdHdhcmVBZ2VudD5B'
    'ZG9iZSBQaG90b3Nob3AgMjQuMSAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVB'
    'Z2VudD4KICAgICAgICAgICAgICAgICAgPHN0RXZ0OmNoYW5nZWQ+Lzwvc3RFdnQ6'
    'Y2hhbmdlZD4KICAgICAgICAgICAgICAgICAgPHN0RXZ0OndoZW4+MjAyMy0wMi0w'
    'OFQxMDoxNjo0NC0wODowMDwvc3RFdnQ6d2hlbj4KICAgICAgICAgICAgICAgICAg'
    'PHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpmM2UxNjllMy1jZmZhLTRjZjUtYmQ1'
    'OS02YzgzODljYjk1MDk8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAg'
    'ICAgIDxzdEV2dDphY3Rpb24+c2F2ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAg'
    'ICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAg'
    'PC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8eG1wTU06T3JpZ2luYWxEb2N1bWVu'
    'dElEPnhtcC5kaWQ6Q0RBNDM3N0E4QUY3MTFFREE1NDdCQjYwRDI1MDkyNkQ8L3ht'
    'cE1NOk9yaWdpbmFsRG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOkRvY3VtZW50'
    'SUQ+YWRvYmU6ZG9jaWQ6cGhvdG9zaG9wOjA2M2QwNTUwLTVlNjctMjA0Mi1iN2Vl'
    'LTg4ZDE2ZDc5MDAyYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1N'
    'OkRlcml2ZWRGcm9tIHJkZjpwYXJzZVR5cGU9IlJlc291cmNlIj4KICAgICAgICAg'
    'ICAgPHN0UmVmOmluc3RhbmNlSUQ+eG1wLmlpZDpDREE0Mzc3NzhBRjcxMUVEQTU0'
    'N0JCNjBEMjUwOTI2RDwvc3RSZWY6aW5zdGFuY2VJRD4KICAgICAgICAgICAgPHN0'
    'UmVmOmRvY3VtZW50SUQ+eG1wLmRpZDpDREE0Mzc3ODhBRjcxMUVEQTU0N0JCNjBE'
    'MjUwOTI2RDwvc3RSZWY6ZG9jdW1lbnRJRD4KICAgICAgICAgPC94bXBNTTpEZXJp'
    'dmVkRnJvbT4KICAgICAgICAgPHhtcE1NOkluc3RhbmNlSUQ+eG1wLmlpZDpmM2Ux'
    'NjllMy1jZmZhLTRjZjUtYmQ1OS02YzgzODljYjk1MDk8L3htcE1NOkluc3RhbmNl'
    'SUQ+CiAgICAgICAgIDxwaG90b3Nob3A6SUNDUHJvZmlsZT5EaXNwbGF5IFAzPC9w'
    'aG90b3Nob3A6SUNDUHJvZmlsZT4KICAgICAgICAgPHBob3Rvc2hvcDpDb2xvck1v'
    'ZGU+MzwvcGhvdG9zaG9wOkNvbG9yTW9kZT4KICAgICAgICAgPHBob3Rvc2hvcDpE'
    'b2N1bWVudEFuY2VzdG9ycz4KICAgICAgICAgICAgPHJkZjpCYWc+CiAgICAgICAg'
    'ICAgICAgIDxyZGY6bGk+eG1wLmRpZDpDREE0Mzc3QThBRjcxMUVEQTU0N0JCNjBE'
    'MjUwOTI2RDwvcmRmOmxpPgogICAgICAgICAgICA8L3JkZjpCYWc+CiAgICAgICAg'
    'IDwvcGhvdG9zaG9wOkRvY3VtZW50QW5jZXN0b3JzPgogICAgICA8L3JkZjpEZXNj'
    'cmlwdGlvbj4KICAgPC9yZGY6UkRGPgo8L3g6eG1wbWV0YT4Ka44gfwAACRBJREFU'
    'eAHtnMtyHDUUhtvjccItUClSBQtY5wHCBpOq7L1K+UWy4QH8CjwHWeQVsiBskmUW'
    'WcMCqgzhGiBz4/9knUHTTHvssW496VNR1K1WS0fn05Fa6h7vLZpG/wapxQKjWhQZ'
    '9DizwACksp4wABmAVGaBytQZPGQAUpkFKlNn8JABSGUWqEydwUMqAzKuTJ8k6rAV'
    'MVfYS1A6ZdOrY5W9s0AwFEaaKbxSiL0/RNlA3ld4R2EAIiN0SQjjT2Wy867826Qb'
    'jHd1cywY6LFzc4gZH89IBYM68IzYMFTkbgHpO4ydAlICBnXGlp0YskrBiDl3GNje'
    'AykBA+OlgEG5vQZSCgaGSyW9BVICBnWmll4CKQUj1TAVQu4dkBIwMFgOGNTTKyAl'
    'YOQYpgBh0hsgpWDk8oxeAXlTYABlT43N7ZXWGS4V93Vv6lKNVOaqhyzrKTlhWJ2X'
    'NWSs/NUCwTCM37lh5J4z2iCrfEF1HgwMputkgdVc5wtOEH+NiI62rwOG5LVCensL'
    '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';

void main() => run(Setup.sweepGradient);

enum Setup {
  none,
  image,
  canvasSaveLayer,
  blur,
  drawnImage,
  container,
  linearGradient,
  radialGradient,
  conicalGradient,
  sweepGradient,
}

void run(Setup setup) {
  runApp(MyApp(setup));
}

class MyApp extends StatelessWidget {
  const MyApp(this._setup, {super.key});

  final Setup _setup;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Wide Gamut Test',
      theme: ThemeData(primarySwatch: Colors.blue),
      home: MyHomePage(_setup, title: 'Wide Gamut Test'),
    );
  }
}

class _SaveLayerDrawer extends CustomPainter {
  _SaveLayerDrawer(this._image);

  final ui.Image? _image;

  @override
  void paint(Canvas canvas, Size size) {
    if (_image != null) {
      final Rect imageRect = Rect.fromCenter(
        center: Offset.zero,
        width: _image!.width.toDouble(),
        height: _image!.height.toDouble(),
      );
      canvas.saveLayer(imageRect, Paint());
      canvas.drawRect(
        imageRect.inflate(-_image!.width.toDouble() / 4.0),
        Paint()
          ..style = PaintingStyle.stroke
          ..color = const Color(0xffffffff)
          ..strokeWidth = 3,
      );
      canvas.saveLayer(imageRect, Paint()..blendMode = BlendMode.dstOver);
      canvas.drawImage(_image!, Offset(-_image!.width / 2.0, -_image!.height / 2.0), Paint());
      canvas.restore();
      canvas.restore();
    }
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

Future<ui.Image> _drawImage() async {
  final ui.PictureRecorder recorder = ui.PictureRecorder();
  const Size markerSize = Size(120, 120);
  final double canvasSize = markerSize.height + 3;
  final Canvas canvas = Canvas(recorder, Rect.fromLTWH(0, 0, canvasSize, canvasSize));

  final Paint ovalPaint = Paint()..color = const Color(0xff00ff00);
  final Path ovalPath =
      Path()..addOval(
        Rect.fromLTWH((canvasSize - markerSize.width) / 2, 1, markerSize.width, markerSize.height),
      );
  canvas.drawPath(ovalPath, ovalPaint);

  final ui.Picture picture = recorder.endRecording();
  final ui.Image image = await picture.toImage(canvasSize.toInt(), (canvasSize + 0).toInt());
  final ByteData? byteData = await image.toByteData(format: ui.ImageByteFormat.rawExtendedRgba128);
  final Completer<ui.Image> completer = Completer<ui.Image>();
  ui.decodeImageFromPixels(
    Uint8List.view(byteData!.buffer),
    canvasSize.toInt(),
    canvasSize.toInt(),
    ui.PixelFormat.rgbaFloat32,
    (ui.Image image) {
      completer.complete(image);
    },
  );
  return completer.future;
}

Future<ui.Image> _loadImage() async {
  final ui.ImmutableBuffer buffer = await ui.ImmutableBuffer.fromUint8List(
    base64Decode(_displayP3Logo),
  );
  final ui.ImageDescriptor descriptor = await ui.ImageDescriptor.encoded(buffer);
  final ui.Codec codec = await descriptor.instantiateCodec();
  return (await codec.getNextFrame()).image;
}

class MyHomePage extends StatefulWidget {
  const MyHomePage(this.setup, {super.key, required this.title});

  final Setup setup;
  final String title;

  @override
  State<StatefulWidget> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  ui.Image? _image;

  @override
  void initState() {
    switch (widget.setup) {
      case Setup.canvasSaveLayer:
        _loadImage().then(
          (ui.Image? value) => setState(() {
            _image = value;
          }),
        );
      case Setup.drawnImage:
        _drawImage().then(
          (ui.Image? value) => setState(() {
            _image = value;
          }),
        );
      case Setup.image ||
          Setup.blur ||
          Setup.none ||
          Setup.container ||
          Setup.linearGradient ||
          Setup.radialGradient ||
          Setup.conicalGradient ||
          Setup.sweepGradient:
        break;
    }
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    late Widget imageWidget;
    switch (widget.setup) {
      case Setup.none:
        imageWidget = Container();
      case Setup.image:
        imageWidget = Image.memory(base64Decode(_displayP3Logo));
      case Setup.drawnImage:
        imageWidget = CustomPaint(painter: _SaveLayerDrawer(_image));
      case Setup.canvasSaveLayer:
        imageWidget = CustomPaint(painter: _SaveLayerDrawer(_image));
      case Setup.blur:
        imageWidget = Stack(
          children: <Widget>[
            const ColoredBox(color: Color(0xff00ff00), child: SizedBox(width: 100, height: 100)),
            ImageFiltered(
              imageFilter: ui.ImageFilter.blur(sigmaX: 6, sigmaY: 6),
              child: Image.memory(base64Decode(_displayP3Logo)),
            ),
          ],
        );
      case Setup.container:
        imageWidget = Container(
          width: 100,
          height: 100,
          color: const Color.from(
            alpha: 1,
            red: 1,
            green: 0,
            blue: 0,
            colorSpace: ui.ColorSpace.displayP3,
          ),
        );
      case Setup.linearGradient:
        imageWidget = Container(
          width: 100,
          height: 100,
          decoration: const BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
              colors: <Color>[
                Color.from(
                  alpha: 1,
                  red: 1,
                  green: 0,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
                Color.from(
                  alpha: 1,
                  red: 0,
                  green: 1,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
              ],
            ),
          ),
        );
      case Setup.radialGradient:
        imageWidget = Container(
          width: 100,
          height: 100,
          decoration: const BoxDecoration(
            gradient: RadialGradient(
              colors: <Color>[
                Color.from(
                  alpha: 1,
                  red: 1,
                  green: 0,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
                Color.from(
                  alpha: 1,
                  red: 0,
                  green: 1,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
              ],
            ),
          ),
        );
      case Setup.conicalGradient:
        imageWidget = Container(
          width: 100,
          height: 100,
          decoration: const BoxDecoration(
            gradient: RadialGradient(
              focal: Alignment(0.2, 0.2),
              colors: <Color>[
                Color.from(
                  alpha: 1,
                  red: 1,
                  green: 0,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
                Color.from(
                  alpha: 1,
                  red: 0,
                  green: 1,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
              ],
            ),
          ),
        );
      case Setup.sweepGradient:
        imageWidget = Container(
          width: 100,
          height: 100,
          decoration: const BoxDecoration(
            gradient: SweepGradient(
              colors: <Color>[
                Color.from(
                  alpha: 1,
                  red: 1,
                  green: 0,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
                Color.from(
                  alpha: 1,
                  red: 0,
                  green: 1,
                  blue: 0,
                  colorSpace: ui.ColorSpace.displayP3,
                ),
              ],
            ),
          ),
        );
    }

    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: Center(
        child: Column(mainAxisAlignment: MainAxisAlignment.center, children: <Widget>[imageWidget]),
      ),
    );
  }
}
