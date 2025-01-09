import 'package:flutter/material.dart';
import 'package:multi_window_ref_app/app/window_controller_render.dart';
import 'package:multi_window_ref_app/app/window_manager_model.dart';
import 'package:multi_window_ref_app/app/window_settings.dart';
import 'dart:math';
import 'package:vector_math/vector_math_64.dart';

class RegularWindowContent extends StatefulWidget {
  const RegularWindowContent(
      {super.key,
      required this.window,
      required this.windowSettings,
      required this.windowManagerModel});

  final RegularWindowController window;
  final WindowSettings windowSettings;
  final WindowManagerModel windowManagerModel;

  @override
  State<StatefulWidget> createState() => _RegularWindowContentState();
}

class _RegularWindowContentState extends State<RegularWindowContent>
    with SingleTickerProviderStateMixin {
  late final AnimationController _animation;
  late final Color cubeColor;

  @override
  void initState() {
    super.initState();
    _animation = AnimationController(
      vsync: this,
      lowerBound: 0,
      upperBound: 2 * pi,
      duration: const Duration(seconds: 15),
    )..repeat();
    cubeColor = _generateRandomDarkColor();
  }

  @override
  void dispose() {
    _animation.dispose();
    super.dispose();
  }

  Color _generateRandomDarkColor() {
    final random = Random();
    const int lowerBound = 32;
    const int span = 160;
    int red = lowerBound + random.nextInt(span);
    int green = lowerBound + random.nextInt(span);
    int blue = lowerBound + random.nextInt(span);
    return Color.fromARGB(255, red, green, blue);
  }

  @override
  Widget build(BuildContext context) {
    final dpr = MediaQuery.of(context).devicePixelRatio;

    final child = Scaffold(
      appBar: AppBar(title: Text('${widget.window.type}')),
      body: Center(
          child: Row(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              AnimatedBuilder(
                animation: _animation,
                builder: (context, child) {
                  return CustomPaint(
                    size: const Size(200, 200),
                    painter: _RotatedWireCube(
                        angle: _animation.value, color: cubeColor),
                  );
                },
              ),
            ],
          ),
          Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              ElevatedButton(
                onPressed: () {
                  widget.windowManagerModel.add(KeyedWindowController(
                      controller: RegularWindowController()));
                },
                child: const Text('Create Regular Window'),
              ),
              const SizedBox(height: 20),
              ListenableBuilder(
                  listenable: widget.window,
                  builder: (BuildContext context, Widget? _) {
                    return Text(
                      'View #${widget.window.view?.viewId ?? "Unknown"}\n'
                      'Parent View: ${widget.window.parentViewId}\n'
                      'View Size: ${(widget.window.view!.physicalSize.width / dpr).toStringAsFixed(1)}\u00D7${(widget.window.view!.physicalSize.height / dpr).toStringAsFixed(1)}\n'
                      'Window Size: ${widget.window.size?.width}\u00D7${widget.window.size?.height}\n'
                      'Device Pixel Ratio: $dpr',
                      textAlign: TextAlign.center,
                    );
                  })
            ],
          ),
        ],
      )),
    );

    return ViewAnchor(
        view: ListenableBuilder(
            listenable: widget.windowManagerModel,
            builder: (BuildContext context, Widget? _) {
              final List<Widget> childViews = <Widget>[];
              for (final KeyedWindowController controller
                  in widget.windowManagerModel.windows) {
                if (controller.parent == widget.window) {
                  childViews.add(WindowControllerRender(
                    controller: controller.controller,
                    key: controller.key,
                    windowSettings: widget.windowSettings,
                    windowManagerModel: widget.windowManagerModel,
                    onDestroyed: () =>
                        widget.windowManagerModel.remove(controller),
                    onError: () => widget.windowManagerModel.remove(controller),
                  ));
                }
              }

              return ViewCollection(views: childViews);
            }),
        child: child);
  }
}

class _RotatedWireCube extends CustomPainter {
  static List<Vector3> vertices = [
    Vector3(-0.5, -0.5, -0.5),
    Vector3(0.5, -0.5, -0.5),
    Vector3(0.5, 0.5, -0.5),
    Vector3(-0.5, 0.5, -0.5),
    Vector3(-0.5, -0.5, 0.5),
    Vector3(0.5, -0.5, 0.5),
    Vector3(0.5, 0.5, 0.5),
    Vector3(-0.5, 0.5, 0.5),
  ];

  static const List<List<int>> edges = [
    [0, 1], [1, 2], [2, 3], [3, 0], // Front face
    [4, 5], [5, 6], [6, 7], [7, 4], // Back face
    [0, 4], [1, 5], [2, 6], [3, 7], // Connecting front and back
  ];

  final double angle;
  final Color color;

  _RotatedWireCube({required this.angle, required this.color});

  Offset scaleAndCenter(Vector3 point, double size, Offset center) {
    final scale = size / 2;
    return Offset(center.dx + point.x * scale, center.dy - point.y * scale);
  }

  @override
  void paint(Canvas canvas, Size size) {
    final rotatedVertices = vertices
        .map((vertex) => Matrix4.rotationX(angle).transformed3(vertex))
        .map((vertex) => Matrix4.rotationY(angle).transformed3(vertex))
        .map((vertex) => Matrix4.rotationZ(angle).transformed3(vertex))
        .toList();

    final center = Offset(size.width / 2, size.height / 2);

    final paint = Paint()
      ..color = color
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    for (var edge in edges) {
      final p1 = scaleAndCenter(rotatedVertices[edge[0]], size.width, center);
      final p2 = scaleAndCenter(rotatedVertices[edge[1]], size.width, center);
      canvas.drawLine(p1, p2, paint);
    }
  }

  @override
  bool shouldRepaint(_RotatedWireCube oldDelegate) => true;
}
