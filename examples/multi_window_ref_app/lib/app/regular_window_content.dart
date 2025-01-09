import 'package:flutter/material.dart';
import 'package:multi_window_ref_app/app/positioner_settings.dart';
import 'child_window_renderer.dart';
import 'window_manager_model.dart';
import 'window_settings.dart';
import 'dart:math';
import 'package:vector_math/vector_math_64.dart';

class RegularWindowContent extends StatefulWidget {
  const RegularWindowContent(
      {super.key,
      required this.controller,
      required this.windowSettings,
      required this.positionerSettingsModifier,
      required this.windowManagerModel});

  final RegularWindowController controller;
  final WindowSettings windowSettings;
  final PositionerSettingsModifier positionerSettingsModifier;
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
    return ViewAnchor(
        view: ChildWindowRenderer(
            windowManagerModel: widget.windowManagerModel,
            windowSettings: widget.windowSettings,
            positionerSettingsModifier: widget.positionerSettingsModifier,
            controller: widget.controller),
        child: Scaffold(
            appBar: AppBar(title: Text('${widget.controller.type}')),
            body: SingleChildScrollView(
                child: Center(
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
                          listenable: widget.controller,
                          builder: (BuildContext context, Widget? _) {
                            return Text(
                              'View #${widget.controller.view?.viewId ?? "Unknown"}\n'
                              'Parent View: ${widget.controller.parentViewId}\n'
                              'Logical Size: ${widget.controller.size?.width ?? "?"}\u00D7${widget.controller.size?.height ?? "?"}\n'
                              'DPR: ${MediaQuery.of(context).devicePixelRatio}',
                              textAlign: TextAlign.center,
                            );
                          })
                    ],
                  ),
                ],
              ),
            ))));
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
