import 'package:flutter/material.dart';
import 'app/main_window.dart';

void main() {
  final RegularWindowController controller = RegularWindowController(
    size: const Size(800, 600),
    sizeConstraints: const BoxConstraints(minWidth: 640, minHeight: 480),
    title: "Multi-Window Reference Application",
  );
  runWidget(RegularWindow(
      controller: controller,
      child: MaterialApp(home: MainWindow(mainController: controller))));
}
