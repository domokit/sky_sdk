import 'package:flutter/material.dart';
import 'app/main_window.dart';

void main() {
  final RegularWindowController controller = RegularWindowController();
  runWidget(WindowingApp(children: <Widget>[
    RegularWindow(
        controller: controller,
        preferredSize: const Size(800, 600),
        child: MaterialApp(home: MainWindow(mainController: controller)))
  ]));
}
