import 'package:flutter/material.dart';
import 'regular_window_content.dart';
import 'window_manager_model.dart';
import 'window_settings.dart';

class WindowControllerRender extends StatelessWidget {
  const WindowControllerRender(
      {required this.controller,
      required this.onDestroyed,
      required this.onError,
      required this.windowSettings,
      required this.windowManagerModel,
      required super.key});

  final WindowController controller;
  final VoidCallback onDestroyed;
  final VoidCallback onError;
  final WindowSettings windowSettings;
  final WindowManagerModel windowManagerModel;

  @override
  Widget build(BuildContext context) {
    switch (controller.type) {
      case WindowArchetype.regular:
        return RegularWindow(
            key: key,
            onDestroyed: onDestroyed,
            onError: (String? reason) => onError(),
            preferredSize: windowSettings.regularSize,
            controller: controller as RegularWindowController,
            child: RegularWindowContent(
                window: controller as RegularWindowController,
                windowSettings: windowSettings,
                windowManagerModel: windowManagerModel));
      default:
        throw UnimplementedError(
            "The provided window type does not have an implementation");
    }
  }
}
