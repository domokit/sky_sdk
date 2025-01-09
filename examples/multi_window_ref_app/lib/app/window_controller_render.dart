import 'package:flutter/material.dart';
import 'package:multi_window_ref_app/app/popup_window_content.dart';
import 'package:multi_window_ref_app/app/positioner_settings.dart';
import 'regular_window_content.dart';
import 'window_manager_model.dart';
import 'window_settings.dart';

class WindowControllerRender extends StatelessWidget {
  const WindowControllerRender(
      {required this.controller,
      required this.onDestroyed,
      required this.onError,
      required this.windowSettings,
      required this.positionerSettingsModifier,
      required this.windowManagerModel,
      required super.key});

  final WindowController controller;
  final VoidCallback onDestroyed;
  final VoidCallback onError;
  final WindowSettings windowSettings;
  final PositionerSettingsModifier positionerSettingsModifier;
  final WindowManagerModel windowManagerModel;

  @override
  Widget build(BuildContext context) {
    switch (controller.type) {
      case WindowArchetype.regular:
        return RegularWindow(
            key: key,
            onDestroyed: onDestroyed,
            onError: (String? reason) => onError(),
            preferredSize: windowSettings.regularSizeNotifier.value,
            controller: controller as RegularWindowController,
            child: RegularWindowContent(
                controller: controller as RegularWindowController,
                windowSettings: windowSettings,
                positionerSettingsModifier: positionerSettingsModifier,
                windowManagerModel: windowManagerModel));
      case WindowArchetype.popup:
        return PopupWindow(
            key: key,
            onDestroyed: onDestroyed,
            onError: (String? reason) => onError(),
            preferredSize: windowSettings.popupSizeNotifier.value,
            anchorRect: windowSettings.anchorToWindowNotifier.value
                ? null
                : windowSettings.anchorRectNotifier.value,
            positioner: WindowPositioner(
                parentAnchor: positionerSettingsModifier.selected.parentAnchor,
                childAnchor: positionerSettingsModifier.selected.childAnchor,
                offset: positionerSettingsModifier.selected.offset,
                constraintAdjustment:
                    positionerSettingsModifier.selected.constraintAdjustments),
            controller: controller as PopupWindowController,
            child: PopupWindowContent(
                controller: controller as PopupWindowController,
                windowSettings: windowSettings,
                positionerSettingsModifier: positionerSettingsModifier,
                windowManagerModel: windowManagerModel));
      default:
        throw UnimplementedError(
            "The provided window type does not have an implementation");
    }
  }
}
