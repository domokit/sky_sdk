import 'package:flutter/material.dart';

class PositionerSetting {
  PositionerSetting(
      {required this.name,
      required this.parentAnchor,
      required this.childAnchor,
      required this.offset,
      required this.constraintAdjustments});
  final String name;
  final WindowPositionerAnchor parentAnchor;
  final WindowPositionerAnchor childAnchor;
  final Offset offset;
  final Set<WindowPositionerConstraintAdjustment> constraintAdjustments;
}

class PositionerSettingsContainer {
  final List<PositionerSetting> positionerSettingsList = [
    PositionerSetting(
        name: "Left",
        parentAnchor: WindowPositionerAnchor.left,
        childAnchor: WindowPositionerAnchor.right,
        offset: const Offset(0, 0),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        }),
    PositionerSetting(
        name: "Right",
        parentAnchor: WindowPositionerAnchor.right,
        childAnchor: WindowPositionerAnchor.left,
        offset: const Offset(0, 0),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        }),
    PositionerSetting(
        name: "Bottom Left",
        parentAnchor: WindowPositionerAnchor.bottomLeft,
        childAnchor: WindowPositionerAnchor.topRight,
        offset: const Offset(0, 0),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        }),
    PositionerSetting(
        name: "Bottom",
        parentAnchor: WindowPositionerAnchor.bottom,
        childAnchor: WindowPositionerAnchor.top,
        offset: const Offset(0, 0),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        }),
    PositionerSetting(
        name: "Bottom Right",
        parentAnchor: WindowPositionerAnchor.bottomRight,
        childAnchor: WindowPositionerAnchor.topLeft,
        offset: const Offset(0, 0),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        }),
    PositionerSetting(
        name: "Center",
        parentAnchor: WindowPositionerAnchor.center,
        childAnchor: WindowPositionerAnchor.center,
        offset: const Offset(0, 0),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        }),
    PositionerSetting(
        name: "Custom",
        parentAnchor: WindowPositionerAnchor.left,
        childAnchor: WindowPositionerAnchor.right,
        offset: const Offset(0, 50),
        constraintAdjustments: {
          WindowPositionerConstraintAdjustment.slideX,
          WindowPositionerConstraintAdjustment.slideY
        })
  ];
}

class PositionerSettingsModifier with ChangeNotifier {
  int _positionerIndex = 0;
  int get positionerIndex => _positionerIndex;

  final PositionerSettingsContainer _mapping = PositionerSettingsContainer();
  PositionerSettingsContainer get mapping => _mapping;

  void setAtIndex(PositionerSetting setting, int index) {
    if (index >= 0 && index < _mapping.positionerSettingsList.length) {
      _mapping.positionerSettingsList[index] = setting;
      notifyListeners();
    }
  }

  void setSelectedIndex(int index) {
    _positionerIndex =
        index.clamp(0, _mapping.positionerSettingsList.length - 1);
    notifyListeners();
  }

  PositionerSetting? getPositionerSetting(int? index) =>
      index == null ? null : _mapping.positionerSettingsList[index];

  PositionerSetting get selected => getPositionerSetting(_positionerIndex)!;
}
