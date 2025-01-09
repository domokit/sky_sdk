import 'package:flutter/material.dart';

class SettingsValueNotifier<T> extends ChangeNotifier {
  SettingsValueNotifier({required T value}) : _value = value;

  T _value;
  T get value => _value;
  set value(T v) {
    _value = v;
    notifyListeners();
  }
}

class WindowSettings {
  WindowSettings(
      {Size regularSize = const Size(400, 300),
      Size popupSize = const Size(200, 300),
      Rect anchorRect = const Rect.fromLTWH(0, 0, 1000, 1000),
      bool anchorToWindow = true})
      : _regularSize = SettingsValueNotifier(value: regularSize),
        _popupSize = SettingsValueNotifier(value: popupSize),
        _anchorRect = SettingsValueNotifier(value: anchorRect),
        _anchorToWindow = SettingsValueNotifier(value: anchorToWindow);

  final SettingsValueNotifier<Size> _regularSize;
  SettingsValueNotifier<Size> get regularSizeNotifier => _regularSize;
  set regularSize(Size value) {
    _regularSize.value = value;
  }

  final SettingsValueNotifier<Size> _popupSize;
  SettingsValueNotifier<Size> get popupSizeNotifier => _popupSize;
  set popupSize(Size value) {
    _popupSize.value = value;
  }

  final SettingsValueNotifier<Rect> _anchorRect;
  SettingsValueNotifier<Rect> get anchorRectNotifier => _anchorRect;
  set anchorRect(Rect value) {
    _anchorRect.value = value;
  }

  final SettingsValueNotifier<bool> _anchorToWindow;
  SettingsValueNotifier<bool> get anchorToWindowNotifier => _anchorToWindow;
  set anchorToWindow(bool value) {
    _anchorToWindow.value = value;
  }
}
