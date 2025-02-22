import 'package:flutter/widgets.dart';

class KeyedWindowController {
  KeyedWindowController(
      {this.parent,
      this.isMainWindow = false,
      required this.key,
      required this.controller});

  final WindowController? parent;
  final bool isMainWindow;
  final UniqueKey key;
  final WindowController controller;
}

/// Manages a flat list of all of the [WindowController]s that have been
/// created by the application as well as which controller is currently
/// selected by the UI.
class WindowManagerModel extends ChangeNotifier {
  final List<KeyedWindowController> _windows = <KeyedWindowController>[];
  List<KeyedWindowController> get windows => _windows;
  int? _selectedViewId;
  WindowController? get selected {
    if (_selectedViewId == null) {
      return null;
    }

    for (final KeyedWindowController controller in _windows) {
      if (controller.controller.rootView.viewId == _selectedViewId) {
        return controller.controller;
      }
    }

    return null;
  }

  void add(KeyedWindowController window) {
    _windows.add(window);
    notifyListeners();
  }

  void remove(UniqueKey key) {
    _windows.removeWhere((KeyedWindowController window) => window.key == key);
    notifyListeners();
  }

  void select(int? viewId) {
    _selectedViewId = viewId;
    notifyListeners();
  }
}
