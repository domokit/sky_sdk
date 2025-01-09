import 'package:flutter/material.dart';

class WindowSettings extends ChangeNotifier {
  WindowSettings({Size regularSize = const Size(400, 300)})
      : _regularSize = regularSize;

  Size _regularSize;
  Size get regularSize => _regularSize;
  set regularSize(Size value) {
    _regularSize = value;
    notifyListeners();
  }
}
