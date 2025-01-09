// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:collection/collection.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() {
  // Disconnects semantics listener for testing purposes.
  // If the test passes, LifeCycleSpy will rewire the semantics listener back.
  SwitchableSemanticsBinding.ensureInitialized();
  assert(!SwitchableSemanticsBinding.instance.semanticsEnabled);

  runApp(const LifeCycleSpy());
}

/// A Test widget that spies on app life cycle changes.
///
/// It will collect the AppLifecycleState sequence during its lifetime, and it
/// will rewire semantics harness if the sequence it receives matches the
/// expected list.
///
/// Rewiring semantics is a signal to native IOS test that the test has passed.
class LifeCycleSpy extends StatefulWidget {
  const LifeCycleSpy({super.key});

  @override
  State<LifeCycleSpy> createState() => _LifeCycleSpyState();
}

class _LifeCycleSpyState extends State<LifeCycleSpy> with WidgetsBindingObserver {
  final List<AppLifecycleState> _expectedLifeCycleSequence = <AppLifecycleState>[
    AppLifecycleState.detached,
    AppLifecycleState.inactive,
    AppLifecycleState.resumed,
  ];
  List<AppLifecycleState?>? _actualLifeCycleSequence;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _actualLifeCycleSequence = <AppLifecycleState?>[ServicesBinding.instance.lifecycleState];
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    setState(() {
      _actualLifeCycleSequence = List<AppLifecycleState>.from(_actualLifeCycleSequence!);
      _actualLifeCycleSequence?.add(state);
    });
  }

  @override
  Widget build(BuildContext context) {
    if (const ListEquality<AppLifecycleState?>().equals(
      _actualLifeCycleSequence,
      _expectedLifeCycleSequence,
    )) {
      // Rewires the semantics harness if test passes.
      SwitchableSemanticsBinding.instance.semanticsEnabled = true;
    }
    return const MaterialApp(title: 'Flutter View', home: Text('test'));
  }
}

class SwitchableSemanticsBinding extends WidgetsFlutterBinding {
  static SwitchableSemanticsBinding get instance => BindingBase.checkInstance(_instance);
  static SwitchableSemanticsBinding? _instance;

  static SwitchableSemanticsBinding ensureInitialized() {
    if (_instance == null) {
      SwitchableSemanticsBinding();
    }
    return SwitchableSemanticsBinding.instance;
  }

  VoidCallback? _originalSemanticsListener;

  @override
  void initInstances() {
    super.initInstances();
    _instance = this;
    _updateHandler();
  }

  @override
  bool get semanticsEnabled => _semanticsEnabled.value;
  final ValueNotifier<bool> _semanticsEnabled = ValueNotifier<bool>(false);
  set semanticsEnabled(bool value) {
    _semanticsEnabled.value = value;
    _updateHandler();
  }

  void _updateHandler() {
    if (_semanticsEnabled.value) {
      platformDispatcher.onSemanticsEnabledChanged = _originalSemanticsListener;
      _originalSemanticsListener = null;
    } else {
      _originalSemanticsListener = platformDispatcher.onSemanticsEnabledChanged;
      platformDispatcher.onSemanticsEnabledChanged = null;
    }
  }

  @override
  void addSemanticsEnabledListener(VoidCallback listener) {
    _semanticsEnabled.addListener(listener);
  }

  @override
  void removeSemanticsEnabledListener(VoidCallback listener) {
    _semanticsEnabled.removeListener(listener);
  }
}
