// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'dart:io';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:vector_math/vector_math_64.dart' hide Colors;

/// Run:
/// flutter test /Users/polinach/_/flutter_dev/packages/flutter/test/widgets/ticker_provider_memory_test.dart --dart-define LEAK_TRACKING=true
void main() {
  testWidgets('memory', (WidgetTester tester) async {

    await tester.pumpWidget(MyApp());
    tester.runAsync(() => Future.delayed(const Duration(seconds: 10)));
  });

}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home: const MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}

class MyHomePage extends StatelessWidget {
  final String title;

  const MyHomePage({super.key, required this.title});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(title),
      ),
      body: const Center(
        child: MemoryPressureWidget(),
      ),
    );
  }
}

class MemoryPressureWidget extends StatefulWidget {
  const MemoryPressureWidget({super.key});

  @override
  State<MemoryPressureWidget> createState() => _MemoryPressureWidgetState();
}

class _MemoryPressureWidgetState extends State<MemoryPressureWidget>
    with SingleTickerProviderStateMixin {
  bool _on = true;
  late AnimationController _controller;

  final List<PairedWanderer> wanderers = [];

  void _checkRss() {}

  @override
  Widget build(BuildContext context) {
    _checkRss();
    return Stack(
      children: [
        for (final wanderer in wanderers)
          PairedWandererWidget(wanderer: wanderer),
      ],
    );
  }

  @override
  void dispose() {
    if (_on) _controller.dispose();
    super.dispose();
  }

  @override
  void initState() {
    super.initState();
    _createBatch(1000, const Size(500, 500));

    if (_on) _controller = AnimationController(vsync: this);
  }

  void _createBatch(int batchSize, Size worldSize) {
    assert(batchSize.isEven);
    final random = Random(42);
    for (var i = 0; i < batchSize / 2; i++) {
      final a = PairedWanderer(
        velocity: (Vector2.random() - Vector2.all(0.5))..scale(100),
        worldSize: worldSize,
        position: Vector2(worldSize.width * random.nextDouble(),
            worldSize.height * random.nextDouble()),
      );
      final b = PairedWanderer(
        velocity: (Vector2.random() - Vector2.all(0.5))..scale(100),
        worldSize: worldSize,
        position: Vector2(worldSize.width * random.nextDouble(),
            worldSize.height * random.nextDouble()),
      );
      a.otherWanderer = b;
      b.otherWanderer = a;
      wanderers.add(a);
      wanderers.add(b);
    }
  }
}

class PairedWanderer {
  PairedWanderer? otherWanderer;

  final Vector2 position;

  final Vector2 velocity;

  final Size worldSize;

  PairedWanderer({
    required this.position,
    required this.velocity,
    required this.worldSize,
  });

  void update(double dt) {
    position.addScaled(velocity, dt);
    if (otherWanderer != null) {
      position.addScaled(otherWanderer!.velocity, dt * 0.25);
    }

    if (position.x < 0 && velocity.x < 0) {
      velocity.x = -velocity.x;
    } else if (position.x > worldSize.width && velocity.x > 0) {
      velocity.x = -velocity.x;
    }
    if (position.y < 0 && velocity.y < 0) {
      velocity.y = -velocity.y;
    } else if (position.y > worldSize.height && velocity.y > 0) {
      velocity.y = -velocity.y;
    }
  }
}

class PairedWandererWidget extends StatefulWidget {
  final PairedWanderer wanderer;

  const PairedWandererWidget({required this.wanderer, super.key});

  @override
  State<PairedWandererWidget> createState() => _PairedWandererWidgetState();
}

class _PairedWandererWidgetState extends State<PairedWandererWidget>
    with SingleTickerProviderStateMixin {
  late Ticker _ticker;

  Duration _lastElapsed = Duration.zero;

  @override
  void initState() {
    super.initState();
    _ticker = createTicker(_onTick);
    _ticker.start();
  }

  @override
  void dispose() {
    _ticker.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    print('Building widget for ${widget.wanderer}');
    return Positioned(
      left: widget.wanderer.position.x - 128 / 4,
      top: widget.wanderer.position.y - 128 / 4,
      child: const SizedBox(
        width: 8,
        height: 8,
        child: Placeholder(),
      ),
    );
  }

  void _onTick(Duration elapsed) {
    var dt = (elapsed - _lastElapsed).inMicroseconds / 1000000;
    dt = min(dt, 1 / 60);
    widget.wanderer.update(dt);
    _lastElapsed = elapsed;
    setState(() {});
  }
}

