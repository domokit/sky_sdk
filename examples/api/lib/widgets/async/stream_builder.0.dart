// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';

/// Flutter code sample for [StreamBuilder].

void main() => runApp(const StreamBuilderExampleApp());

class StreamBuilderExampleApp extends StatelessWidget {
  const StreamBuilderExampleApp({super.key});

  static const Duration delay = Duration(seconds: 1);

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: StreamBuilderExample(delay: delay));
  }
}

class StreamBuilderExample extends StatefulWidget {
  const StreamBuilderExample({required this.delay, super.key});

  final Duration delay;

  @override
  State<StreamBuilderExample> createState() => _StreamBuilderExampleState();
}

class _StreamBuilderExampleState extends State<StreamBuilderExample> {
  late final StreamController<int> _controller = StreamController<int>(
    onListen: () async {
      await Future<void>.delayed(widget.delay);

      if (!_controller.isClosed) {
        _controller.add(1);
      }

      await Future<void>.delayed(widget.delay);

      if (!_controller.isClosed) {
        _controller.close();
      }
    },
  );

  Stream<int> get _bids => _controller.stream;

  @override
  void dispose() {
    if (!_controller.isClosed) {
      _controller.close();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTextStyle(
      style: Theme.of(context).textTheme.displayMedium!,
      textAlign: TextAlign.center,
      child: Container(
        alignment: FractionalOffset.center,
        color: Colors.white,
        child: BidsStatus(bids: _bids),
      ),
    );
  }
}

class BidsStatus extends StatelessWidget {
  const BidsStatus({required this.bids, super.key});

  final Stream<int>? bids;

  @override
  Widget build(BuildContext context) {
    return StreamBuilder<int>(
      stream: bids,
      builder: (BuildContext context, AsyncSnapshot<int> snapshot) {
        List<Widget> children;
        if (snapshot.hasError) {
          children = <Widget>[
            const Icon(Icons.error_outline, color: Colors.red, size: 60),
            Padding(
              padding: const EdgeInsets.only(top: 16),
              child: Text('Error: ${snapshot.error}'),
            ),
            Padding(
              padding: const EdgeInsets.only(top: 8),
              child: Text(
                'Stack trace: ${snapshot.stackTrace}',
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
              ),
            ),
          ];
        } else {
          switch (snapshot.connectionState) {
            case ConnectionState.none:
              children = const <Widget>[
                Icon(Icons.info, color: Colors.blue, size: 60),
                Padding(padding: EdgeInsets.only(top: 16), child: Text('Select a lot')),
              ];
            case ConnectionState.waiting:
              children = const <Widget>[
                SizedBox(width: 60, height: 60, child: CircularProgressIndicator()),
                Padding(padding: EdgeInsets.only(top: 16), child: Text('Awaiting bids...')),
              ];
            case ConnectionState.active:
              children = <Widget>[
                const Icon(Icons.check_circle_outline, color: Colors.green, size: 60),
                Padding(padding: const EdgeInsets.only(top: 16), child: Text('\$${snapshot.data}')),
              ];
            case ConnectionState.done:
              children = <Widget>[
                const Icon(Icons.info, color: Colors.blue, size: 60),
                Padding(
                  padding: const EdgeInsets.only(top: 16),
                  child: Text(snapshot.hasData ? '\$${snapshot.data} (closed)' : '(closed)'),
                ),
              ];
          }
        }

        return Column(mainAxisAlignment: MainAxisAlignment.center, children: children);
      },
    );
  }
}
