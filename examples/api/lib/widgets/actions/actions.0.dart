// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Actions].

void main() => runApp(const ActionsExampleApp());

class ActionsExampleApp extends StatelessWidget {
  const ActionsExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Actions Sample')),
        body: const Center(child: ActionsExample()),
      ),
    );
  }
}

// A simple model class that notifies listeners when it changes.
class Model {
  final ValueNotifier<bool> isDirty = ValueNotifier<bool>(false);
  final ValueNotifier<int> data = ValueNotifier<int>(0);

  int save() {
    if (isDirty.value) {
      debugPrint('Saved Data: ${data.value}');
      isDirty.value = false;
    }
    return data.value;
  }

  void setValue(int newValue) {
    isDirty.value = data.value != newValue;
    data.value = newValue;
  }

  void dispose() {
    isDirty.dispose();
    data.dispose();
  }
}

class ModifyIntent extends Intent {
  const ModifyIntent(this.value);

  final int value;
}

// An Action that modifies the model by setting it to the value that it gets
// from the Intent passed to it when invoked.
class ModifyAction extends Action<ModifyIntent> {
  ModifyAction(this.model);

  final Model model;

  @override
  void invoke(covariant ModifyIntent intent) {
    model.setValue(intent.value);
  }
}

// An intent for saving data.
class SaveIntent extends Intent {
  const SaveIntent();
}

// An Action that saves the data in the model it is created with.
class SaveAction extends Action<SaveIntent> {
  SaveAction(this.model);

  final Model model;

  @override
  int invoke(covariant SaveIntent intent) => model.save();
}

class SaveButton extends StatefulWidget {
  const SaveButton(this.valueNotifier, {super.key});

  final ValueNotifier<bool> valueNotifier;

  @override
  State<SaveButton> createState() => _SaveButtonState();
}

class _SaveButtonState extends State<SaveButton> {
  int _savedValue = 0;

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
      listenable: widget.valueNotifier,
      builder: (BuildContext context, Widget? child) {
        return TextButton.icon(
          icon: const Icon(Icons.save),
          label: Text('$_savedValue'),
          style: ButtonStyle(
            foregroundColor: WidgetStatePropertyAll<Color>(
              widget.valueNotifier.value ? Colors.red : Colors.green,
            ),
          ),
          onPressed: () {
            setState(() {
              _savedValue = Actions.invoke(context, const SaveIntent())! as int;
            });
          },
        );
      },
    );
  }
}

class ActionsExample extends StatefulWidget {
  const ActionsExample({super.key});

  @override
  State<ActionsExample> createState() => _ActionsExampleState();
}

class _ActionsExampleState extends State<ActionsExample> {
  final Model _model = Model();
  int _count = 0;

  @override
  void dispose() {
    _model.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Actions(
      actions: <Type, Action<Intent>>{
        ModifyIntent: ModifyAction(_model),
        SaveIntent: SaveAction(_model),
      },
      child: Builder(
        builder: (BuildContext context) {
          return Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: <Widget>[
              const Spacer(),
              Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: <Widget>[
                  IconButton(
                    icon: const Icon(Icons.exposure_plus_1),
                    onPressed: () {
                      Actions.invoke(context, ModifyIntent(++_count));
                    },
                  ),
                  ListenableBuilder(
                    listenable: _model.data,
                    builder: (BuildContext context, Widget? child) {
                      return Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: Text(
                          'Value: ${_model.data.value}',
                          style: Theme.of(context).textTheme.headlineMedium,
                        ),
                      );
                    },
                  ),
                  IconButton(
                    icon: const Icon(Icons.exposure_minus_1),
                    onPressed: () {
                      Actions.invoke(context, ModifyIntent(--_count));
                    },
                  ),
                ],
              ),
              SaveButton(_model.isDirty),
              const Spacer(),
            ],
          );
        },
      ),
    );
  }
}
