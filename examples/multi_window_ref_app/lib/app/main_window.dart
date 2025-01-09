import 'package:flutter/material.dart';
import 'package:multi_window_ref_app/app/child_window_renderer.dart';

import 'window_settings.dart';
import 'window_settings_dialog.dart';
import 'window_manager_model.dart';
import 'positioner_settings.dart';
import 'custom_positioner_dialog.dart';

class MainWindow extends StatefulWidget {
  MainWindow({super.key, required WindowController mainController}) {
    _windowManagerModel.add(
        KeyedWindowController(isMainWindow: true, controller: mainController));
  }

  final WindowManagerModel _windowManagerModel = WindowManagerModel();
  final WindowSettings _settings = WindowSettings();
  final PositionerSettingsModifier _positionerSettingsModifier =
      PositionerSettingsModifier();

  @override
  State<MainWindow> createState() => _MainWindowState();
}

class _MainWindowState extends State<MainWindow> {
  @override
  Widget build(BuildContext context) {
    final child = Scaffold(
      appBar: AppBar(
        title: const Text('Multi Window Reference App'),
      ),
      body: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Expanded(
            flex: 60,
            child: SingleChildScrollView(
              scrollDirection: Axis.vertical,
              child: _ActiveWindowsTable(
                  windowManagerModel: widget._windowManagerModel),
            ),
          ),
          Expanded(
            flex: 40,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                ListenableBuilder(
                    listenable: widget._windowManagerModel,
                    builder: (BuildContext context, Widget? child) {
                      return _WindowCreatorCard(
                          selectedWindow: widget._windowManagerModel.selected,
                          windowManagerModel: widget._windowManagerModel,
                          windowSettings: widget._settings);
                    }),
                const SizedBox(height: 12),
                _PositionerEditorCard(
                    positionerSettingsModifier:
                        widget._positionerSettingsModifier)
              ],
            ),
          ),
        ],
      ),
    );

    return ViewAnchor(
        view: ChildWindowRenderer(
            windowManagerModel: widget._windowManagerModel,
            windowSettings: widget._settings,
            positionerSettingsModifier: widget._positionerSettingsModifier,
            controller: widget._windowManagerModel.windows[0].controller,
            renderParentlessWindows: true),
        child: child);
  }
}

class _ActiveWindowsTable extends StatelessWidget {
  const _ActiveWindowsTable({required this.windowManagerModel});

  final WindowManagerModel windowManagerModel;

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
        listenable: windowManagerModel,
        builder: (BuildContext context, Widget? widget) {
          return DataTable(
            showBottomBorder: true,
            onSelectAll: (selected) {
              windowManagerModel.select(null);
            },
            columns: const [
              DataColumn(
                label: SizedBox(
                  width: 20,
                  child: Text(
                    'ID',
                    style: TextStyle(
                      fontSize: 16,
                    ),
                  ),
                ),
              ),
              DataColumn(
                label: SizedBox(
                  width: 120,
                  child: Text(
                    'Type',
                    style: TextStyle(
                      fontSize: 16,
                    ),
                  ),
                ),
              ),
              DataColumn(
                  label: SizedBox(
                    width: 20,
                    child: Text(''),
                  ),
                  numeric: true),
            ],
            rows: (windowManagerModel.windows)
                .map<DataRow>((KeyedWindowController controller) {
              return DataRow(
                key: controller.key,
                color: WidgetStateColor.resolveWith((states) {
                  if (states.contains(WidgetState.selected)) {
                    return Theme.of(context).colorScheme.primary.withAlpha(20);
                  }
                  return Colors.transparent;
                }),
                selected: controller.controller == windowManagerModel.selected,
                onSelectChanged: (selected) {
                  if (selected != null) {
                    windowManagerModel.select(
                        selected ? controller.controller.view?.viewId : null);
                  }
                },
                cells: [
                  DataCell(
                    ListenableBuilder(
                        listenable: controller.controller,
                        builder: (BuildContext context, Widget? _) => Text(
                            controller.controller.view != null
                                ? '${controller.controller.view?.viewId}'
                                : 'Loading...')),
                  ),
                  DataCell(
                    ListenableBuilder(
                        listenable: controller.controller,
                        builder: (BuildContext context, Widget? _) => Text(
                            controller.controller.type
                                .toString()
                                .replaceFirst('WindowArchetype.', ''))),
                  ),
                  DataCell(
                    ListenableBuilder(
                        listenable: controller.controller,
                        builder: (BuildContext context, Widget? _) =>
                            IconButton(
                              icon: const Icon(Icons.delete_outlined),
                              onPressed: () async {
                                await controller.controller.destroy();
                              },
                            )),
                  ),
                ],
              );
            }).toList(),
          );
        });
  }
}

class _WindowCreatorCard extends StatelessWidget {
  const _WindowCreatorCard(
      {required this.selectedWindow,
      required this.windowManagerModel,
      required this.windowSettings});

  final WindowController? selectedWindow;
  final WindowManagerModel windowManagerModel;
  final WindowSettings windowSettings;

  @override
  Widget build(BuildContext context) {
    return Card.outlined(
      margin: const EdgeInsets.symmetric(horizontal: 25),
      child: Padding(
        padding: const EdgeInsets.fromLTRB(25, 0, 25, 5),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            const Padding(
              padding: EdgeInsets.only(top: 10, bottom: 10),
              child: Text(
                'New Window',
                style: TextStyle(
                  fontWeight: FontWeight.bold,
                  fontSize: 16.0,
                ),
              ),
            ),
            Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                OutlinedButton(
                  onPressed: () async {
                    windowManagerModel.add(KeyedWindowController(
                        controller: RegularWindowController()));
                  },
                  child: const Text('Regular'),
                ),
                const SizedBox(height: 8),
                OutlinedButton(
                  onPressed: () {
                    windowManagerModel.add(KeyedWindowController(
                        parent: windowManagerModel.selected,
                        controller: PopupWindowController()));
                  },
                  child: Text(windowManagerModel.selected?.view?.viewId != null
                      ? 'Popup of ID ${windowManagerModel.selected!.view?.viewId}'
                      : 'Popup'),
                ),
                const SizedBox(height: 8),
                Container(
                  alignment: Alignment.bottomRight,
                  child: TextButton(
                    child: const Text('SETTINGS'),
                    onPressed: () {
                      windowSettingsDialog(context, windowSettings);
                    },
                  ),
                ),
                const SizedBox(width: 8),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _PositionerEditorCard extends StatefulWidget {
  const _PositionerEditorCard({required this.positionerSettingsModifier});

  final PositionerSettingsModifier positionerSettingsModifier;

  @override
  State<_PositionerEditorCard> createState() => _PositionerEditorCardState();
}

class _PositionerEditorCardState extends State<_PositionerEditorCard> {
  @override
  Widget build(BuildContext context) {
    return Card.outlined(
      margin: const EdgeInsets.symmetric(horizontal: 20),
      child: Padding(
          padding: const EdgeInsets.fromLTRB(20, 0, 15, 5),
          child: ListenableBuilder(
              listenable: widget.positionerSettingsModifier,
              builder: (BuildContext context, _) {
                final positionerSettingsList = widget
                    .positionerSettingsModifier.mapping.positionerSettingsList;
                final selectedName = positionerSettingsList[
                        widget.positionerSettingsModifier.positionerIndex]
                    .name;
                return Column(
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                    const Padding(
                      padding: EdgeInsets.only(top: 10),
                      child: Text(
                        'Positioner',
                        style: TextStyle(
                          fontWeight: FontWeight.bold,
                          fontSize: 16.0,
                        ),
                      ),
                    ),
                    ListTile(
                      title: const Text('Preset'),
                      subtitle: DropdownButton(
                        items: positionerSettingsList
                            .map((PositionerSetting setting) =>
                                DropdownMenuItem<String>(
                                  value: setting.name,
                                  child: Text(setting.name),
                                ))
                            .toList(),
                        value: selectedName,
                        isExpanded: true,
                        focusColor: Colors.transparent,
                        onChanged: (String? value) {
                          setState(() {
                            widget.positionerSettingsModifier.setSelectedIndex(
                              positionerSettingsList.indexWhere(
                                  (setting) => setting.name == value),
                            );
                          });
                        },
                      ),
                    ),
                    Align(
                      alignment: Alignment.bottomRight,
                      child: Padding(
                        padding: const EdgeInsets.only(right: 10),
                        child: TextButton(
                          child: const Text('CUSTOM PRESET'),
                          onPressed: () async {
                            final settings = await customPositionerDialog(
                              context,
                              positionerSettingsList.last,
                            );
                            if (settings != null) {
                              setState(() {
                                final pos = positionerSettingsList.length - 1;
                                widget.positionerSettingsModifier
                                    .setAtIndex(settings, pos);
                                widget.positionerSettingsModifier
                                    .setSelectedIndex(pos);
                              });
                            }
                          },
                        ),
                      ),
                    ),
                    const SizedBox(width: 8),
                  ],
                );
              })),
    );
  }
}
