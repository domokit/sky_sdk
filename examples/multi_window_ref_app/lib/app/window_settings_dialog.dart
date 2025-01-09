import 'package:flutter/material.dart';
import 'window_settings.dart';

Future<void> windowSettingsDialog(
    BuildContext context, WindowSettings settings) async {
  return await showDialog(
      barrierDismissible: true,
      context: context,
      builder: (BuildContext ctx) {
        return AlertDialog(
          contentPadding: const EdgeInsets.all(4),
          titlePadding: const EdgeInsets.fromLTRB(24, 10, 24, 0),
          title: const Center(
            child: Text('Window Settings'),
          ),
          content: SingleChildScrollView(
              child: ListBody(children: [
            _WindowSettingsTile(
                title: "Regular",
                size: settings.regularSizeNotifier,
                onChange: (Size size) => settings.regularSize = size),
            _WindowSettingsTile(
                title: "Popup",
                size: settings.popupSizeNotifier,
                onChange: (Size size) => settings.popupSize = size),
            const Divider(),
            _AnchorSettingsTile(settings: settings),
          ])),
          actions: [
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: TextButton(
                onPressed: () {
                  Navigator.of(context, rootNavigator: true).pop();
                },
                child: const Text('Apply'),
              ),
            )
          ],
        );
      });
}

class _WindowSettingsTile extends StatelessWidget {
  const _WindowSettingsTile(
      {required this.title, required this.size, required this.onChange});

  final String title;
  final SettingsValueNotifier<Size> size;
  final void Function(Size) onChange;

  @override
  Widget build(BuildContext context) {
    return ListTile(
      title: Text(title),
      subtitle: ListenableBuilder(
          listenable: size,
          builder: (BuildContext ctx, Widget? _) {
            return Row(
              children: [
                Expanded(
                  child: TextFormField(
                    initialValue: size.value.width.toString(),
                    decoration: const InputDecoration(
                      labelText: 'Initial width',
                    ),
                    onChanged: (String value) => onChange(
                        Size(double.tryParse(value) ?? 0, size.value.height)),
                  ),
                ),
                const SizedBox(
                  width: 20,
                ),
                Expanded(
                  child: TextFormField(
                    initialValue: size.value.height.toString(),
                    decoration: const InputDecoration(
                      labelText: 'Initial height',
                    ),
                    onChanged: (String value) => onChange(
                        Size(size.value.width, double.tryParse(value) ?? 0)),
                  ),
                ),
              ],
            );
          }),
    );
  }
}

class _AnchorSettingsTile extends StatelessWidget {
  const _AnchorSettingsTile({required this.settings});

  final WindowSettings settings;

  @override
  Widget build(BuildContext context) {
    return ListenableBuilder(
        listenable: settings.anchorRectNotifier,
        builder: (BuildContext context, Widget? _) {
          return ListenableBuilder(
              listenable: settings.anchorToWindowNotifier,
              builder: (BuildContext context, Widget? _) {
                return ListTile(
                  title: const Text('Anchoring'),
                  subtitle: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Align(
                        alignment: Alignment.centerLeft,
                        child: CheckboxListTile(
                          title: const Text('Anchor to Window'),
                          subtitle: const Text(
                              "Use the parent's window frame as the anchor rectangle"),
                          contentPadding: EdgeInsets.zero,
                          value: settings.anchorToWindowNotifier.value,
                          onChanged: (bool? value) {
                            settings.anchorToWindow = value ?? false;
                          },
                          controlAffinity: ListTileControlAffinity.leading,
                        ),
                      ),
                      Align(
                        alignment: Alignment.centerLeft,
                        child: Text(
                          "View Anchor Rectangle (values will be clamped to the size of the parent view)",
                          style: TextStyle(
                            color: settings.anchorToWindowNotifier.value
                                ? Theme.of(context).disabledColor
                                : Theme.of(context).textTheme.bodyMedium?.color,
                          ),
                        ),
                      ),
                      Row(
                        children: [
                          Expanded(
                            child: TextFormField(
                              enabled: !settings.anchorToWindowNotifier.value,
                              initialValue: settings
                                  .anchorRectNotifier.value.left
                                  .toString(),
                              decoration: const InputDecoration(
                                labelText: 'Left',
                              ),
                              onChanged: settings.anchorToWindowNotifier.value
                                  ? null
                                  : (String value) => settings.anchorRect =
                                      Rect.fromLTWH(
                                          double.tryParse(value) ?? 0,
                                          settings.anchorRectNotifier.value.top,
                                          settings
                                              .anchorRectNotifier.value.width,
                                          settings
                                              .anchorRectNotifier.value.height),
                            ),
                          ),
                          const SizedBox(
                            width: 20,
                          ),
                          Expanded(
                            child: TextFormField(
                              enabled: !settings.anchorToWindowNotifier.value,
                              initialValue: settings
                                  .anchorRectNotifier.value.top
                                  .toString(),
                              decoration: const InputDecoration(
                                labelText: 'Top',
                              ),
                              onChanged: settings.anchorToWindowNotifier.value
                                  ? null
                                  : (String value) => settings.anchorRect =
                                      Rect.fromLTWH(
                                          settings
                                              .anchorRectNotifier.value.left,
                                          double.tryParse(value) ?? 0,
                                          settings
                                              .anchorRectNotifier.value.width,
                                          settings
                                              .anchorRectNotifier.value.height),
                            ),
                          ),
                          const SizedBox(
                            width: 20,
                          ),
                          Expanded(
                            child: TextFormField(
                              enabled: !settings.anchorToWindowNotifier.value,
                              initialValue: settings
                                  .anchorRectNotifier.value.width
                                  .toString(),
                              decoration: const InputDecoration(
                                labelText: 'Width',
                              ),
                              onChanged: settings.anchorToWindowNotifier.value
                                  ? null
                                  : (String value) => settings.anchorRect =
                                      Rect.fromLTWH(
                                          settings
                                              .anchorRectNotifier.value.left,
                                          settings.anchorRectNotifier.value.top,
                                          double.tryParse(value) ?? 0,
                                          settings
                                              .anchorRectNotifier.value.height),
                            ),
                          ),
                          const SizedBox(
                            width: 20,
                          ),
                          Expanded(
                            child: TextFormField(
                              enabled: !settings.anchorToWindowNotifier.value,
                              initialValue: settings
                                  .anchorRectNotifier.value.height
                                  .toString(),
                              decoration: const InputDecoration(
                                labelText: 'Height',
                              ),
                              onChanged: settings.anchorToWindowNotifier.value
                                  ? null
                                  : (String value) => settings
                                          .anchorRectNotifier.value =
                                      Rect.fromLTWH(
                                          settings
                                              .anchorRectNotifier.value.left,
                                          settings.anchorRectNotifier.value.top,
                                          settings
                                              .anchorRectNotifier.value.width,
                                          double.tryParse(value) ?? 0),
                            ),
                          ),
                        ],
                      ),
                    ],
                  ),
                );
              });
        });
  }
}
