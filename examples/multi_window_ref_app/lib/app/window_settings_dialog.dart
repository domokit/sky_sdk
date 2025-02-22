import 'package:flutter/material.dart';
import 'package:multi_window_ref_app/app/window_settings.dart';

Future<void> windowSettingsDialog(
    BuildContext context, WindowSettings settings) async {
  return await showDialog(
      barrierDismissible: true,
      context: context,
      builder: (BuildContext ctx) {
        return SimpleDialog(
          contentPadding: const EdgeInsets.all(4),
          titlePadding: const EdgeInsets.fromLTRB(24, 10, 24, 0),
          title: const Center(
            child: Text('Window Settings'),
          ),
          children: [
            SizedBox(
              width: 600,
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Row(
                    children: [
                      Expanded(
                        child: ListTile(
                          title: const Text('Regular'),
                          subtitle: ListenableBuilder(
                              listenable: settings,
                              builder: (BuildContext ctx, Widget? _) {
                                return Row(
                                  children: [
                                    Expanded(
                                      child: TextFormField(
                                        initialValue: settings.regularSize.width
                                            .toString(),
                                        decoration: const InputDecoration(
                                          labelText: 'Initial width',
                                        ),
                                        onChanged: (String value) =>
                                            settings.regularSize = Size(
                                                double.tryParse(value) ?? 0,
                                                settings.regularSize.height),
                                      ),
                                    ),
                                    const SizedBox(
                                      width: 20,
                                    ),
                                    Expanded(
                                      child: TextFormField(
                                        initialValue: settings
                                            .regularSize.height
                                            .toString(),
                                        decoration: const InputDecoration(
                                          labelText: 'Initial height',
                                        ),
                                        onChanged: (String value) =>
                                            settings.regularSize = Size(
                                                settings.regularSize.width,
                                                double.tryParse(value) ?? 0),
                                      ),
                                    ),
                                  ],
                                );
                              }),
                        ),
                      ),
                      const SizedBox(
                        width: 10,
                      ),
                    ],
                  ),
                ],
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: TextButton(
                onPressed: () {
                  Navigator.of(context, rootNavigator: true).pop();
                },
                child: const Text('Apply'),
              ),
            ),
            const SizedBox(
              height: 2,
            ),
          ],
        );
      });
}
