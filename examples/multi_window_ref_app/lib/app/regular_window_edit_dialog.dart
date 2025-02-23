import 'package:flutter/material.dart';

void showRegularWindowEditDialog(BuildContext context,
    {double? initialWidth,
    double? initialHeight,
    String? initialTitle,
    WindowState? initialState,
    Function(double?, double?, String?, WindowState)? onSave}) {
  final TextEditingController widthController =
      TextEditingController(text: initialWidth?.toString() ?? '');
  final TextEditingController heightController =
      TextEditingController(text: initialHeight?.toString() ?? '');
  final TextEditingController titleController =
      TextEditingController(text: initialTitle ?? '');

  showDialog(
    context: context,
    builder: (context) {
      WindowState selectedState = initialState ?? WindowState.restored;

      return AlertDialog(
        title: Text("Edit Window Properties"),
        content: StatefulBuilder(
          builder: (context, setState) {
            return Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                TextField(
                  controller: widthController,
                  keyboardType: TextInputType.number,
                  decoration: InputDecoration(labelText: "Width"),
                ),
                TextField(
                  controller: heightController,
                  keyboardType: TextInputType.number,
                  decoration: InputDecoration(labelText: "Height"),
                ),
                TextField(
                  controller: titleController,
                  decoration: InputDecoration(labelText: "Title"),
                ),
                DropdownButton<WindowState>(
                  value: selectedState,
                  onChanged: (WindowState? newState) {
                    if (newState != null) {
                      setState(() {
                        selectedState = newState;
                      });
                    }
                  },
                  items: WindowState.values.map((WindowState state) {
                    return DropdownMenuItem<WindowState>(
                      value: state,
                      child: Text(
                        state.toString().split('.').last[0].toUpperCase() +
                            state.toString().split('.').last.substring(1),
                      ),
                    );
                  }).toList(),
                ),
              ],
            );
          },
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(context).pop(),
            child: Text("Cancel"),
          ),
          TextButton(
            onPressed: () {
              double? width = double.tryParse(widthController.text);
              double? height = double.tryParse(heightController.text);
              String? title =
                  titleController.text.isEmpty ? null : titleController.text;

              onSave?.call(width, height, title, selectedState);
              Navigator.of(context).pop();
            },
            child: Text("Save"),
          ),
        ],
      );
    },
  );
}
