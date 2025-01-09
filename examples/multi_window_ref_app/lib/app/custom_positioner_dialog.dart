import 'package:flutter/material.dart';
import 'positioner_settings.dart';

Future<PositionerSetting?> customPositionerDialog(
  BuildContext context,
  PositionerSetting settings,
) async {
  return await showDialog<PositionerSetting>(
      barrierDismissible: true,
      context: context,
      builder: (BuildContext ctx) {
        String name = settings.name;
        Offset offset = settings.offset;
        final TextEditingController controllerX = TextEditingController();
        final TextEditingController controllerY = TextEditingController();
        final List<String> anchor = WindowPositionerAnchor.values
            .map((e) => e.toString().split('.').last)
            .toList();
        bool slideX = settings.constraintAdjustments
            .contains(WindowPositionerConstraintAdjustment.slideX);
        bool slideY = settings.constraintAdjustments
            .contains(WindowPositionerConstraintAdjustment.slideY);
        bool flipX = settings.constraintAdjustments
            .contains(WindowPositionerConstraintAdjustment.flipX);
        bool flipY = settings.constraintAdjustments
            .contains(WindowPositionerConstraintAdjustment.flipY);
        bool resizeX = settings.constraintAdjustments
            .contains(WindowPositionerConstraintAdjustment.resizeX);
        bool resizeY = settings.constraintAdjustments
            .contains(WindowPositionerConstraintAdjustment.resizeY);
        String parentAnchor = settings.parentAnchor.toString().split('.').last;
        String childAnchor = settings.childAnchor.toString().split('.').last;
        controllerX.text = offset.dx.toString();
        controllerY.text = offset.dy.toString();

        return StatefulBuilder(
            builder: (BuildContext ctx, StateSetter setState) {
          return SimpleDialog(
            contentPadding: const EdgeInsets.all(4),
            titlePadding: const EdgeInsets.fromLTRB(24, 12, 24, 0),
            title: const Center(
              child: Text('Custom Positioner'),
            ),
            children: [
              ListTile(
                title: const Text('Parent Anchor'),
                subtitle: DropdownButton(
                  items: anchor.map<DropdownMenuItem<String>>((String value) {
                    return DropdownMenuItem<String>(
                      value: value,
                      child: Text(value),
                    );
                  }).toList(),
                  value: parentAnchor,
                  isExpanded: true,
                  focusColor: Colors.transparent,
                  onChanged: (String? value) {
                    setState(() {
                      parentAnchor = value!;
                    });
                  },
                ),
              ),
              ListTile(
                title: const Text('Child Anchor'),
                subtitle: DropdownButton(
                  items: anchor.map<DropdownMenuItem<String>>((String value) {
                    return DropdownMenuItem<String>(
                      value: value,
                      child: Text(value),
                    );
                  }).toList(),
                  value: childAnchor,
                  isExpanded: true,
                  focusColor: Colors.transparent,
                  onChanged: (String? value) {
                    setState(() {
                      childAnchor = value!;
                    });
                  },
                ),
              ),
              ListTile(
                title: const Text('Offset'),
                subtitle: Row(
                  children: [
                    Expanded(
                      child: TextFormField(
                        controller: controllerX,
                        decoration: const InputDecoration(
                          labelText: 'X',
                        ),
                        onChanged: (String value) => setState(
                          () => offset =
                              Offset(double.tryParse(value) ?? 0, offset.dy),
                        ),
                      ),
                    ),
                    const SizedBox(
                      width: 20,
                    ),
                    Expanded(
                      child: TextFormField(
                        controller: controllerY,
                        decoration: const InputDecoration(
                          labelText: 'Y',
                        ),
                        onChanged: (String value) => setState(
                          () => offset =
                              Offset(offset.dx, double.tryParse(value) ?? 0),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
              const SizedBox(
                height: 4,
              ),
              ListTile(
                title: const Text('Constraint Adjustments'),
                subtitle: Column(
                  children: [
                    const SizedBox(
                      height: 4,
                    ),
                    Row(
                      children: [
                        Expanded(
                          child: Table(
                              defaultVerticalAlignment:
                                  TableCellVerticalAlignment.middle,
                              children: [
                                const TableRow(children: [
                                  TableCell(
                                    child: Text(''),
                                  ),
                                  TableCell(
                                    child: Center(
                                      child: Text('X'),
                                    ),
                                  ),
                                  TableCell(
                                    child: Center(
                                      child: Text('Y'),
                                    ),
                                  ),
                                ]),
                                TableRow(children: [
                                  const TableCell(
                                    child: Text('Slide'),
                                  ),
                                  TableCell(
                                    child: Checkbox(
                                      value: slideX,
                                      onChanged: (bool? value) =>
                                          setState(() => slideX = value!),
                                    ),
                                  ),
                                  TableCell(
                                    child: Checkbox(
                                      value: slideY,
                                      onChanged: (bool? value) =>
                                          setState(() => slideY = value!),
                                    ),
                                  ),
                                ]),
                                TableRow(children: [
                                  const TableCell(
                                    child: Text('Flip'),
                                  ),
                                  TableCell(
                                    child: Checkbox(
                                      value: flipX,
                                      onChanged: (bool? value) =>
                                          setState(() => flipX = value!),
                                    ),
                                  ),
                                  TableCell(
                                    child: Checkbox(
                                      value: flipY,
                                      onChanged: (bool? value) =>
                                          setState(() => flipY = value!),
                                    ),
                                  ),
                                ]),
                                TableRow(children: [
                                  const TableCell(
                                    child: Text('Resize'),
                                  ),
                                  TableCell(
                                    child: Checkbox(
                                      value: resizeX,
                                      onChanged: (bool? value) =>
                                          setState(() => resizeX = value!),
                                    ),
                                  ),
                                  TableCell(
                                    child: Checkbox(
                                      value: resizeY,
                                      onChanged: (bool? value) =>
                                          setState(() => resizeY = value!),
                                    ),
                                  ),
                                ]),
                              ]),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
              Padding(
                padding: const EdgeInsets.fromLTRB(16, 0, 16, 4),
                child: Row(
                  children: [
                    Expanded(
                      child: TextButton(
                        onPressed: () {
                          setState(() {
                            parentAnchor = 'left';
                            childAnchor = 'right';
                            offset = const Offset(0, 50);
                            controllerX.text = offset.dx.toString();
                            controllerY.text = offset.dy.toString();
                            slideX = true;
                            slideY = true;
                            flipX = false;
                            flipY = false;
                            resizeX = false;
                            resizeY = false;
                          });
                        },
                        child: const Text('Set Defaults'),
                      ),
                    ),
                    Expanded(
                      child: TextButton(
                        onPressed: () {
                          Set<WindowPositionerConstraintAdjustment>
                              constraintAdjustments = {};
                          if (slideX) {
                            constraintAdjustments.add(
                                WindowPositionerConstraintAdjustment.slideX);
                          }
                          if (slideY) {
                            constraintAdjustments.add(
                                WindowPositionerConstraintAdjustment.slideY);
                          }
                          if (flipX) {
                            constraintAdjustments.add(
                                WindowPositionerConstraintAdjustment.flipX);
                          }
                          if (flipY) {
                            constraintAdjustments.add(
                                WindowPositionerConstraintAdjustment.flipY);
                          }
                          if (resizeX) {
                            constraintAdjustments.add(
                                WindowPositionerConstraintAdjustment.resizeX);
                          }
                          if (resizeY) {
                            constraintAdjustments.add(
                                WindowPositionerConstraintAdjustment.resizeY);
                          }
                          Navigator.of(context, rootNavigator: true)
                              .pop(PositionerSetting(
                            name: name,
                            parentAnchor:
                                WindowPositionerAnchor.values.firstWhere(
                              (e) =>
                                  e.toString() ==
                                  'WindowPositionerAnchor.$parentAnchor',
                              orElse: () => WindowPositionerAnchor.left,
                            ),
                            childAnchor:
                                WindowPositionerAnchor.values.firstWhere(
                              (e) =>
                                  e.toString() ==
                                  'WindowPositionerAnchor.$childAnchor',
                              orElse: () => WindowPositionerAnchor.left,
                            ),
                            offset: offset,
                            constraintAdjustments: constraintAdjustments,
                          ));
                        },
                        child: const Text('Apply'),
                      ),
                    ),
                  ],
                ),
              ),
            ],
          );
        });
      });
}
