// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../layout/adaptive.dart';
import 'colors.dart';

const double textFieldHeight = 60.0;
const double appPaddingLarge = 120.0;
const double appPaddingSmall = 24.0;

class HeaderFormField {
  const HeaderFormField({
    required this.index,
    required this.iconData,
    required this.title,
    required this.textController,
  });
  final int index;
  final IconData iconData;
  final String title;
  final TextEditingController textController;
}

class HeaderForm extends StatelessWidget {
  const HeaderForm({super.key, required this.fields});
  final List<HeaderFormField> fields;

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final bool isSmallDesktop = isDisplaySmallDesktop(context);

    return Padding(
      padding: EdgeInsets.symmetric(
        horizontal: isDesktop && !isSmallDesktop ? appPaddingLarge : appPaddingSmall,
      ),
      child:
          isDesktop
              ? LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  int crossAxisCount = isSmallDesktop ? 2 : 4;
                  if (fields.length < crossAxisCount) {
                    crossAxisCount = fields.length;
                  }
                  final double itemWidth = constraints.maxWidth / crossAxisCount;
                  return GridView.count(
                    crossAxisCount: crossAxisCount,
                    childAspectRatio: itemWidth / textFieldHeight,
                    physics: const NeverScrollableScrollPhysics(),
                    children: <Widget>[
                      for (final HeaderFormField field in fields)
                        if ((field.index + 1) % crossAxisCount == 0)
                          _HeaderTextField(field: field)
                        else
                          Padding(
                            padding: const EdgeInsetsDirectional.only(end: 16),
                            child: _HeaderTextField(field: field),
                          ),
                    ],
                  );
                },
              )
              : Column(
                mainAxisSize: MainAxisSize.min,
                children: <Widget>[
                  for (final HeaderFormField field in fields)
                    Padding(
                      padding: const EdgeInsets.only(bottom: 8),
                      child: _HeaderTextField(field: field),
                    ),
                ],
              ),
    );
  }
}

class _HeaderTextField extends StatelessWidget {
  const _HeaderTextField({required this.field});
  final HeaderFormField field;

  @override
  Widget build(BuildContext context) {
    return TextField(
      controller: field.textController,
      cursorColor: Theme.of(context).colorScheme.secondary,
      style: Theme.of(context).textTheme.bodyLarge!.copyWith(color: Colors.white),
      onTap: () {},
      decoration: InputDecoration(
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(4),
          borderSide: BorderSide.none,
        ),
        contentPadding: const EdgeInsets.all(16),
        fillColor: cranePurple700,
        filled: true,
        hintText: field.title,
        floatingLabelBehavior: FloatingLabelBehavior.never,
        prefixIcon: Icon(field.iconData, size: 24, color: Theme.of(context).iconTheme.color),
      ),
    );
  }
}
