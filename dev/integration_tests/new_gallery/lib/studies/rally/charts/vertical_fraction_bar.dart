// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class VerticalFractionBar extends StatelessWidget {
  const VerticalFractionBar({super.key, this.color, required this.fraction});

  final Color? color;
  final double fraction;

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        return SizedBox(
          height: constraints.maxHeight,
          width: 4,
          child: Column(
            children: <Widget>[
              SizedBox(
                height: (1 - fraction) * constraints.maxHeight,
                child: Container(color: Colors.black),
              ),
              SizedBox(height: fraction * constraints.maxHeight, child: Container(color: color)),
            ],
          ),
        );
      },
    );
  }
}
