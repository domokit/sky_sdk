// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../../gallery_localizations.dart';

// Duration of time (e.g. 16h 12m)
String formattedDuration(BuildContext context, Duration duration, {bool? abbreviated}) {
  final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

  final String hoursShortForm = localizations.craneHours(duration.inHours);
  final String minutesShortForm = localizations.craneMinutes(duration.inMinutes % 60);
  return localizations.craneFlightDuration(hoursShortForm, minutesShortForm);
}
