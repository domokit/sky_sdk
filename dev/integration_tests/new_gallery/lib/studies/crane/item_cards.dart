// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../../layout/adaptive.dart';
import '../../layout/highlight_focus.dart';
import '../../layout/image_placeholder.dart';
import 'model/destination.dart';

// Width and height for thumbnail images.
const double mobileThumbnailSize = 60.0;

class DestinationCard extends StatelessWidget {
  const DestinationCard({super.key, required this.destination});

  final Destination destination;

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);
    final TextTheme textTheme = Theme.of(context).textTheme;

    final Widget card =
        isDesktop
            ? Padding(
              padding: const EdgeInsets.only(bottom: 40),
              child: Semantics(
                container: true,
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: <Widget>[
                    ClipRRect(
                      borderRadius: const BorderRadius.all(Radius.circular(4)),
                      child: _DestinationImage(destination: destination),
                    ),
                    Padding(
                      padding: const EdgeInsets.only(top: 20, bottom: 10),
                      child: SelectableText(destination.destination, style: textTheme.titleMedium),
                    ),
                    SelectableText(
                      destination.subtitle(context),
                      semanticsLabel: destination.subtitleSemantics(context),
                      style: textTheme.titleSmall,
                    ),
                  ],
                ),
              ),
            )
            : Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                ListTile(
                  contentPadding: const EdgeInsetsDirectional.only(end: 8),
                  leading: ClipRRect(
                    borderRadius: const BorderRadius.all(Radius.circular(4)),
                    child: SizedBox(
                      width: mobileThumbnailSize,
                      height: mobileThumbnailSize,
                      child: _DestinationImage(destination: destination),
                    ),
                  ),
                  title: SelectableText(destination.destination, style: textTheme.titleMedium),
                  subtitle: SelectableText(
                    destination.subtitle(context),
                    semanticsLabel: destination.subtitleSemantics(context),
                    style: textTheme.titleSmall,
                  ),
                ),
                const Divider(thickness: 1),
              ],
            );

    return HighlightFocus(
      debugLabel: 'DestinationCard: ${destination.destination}',
      highlightColor: Colors.red.withOpacity(0.1),
      onPressed: () {},
      child: card,
    );
  }
}

class _DestinationImage extends StatelessWidget {
  const _DestinationImage({required this.destination});

  final Destination destination;

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    return Semantics(
      label: destination.assetSemanticLabel,
      child: ExcludeSemantics(
        child: FadeInImagePlaceholder(
          image: ResizeImage(
            AssetImage(destination.assetName, package: 'flutter_gallery_assets'),
            width: isDesktop ? null : mobileThumbnailSize.toInt(),
            height: isDesktop ? null : mobileThumbnailSize.toInt(),
          ),
          fit: BoxFit.cover,
          width: isDesktop ? null : mobileThumbnailSize,
          height: isDesktop ? null : mobileThumbnailSize,
          placeholder: LayoutBuilder(
            builder: (BuildContext context, BoxConstraints constraints) {
              return Container(
                color: Colors.black.withOpacity(0.1),
                width: constraints.maxWidth,
                height: constraints.maxWidth / destination.imageAspectRatio,
              );
            },
          ),
        ),
      ),
    );
  }
}
