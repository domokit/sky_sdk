// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN openContainerTransformDemo

const String _loremIpsumParagraph =
    'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod '
    'tempor incididunt ut labore et dolore magna aliqua. Vulputate dignissim '
    'suspendisse in est. Ut ornare lectus sit amet. Eget nunc lobortis mattis '
    'aliquam faucibus purus in. Hendrerit gravida rutrum quisque non tellus '
    'orci ac auctor. Mattis aliquam faucibus purus in massa. Tellus rutrum '
    'tellus pellentesque eu tincidunt tortor. Nunc eget lorem dolor sed. Nulla '
    'at volutpat diam ut venenatis tellus in metus. Tellus cras adipiscing enim '
    'eu turpis. Pretium fusce id velit ut tortor. Adipiscing enim eu turpis '
    'egestas pretium. Quis varius quam quisque id. Blandit aliquam etiam erat '
    'velit scelerisque. In nisl nisi scelerisque eu. Semper risus in hendrerit '
    'gravida rutrum quisque. Suspendisse in est ante in nibh mauris cursus '
    'mattis molestie. Adipiscing elit duis tristique sollicitudin nibh sit '
    'amet commodo nulla. Pretium viverra suspendisse potenti nullam ac tortor '
    'vitae.\n'
    '\n'
    'Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod '
    'tempor incididunt ut labore et dolore magna aliqua. Vulputate dignissim '
    'suspendisse in est. Ut ornare lectus sit amet. Eget nunc lobortis mattis '
    'aliquam faucibus purus in. Hendrerit gravida rutrum quisque non tellus '
    'orci ac auctor. Mattis aliquam faucibus purus in massa. Tellus rutrum '
    'tellus pellentesque eu tincidunt tortor. Nunc eget lorem dolor sed. Nulla '
    'at volutpat diam ut venenatis tellus in metus. Tellus cras adipiscing enim '
    'eu turpis. Pretium fusce id velit ut tortor. Adipiscing enim eu turpis '
    'egestas pretium. Quis varius quam quisque id. Blandit aliquam etiam erat '
    'velit scelerisque. In nisl nisi scelerisque eu. Semper risus in hendrerit '
    'gravida rutrum quisque. Suspendisse in est ante in nibh mauris cursus '
    'mattis molestie. Adipiscing elit duis tristique sollicitudin nibh sit '
    'amet commodo nulla. Pretium viverra suspendisse potenti nullam ac tortor '
    'vitae';

const double _fabDimension = 56;

class OpenContainerTransformDemo extends StatefulWidget {
  const OpenContainerTransformDemo({super.key});

  @override
  State<OpenContainerTransformDemo> createState() => _OpenContainerTransformDemoState();
}

class _OpenContainerTransformDemoState extends State<OpenContainerTransformDemo> {
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();
  ContainerTransitionType _transitionType = ContainerTransitionType.fade;

  void _showSettingsBottomModalSheet(BuildContext context) {
    final GalleryLocalizations? localizations = GalleryLocalizations.of(context);

    showModalBottomSheet<void>(
      context: context,
      builder: (BuildContext context) {
        return StatefulBuilder(
          builder: (BuildContext context, void Function(void Function()) setModalState) {
            return Container(
              height: 125,
              padding: const EdgeInsets.all(15),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Text(
                    localizations!.demoContainerTransformModalBottomSheetTitle,
                    style: Theme.of(context).textTheme.bodySmall,
                  ),
                  const SizedBox(height: 12),
                  ToggleButtons(
                    borderRadius: BorderRadius.circular(2),
                    selectedBorderColor: Theme.of(context).colorScheme.primary,
                    onPressed: (int index) {
                      setModalState(() {
                        setState(() {
                          _transitionType =
                              index == 0
                                  ? ContainerTransitionType.fade
                                  : ContainerTransitionType.fadeThrough;
                        });
                      });
                    },
                    isSelected: <bool>[
                      _transitionType == ContainerTransitionType.fade,
                      _transitionType == ContainerTransitionType.fadeThrough,
                    ],
                    children: <Widget>[
                      Text(localizations.demoContainerTransformTypeFade),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 10),
                        child: Text(localizations.demoContainerTransformTypeFadeThrough),
                      ),
                    ],
                  ),
                ],
              ),
            );
          },
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations? localizations = GalleryLocalizations.of(context);
    final ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Navigator(
      // Adding [ValueKey] to make sure that the widget gets rebuilt when
      // changing type.
      key: ValueKey<ContainerTransitionType>(_transitionType),
      onGenerateRoute: (RouteSettings settings) {
        return MaterialPageRoute<void>(
          builder:
              (BuildContext context) => Scaffold(
                key: _scaffoldKey,
                appBar: AppBar(
                  automaticallyImplyLeading: false,
                  title: Column(
                    children: <Widget>[
                      Text(localizations!.demoContainerTransformTitle),
                      Text(
                        '(${localizations.demoContainerTransformDemoInstructions})',
                        style: Theme.of(
                          context,
                        ).textTheme.titleSmall!.copyWith(color: Colors.white),
                      ),
                    ],
                  ),
                  actions: <Widget>[
                    IconButton(
                      icon: const Icon(Icons.settings),
                      onPressed: () {
                        _showSettingsBottomModalSheet(context);
                      },
                    ),
                  ],
                ),
                body: ListView(
                  padding: const EdgeInsets.all(8),
                  children: <Widget>[
                    _OpenContainerWrapper(
                      transitionType: _transitionType,
                      closedBuilder: (BuildContext context, void Function() openContainer) {
                        return _DetailsCard(openContainer: openContainer);
                      },
                    ),
                    const SizedBox(height: 16),
                    _OpenContainerWrapper(
                      transitionType: _transitionType,
                      closedBuilder: (BuildContext context, void Function() openContainer) {
                        return _DetailsListTile(openContainer: openContainer);
                      },
                    ),
                    const SizedBox(height: 16),
                    Row(
                      children: <Widget>[
                        Expanded(
                          child: _OpenContainerWrapper(
                            transitionType: _transitionType,
                            closedBuilder: (BuildContext context, void Function() openContainer) {
                              return _SmallDetailsCard(
                                openContainer: openContainer,
                                subtitle: localizations.demoMotionPlaceholderSubtitle,
                              );
                            },
                          ),
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: _OpenContainerWrapper(
                            transitionType: _transitionType,
                            closedBuilder: (BuildContext context, void Function() openContainer) {
                              return _SmallDetailsCard(
                                openContainer: openContainer,
                                subtitle: localizations.demoMotionPlaceholderSubtitle,
                              );
                            },
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 16),
                    Row(
                      children: <Widget>[
                        Expanded(
                          child: _OpenContainerWrapper(
                            transitionType: _transitionType,
                            closedBuilder: (BuildContext context, void Function() openContainer) {
                              return _SmallDetailsCard(
                                openContainer: openContainer,
                                subtitle: localizations.demoMotionSmallPlaceholderSubtitle,
                              );
                            },
                          ),
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: _OpenContainerWrapper(
                            transitionType: _transitionType,
                            closedBuilder: (BuildContext context, void Function() openContainer) {
                              return _SmallDetailsCard(
                                openContainer: openContainer,
                                subtitle: localizations.demoMotionSmallPlaceholderSubtitle,
                              );
                            },
                          ),
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: _OpenContainerWrapper(
                            transitionType: _transitionType,
                            closedBuilder: (BuildContext context, void Function() openContainer) {
                              return _SmallDetailsCard(
                                openContainer: openContainer,
                                subtitle: localizations.demoMotionSmallPlaceholderSubtitle,
                              );
                            },
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 16),
                    ...List<OpenContainer<bool>>.generate(10, (int index) {
                      return OpenContainer<bool>(
                        transitionType: _transitionType,
                        openBuilder:
                            (BuildContext context, void Function() openContainer) =>
                                const _DetailsPage(),
                        tappable: false,
                        closedShape: const RoundedRectangleBorder(),
                        closedElevation: 0,
                        closedBuilder: (BuildContext context, void Function() openContainer) {
                          return ListTile(
                            leading: Image.asset(
                              'placeholders/avatar_logo.png',
                              package: 'flutter_gallery_assets',
                              width: 40,
                            ),
                            onTap: openContainer,
                            title: Text('${localizations.demoMotionListTileTitle} ${index + 1}'),
                            subtitle: Text(localizations.demoMotionPlaceholderSubtitle),
                          );
                        },
                      );
                    }),
                  ],
                ),
                floatingActionButton: OpenContainer(
                  transitionType: _transitionType,
                  openBuilder:
                      (BuildContext context, void Function() openContainer) => const _DetailsPage(),
                  closedElevation: 6,
                  closedShape: const RoundedRectangleBorder(
                    borderRadius: BorderRadius.all(Radius.circular(_fabDimension / 2)),
                  ),
                  closedColor: colorScheme.secondary,
                  closedBuilder: (BuildContext context, void Function() openContainer) {
                    return SizedBox(
                      height: _fabDimension,
                      width: _fabDimension,
                      child: Center(child: Icon(Icons.add, color: colorScheme.onSecondary)),
                    );
                  },
                ),
              ),
        );
      },
    );
  }
}

class _OpenContainerWrapper extends StatelessWidget {
  const _OpenContainerWrapper({required this.closedBuilder, required this.transitionType});

  final CloseContainerBuilder closedBuilder;
  final ContainerTransitionType transitionType;

  @override
  Widget build(BuildContext context) {
    return OpenContainer<bool>(
      transitionType: transitionType,
      openBuilder: (BuildContext context, void Function() openContainer) => const _DetailsPage(),
      tappable: false,
      closedBuilder: closedBuilder,
    );
  }
}

class _DetailsCard extends StatelessWidget {
  const _DetailsCard({required this.openContainer});

  final VoidCallback openContainer;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return _InkWellOverlay(
      openContainer: openContainer,
      height: 300,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: <Widget>[
          Expanded(
            child: ColoredBox(
              color: Colors.black38,
              child: Center(
                child: Image.asset(
                  'placeholders/placeholder_image.png',
                  package: 'flutter_gallery_assets',
                  width: 100,
                ),
              ),
            ),
          ),
          ListTile(
            title: Text(localizations.demoMotionPlaceholderTitle),
            subtitle: Text(localizations.demoMotionPlaceholderSubtitle),
          ),
          Padding(
            padding: const EdgeInsets.only(left: 16, right: 16, bottom: 16),
            child: Text(
              'Lorem ipsum dolor sit amet, consectetur '
              'adipiscing elit, sed do eiusmod tempor.',
              style: Theme.of(
                context,
              ).textTheme.bodyMedium!.copyWith(color: Colors.black54, inherit: false),
            ),
          ),
        ],
      ),
    );
  }
}

class _SmallDetailsCard extends StatelessWidget {
  const _SmallDetailsCard({required this.openContainer, required this.subtitle});

  final VoidCallback openContainer;
  final String subtitle;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;

    return _InkWellOverlay(
      openContainer: openContainer,
      height: 225,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Container(
            color: Colors.black38,
            height: 150,
            child: Center(
              child: Image.asset(
                'placeholders/placeholder_image.png',
                package: 'flutter_gallery_assets',
                width: 80,
              ),
            ),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.all(10),
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Text(
                    GalleryLocalizations.of(context)!.demoMotionPlaceholderTitle,
                    style: textTheme.titleLarge,
                  ),
                  const SizedBox(height: 4),
                  Text(subtitle, style: textTheme.bodySmall),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _DetailsListTile extends StatelessWidget {
  const _DetailsListTile({required this.openContainer});

  final VoidCallback openContainer;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    const double height = 120.0;

    return _InkWellOverlay(
      openContainer: openContainer,
      height: height,
      child: Row(
        children: <Widget>[
          Container(
            color: Colors.black38,
            height: height,
            width: height,
            child: Center(
              child: Image.asset(
                'placeholders/placeholder_image.png',
                package: 'flutter_gallery_assets',
                width: 60,
              ),
            ),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.all(20),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Text(
                    GalleryLocalizations.of(context)!.demoMotionPlaceholderTitle,
                    style: textTheme.titleMedium,
                  ),
                  const SizedBox(height: 8),
                  Text(
                    'Lorem ipsum dolor sit amet, consectetur '
                    'adipiscing elit,',
                    style: textTheme.bodySmall,
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _InkWellOverlay extends StatelessWidget {
  const _InkWellOverlay({required this.openContainer, required this.height, required this.child});

  final VoidCallback openContainer;
  final double height;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    return SizedBox(height: height, child: InkWell(onTap: openContainer, child: child));
  }
}

class _DetailsPage extends StatelessWidget {
  const _DetailsPage();

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    final TextTheme textTheme = Theme.of(context).textTheme;

    return Scaffold(
      appBar: AppBar(title: Text(localizations.demoMotionDetailsPageTitle)),
      body: ListView(
        children: <Widget>[
          Container(
            color: Colors.black38,
            height: 250,
            child: Padding(
              padding: const EdgeInsets.all(70),
              child: Image.asset(
                'placeholders/placeholder_image.png',
                package: 'flutter_gallery_assets',
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(20),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Text(
                  localizations.demoMotionPlaceholderTitle,
                  style: textTheme.headlineSmall!.copyWith(color: Colors.black54, fontSize: 30),
                ),
                const SizedBox(height: 10),
                Text(
                  _loremIpsumParagraph,
                  style: textTheme.bodyMedium!.copyWith(
                    color: Colors.black54,
                    height: 1.5,
                    fontSize: 16,
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

// END openContainerTransformDemo
