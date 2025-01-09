// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:animations/animations.dart';
import 'package:flutter/material.dart';
import '../../gallery_localizations.dart';

// BEGIN sharedXAxisTransitionDemo

class SharedXAxisTransitionDemo extends StatefulWidget {
  const SharedXAxisTransitionDemo({super.key});
  @override
  State<SharedXAxisTransitionDemo> createState() => _SharedXAxisTransitionDemoState();
}

class _SharedXAxisTransitionDemoState extends State<SharedXAxisTransitionDemo> {
  bool _isLoggedIn = false;

  void _toggleLoginStatus() {
    setState(() {
      _isLoggedIn = !_isLoggedIn;
    });
  }

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return Scaffold(
      resizeToAvoidBottomInset: false,
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Column(
          children: <Widget>[
            Text(localizations.demoSharedXAxisTitle),
            Text(
              '(${localizations.demoSharedXAxisDemoInstructions})',
              style: Theme.of(context).textTheme.titleSmall!.copyWith(color: Colors.white),
            ),
          ],
        ),
      ),
      body: SafeArea(
        child: Column(
          children: <Widget>[
            Expanded(
              child: PageTransitionSwitcher(
                reverse: !_isLoggedIn,
                transitionBuilder: (
                  Widget child,
                  Animation<double> animation,
                  Animation<double> secondaryAnimation,
                ) {
                  return SharedAxisTransition(
                    animation: animation,
                    secondaryAnimation: secondaryAnimation,
                    transitionType: SharedAxisTransitionType.horizontal,
                    child: child,
                  );
                },
                child: _isLoggedIn ? const _CoursePage() : const _SignInPage(),
              ),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 15, vertical: 20),
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  TextButton(
                    onPressed: _isLoggedIn ? _toggleLoginStatus : null,
                    child: Text(localizations.demoSharedXAxisBackButtonText),
                  ),
                  ElevatedButton(
                    onPressed: _isLoggedIn ? null : _toggleLoginStatus,
                    child: Text(localizations.demoSharedXAxisNextButtonText),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _CoursePage extends StatelessWidget {
  const _CoursePage();

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return ListView(
      children: <Widget>[
        const SizedBox(height: 16),
        Text(
          localizations.demoSharedXAxisCoursePageTitle,
          style: Theme.of(context).textTheme.headlineSmall,
          textAlign: TextAlign.center,
        ),
        const SizedBox(height: 10),
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 10),
          child: Text(
            localizations.demoSharedXAxisCoursePageSubtitle,
            style: const TextStyle(fontSize: 12, color: Colors.grey),
            textAlign: TextAlign.center,
          ),
        ),
        _CourseSwitch(course: localizations.demoSharedXAxisArtsAndCraftsCourseTitle),
        _CourseSwitch(course: localizations.demoSharedXAxisBusinessCourseTitle),
        _CourseSwitch(course: localizations.demoSharedXAxisIllustrationCourseTitle),
        _CourseSwitch(course: localizations.demoSharedXAxisDesignCourseTitle),
        _CourseSwitch(course: localizations.demoSharedXAxisCulinaryCourseTitle),
      ],
    );
  }
}

class _CourseSwitch extends StatefulWidget {
  const _CourseSwitch({this.course});

  final String? course;

  @override
  _CourseSwitchState createState() => _CourseSwitchState();
}

class _CourseSwitchState extends State<_CourseSwitch> {
  bool _isCourseBundled = true;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations? localizations = GalleryLocalizations.of(context);
    final String subtitle =
        _isCourseBundled
            ? localizations!.demoSharedXAxisBundledCourseSubtitle
            : localizations!.demoSharedXAxisIndividualCourseSubtitle;

    return SwitchListTile(
      title: Text(widget.course!),
      subtitle: Text(subtitle),
      value: _isCourseBundled,
      onChanged: (bool newValue) {
        setState(() {
          _isCourseBundled = newValue;
        });
      },
    );
  }
}

class _SignInPage extends StatelessWidget {
  const _SignInPage();

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations? localizations = GalleryLocalizations.of(context);

    return LayoutBuilder(
      builder: (BuildContext context, BoxConstraints constraints) {
        final double maxHeight = constraints.maxHeight;
        const SizedBox spacing = SizedBox(height: 10);

        return Container(
          constraints: const BoxConstraints(maxWidth: 400),
          child: Column(
            children: <Widget>[
              SizedBox(height: maxHeight / 10),
              Image.asset(
                'placeholders/avatar_logo.png',
                package: 'flutter_gallery_assets',
                width: 80,
                height: 80,
              ),
              spacing,
              Text(
                localizations!.demoSharedXAxisSignInWelcomeText,
                style: Theme.of(context).textTheme.headlineSmall,
              ),
              spacing,
              Text(
                localizations.demoSharedXAxisSignInSubtitleText,
                style: const TextStyle(fontSize: 12, color: Colors.grey),
              ),
              Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: <Widget>[
                  Padding(
                    padding: const EdgeInsetsDirectional.only(
                      top: 40,
                      start: 10,
                      end: 10,
                      bottom: 10,
                    ),
                    child: TextField(
                      decoration: InputDecoration(
                        suffixIcon: const Icon(Icons.visibility, size: 20, color: Colors.black54),
                        labelText: localizations.demoSharedXAxisSignInTextFieldLabel,
                        border: const OutlineInputBorder(),
                      ),
                    ),
                  ),
                  TextButton(
                    onPressed: () {},
                    child: Text(localizations.demoSharedXAxisForgotEmailButtonText),
                  ),
                  spacing,
                  TextButton(
                    onPressed: () {},
                    child: Text(localizations.demoSharedXAxisCreateAccountButtonText),
                  ),
                ],
              ),
            ],
          ),
        );
      },
    );
  }
}

// END sharedXAxisTransitionDemo
