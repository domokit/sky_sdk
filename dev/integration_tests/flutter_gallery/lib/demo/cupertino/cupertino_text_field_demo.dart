// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

class CupertinoTextFieldDemo extends StatefulWidget {
  const CupertinoTextFieldDemo({super.key});

  static const String routeName = '/cupertino/text_fields';

  @override
  State<CupertinoTextFieldDemo> createState() {
    return _CupertinoTextFieldDemoState();
  }
}

class _CupertinoTextFieldDemoState extends State<CupertinoTextFieldDemo> {
  TextEditingController? _chatTextController;
  TextEditingController? _locationTextController;

  @override
  void initState() {
    super.initState();
    _chatTextController = TextEditingController();
    _locationTextController = TextEditingController(text: 'Montreal, Canada');
  }

  Widget _buildChatTextField() {
    return CupertinoTextField(
      controller: _chatTextController,
      textCapitalization: TextCapitalization.sentences,
      placeholder: 'Text Message',
      decoration: BoxDecoration(
        border: Border.all(width: 0.0, color: CupertinoColors.inactiveGray),
        borderRadius: BorderRadius.circular(15.0),
      ),
      maxLines: null,
      keyboardType: TextInputType.multiline,
      prefix: const Padding(padding: EdgeInsets.symmetric(horizontal: 4.0)),
      suffix: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 2.0),
        child: CupertinoButton(
          minSize: 0.0,
          padding: const EdgeInsets.only(bottom: 4),
          onPressed: () => setState(() => _chatTextController!.clear()),
          child: const Icon(
            CupertinoIcons.arrow_up_circle_fill,
            size: 28.0,
            color: CupertinoColors.activeGreen,
          ),
        ),
      ),
      autofocus: true,
      suffixMode: OverlayVisibilityMode.editing,
      onSubmitted: (String text) => setState(() => _chatTextController!.clear()),
    );
  }

  Widget _buildNameField() {
    return const CupertinoTextField(
      prefix: Icon(
        CupertinoIcons.person_fill,
        color: CupertinoColors.lightBackgroundGray,
        size: 28.0,
      ),
      padding: EdgeInsets.symmetric(horizontal: 6.0, vertical: 12.0),
      clearButtonMode: OverlayVisibilityMode.editing,
      textCapitalization: TextCapitalization.words,
      autocorrect: false,
      decoration: BoxDecoration(
        border: Border(bottom: BorderSide(width: 0.0, color: CupertinoColors.inactiveGray)),
      ),
      placeholder: 'Name',
    );
  }

  Widget _buildEmailField() {
    return const CupertinoTextField(
      prefix: Icon(
        CupertinoIcons.envelope_fill,
        color: CupertinoColors.lightBackgroundGray,
        size: 26,
      ),
      padding: EdgeInsets.symmetric(horizontal: 6.0, vertical: 12.0),
      clearButtonMode: OverlayVisibilityMode.editing,
      keyboardType: TextInputType.emailAddress,
      autocorrect: false,
      decoration: BoxDecoration(
        border: Border(bottom: BorderSide(width: 0.0, color: CupertinoColors.inactiveGray)),
      ),
      placeholder: 'Email',
    );
  }

  Widget _buildLocationField() {
    return CupertinoTextField(
      controller: _locationTextController,
      prefix: const Icon(
        CupertinoIcons.location_fill,
        color: CupertinoColors.lightBackgroundGray,
        size: 26,
      ),
      padding: const EdgeInsets.symmetric(horizontal: 6.0, vertical: 12.0),
      clearButtonMode: OverlayVisibilityMode.editing,
      textCapitalization: TextCapitalization.words,
      decoration: const BoxDecoration(
        border: Border(bottom: BorderSide(width: 0.0, color: CupertinoColors.inactiveGray)),
      ),
      placeholder: 'Location',
    );
  }

  Widget _buildPinField() {
    return const CupertinoTextField(
      prefix: Icon(
        CupertinoIcons.lock_open_fill,
        color: CupertinoColors.lightBackgroundGray,
        size: 26,
      ),
      padding: EdgeInsets.symmetric(horizontal: 6.0, vertical: 12.0),
      clearButtonMode: OverlayVisibilityMode.editing,
      keyboardType: TextInputType.number,
      autocorrect: false,
      obscureText: true,
      decoration: BoxDecoration(
        border: Border(bottom: BorderSide(width: 0.0, color: CupertinoColors.inactiveGray)),
      ),
      placeholder: 'Create a PIN',
    );
  }

  Widget _buildTagsField() {
    return CupertinoTextField(
      controller: TextEditingController(text: 'colleague, reading club'),
      prefix: const Icon(
        CupertinoIcons.tag_fill,
        color: CupertinoColors.lightBackgroundGray,
        size: 26,
      ),
      enabled: false,
      padding: const EdgeInsets.symmetric(horizontal: 6.0, vertical: 12.0),
      decoration: const BoxDecoration(
        border: Border(bottom: BorderSide(width: 0.0, color: CupertinoColors.inactiveGray)),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTextStyle(
      style: const TextStyle(
        fontFamily: 'CupertinoSystemText',
        inherit: false,
        fontSize: 17.0,
        color: CupertinoColors.black,
      ),
      child: CupertinoPageScaffold(
        navigationBar: const CupertinoNavigationBar(
          // We're specifying a back label here because the previous page is a
          // Material page. CupertinoPageRoutes could auto-populate these back
          // labels.
          previousPageTitle: 'Cupertino',
          middle: Text('Text Fields'),
        ),
        child: CupertinoScrollbar(
          child: ListView(
            primary: true,
            children: <Widget>[
              Padding(
                padding: const EdgeInsets.symmetric(vertical: 32.0, horizontal: 16.0),
                child: Column(
                  children: <Widget>[
                    _buildNameField(),
                    _buildEmailField(),
                    _buildLocationField(),
                    _buildPinField(),
                    _buildTagsField(),
                  ],
                ),
              ),
              Padding(
                padding: const EdgeInsets.symmetric(vertical: 32.0, horizontal: 16.0),
                child: _buildChatTextField(),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
