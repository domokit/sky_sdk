// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:url_launcher/url_launcher.dart';

import 'demos.dart';
import 'example_code_parser.dart';
import 'syntax_highlighter.dart';

@immutable
class ComponentDemoTabData {
  const ComponentDemoTabData({
    this.demoWidget,
    this.exampleCodeTag,
    this.description,
    this.tabName,
    this.documentationUrl,
  });

  final Widget? demoWidget;
  final String? exampleCodeTag;
  final String? description;
  final String? tabName;
  final String? documentationUrl;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ComponentDemoTabData &&
        other.tabName == tabName &&
        other.description == description &&
        other.documentationUrl == documentationUrl;
  }

  @override
  int get hashCode => Object.hash(tabName, description, documentationUrl);
}

class TabbedComponentDemoScaffold extends StatefulWidget {
  const TabbedComponentDemoScaffold({
    super.key,
    this.title,
    this.demos,
    this.actions,
    this.isScrollable = true,
    this.showExampleCodeAction = true,
  });

  final List<ComponentDemoTabData>? demos;
  final String? title;
  final List<Widget>? actions;
  final bool isScrollable;
  final bool showExampleCodeAction;

  @override
  State<TabbedComponentDemoScaffold> createState() => _TabbedComponentDemoScaffoldState();
}

class _TabbedComponentDemoScaffoldState extends State<TabbedComponentDemoScaffold> {
  void _showExampleCode(BuildContext context) {
    final String? tag = widget.demos![DefaultTabController.of(context).index].exampleCodeTag;
    if (tag != null) {
      Navigator.push(
        context,
        MaterialPageRoute<FullScreenCodeDialog>(
          builder: (BuildContext context) => FullScreenCodeDialog(exampleCodeTag: tag),
        ),
      );
    }
  }

  Future<void> _showApiDocumentation(BuildContext context) async {
    final String? url = widget.demos![DefaultTabController.of(context).index].documentationUrl;
    if (url == null) {
      return;
    }

    final Uri uri = Uri.parse(url);
    if (await canLaunchUrl(uri)) {
      await launchUrl(uri);
    } else if (context.mounted) {
      showDialog<void>(
        context: context,
        builder: (BuildContext context) {
          return SimpleDialog(
            title: const Text("Couldn't display URL:"),
            children: <Widget>[
              Padding(padding: const EdgeInsets.symmetric(horizontal: 16.0), child: Text(url)),
            ],
          );
        },
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: widget.demos!.length,
      child: Scaffold(
        appBar: AppBar(
          title: Text(widget.title!),
          actions: <Widget>[
            ...?widget.actions,
            Builder(
              builder: (BuildContext context) {
                return IconButton(
                  icon: const Icon(Icons.library_books, semanticLabel: 'Show documentation'),
                  onPressed: () => _showApiDocumentation(context),
                );
              },
            ),
            if (widget.showExampleCodeAction)
              Builder(
                builder: (BuildContext context) {
                  return IconButton(
                    icon: const Icon(Icons.code),
                    tooltip: 'Show example code',
                    onPressed: () => _showExampleCode(context),
                  );
                },
              ),
          ],
          bottom: TabBar(
            isScrollable: widget.isScrollable,
            tabs:
                widget.demos!
                    .map<Widget>((ComponentDemoTabData data) => Tab(text: data.tabName))
                    .toList(),
          ),
        ),
        body: TabBarView(
          children:
              widget.demos!.map<Widget>((ComponentDemoTabData demo) {
                return SafeArea(
                  top: false,
                  bottom: false,
                  child: Column(
                    children: <Widget>[
                      Padding(
                        padding: const EdgeInsets.all(16.0),
                        child: Text(
                          demo.description!,
                          style: Theme.of(context).textTheme.titleMedium,
                        ),
                      ),
                      Expanded(child: demo.demoWidget!),
                    ],
                  ),
                );
              }).toList(),
        ),
      ),
    );
  }
}

class FullScreenCodeDialog extends StatefulWidget {
  const FullScreenCodeDialog({super.key, this.exampleCodeTag});

  final String? exampleCodeTag;

  @override
  FullScreenCodeDialogState createState() => FullScreenCodeDialogState();
}

class FullScreenCodeDialogState extends State<FullScreenCodeDialog> {
  String? _exampleCode;

  @override
  void didChangeDependencies() {
    getExampleCode(widget.exampleCodeTag, DefaultAssetBundle.of(context)).then((String? code) {
      if (mounted) {
        setState(() {
          _exampleCode = code ?? 'Example code not found';
        });
      }
    });
    super.didChangeDependencies();
  }

  @override
  Widget build(BuildContext context) {
    final SyntaxHighlighterStyle style =
        Theme.of(context).brightness == Brightness.dark
            ? SyntaxHighlighterStyle.darkThemeStyle
            : SyntaxHighlighterStyle.lightThemeStyle;

    Widget body;
    if (_exampleCode == null) {
      body = const Center(child: CircularProgressIndicator());
    } else {
      body = SingleChildScrollView(
        child: Padding(
          padding: const EdgeInsets.all(16.0),
          child: RichText(
            text: TextSpan(
              style: const TextStyle(fontFamily: 'monospace', fontSize: 10.0),
              children: <TextSpan>[DartSyntaxHighlighter(style).format(_exampleCode)],
            ),
          ),
        ),
      );
    }

    return Scaffold(
      appBar: AppBar(
        leading: IconButton(
          icon: const Icon(Icons.clear, semanticLabel: 'Close'),
          onPressed: () {
            Navigator.pop(context);
          },
        ),
        title: const Text('Example code'),
      ),
      body: body,
    );
  }
}

class MaterialDemoDocumentationButton extends StatelessWidget {
  MaterialDemoDocumentationButton(String routeName, {super.key})
    : documentationUrl = kDemoDocumentationUrl[routeName],
      assert(
        kDemoDocumentationUrl[routeName] != null,
        'A documentation URL was not specified for demo route $routeName in kAllGalleryDemos',
      );

  final String? documentationUrl;

  @override
  Widget build(BuildContext context) {
    return IconButton(
      icon: const Icon(Icons.library_books),
      tooltip: 'API documentation',
      onPressed: () => launchUrl(Uri.parse(documentationUrl!), mode: LaunchMode.inAppWebView),
    );
  }
}

class CupertinoDemoDocumentationButton extends StatelessWidget {
  CupertinoDemoDocumentationButton(String routeName, {super.key})
    : documentationUrl = kDemoDocumentationUrl[routeName],
      assert(
        kDemoDocumentationUrl[routeName] != null,
        'A documentation URL was not specified for demo route $routeName in kAllGalleryDemos',
      );

  final String? documentationUrl;

  @override
  Widget build(BuildContext context) {
    return CupertinoButton(
      padding: EdgeInsets.zero,
      child: Semantics(label: 'API documentation', child: const Icon(CupertinoIcons.book)),
      onPressed: () => launchUrl(Uri.parse(documentationUrl!), mode: LaunchMode.inAppWebView),
    );
  }
}
