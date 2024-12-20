// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [CarouselView].

void main() => runApp(const CarouselExampleApp());

class CarouselExampleApp extends StatelessWidget {
  const CarouselExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: Scaffold(
        appBar: AppBar(
          leading: const Icon(Icons.cast),
          title: const Text('Flutter TV'),
          actions: const <Widget>[
            Padding(
              padding: EdgeInsetsDirectional.only(end: 16.0),
              child: CircleAvatar(child: Icon(Icons.account_circle)),
            ),
          ],
        ),
        body: const CarouselExample(),
      ),
    );
  }
}

class CarouselExample extends StatefulWidget {
  const CarouselExample({super.key});

  @override
  State<CarouselExample> createState() => _CarouselExampleState();
}

class _CarouselExampleState extends State<CarouselExample> {
  final CarouselController controller = CarouselController(initialItem: 1);

  @override
  void dispose() {
    controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final double height = MediaQuery.sizeOf(context).height;

    return ListView(
      children: <Widget>[
        ConstrainedBox(
          constraints: BoxConstraints(maxHeight: height / 2),
          child: CarouselView.weighted(
            controller: controller,
            itemSnapping: true,
            flexWeights: const <int>[1, 7, 1],
            children:
                ImageInfo.values.map((ImageInfo image) {
                  return HeroLayoutCard(imageInfo: image);
                }).toList(),
          ),
        ),
        const SizedBox(height: 20),
        const Padding(
          padding: EdgeInsetsDirectional.only(top: 8.0, start: 8.0),
          child: Text('Multi-browse layout'),
        ),
        ConstrainedBox(
          constraints: const BoxConstraints(maxHeight: 50),
          child: CarouselView.weighted(
            flexWeights: const <int>[1, 2, 3, 2, 1],
            consumeMaxWeight: false,
            children: List<Widget>.generate(20, (int index) {
              return ColoredBox(
                color: Colors.primaries[index % Colors.primaries.length].withOpacity(0.8),
                child: const SizedBox.expand(),
              );
            }),
          ),
        ),
        const SizedBox(height: 20),
        ConstrainedBox(
          constraints: const BoxConstraints(maxHeight: 200),
          child: CarouselView.weighted(
            flexWeights: const <int>[3, 3, 3, 2, 1],
            consumeMaxWeight: false,
            children:
                CardInfo.values.map((CardInfo info) {
                  return ColoredBox(
                    color: info.backgroundColor,
                    child: Center(
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: <Widget>[
                          Icon(info.icon, color: info.color, size: 32.0),
                          Text(
                            info.label,
                            style: const TextStyle(fontWeight: FontWeight.bold),
                            overflow: TextOverflow.clip,
                            softWrap: false,
                          ),
                        ],
                      ),
                    ),
                  );
                }).toList(),
          ),
        ),
        const SizedBox(height: 20),
        const Padding(
          padding: EdgeInsetsDirectional.only(top: 8.0, start: 8.0),
          child: Text('Uncontained layout'),
        ),
        ConstrainedBox(
          constraints: const BoxConstraints(maxHeight: 200),
          child: CarouselView(
            itemExtent: 330,
            shrinkExtent: 200,
            children: List<Widget>.generate(20, (int index) {
              return UncontainedLayoutCard(index: index, label: 'Show $index');
            }),
          ),
        ),
      ],
    );
  }
}

class HeroLayoutCard extends StatelessWidget {
  const HeroLayoutCard({super.key, required this.imageInfo});

  final ImageInfo imageInfo;

  @override
  Widget build(BuildContext context) {
    final double width = MediaQuery.sizeOf(context).width;
    return Stack(
      alignment: AlignmentDirectional.bottomStart,
      children: <Widget>[
        ClipRect(
          child: OverflowBox(
            maxWidth: width * 7 / 8,
            minWidth: width * 7 / 8,
            child: Image(
              fit: BoxFit.cover,
              image: NetworkImage(
                'https://flutter.github.io/assets-for-api-docs/assets/material/${imageInfo.url}',
              ),
            ),
          ),
        ),
        Padding(
          padding: const EdgeInsets.all(18.0),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              Text(
                imageInfo.title,
                overflow: TextOverflow.clip,
                softWrap: false,
                style: Theme.of(context).textTheme.headlineLarge?.copyWith(color: Colors.white),
              ),
              const SizedBox(height: 10),
              Text(
                imageInfo.subtitle,
                overflow: TextOverflow.clip,
                softWrap: false,
                style: Theme.of(context).textTheme.bodyMedium?.copyWith(color: Colors.white),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

class UncontainedLayoutCard extends StatelessWidget {
  const UncontainedLayoutCard({super.key, required this.index, required this.label});

  final int index;
  final String label;

  @override
  Widget build(BuildContext context) {
    return ColoredBox(
      color: Colors.primaries[index % Colors.primaries.length].withOpacity(0.5),
      child: Center(
        child: Text(
          label,
          style: const TextStyle(color: Colors.white, fontSize: 20),
          overflow: TextOverflow.clip,
          softWrap: false,
        ),
      ),
    );
  }
}

enum CardInfo {
  camera('Cameras', Icons.video_call, Color(0xff2354C7), Color(0xffECEFFD)),
  lighting('Lighting', Icons.lightbulb, Color(0xff806C2A), Color(0xffFAEEDF)),
  climate('Climate', Icons.thermostat, Color(0xffA44D2A), Color(0xffFAEDE7)),
  wifi('Wifi', Icons.wifi, Color(0xff417345), Color(0xffE5F4E0)),
  media('Media', Icons.library_music, Color(0xff2556C8), Color(0xffECEFFD)),
  security('Security', Icons.crisis_alert, Color(0xff794C01), Color(0xffFAEEDF)),
  safety('Safety', Icons.medical_services, Color(0xff2251C5), Color(0xffECEFFD)),
  more('', Icons.add, Color(0xff201D1C), Color(0xffE3DFD8));

  const CardInfo(this.label, this.icon, this.color, this.backgroundColor);
  final String label;
  final IconData icon;
  final Color color;
  final Color backgroundColor;
}

enum ImageInfo {
  image0('The Flow', 'Sponsored | Season 1 Now Streaming', 'content_based_color_scheme_1.png'),
  image1(
    'Through the Pane',
    'Sponsored | Season 1 Now Streaming',
    'content_based_color_scheme_2.png',
  ),
  image2('Iridescence', 'Sponsored | Season 1 Now Streaming', 'content_based_color_scheme_3.png'),
  image3('Sea Change', 'Sponsored | Season 1 Now Streaming', 'content_based_color_scheme_4.png'),
  image4('Blue Symphony', 'Sponsored | Season 1 Now Streaming', 'content_based_color_scheme_5.png'),
  image5('When It Rains', 'Sponsored | Season 1 Now Streaming', 'content_based_color_scheme_6.png');

  const ImageInfo(this.title, this.subtitle, this.url);
  final String title;
  final String subtitle;
  final String url;
}
