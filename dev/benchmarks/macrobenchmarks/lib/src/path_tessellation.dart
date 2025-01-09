// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class PathTessellationPage extends StatefulWidget {
  const PathTessellationPage({super.key});

  @override
  State<PathTessellationPage> createState() => _PathTessellationPageState();
}

class _PathTessellationPageState extends State<PathTessellationPage>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(vsync: this, lowerBound: 1.0, upperBound: 1.3);
    _controller.addListener(() {
      setState(() {});
    });
  }

  @override
  Widget build(BuildContext context) {
    final double scale = _controller.value;
    return SafeArea(
      child: ColoredBox(
        color: Colors.black,
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[
            ListView.builder(
              key: const Key('list_view'), // this key is used by the driver test,
              itemBuilder: (BuildContext context, int index) {
                return Container(
                  margin: const EdgeInsets.all(1.0),
                  decoration: BoxDecoration(color: Colors.white.withOpacity(0.2)),
                  child: IconRow(iconSize: (30 + 0.5 * (index % 10)) * scale),
                );
              },
              itemCount: 200,
              itemExtent: 50,
            ),
            Positioned(
              top: 0,
              left: 0,
              right: 0,
              child: Container(
                color: Colors.black.withOpacity(0.7),
                height: 100,
                child: IconRow(iconSize: 50.0 * scale),
              ),
            ),
            Positioned(
              bottom: 0,
              left: 0,
              right: 0,
              child: ColoredBox(
                color: Colors.black.withOpacity(0.7),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: <Widget>[
                    SizedBox(height: 100, child: IconRow(iconSize: 55.0 * scale)),
                    MaterialButton(
                      textColor: Colors.white,
                      key: const Key('animate_button'), // this key is used by the driver test
                      child: const Text('Animate'),
                      onPressed: () {
                        if (_controller.isAnimating) {
                          _controller.stop();
                        } else {
                          _controller.repeat(period: const Duration(seconds: 1), reverse: true);
                        }
                      },
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class IconRow extends StatelessWidget {
  const IconRow({super.key, required this.iconSize});

  final double iconSize;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: <Widget>[
        SizedBox.square(
          dimension: iconSize,
          child: CustomPaint(painter: _SettingsIconPainter(), willChange: true),
        ),
        SizedBox.square(
          dimension: iconSize,
          child: CustomPaint(painter: _CameraIconPainter(), willChange: true),
        ),
        SizedBox.square(
          dimension: iconSize,
          child: CustomPaint(painter: _CalendarIconPainter(), willChange: true),
        ),
        SizedBox.square(
          dimension: iconSize,
          child: CustomPaint(painter: _ConversationIconPainter(), willChange: true),
        ),
        SizedBox.square(
          dimension: iconSize,
          child: CustomPaint(painter: _GeometryIconPainter(), willChange: true),
        ),
      ],
    );
  }
}

/// Parses SVG path data into a [Path] object.
Path _pathFromString(String pathString) {
  int start = 0;
  final RegExp pattern = RegExp('[MLCHVZ]');
  Offset current = Offset.zero;
  final Path path = Path();

  void performCommand(String command) {
    final String type = command[0];
    final List<double> arguments = command
        .substring(1)
        .split(' ')
        .where((String element) => element.isNotEmpty)
        .map((String e) => double.parse(e))
        .toList(growable: false);
    switch (type) {
      case 'M':
        path.moveTo(arguments[0], arguments[1]);
        current = Offset(arguments[0], arguments[1]);
      case 'L':
        path.lineTo(arguments[0], arguments[1]);
        current = Offset(arguments[0], arguments[1]);
      case 'C':
        path.cubicTo(
          arguments[0],
          arguments[1],
          arguments[2],
          arguments[3],
          arguments[4],
          arguments[5],
        );
        current = Offset(arguments[4], arguments[5]);
      case 'H':
        path.lineTo(arguments[0], current.dy);
        current = Offset(arguments[0], current.dy);
      case 'V':
        path.lineTo(current.dx, arguments[0]);
        current = Offset(current.dx, arguments[0]);
    }
  }

  while (true) {
    start = pathString.indexOf(pattern, start);
    if (start == -1) {
      break;
    }
    int end = pathString.indexOf(pattern, start + 1);
    if (end == -1) {
      end = pathString.length;
    }
    final String command = pathString.substring(start, end);
    performCommand(command);
    start = end;
  }
  return path;
}

class _SettingsIconPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final Matrix4 scale = Matrix4.diagonal3Values(size.width / 20, size.height / 20, 1.0);

    Path path;
    path = _path1.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0x60F84F39));

    path = _path2.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0xFFF84F39));

    path = _path3.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0xFFF84F39));
  }

  static final Path _path1 = _pathFromString(
    'M8.3252 2.675L7.7877 4.0625L5.93771 5.1125L4.4627 4.8875C4.2171 4.85416 3.96713 4.89459 3.74456 5.00365C3.52199 5.11271 3.33686 5.28548 3.21271 5.5L2.7127 6.375C2.58458 6.59294 2.52555 6.8446 2.5434 7.09678C2.56126 7.34895 2.65516 7.58979 2.8127 7.7875L3.7502 8.95V11.05L2.8377 12.2125C2.68016 12.4102 2.58626 12.651 2.5684 12.9032C2.55055 13.1554 2.60958 13.4071 2.73771 13.625L3.2377 14.5C3.36186 14.7145 3.54699 14.8873 3.76956 14.9963C3.99213 15.1054 4.2421 15.1458 4.4877 15.1125L5.96271 14.8875L7.7877 15.9375L8.3252 17.325C8.41585 17.5599 8.57534 17.762 8.78277 17.9047C8.9902 18.0475 9.2359 18.1243 9.48771 18.125H10.5377C10.7895 18.1243 11.0352 18.0475 11.2426 17.9047C11.4501 17.762 11.6096 17.5599 11.7002 17.325L12.2377 15.9375L14.0627 14.8875L15.5377 15.1125C15.7833 15.1458 16.0333 15.1054 16.2559 14.9963C16.4784 14.8873 16.6636 14.7145 16.7877 14.5L17.2877 13.625C17.4158 13.4071 17.4749 13.1554 17.457 12.9032C17.4392 12.651 17.3453 12.4102 17.1877 12.2125L16.2502 11.05V8.95L17.1627 7.7875C17.3203 7.58979 17.4142 7.34895 17.432 7.09678C17.4499 6.8446 17.3908 6.59294 17.2627 6.375L16.7627 5.5C16.6386 5.28548 16.4534 5.11271 16.2309 5.00365C16.0083 4.89459 15.7583 4.85416 15.5127 4.8875L14.0377 5.1125L12.2127 4.0625L11.6752 2.675C11.5846 2.44008 11.4251 2.23801 11.2176 2.09527C11.0102 1.95252 10.7645 1.87574 10.5127 1.875H9.48771C9.2359 1.87574 8.9902 1.95252 8.78277 2.09527C8.57534 2.23801 8.41585 2.44008 8.3252 2.675ZM10 12.5C11.3807 12.5 12.5 11.3807 12.5 10C12.5 8.61929 11.3807 7.5 10 7.5C8.61929 7.5 7.5 8.61929 7.5 10C7.5 11.3807 8.61929 12.5 10 12.5Z',
  );
  static final Path _path2 = _pathFromString(
    'M9.48771 1.25L9.48586 1.25001C9.10816 1.25112 8.7396 1.36628 8.42845 1.5804C8.11747 1.79441 7.87833 2.0973 7.74232 2.44945L7.2854 3.62894L5.81769 4.46197L4.55695 4.26965L4.54677 4.26818C4.17836 4.21818 3.80341 4.27882 3.46955 4.44241C3.13569 4.606 2.858 4.86515 2.67177 5.18693L2.17177 6.06191C1.98107 6.38797 1.89329 6.76406 1.91997 7.14092C1.94675 7.51918 2.08759 7.88043 2.32392 8.177L3.12521 9.17062V10.834L2.34736 11.825C2.11197 12.1212 1.97169 12.4817 1.94497 12.8591C1.91828 13.236 2.00608 13.6121 2.1968 13.9381L2.69505 14.8101L2.69677 14.8131C2.883 15.1349 3.16069 15.394 3.49455 15.5576C3.82841 15.7212 4.20336 15.7818 4.57177 15.7318L5.84067 15.5383L7.28466 16.3691L7.28539 16.3711L7.74211 17.55C7.87812 17.9021 8.11745 18.2056 8.42843 18.4196C8.73958 18.6337 9.10814 18.7489 9.48584 18.75L9.48769 18.75L10.5145 18.75C10.8922 18.7489 11.2608 18.6337 11.5719 18.4196C11.8829 18.2056 12.1473 17.9021 12.2833 17.55L12.7408 16.3691L14.1847 15.5383L15.4434 15.7304L15.4536 15.7318C15.822 15.7818 16.197 15.7212 16.5309 15.5576C16.8647 15.394 17.1424 15.1349 17.3286 14.8131L17.3304 14.8101L17.8287 13.9381C18.0193 13.612 18.1071 13.2359 18.0804 12.8591C18.0537 12.4808 17.9128 12.1196 17.6765 11.823L16.8752 10.8294V9.166L17.6515 8.177L17.6531 8.17502C17.8884 7.87883 18.0287 7.51834 18.0554 7.14092C18.0821 6.76407 17.9943 6.38798 17.8036 6.06192L17.3054 5.18991L17.3036 5.18693C17.1174 4.86515 16.8397 4.606 16.5059 4.44241C16.172 4.27882 15.797 4.21809 15.4286 4.2681L14.1597 4.46166L12.7158 3.63087L12.258 2.44923C12.122 2.09718 11.8829 1.79437 11.572 1.5804C11.2608 1.36628 10.8923 1.25112 10.5146 1.25L9.48771 1.25ZM10.5116 2.5H9.48879C9.36315 2.50053 9.24059 2.5389 9.13708 2.61013C9.03337 2.68151 8.95363 2.78254 8.9083 2.9L8.3705 4.28827C8.31845 4.42266 8.22154 4.53492 8.09621 4.60606L6.24621 5.65606C6.12411 5.72535 5.98224 5.75153 5.84346 5.73036L4.37442 5.50627C4.25298 5.49062 4.12958 5.51099 4.01957 5.5649C3.90871 5.61922 3.81643 5.70515 3.75436 5.81183L3.25154 6.69178C3.18747 6.80075 3.15792 6.92655 3.16684 7.05264C3.17574 7.17824 3.22235 7.2982 3.30057 7.39684L4.23671 8.55766C4.32633 8.66878 4.37521 8.80724 4.37521 8.95V11.05C4.37521 11.1899 4.32824 11.3258 4.24184 11.4359L3.32651 12.602C3.24773 12.7009 3.20077 12.8213 3.19184 12.9474C3.18292 13.0735 3.21243 13.1993 3.27649 13.3083L3.2804 13.3149L3.77864 14.1869L3.77933 14.1881C3.8414 14.2948 3.93369 14.3808 4.04457 14.4351C4.15457 14.489 4.27796 14.5094 4.39939 14.4937L5.86846 14.2696C6.00848 14.2483 6.15161 14.2751 6.27439 14.3458L8.09939 15.3958C8.22322 15.467 8.3189 15.5785 8.3705 15.7117L8.908 17.0992C8.95333 17.2167 9.03337 17.3185 9.13708 17.3899C9.24061 17.4611 9.36321 17.4995 9.48887 17.5H10.5365C10.6622 17.4995 10.7848 17.4611 10.8883 17.3899C10.992 17.3185 11.0718 17.2175 11.1171 17.1L11.6549 15.7117C11.7065 15.5785 11.8022 15.467 11.926 15.3958L13.751 14.3458C13.8738 14.2751 14.0169 14.2483 14.157 14.2696L15.626 14.4937C15.7475 14.5094 15.8708 14.489 15.9808 14.4351C16.0917 14.3808 16.184 14.2949 16.2461 14.1882L16.2468 14.1869L16.7489 13.3082C16.8129 13.1993 16.8425 13.0735 16.8336 12.9474C16.8247 12.8218 16.7781 12.7019 16.6999 12.6032L16.6989 12.602L15.7637 11.4423C15.6741 11.3312 15.6252 11.1928 15.6252 11.05V8.95C15.6252 8.81006 15.6722 8.67418 15.7586 8.5641L16.6711 7.4016L16.6739 7.398C16.7527 7.29915 16.7996 7.17873 16.8086 7.05264C16.8175 6.92655 16.788 6.80072 16.7239 6.69175L16.72 6.68511L16.2218 5.81307L16.2211 5.81187C16.159 5.70517 16.0667 5.61923 15.9558 5.5649C15.8458 5.51099 15.7224 5.49062 15.601 5.50627L14.132 5.73036C13.9919 5.75172 13.8488 5.72488 13.726 5.65424L11.901 4.60424C11.7772 4.533 11.6815 4.42148 11.6299 4.28827L11.0924 2.90077C11.0471 2.78331 10.967 2.68151 10.8633 2.61013C10.7598 2.5389 10.6373 2.50053 10.5116 2.5Z',
  );
  static final Path _path3 = _pathFromString(
    'M10.0002 6.875C9.1714 6.875 8.37655 7.20424 7.7905 7.79029C7.20445 8.37635 6.87521 9.1712 6.87521 10C6.87521 10.6181 7.05848 11.2223 7.40186 11.7362C7.74524 12.2501 8.2333 12.6506 8.80432 12.8871C9.37534 13.1237 10.0037 13.1855 10.6099 13.065C11.2161 12.9444 11.7729 12.6467 12.2099 12.2097C12.647 11.7727 12.9446 11.2159 13.0652 10.6097C13.1857 10.0035 13.1239 9.37514 12.8873 8.80412C12.6508 8.2331 12.2503 7.74504 11.7364 7.40166C11.2225 7.05828 10.6183 6.875 10.0002 6.875ZM10.0002 8.125C9.50292 8.125 9.02601 8.32255 8.67438 8.67418C8.32275 9.02581 8.12521 9.50272 8.12521 10C8.12521 10.3708 8.23517 10.7334 8.4412 11.0417C8.64723 11.35 8.94006 11.5904 9.28267 11.7323C9.62529 11.8742 10.0023 11.9113 10.366 11.839C10.7297 11.7666 11.0638 11.5881 11.326 11.3258C11.5883 11.0636 11.7668 10.7295 11.8392 10.3658C11.9115 10.0021 11.8744 9.62508 11.7325 9.28247C11.5906 8.93986 11.3502 8.64703 11.0419 8.441C10.7336 8.23497 10.371 8.125 10.0002 8.125Z',
  );

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class _CameraIconPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final Matrix4 scale = Matrix4.diagonal3Values(size.width / 20, size.height / 20, 1.0);

    Path path;
    path = _path1.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0xFFF84F39));

    path = _path2.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0x60F84F39));
  }

  static final Path _path1 = _pathFromString(
    'M3.26366 17H16.7363C17.4857 17 18.0503 16.8123 18.4302 16.4369C18.8101 16.0615 19 15.5035 19 14.7631V7.01229C19 6.27188 18.8101 5.71397 18.4302 5.33855C18.0503 4.96313 17.4857 4.77542 16.7363 4.77542H14.6132C14.4154 4.77542 14.2567 4.76238 14.137 4.73631C14.0173 4.70503 13.9107 4.65549 13.817 4.58771C13.7233 4.51471 13.6219 4.41825 13.5126 4.29832L12.9115 3.61788C12.7242 3.41453 12.5212 3.26071 12.3027 3.15642C12.0893 3.05214 11.7953 3 11.4206 3H8.50911C8.13443 3 7.83781 3.05214 7.61925 3.15642C7.4059 3.26071 7.20555 3.41453 7.01821 3.61788L6.41717 4.29832C6.24545 4.48082 6.09193 4.60596 5.95663 4.67374C5.82134 4.74153 5.61058 4.77542 5.32437 4.77542H3.26366C2.50911 4.77542 1.94189 4.96313 1.56201 5.33855C1.18734 5.71397 1 6.27188 1 7.01229V14.7631C1 15.5035 1.18734 16.0615 1.56201 16.4369C1.94189 16.8123 2.50911 17 3.26366 17ZM3.27927 15.9207C2.89419 15.9207 2.59757 15.819 2.38942 15.6156C2.18647 15.4123 2.085 15.1099 2.085 14.7084V7.07486C2.085 6.67337 2.18647 6.37095 2.38942 6.1676C2.59757 5.95903 2.89419 5.85475 3.27927 5.85475H5.57415C5.9072 5.85475 6.1752 5.81825 6.37814 5.74525C6.58109 5.67225 6.77624 5.53147 6.96357 5.32291L7.55681 4.6581C7.76496 4.42346 7.9497 4.26965 8.11101 4.19665C8.27233 4.12365 8.50911 4.08715 8.82134 4.08715H11.1084C11.4258 4.08715 11.6626 4.12365 11.8187 4.19665C11.9801 4.26965 12.1648 4.42346 12.3729 4.6581L12.9662 5.32291C13.1535 5.53147 13.3487 5.67225 13.5516 5.74525C13.7546 5.81825 14.0225 5.85475 14.3556 5.85475H16.7207C17.1058 5.85475 17.4024 5.95903 17.6106 6.1676C17.8187 6.37095 17.9228 6.67337 17.9228 7.07486V14.7084C17.9228 15.1099 17.8187 15.4123 17.6106 15.6156C17.4024 15.819 17.1058 15.9207 16.7207 15.9207H3.27927ZM10 14.8101C10.7493 14.8101 11.4284 14.6302 12.0373 14.2704C12.6461 13.9054 13.1301 13.4179 13.4892 12.8078C13.8534 12.1926 14.0356 11.5095 14.0356 10.7587C14.0356 10.0078 13.8534 9.32477 13.4892 8.7095C13.1301 8.09423 12.6461 7.6067 12.0373 7.24693C11.4284 6.88194 10.7493 6.69944 10 6.69944C9.25585 6.69944 8.57676 6.88194 7.96271 7.24693C7.35386 7.6067 6.8673 8.09423 6.50304 8.7095C6.14397 9.32477 5.96444 10.0078 5.96444 10.7587C5.96444 11.5095 6.14397 12.1926 6.50304 12.8078C6.8673 13.4179 7.35386 13.9054 7.96271 14.2704C8.57676 14.6302 9.25585 14.8101 10 14.8101ZM10 13.7855C9.4484 13.7855 8.94363 13.6499 8.48569 13.3788C8.03296 13.1076 7.66869 12.7426 7.39289 12.2838C7.12229 11.825 6.98699 11.3166 6.98699 10.7587C6.98699 10.1955 7.12229 9.68454 7.39289 9.2257C7.66349 8.76685 8.02775 8.40447 8.48569 8.13855C8.94363 7.86741 9.4484 7.73184 10 7.73184C10.5568 7.73184 11.0616 7.86741 11.5143 8.13855C11.9722 8.40447 12.3365 8.76685 12.6071 9.2257C12.8829 9.68454 13.0208 10.1955 13.0208 10.7587C13.0208 11.3166 12.8829 11.825 12.6071 12.2838C12.3365 12.7426 11.9722 13.1076 11.5143 13.3788C11.0616 13.6499 10.5568 13.7855 10 13.7855ZM14.3556 8.04469C14.3556 8.30019 14.4467 8.51657 14.6288 8.69385C14.8109 8.86592 15.0269 8.95196 15.2767 8.95196C15.516 8.94674 15.7242 8.8581 15.9011 8.68603C16.0833 8.50875 16.1743 8.29497 16.1743 8.04469C16.1743 7.79963 16.0833 7.58845 15.9011 7.41117C15.7242 7.22868 15.516 7.13743 15.2767 7.13743C15.0269 7.13743 14.8109 7.22868 14.6288 7.41117C14.4467 7.58845 14.3556 7.79963 14.3556 8.04469Z',
  );
  static final Path _path2 = _pathFromString(
    'M2.30754 15.6907C2.51782 15.8969 2.81748 16 3.20651 16H16.7856C17.1746 16 17.4743 15.8969 17.6846 15.6907C17.8949 15.4845 18 15.1778 18 14.7707V7.02974C18 6.6226 17.8949 6.31593 17.6846 6.10972C17.4743 5.89822 17.1746 5.79247 16.7856 5.79247H14.3963C14.0598 5.79247 13.7891 5.75545 13.584 5.68143C13.379 5.6074 13.1819 5.46464 12.9926 5.25314L12.3933 4.57898C12.183 4.34104 11.9964 4.18506 11.8334 4.11104C11.6757 4.03701 11.4365 4 11.1158 4H8.80532C8.4899 4 8.2507 4.03701 8.08773 4.11104C7.92476 4.18506 7.73813 4.34104 7.52785 4.57898L6.92854 5.25314C6.73928 5.46464 6.54214 5.6074 6.33711 5.68143C6.13208 5.75545 5.86134 5.79247 5.52489 5.79247H3.20651C2.81748 5.79247 2.51782 5.89822 2.30754 6.10972C2.10251 6.31593 2 6.6226 2 7.02974V14.7707C2 15.1778 2.10251 15.4845 2.30754 15.6907ZM9.99547 14C9.99547 14 8.76994 13.8432 8.23868 13.5297C7.71345 13.2162 7.29086 12.7941 6.97089 12.2636C6.65696 11.733 6.5 11.1451 6.5 10.5C6.5 9.84884 6.65696 9.25797 6.97089 8.72739C7.28482 8.19681 7.70742 7.77778 8.23868 7.47028C8.76994 7.15676 9.35554 7 9.99547 7C10.6414 7 11.7523 7.47028 11.7523 7.47028C11.7523 7.47028 12.7061 8.19681 13.0201 8.72739C13.34 9.25797 13.5 9.84884 13.5 10.5C13.5 11.1451 13.34 11.733 13.0201 12.2636C12.7061 12.7941 12.2835 13.2162 11.7523 13.5297C11.227 13.8432 9.99547 14 9.99547 14Z',
  );

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class _CalendarIconPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final Matrix4 scale = Matrix4.diagonal3Values(size.width / 20, size.height / 20, 1.0);

    Path path;
    path = _path1.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0x60F84F39));

    path = _path2.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0xFFF84F39));
  }

  static final Path _path1 = _pathFromString(
    'M16.7812 6.85938H3.28125V4.5L5 3H15L16.7812 4.5V6.85938Z',
  );
  static final Path _path2 = _pathFromString(
    'M6.5606 11.2462C7.07732 11.2462 7.4962 10.8273 7.4962 10.3106C7.4962 9.79388 7.07732 9.375 6.5606 9.375C6.04388 9.375 5.625 9.79388 5.625 10.3106C5.625 10.8273 6.04388 11.2462 6.5606 11.2462ZM7.4962 13.4356C7.4962 13.9523 7.07732 14.3712 6.5606 14.3712C6.04388 14.3712 5.625 13.9523 5.625 13.4356C5.625 12.9189 6.04388 12.5 6.5606 12.5C7.07732 12.5 7.4962 12.9189 7.4962 13.4356ZM10.0005 11.2462C10.5173 11.2462 10.9361 10.8273 10.9361 10.3106C10.9361 9.79388 10.5173 9.375 10.0005 9.375C9.48382 9.375 9.06494 9.79388 9.06494 10.3106C9.06494 10.8273 9.48382 11.2462 10.0005 11.2462ZM10.9361 13.4356C10.9361 13.9523 10.5173 14.3712 10.0005 14.3712C9.48382 14.3712 9.06494 13.9523 9.06494 13.4356C9.06494 12.9189 9.48382 12.5 10.0005 12.5C10.5173 12.5 10.9361 12.9189 10.9361 13.4356ZM13.4356 11.2462C13.9523 11.2462 14.3712 10.8273 14.3712 10.3106C14.3712 9.79388 13.9523 9.375 13.4356 9.375C12.9189 9.375 12.5 9.79388 12.5 10.3106C12.5 10.8273 12.9189 11.2462 13.4356 11.2462ZM17.5 5.625C17.5 3.89911 16.1009 2.5 14.375 2.5H5.625C3.89911 2.5 2.5 3.89911 2.5 5.625V14.375C2.5 16.1009 3.89911 17.5 5.625 17.5H14.375C16.1009 17.5 17.5 16.1009 17.5 14.375V5.625ZM3.75 7.5H16.25V14.375C16.25 15.4105 15.4105 16.25 14.375 16.25H5.625C4.58947 16.25 3.75 15.4105 3.75 14.375V7.5ZM5.625 3.75H14.375C15.4105 3.75 16.25 4.58947 16.25 5.625V6.25H3.75V5.625C3.75 4.58947 4.58947 3.75 5.625 3.75Z',
  );

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class _ConversationIconPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final Matrix4 scale = Matrix4.diagonal3Values(size.width / 20, size.height / 20, 1.0);

    Path path;
    path = _path1.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0x60F84F39));

    path = _path2.transform(scale.storage)..fillType = PathFillType.evenOdd;
    canvas.drawPath(path, Paint()..color = const Color(0xFFF84F39));
  }

  static final Path _path1 = _pathFromString(
    'M14.4141 8.33333C14.4141 11.555 11.8024 14.1667 8.58073 14.1667C7.64487 14.1667 6.76047 13.9463 5.97663 13.5546L2.65625 14.4661L3.70864 11.5424C3.10106 10.6218 2.7474 9.51887 2.7474 8.33333C2.7474 5.11167 5.35907 2.5 8.58073 2.5C11.8024 2.5 14.4141 5.11167 14.4141 8.33333Z',
  );
  static final Path _path2 = _pathFromString(
    'M8.5382 1.81665C4.84709 1.81665 1.85486 4.80888 1.85486 8.49998C1.85486 9.65241 2.1471 10.7384 2.66182 11.686L1.89645 13.6887C1.5494 14.5968 2.38481 15.5128 3.32097 15.2506L5.74289 14.5723C6.59409 14.9647 7.54146 15.1833 8.5382 15.1833C12.2293 15.1833 15.2215 12.1911 15.2215 8.49998C15.2215 4.80888 12.2293 1.81665 8.5382 1.81665ZM3.22153 8.49998C3.22153 5.56367 5.60188 3.18332 8.5382 3.18332C11.4745 3.18332 13.8549 5.56367 13.8549 8.49998C13.8549 11.4363 11.4745 13.8167 8.5382 13.8167C7.66578 13.8167 6.84431 13.607 6.11952 13.2361L5.88143 13.1142L3.30309 13.8364L4.17436 11.5565L3.99903 11.2698C3.5059 10.4636 3.22153 9.51607 3.22153 8.49998ZM16.5636 7.07206L16.1464 6.61586L16.2475 7.22577C16.317 7.64558 16.3533 8.07677 16.3533 8.51656C16.3533 8.69251 16.3475 8.86707 16.3361 9.04007L16.3328 9.08869L16.3542 9.13249C16.6951 9.83163 16.8175 10.5975 16.8175 11.5C16.8175 12.5161 16.5332 13.4636 16.04 14.2698L15.8647 14.5565L16.736 16.8363L14.1576 16.1142L13.9195 16.2361C13.1948 16.607 12.3733 16.8166 11.5009 16.8166C10.5879 16.8166 9.87033 16.6947 9.19041 16.3496L9.14498 16.3266L9.09417 16.3303C8.90419 16.3441 8.71231 16.3511 8.51875 16.3511C8.10958 16.3511 7.70785 16.3197 7.31582 16.2593L6.72389 16.1681L7.16334 16.575C8.31731 17.6436 9.75888 18.1833 11.5009 18.1833C12.4976 18.1833 13.445 17.9647 14.2962 17.5723L16.7181 18.2506C17.6543 18.5128 18.4897 17.5968 18.1426 16.6887L17.3772 14.6859C17.892 13.7384 18.1842 12.6524 18.1842 11.5C18.1842 9.75761 17.6387 8.24759 16.5636 7.07206Z',
  );

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}

class _GeometryIconPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size canvasSize) {
    const Size size = Size(20, 20);
    canvas.scale(canvasSize.width / size.width, canvasSize.height / size.height);

    final Paint paint = Paint()..color = const Color(0xFFF84F39);
    final Rect frame = Offset.zero & size;
    canvas.drawDRRect(
      RRect.fromRectAndRadius(frame, const Radius.elliptical(5, 4)),
      RRect.fromRectAndRadius(frame.deflate(1), const Radius.elliptical(4, 3)),
      paint,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(const Rect.fromLTWH(3, 3, 6, 6), const Radius.elliptical(2, 1)),
      paint,
    );
    canvas.drawRRect(
      RRect.fromRectAndRadius(const Rect.fromLTWH(11, 11, 6, 6), const Radius.elliptical(2, 1)),
      paint,
    );
    canvas.drawCircle(const Offset(14, 6), 3, paint);
    canvas.drawCircle(const Offset(6, 14), 3, paint);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) {
    return false;
  }
}
