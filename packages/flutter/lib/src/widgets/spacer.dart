// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'basic.dart';
import 'framework.dart';

/// Spacer creates an adjustable, empty spacer that can be used to tune the
/// spacing between widgets in a [Flex] container, like [Row] or [Column].
///
/// The [Spacer] widget will take up any available space, so setting the
/// [Flex.mainAxisAlignment] on a flex container that contains a [Spacer] to
/// [MainAxisAlignment.spaceAround], [MainAxisAlignment.spaceBetween], or
/// [MainAxisAlignment.spaceEvenly] will not have any visible effect: the
/// [Spacer] has taken up all of the additional space, therefore there is none
/// left to redistribute.
///
/// {@tool snippet}
///
/// ```dart
/// const Row(
///   children: <Widget>[
///     Text('Begin'),
///     Spacer(), // Defaults to a flex of one.
///     Text('Middle'),
///     // Gives twice the space between Middle and End than Begin and Middle.
///     Spacer(flex: 2),
///     Text('End'),
///   ],
/// )
/// ```
/// {@end-tool}
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=7FJgd7QN1zI}
///
/// See also:
///
///  * [Row] and [Column], which are the most common containers to use a Spacer
///    in.
///  * [SizedBox], to create a box with a specific size and an optional child.
class Spacer extends StatelessWidget {
  /// Creates a flexible space to insert into a [Flexible] widget.
  ///
  /// The [flex] parameter may not be null or less than one.
  const Spacer({super.key, this.flex = 1}) : assert(flex > 0);

  /// The flex factor to use in determining how much space to take up.
  ///
  /// The amount of space the [Spacer] can occupy in the main axis is determined
  /// by dividing the free space proportionately, after placing the inflexible
  /// children, according to the flex factors of the flexible children.
  ///
  /// Defaults to one.
  final int flex;

  @override
  Widget build(BuildContext context) {
    return Expanded(flex: flex, child: const SizedBox.shrink());
  }
}
