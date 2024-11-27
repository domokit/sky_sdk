// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../widgets.dart';

/// Returns [true] if the bytes at [url] can be fetched.
///
/// The bytes may be unable to be fetched if they aren't from the same origin
/// and the sever hosting them does not allow cross-origin requests.
Future<bool> checkIfImageBytesCanBeFetched(String url) {
  // On mobile and desktop there are no cross-origin restrictions.
  return Future<bool>.value(true);
}

/// Returns a widget which displays the [src] in an <img> tag.
Widget createImgElementWidget(String src,
        {Key? key,
        ImageLoadingBuilder? loadingBuilder,
        ImageFrameBuilder? frameBuilder,
        ImageErrorWidgetBuilder? errorBuilder}) =>
    throw UnsupportedError('Creating an Image widget using an <img> tag is '
        'only supported on Web.');
