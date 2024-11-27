// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../foundation.dart';
import '../../widgets.dart';
import '_web_image_io.dart' if (dart.library.js_interop) '_web_image_web.dart'
    as impl;

/// A substitute for [Image.network] that follows the web's [same origin policy][].
///
/// On mobile this widget delegates to [Image.network]. On the web this widget
/// delegates to [Image.network] if the server allows fetching the image using
/// a XHR. Otherwise, it uses an `<img>` element to display the image.
///
/// If the [src] of the image is same-origin, or if a HEAD request for [src]
/// indicates that a GET request would succeed, then this widget works exactly
/// the same as [Image.network].
///
/// # Multi-frame Images
///
/// In the case that the [src] can't be fetched directly and a platform view is
/// used to display the image, then if the [src] points to a multi-frame image
/// (e.g. an animated GIF), then there is no way to control the display of the
/// frames of the image. For example, if [src] points to an animated GIF, this
/// widget will just play the GIF animation.
///
/// # Performance Implications
///
/// Rendering many platform views can be a performance burden. Flutter attempts
/// to optimize platform view rendering, but cannot do so in all cases. It is
/// difficult for Flutter to optimize platform views in cases where other
/// content is rendered on top of multiple platform views, so consider laying
/// out [WebImage] widgets so there are no other widgets drawn on top of them.
///
/// # Usage Guidelines
///
/// When possible, consider using an [Image.network] widget instead of
/// [WebImage]. If you control the server hosting the images you want to
/// display, consider configuring the server to support cross-origin requests
/// to enable [Image.network].
///
/// [same origin policy]: https://developer.mozilla.org/en-US/docs/Web/Security/Same-origin_policy
class WebImage extends StatefulWidget {
  /// Creates a web image.
  const WebImage(
    this.provider, {
    super.key,
    this.loadingBuilder,
    this.frameBuilder,
    this.errorBuilder,
  });

  WebImage.network(
    String src, {
    Key? key,
    ImageLoadingBuilder? loadingBuilder,
    ImageFrameBuilder? frameBuilder,
    ImageErrorWidgetBuilder? errorBuilder,
  }) : this(
          WebImageProvider(src),
          key: key,
          loadingBuilder: loadingBuilder,
          frameBuilder: frameBuilder,
          errorBuilder: errorBuilder,
        );

  /// The resource which provides the image.
  final WebImageProvider provider;

  /// {@macro flutter.widgets.Image.loadingBuilder}
  final ImageLoadingBuilder? loadingBuilder;

  /// {@macro flutter.widgets.Image.frameBuilder}
  final ImageFrameBuilder? frameBuilder;

  /// {@macro flutter.widgets.Image.errorBuilder}
  final ImageErrorWidgetBuilder? errorBuilder;

  @override
  State<WebImage> createState() => _WebImageState();
}

class _WebImageState extends State<WebImage> {
  bool checkedIfBytesCanBeFetched = false;
  bool imageBytesCanBeFetched = true;
  @override
  void initState() {
    super.initState();
    widget.provider.checkIfImageBytesCanBeFetched().then((bool canBeFetched) {
      // We may have become unmounted in the time it took to check if the bytes
      // could be fetched.
      if (mounted) {
        setState(() {
          checkedIfBytesCanBeFetched = true;
          imageBytesCanBeFetched = canBeFetched;
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    if (!checkedIfBytesCanBeFetched) {
      return widget.loadingBuilder
              ?.call(context, const SizedBox.shrink(), null) ??
          const SizedBox.shrink();
    }

    if (imageBytesCanBeFetched) {
      return Image(
        image: widget.provider._networkImage!,
        key: widget.key,
        loadingBuilder: widget.loadingBuilder,
        frameBuilder: widget.frameBuilder,
        errorBuilder: widget.errorBuilder,
      );
    } else {
      return impl.createImgElementWidget(
        widget.provider._imgElementProvider!,
        key: widget.key,
        loadingBuilder: widget.loadingBuilder,
        frameBuilder: widget.frameBuilder,
        errorBuilder: widget.errorBuilder,
      );
    }
  }
}

Future<void> precacheWebImage(
  WebImageProvider provider,
  BuildContext context, {
  Size? size,
  ImageErrorListener? onError,
}) {
  return provider.checkIfImageBytesCanBeFetched().then((bool canBeFetched) {
    if (canBeFetched) {
      if (context.mounted) {
        return precacheImage(
          provider._networkImage!,
          context,
          size: size,
          onError: onError,
        );
      }
    }

    if (context.mounted) {
      return impl.precacheImgElement(
        provider._imgElementProvider!,
        onError: onError,
      );
    }
  });
}

class WebImageProvider {
  WebImageProvider(this.src);

  final String src;

  bool? _imageBytesCanBeFetched;

  NetworkImage? _networkImage;

  ImgElementProvider? _imgElementProvider;

  Future<bool> checkIfImageBytesCanBeFetched() {
    if (_imageBytesCanBeFetched != null) {
      return SynchronousFuture<bool>(_imageBytesCanBeFetched!);
    }
    return impl.checkIfImageBytesCanBeFetched(src).then((bool result) {
      _imageBytesCanBeFetched = result;
      if (_imageBytesCanBeFetched!) {
        _networkImage = NetworkImage(src);
      } else {
        _imgElementProvider = impl.createImgElementProvider(src);
      }
      return result;
    });
  }
}

abstract class ImgElementProvider {}
