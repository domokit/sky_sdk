// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter/material.dart';
import 'package:scoped_model/scoped_model.dart';

import '../../gallery_localizations.dart';
import '../../layout/adaptive.dart';
import '../../layout/text_scale.dart';
import 'colors.dart';
import 'model/app_state_model.dart';
import 'model/product.dart';
import 'page_status.dart';
import 'shopping_cart.dart';

// These curves define the emphasized easing curve.
const Cubic _accelerateCurve = Cubic(0.548, 0, 0.757, 0.464);
const Cubic _decelerateCurve = Cubic(0.23, 0.94, 0.41, 1);
// The time at which the accelerate and decelerate curves switch off
const double _peakVelocityTime = 0.248210;
// Percent (as a decimal) of animation that should be completed at _peakVelocityTime
const double _peakVelocityProgress = 0.379146;
// Radius of the shape on the top start of the sheet for mobile layouts.
const double _mobileCornerRadius = 24.0;
// Radius of the shape on the top start and bottom start of the sheet for mobile layouts.
const double _desktopCornerRadius = 12.0;
// Width for just the cart icon and no thumbnails.
const double _cartIconWidth = 64.0;
// Height for just the cart icon and no thumbnails.
const double _cartIconHeight = 56.0;
// Height of a thumbnail.
const double _defaultThumbnailHeight = 40.0;
// Gap between thumbnails.
const double _thumbnailGap = 16.0;

// Maximum number of thumbnails shown in the cart.
const int _maxThumbnailCount = 3;

double _thumbnailHeight(BuildContext context) {
  return _defaultThumbnailHeight * reducedTextScale(context);
}

double _paddedThumbnailHeight(BuildContext context) {
  return _thumbnailHeight(context) + _thumbnailGap;
}

class ExpandingBottomSheet extends StatefulWidget {
  const ExpandingBottomSheet({
    super.key,
    required this.hideController,
    required this.expandingController,
  });

  final AnimationController hideController;
  final AnimationController expandingController;

  @override
  ExpandingBottomSheetState createState() => ExpandingBottomSheetState();

  static ExpandingBottomSheetState? of(BuildContext context, {bool isNullOk = false}) {
    final ExpandingBottomSheetState? result =
        context.findAncestorStateOfType<ExpandingBottomSheetState>();
    if (isNullOk || result != null) {
      return result;
    }
    throw FlutterError(
      'ExpandingBottomSheet.of() called with a context that does not contain a ExpandingBottomSheet.\n',
    );
  }
}

// Emphasized Easing is a motion curve that has an organic, exciting feeling.
// It's very fast to begin with and then very slow to finish. Unlike standard
// curves, like [Curves.fastOutSlowIn], it can't be expressed in a cubic bezier
// curve formula. It's quintic, not cubic. But it _can_ be expressed as one
// curve followed by another, which we do here.
Animation<T> _getEmphasizedEasingAnimation<T>({
  required T begin,
  required T peak,
  required T end,
  required bool isForward,
  required Animation<double> parent,
}) {
  Curve firstCurve;
  Curve secondCurve;
  double firstWeight;
  double secondWeight;

  if (isForward) {
    firstCurve = _accelerateCurve;
    secondCurve = _decelerateCurve;
    firstWeight = _peakVelocityTime;
    secondWeight = 1 - _peakVelocityTime;
  } else {
    firstCurve = _decelerateCurve.flipped;
    secondCurve = _accelerateCurve.flipped;
    firstWeight = 1 - _peakVelocityTime;
    secondWeight = _peakVelocityTime;
  }

  return TweenSequence<T>(<TweenSequenceItem<T>>[
    TweenSequenceItem<T>(
      weight: firstWeight,
      tween: Tween<T>(begin: begin, end: peak).chain(CurveTween(curve: firstCurve)),
    ),
    TweenSequenceItem<T>(
      weight: secondWeight,
      tween: Tween<T>(begin: peak, end: end).chain(CurveTween(curve: secondCurve)),
    ),
  ]).animate(parent);
}

// Calculates the value where two double Animations should be joined. Used by
// callers of _getEmphasisedEasing<double>().
double _getPeakPoint({required double begin, required double end}) {
  return begin + (end - begin) * _peakVelocityProgress;
}

class ExpandingBottomSheetState extends State<ExpandingBottomSheet> {
  final GlobalKey _expandingBottomSheetKey = GlobalKey(debugLabel: 'Expanding bottom sheet');

  // The width of the Material, calculated by _widthFor() & based on the number
  // of products in the cart. 64.0 is the width when there are 0 products
  // (_kWidthForZeroProducts)
  double _width = _cartIconWidth;
  double _height = _cartIconHeight;

  // Controller for the opening and closing of the ExpandingBottomSheet
  AnimationController get _controller => widget.expandingController;

  // Animations for the opening and closing of the ExpandingBottomSheet
  late Animation<double> _widthAnimation;
  late Animation<double> _heightAnimation;
  late Animation<double> _thumbnailOpacityAnimation;
  late Animation<double> _cartOpacityAnimation;
  late Animation<double> _topStartShapeAnimation;
  late Animation<double> _bottomStartShapeAnimation;
  late Animation<Offset> _slideAnimation;
  late Animation<double> _gapAnimation;

  Animation<double> _getWidthAnimation(double screenWidth) {
    if (_controller.status == AnimationStatus.forward) {
      // Opening animation
      return Tween<double>(begin: _width, end: screenWidth).animate(
        CurvedAnimation(
          parent: _controller.view,
          curve: const Interval(0, 0.3, curve: Curves.fastOutSlowIn),
        ),
      );
    } else {
      // Closing animation
      return _getEmphasizedEasingAnimation(
        begin: _width,
        peak: _getPeakPoint(begin: _width, end: screenWidth),
        end: screenWidth,
        isForward: false,
        parent: CurvedAnimation(parent: _controller.view, curve: const Interval(0, 0.87)),
      );
    }
  }

  Animation<double> _getHeightAnimation(double screenHeight) {
    if (_controller.status == AnimationStatus.forward) {
      // Opening animation

      return _getEmphasizedEasingAnimation(
        begin: _height,
        peak: _getPeakPoint(begin: _height, end: screenHeight),
        end: screenHeight,
        isForward: true,
        parent: _controller.view,
      );
    } else {
      // Closing animation
      return Tween<double>(begin: _height, end: screenHeight).animate(
        CurvedAnimation(
          parent: _controller.view,
          curve: const Interval(0.434, 1), // not used
          // only the reverseCurve will be used
          reverseCurve: Interval(0.434, 1, curve: Curves.fastOutSlowIn.flipped),
        ),
      );
    }
  }

  Animation<double> _getDesktopGapAnimation(double gapHeight) {
    final double collapsedGapHeight = gapHeight;
    const double expandedGapHeight = 0.0;

    if (_controller.status == AnimationStatus.forward) {
      // Opening animation

      return _getEmphasizedEasingAnimation(
        begin: collapsedGapHeight,
        peak: collapsedGapHeight + (expandedGapHeight - collapsedGapHeight) * _peakVelocityProgress,
        end: expandedGapHeight,
        isForward: true,
        parent: _controller.view,
      );
    } else {
      // Closing animation
      return Tween<double>(begin: collapsedGapHeight, end: expandedGapHeight).animate(
        CurvedAnimation(
          parent: _controller.view,
          curve: const Interval(0.434, 1), // not used
          // only the reverseCurve will be used
          reverseCurve: Interval(0.434, 1, curve: Curves.fastOutSlowIn.flipped),
        ),
      );
    }
  }

  // Animation of the top-start cut corner. It's cut when closed and not cut when open.
  Animation<double> _getShapeTopStartAnimation(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    final double cornerRadius = isDesktop ? _desktopCornerRadius : _mobileCornerRadius;

    if (_controller.status == AnimationStatus.forward) {
      return Tween<double>(begin: cornerRadius, end: 0).animate(
        CurvedAnimation(
          parent: _controller.view,
          curve: const Interval(0, 0.3, curve: Curves.fastOutSlowIn),
        ),
      );
    } else {
      return _getEmphasizedEasingAnimation(
        begin: cornerRadius,
        peak: _getPeakPoint(begin: cornerRadius, end: 0),
        end: 0,
        isForward: false,
        parent: _controller.view,
      );
    }
  }

  // Animation of the bottom-start cut corner. It's cut when closed and not cut when open.
  Animation<double> _getShapeBottomStartAnimation(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    final double cornerRadius = isDesktop ? _desktopCornerRadius : 0.0;

    if (_controller.status == AnimationStatus.forward) {
      return Tween<double>(begin: cornerRadius, end: 0).animate(
        CurvedAnimation(
          parent: _controller.view,
          curve: const Interval(0, 0.3, curve: Curves.fastOutSlowIn),
        ),
      );
    } else {
      return _getEmphasizedEasingAnimation(
        begin: cornerRadius,
        peak: _getPeakPoint(begin: cornerRadius, end: 0),
        end: 0,
        isForward: false,
        parent: _controller.view,
      );
    }
  }

  Animation<double> _getThumbnailOpacityAnimation() {
    return Tween<double>(begin: 1, end: 0).animate(
      CurvedAnimation(
        parent: _controller.view,
        curve:
            _controller.status == AnimationStatus.forward
                ? const Interval(0, 0.3)
                : const Interval(0.532, 0.766),
      ),
    );
  }

  Animation<double> _getCartOpacityAnimation() {
    return CurvedAnimation(
      parent: _controller.view,
      curve:
          _controller.status == AnimationStatus.forward
              ? const Interval(0.3, 0.6)
              : const Interval(0.766, 1),
    );
  }

  // Returns the correct width of the ExpandingBottomSheet based on the number of
  // products and the text scaling options in the cart in the mobile layout.
  double _mobileWidthFor(int numProducts, BuildContext context) {
    final int cartThumbnailGap = numProducts > 0 ? 16 : 0;
    final double thumbnailsWidth =
        min(numProducts, _maxThumbnailCount) * _paddedThumbnailHeight(context);
    final num overflowNumberWidth =
        numProducts > _maxThumbnailCount ? 30 * cappedTextScale(context) : 0;
    return _cartIconWidth + cartThumbnailGap + thumbnailsWidth + overflowNumberWidth;
  }

  // Returns the correct height of the ExpandingBottomSheet based on the text scaling
  // options in the mobile layout.
  double _mobileHeightFor(BuildContext context) {
    return _paddedThumbnailHeight(context);
  }

  // Returns the correct width of the ExpandingBottomSheet based on the text scaling
  // options in the desktop layout.
  double _desktopWidthFor(BuildContext context) {
    return _paddedThumbnailHeight(context) + 8;
  }

  // Returns the correct height of the ExpandingBottomSheet based on the number of
  // products and the text scaling options in the cart in the desktop layout.
  double _desktopHeightFor(int numProducts, BuildContext context) {
    final int cartThumbnailGap = numProducts > 0 ? 8 : 0;
    final double thumbnailsHeight =
        min(numProducts, _maxThumbnailCount) * _paddedThumbnailHeight(context);
    final num overflowNumberHeight =
        numProducts > _maxThumbnailCount ? 28 * reducedTextScale(context) : 0;
    return _cartIconHeight + cartThumbnailGap + thumbnailsHeight + overflowNumberHeight;
  }

  // Returns true if the cart is open or opening and false otherwise.
  bool get _isOpen => _controller.isForwardOrCompleted;

  // Opens the ExpandingBottomSheet if it's closed, otherwise does nothing.
  void open() {
    if (!_isOpen) {
      _controller.forward();
    }
  }

  // Closes the ExpandingBottomSheet if it's open or opening, otherwise does nothing.
  void close() {
    if (_isOpen) {
      _controller.reverse();
    }
  }

  // Changes the padding between the start edge of the Material and the cart icon
  // based on the number of products in the cart (padding increases when > 0
  // products.)
  EdgeInsetsDirectional _horizontalCartPaddingFor(int numProducts) {
    return (numProducts == 0)
        ? const EdgeInsetsDirectional.only(start: 20, end: 8)
        : const EdgeInsetsDirectional.only(start: 32, end: 8);
  }

  // Changes the padding above and below the cart icon
  // based on the number of products in the cart (padding increases when > 0
  // products.)
  EdgeInsets _verticalCartPaddingFor(int numProducts) {
    return (numProducts == 0)
        ? const EdgeInsets.only(top: 16, bottom: 16)
        : const EdgeInsets.only(top: 16, bottom: 24);
  }

  bool get _cartIsVisible => _thumbnailOpacityAnimation.value == 0;

  // We take 16 pts off of the bottom padding to ensure the collapsed shopping
  // cart is not too tall.
  double get _bottomSafeArea {
    return max(MediaQuery.of(context).viewPadding.bottom - 16, 0);
  }

  Widget _buildThumbnails(BuildContext context, int numProducts) {
    final bool isDesktop = isDisplayDesktop(context);

    Widget thumbnails;

    if (isDesktop) {
      thumbnails = Column(
        children: <Widget>[
          AnimatedPadding(
            padding: _verticalCartPaddingFor(numProducts),
            duration: const Duration(milliseconds: 225),
            child: const Icon(Icons.shopping_cart),
          ),
          SizedBox(
            width: _width,
            height: min(numProducts, _maxThumbnailCount) * _paddedThumbnailHeight(context),
            child: const ProductThumbnailRow(),
          ),
          const ExtraProductsNumber(),
        ],
      );
    } else {
      thumbnails = Column(
        children: <Widget>[
          Row(
            children: <Widget>[
              AnimatedPadding(
                padding: _horizontalCartPaddingFor(numProducts),
                duration: const Duration(milliseconds: 225),
                child: const Icon(Icons.shopping_cart),
              ),
              Container(
                // Accounts for the overflow number
                width:
                    min(numProducts, _maxThumbnailCount) * _paddedThumbnailHeight(context) +
                    (numProducts > 0 ? _thumbnailGap : 0),
                height: _height - _bottomSafeArea,
                padding: const EdgeInsets.symmetric(vertical: 8),
                child: const ProductThumbnailRow(),
              ),
              const ExtraProductsNumber(),
            ],
          ),
        ],
      );
    }

    return ExcludeSemantics(
      child: Opacity(opacity: _thumbnailOpacityAnimation.value, child: thumbnails),
    );
  }

  Widget _buildShoppingCartPage() {
    return Opacity(opacity: _cartOpacityAnimation.value, child: const ShoppingCartPage());
  }

  Widget _buildCart(BuildContext context) {
    // numProducts is the number of different products in the cart (does not
    // include multiples of the same product).
    final bool isDesktop = isDisplayDesktop(context);

    final AppStateModel model = ScopedModel.of<AppStateModel>(context);
    final int numProducts = model.productsInCart.keys.length;
    final int totalCartQuantity = model.totalCartQuantity;
    final Size screenSize = MediaQuery.of(context).size;
    final double screenWidth = screenSize.width;
    final double screenHeight = screenSize.height;

    final double expandedCartWidth =
        isDesktop
            ? (360 * cappedTextScale(context)).clamp(360, screenWidth).toDouble()
            : screenWidth;

    _width = isDesktop ? _desktopWidthFor(context) : _mobileWidthFor(numProducts, context);
    _widthAnimation = _getWidthAnimation(expandedCartWidth);
    _height =
        isDesktop
            ? _desktopHeightFor(numProducts, context)
            : _mobileHeightFor(context) + _bottomSafeArea;
    _heightAnimation = _getHeightAnimation(screenHeight);
    _topStartShapeAnimation = _getShapeTopStartAnimation(context);
    _bottomStartShapeAnimation = _getShapeBottomStartAnimation(context);
    _thumbnailOpacityAnimation = _getThumbnailOpacityAnimation();
    _cartOpacityAnimation = _getCartOpacityAnimation();
    _gapAnimation =
        isDesktop ? _getDesktopGapAnimation(116) : const AlwaysStoppedAnimation<double>(0);

    final Widget child = SizedBox(
      width: _widthAnimation.value,
      height: _heightAnimation.value,
      child: Material(
        animationDuration: Duration.zero,
        shape: BeveledRectangleBorder(
          borderRadius: BorderRadiusDirectional.only(
            topStart: Radius.circular(_topStartShapeAnimation.value),
            bottomStart: Radius.circular(_bottomStartShapeAnimation.value),
          ),
        ),
        elevation: 4,
        color: shrinePink50,
        child: _cartIsVisible ? _buildShoppingCartPage() : _buildThumbnails(context, numProducts),
      ),
    );

    final Widget childWithInteraction =
        productPageIsVisible(context)
            ? Semantics(
              button: true,
              enabled: true,
              label: GalleryLocalizations.of(context)!.shrineScreenReaderCart(totalCartQuantity),
              child: MouseRegion(
                cursor: SystemMouseCursors.click,
                child: GestureDetector(behavior: HitTestBehavior.opaque, onTap: open, child: child),
              ),
            )
            : child;

    return Padding(padding: EdgeInsets.only(top: _gapAnimation.value), child: childWithInteraction);
  }

  // Builder for the hide and reveal animation when the backdrop opens and closes
  Widget _buildSlideAnimation(BuildContext context, Widget child) {
    final bool isDesktop = isDisplayDesktop(context);

    if (isDesktop) {
      return child;
    } else {
      final int textDirectionScalar = Directionality.of(context) == TextDirection.ltr ? 1 : -1;

      _slideAnimation = _getEmphasizedEasingAnimation(
        begin: Offset(1.0 * textDirectionScalar, 0.0),
        peak: Offset(_peakVelocityProgress * textDirectionScalar, 0),
        end: Offset.zero,
        isForward: widget.hideController.status == AnimationStatus.forward,
        parent: widget.hideController,
      );

      return SlideTransition(position: _slideAnimation, child: child);
    }
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedSize(
      key: _expandingBottomSheetKey,
      duration: const Duration(milliseconds: 225),
      curve: Curves.easeInOut,
      alignment: AlignmentDirectional.topStart,
      child: AnimatedBuilder(
        animation: widget.hideController,
        builder:
            (BuildContext context, Widget? child) => AnimatedBuilder(
              animation: widget.expandingController,
              builder:
                  (BuildContext context, Widget? child) => ScopedModelDescendant<AppStateModel>(
                    builder:
                        (BuildContext context, Widget? child, AppStateModel model) =>
                            _buildSlideAnimation(context, _buildCart(context)),
                  ),
            ),
      ),
    );
  }
}

class ProductThumbnailRow extends StatefulWidget {
  const ProductThumbnailRow({super.key});

  @override
  State<ProductThumbnailRow> createState() => _ProductThumbnailRowState();
}

class _ProductThumbnailRowState extends State<ProductThumbnailRow> {
  final GlobalKey<AnimatedListState> _listKey = GlobalKey<AnimatedListState>();

  // _list represents what's currently on screen. If _internalList updates,
  // it will need to be updated to match it.
  late _ListModel _list;

  // _internalList represents the list as it is updated by the AppStateModel.
  late List<int> _internalList;

  @override
  void initState() {
    super.initState();
    _list = _ListModel(
      listKey: _listKey,
      initialItems: ScopedModel.of<AppStateModel>(context).productsInCart.keys.toList(),
      removedItemBuilder: _buildRemovedThumbnail,
    );
    _internalList = List<int>.from(_list.list);
  }

  Product _productWithId(int productId) {
    final AppStateModel model = ScopedModel.of<AppStateModel>(context);
    final Product product = model.getProductById(productId);
    return product;
  }

  Widget _buildRemovedThumbnail(int item, BuildContext context, Animation<double> animation) {
    return ProductThumbnail(animation, animation, _productWithId(item));
  }

  Widget _buildThumbnail(BuildContext context, int index, Animation<double> animation) {
    final Animation<double> thumbnailSize = Tween<double>(begin: 0.8, end: 1).animate(
      CurvedAnimation(curve: const Interval(0.33, 1, curve: Curves.easeIn), parent: animation),
    );

    final Animation<double> opacity = CurvedAnimation(
      curve: const Interval(0.33, 1),
      parent: animation,
    );

    return ProductThumbnail(thumbnailSize, opacity, _productWithId(_list[index]));
  }

  // If the lists are the same length, assume nothing has changed.
  // If the internalList is shorter than the ListModel, an item has been removed.
  // If the internalList is longer, then an item has been added.
  void _updateLists() {
    // Update _internalList based on the model
    _internalList = ScopedModel.of<AppStateModel>(context).productsInCart.keys.toList();
    final Set<int> internalSet = Set<int>.from(_internalList);
    final Set<int> listSet = Set<int>.from(_list.list);

    final Set<int> difference = internalSet.difference(listSet);
    if (difference.isEmpty) {
      return;
    }

    for (final int product in difference) {
      if (_internalList.length < _list.length) {
        _list.remove(product);
      } else if (_internalList.length > _list.length) {
        _list.add(product);
      }
    }

    while (_internalList.length != _list.length) {
      int index = 0;
      // Check bounds and that the list elements are the same
      while (_internalList.isNotEmpty &&
          _list.length > 0 &&
          index < _internalList.length &&
          index < _list.length &&
          _internalList[index] == _list[index]) {
        index++;
      }
    }
  }

  Widget _buildAnimatedList(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    return AnimatedList(
      key: _listKey,
      shrinkWrap: true,
      itemBuilder: _buildThumbnail,
      initialItemCount: _list.length,
      scrollDirection: isDesktop ? Axis.vertical : Axis.horizontal,
      physics: const NeverScrollableScrollPhysics(), // Cart shouldn't scroll
    );
  }

  @override
  Widget build(BuildContext context) {
    return ScopedModelDescendant<AppStateModel>(
      builder: (BuildContext context, Widget? child, AppStateModel model) {
        _updateLists();
        return _buildAnimatedList(context);
      },
    );
  }
}

class ExtraProductsNumber extends StatelessWidget {
  const ExtraProductsNumber({super.key});

  // Calculates the number to be displayed at the end of the row if there are
  // more than three products in the cart. This calculates overflow products,
  // including their duplicates (but not duplicates of products shown as
  // thumbnails).
  int _calculateOverflow(AppStateModel model) {
    final Map<int, int> productMap = model.productsInCart;
    // List created to be able to access products by index instead of ID.
    // Order is guaranteed because productsInCart returns a LinkedHashMap.
    final List<int> products = productMap.keys.toList();
    int overflow = 0;
    final int numProducts = products.length;
    for (int i = _maxThumbnailCount; i < numProducts; i++) {
      overflow += productMap[products[i]]!;
    }
    return overflow;
  }

  Widget _buildOverflow(AppStateModel model, BuildContext context) {
    if (model.productsInCart.length <= _maxThumbnailCount) {
      return Container();
    }

    final int numOverflowProducts = _calculateOverflow(model);
    // Maximum of 99 so padding doesn't get messy.
    final int displayedOverflowProducts = numOverflowProducts <= 99 ? numOverflowProducts : 99;
    return Text(
      '+$displayedOverflowProducts',
      style: Theme.of(context).primaryTextTheme.labelLarge,
    );
  }

  @override
  Widget build(BuildContext context) {
    return ScopedModelDescendant<AppStateModel>(
      builder:
          (BuildContext builder, Widget? child, AppStateModel model) =>
              _buildOverflow(model, context),
    );
  }
}

class ProductThumbnail extends StatelessWidget {
  const ProductThumbnail(this.animation, this.opacityAnimation, this.product, {super.key});

  final Animation<double> animation;
  final Animation<double> opacityAnimation;
  final Product product;

  @override
  Widget build(BuildContext context) {
    final bool isDesktop = isDisplayDesktop(context);

    return FadeTransition(
      opacity: opacityAnimation,
      child: ScaleTransition(
        scale: animation,
        child: Container(
          width: _thumbnailHeight(context),
          height: _thumbnailHeight(context),
          decoration: BoxDecoration(
            image: DecorationImage(
              image: ExactAssetImage(
                product.assetName, // asset name
                package: product.assetPackage, // asset package
              ),
              fit: BoxFit.cover,
            ),
            borderRadius: const BorderRadius.all(Radius.circular(10)),
          ),
          margin:
              isDesktop
                  ? const EdgeInsetsDirectional.only(start: 12, end: 12, bottom: 16)
                  : const EdgeInsetsDirectional.only(start: 16),
        ),
      ),
    );
  }
}

// _ListModel manipulates an internal list and an AnimatedList
class _ListModel {
  _ListModel({required this.listKey, required this.removedItemBuilder, Iterable<int>? initialItems})
    : _items = initialItems?.toList() ?? <int>[];

  final GlobalKey<AnimatedListState> listKey;
  final Widget Function(int, BuildContext, Animation<double>) removedItemBuilder;
  final List<int> _items;

  AnimatedListState? get _animatedList => listKey.currentState;

  void add(int product) {
    _insert(_items.length, product);
  }

  void _insert(int index, int item) {
    _items.insert(index, item);
    _animatedList!.insertItem(index, duration: const Duration(milliseconds: 225));
  }

  void remove(int product) {
    final int index = _items.indexOf(product);
    if (index >= 0) {
      _removeAt(index);
    }
  }

  void _removeAt(int index) {
    final int removedItem = _items.removeAt(index);
    _animatedList!.removeItem(index, (BuildContext context, Animation<double> animation) {
      return removedItemBuilder(removedItem, context, animation);
    });
  }

  int get length => _items.length;

  int operator [](int index) => _items[index];

  int indexOf(int item) => _items.indexOf(item);

  List<int> get list => _items;
}
