// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'action_buttons.dart';
/// @docImport 'animated_icons.dart';
/// @docImport 'icon_button.dart';
/// @docImport 'list_tile.dart';
library;

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'app_bar.dart';
import 'app_bar_theme.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'debug.dart';
import 'input_border.dart';
import 'input_decorator.dart';
import 'material_localizations.dart';
import 'scaffold.dart';
import 'text_field.dart';
import 'theme.dart';

/// Shows a full screen search page and returns the search result selected by
/// the user when the page is closed.
///
/// The search page consists of an app bar with a search field and a body which
/// can either show suggested search queries or the search results.
///
/// The appearance of the search page is determined by the provided
/// `delegate`. The initial query string is given by `query`, which defaults
/// to the empty string. When `query` is set to null, `delegate.query` will
/// be used as the initial query.
///
/// This method returns the selected search result, which can be set in the
/// [SearchDelegate.close] call. If the search page is closed with the system
/// back button, it returns null.
///
/// A given [SearchDelegate] can only be associated with one active [showSearch]
/// call. Call [SearchDelegate.close] before re-using the same delegate instance
/// for another [showSearch] call.
///
/// The `useRootNavigator` argument is used to determine whether to push the
/// search page to the [Navigator] furthest from or nearest to the given
/// `context`. By default, `useRootNavigator` is `false` and the search page
/// route created by this method is pushed to the nearest navigator to the
/// given `context`. It can not be `null`.
///
/// The `maintainState` argument is used to determine if the route should remain
/// in memory when it is inactive (see [ModalRoute.maintainState] for more details].
/// By default, `maintainState` is `false`.
///
/// The transition to the search page triggered by this method looks best if the
/// screen triggering the transition contains an [AppBar] at the top and the
/// transition is called from an [IconButton] that's part of [AppBar.actions].
/// The animation provided by [SearchDelegate.transitionAnimation] can be used
/// to trigger additional animations in the underlying page while the search
/// page fades in or out. This is commonly used to animate an [AnimatedIcon] in
/// the [AppBar.leading] position e.g. from the hamburger menu to the back arrow
/// used to exit the search page.
///
/// ## Handling emojis and other complex characters
/// {@macro flutter.widgets.EditableText.onChanged}
///
/// See also:
///
///  * [SearchDelegate] to define the content of the search page.
Future<T?> showSearch<T>({
  required BuildContext context,
  required SearchDelegate<T> delegate,
  String? query = '',
  bool useRootNavigator = false,
  bool maintainState = false,
}) {
  delegate.query = query ?? delegate.query;
  delegate._currentBody = _SearchBody.suggestions;
  return Navigator.of(
    context,
    rootNavigator: useRootNavigator,
  ).push(_SearchPageRoute<T>(delegate: delegate, maintainState: maintainState));
}

/// Delegate for [showSearch] to define the content of the search page.
///
/// The search page always shows an [AppBar] at the top where users can
/// enter their search queries. The buttons shown before and after the search
/// query text field can be customized via [SearchDelegate.buildLeading]
/// and [SearchDelegate.buildActions]. Additionally, a widget can be placed
/// across the bottom of the [AppBar] via [SearchDelegate.buildBottom].
///
/// The body below the [AppBar] can either show suggested queries (returned by
/// [SearchDelegate.buildSuggestions]) or - once the user submits a search  - the
/// results of the search as returned by [SearchDelegate.buildResults].
///
/// [SearchDelegate.query] always contains the current query entered by the user
/// and should be used to build the suggestions and results.
///
/// The results can be brought on screen by calling [SearchDelegate.showResults]
/// and you can go back to showing the suggestions by calling
/// [SearchDelegate.showSuggestions].
///
/// Once the user has selected a search result, [SearchDelegate.close] should be
/// called to remove the search page from the top of the navigation stack and
/// to notify the caller of [showSearch] about the selected search result.
///
/// A given [SearchDelegate] can only be associated with one active [showSearch]
/// call. Call [SearchDelegate.close] before re-using the same delegate instance
/// for another [showSearch] call.
///
/// ## Handling emojis and other complex characters
/// {@macro flutter.widgets.EditableText.onChanged}
abstract class SearchDelegate<T> {
  /// Constructor to be called by subclasses which may specify
  /// [searchFieldLabel], either [searchFieldStyle] or [searchFieldDecorationTheme],
  /// [keyboardType] and/or [textInputAction]. Only one of [searchFieldLabel]
  /// and [searchFieldDecorationTheme] may be non-null.
  ///
  /// {@tool snippet}
  /// ```dart
  /// class CustomSearchHintDelegate extends SearchDelegate<String> {
  ///   CustomSearchHintDelegate({
  ///     required String hintText,
  ///   }) : super(
  ///     searchFieldLabel: hintText,
  ///     keyboardType: TextInputType.text,
  ///     textInputAction: TextInputAction.search,
  ///   );
  ///
  ///   @override
  ///   Widget buildLeading(BuildContext context) => const Text('leading');
  ///
  ///   @override
  ///   PreferredSizeWidget buildBottom(BuildContext context) {
  ///     return const PreferredSize(
  ///        preferredSize: Size.fromHeight(56.0),
  ///        child: Text('bottom'));
  ///   }
  ///
  ///   @override
  ///   Widget buildSuggestions(BuildContext context) => const Text('suggestions');
  ///
  ///   @override
  ///   Widget buildResults(BuildContext context) => const Text('results');
  ///
  ///   @override
  ///   List<Widget> buildActions(BuildContext context) => <Widget>[];
  /// }
  /// ```
  /// {@end-tool}
  SearchDelegate({
    this.searchFieldLabel,
    this.searchFieldStyle,
    this.searchFieldDecorationTheme,
    this.keyboardType,
    this.textInputAction = TextInputAction.search,
    this.autocorrect = true,
    this.enableSuggestions = true,
  }) : assert(searchFieldStyle == null || searchFieldDecorationTheme == null);

  /// Suggestions shown in the body of the search page while the user types a
  /// query into the search field.
  ///
  /// The delegate method is called whenever the content of [query] changes.
  /// The suggestions should be based on the current [query] string. If the query
  /// string is empty, it is good practice to show suggested queries based on
  /// past queries or the current context.
  ///
  /// Usually, this method will return a [ListView] with one [ListTile] per
  /// suggestion. When [ListTile.onTap] is called, [query] should be updated
  /// with the corresponding suggestion and the results page should be shown
  /// by calling [showResults].
  Widget buildSuggestions(BuildContext context);

  /// The results shown after the user submits a search from the search page.
  ///
  /// The current value of [query] can be used to determine what the user
  /// searched for.
  ///
  /// This method might be applied more than once to the same query.
  /// If your [buildResults] method is computationally expensive, you may want
  /// to cache the search results for one or more queries.
  ///
  /// Typically, this method returns a [ListView] with the search results.
  /// When the user taps on a particular search result, [close] should be called
  /// with the selected result as argument. This will close the search page and
  /// communicate the result back to the initial caller of [showSearch].
  Widget buildResults(BuildContext context);

  /// A widget to display before the current query in the [AppBar].
  ///
  /// Typically an [IconButton] configured with a [BackButtonIcon] that exits
  /// the search with [close]. One can also use an [AnimatedIcon] driven by
  /// [transitionAnimation], which animates from e.g. a hamburger menu to the
  /// back button as the search overlay fades in.
  ///
  /// Returns null if no widget should be shown.
  ///
  /// See also:
  ///
  ///  * [AppBar.leading], the intended use for the return value of this method.
  Widget? buildLeading(BuildContext context);

  /// {@macro flutter.material.appbar.automaticallyImplyLeading}
  bool? automaticallyImplyLeading;

  /// {@macro flutter.material.appbar.leadingWidth}
  double? leadingWidth;

  /// Widgets to display after the search query in the [AppBar].
  ///
  /// If the [query] is not empty, this should typically contain a button to
  /// clear the query and show the suggestions again (via [showSuggestions]) if
  /// the results are currently shown.
  ///
  /// Returns null if no widget should be shown.
  ///
  /// See also:
  ///
  ///  * [AppBar.actions], the intended use for the return value of this method.
  List<Widget>? buildActions(BuildContext context);

  /// Widget to display across the bottom of the [AppBar].
  ///
  /// Returns null by default, i.e. a bottom widget is not included.
  ///
  /// See also:
  ///
  ///  * [AppBar.bottom], the intended use for the return value of this method.
  ///
  PreferredSizeWidget? buildBottom(BuildContext context) => null;

  /// Widget to display a flexible space in the [AppBar].
  ///
  /// Returns null by default, i.e. a flexible space widget is not included.
  ///
  /// See also:
  ///
  ///  * [AppBar.flexibleSpace], the intended use for the return value of this method.
  Widget? buildFlexibleSpace(BuildContext context) => null;

  /// The theme used to configure the search page.
  ///
  /// The returned [ThemeData] will be used to wrap the entire search page,
  /// so it can be used to configure any of its components with the appropriate
  /// theme properties.
  ///
  /// Unless overridden, the default theme will configure the AppBar containing
  /// the search input text field with a white background and black text on light
  /// themes. For dark themes the default is a dark grey background with light
  /// color text.
  ///
  /// See also:
  ///
  ///  * [AppBarTheme], which configures the AppBar's appearance.
  ///  * [InputDecorationTheme], which configures the appearance of the search
  ///    text field.
  ThemeData appBarTheme(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final ColorScheme colorScheme = theme.colorScheme;
    return theme.copyWith(
      appBarTheme: AppBarTheme(
        systemOverlayStyle:
            colorScheme.brightness == Brightness.dark
                ? SystemUiOverlayStyle.light
                : SystemUiOverlayStyle.dark,
        backgroundColor:
            colorScheme.brightness == Brightness.dark ? Colors.grey[900] : Colors.white,
        iconTheme: theme.primaryIconTheme.copyWith(color: Colors.grey),
        titleTextStyle: theme.textTheme.titleLarge,
        toolbarTextStyle: theme.textTheme.bodyMedium,
      ),
      inputDecorationTheme:
          searchFieldDecorationTheme ??
          InputDecorationTheme(
            hintStyle: searchFieldStyle ?? theme.inputDecorationTheme.hintStyle,
            border: InputBorder.none,
          ),
    );
  }

  /// The current query string shown in the [AppBar].
  ///
  /// The user manipulates this string via the keyboard.
  ///
  /// If the user taps on a suggestion provided by [buildSuggestions] this
  /// string should be updated to that suggestion via the setter.
  String get query => _queryTextController.text;

  /// Changes the current query string.
  ///
  /// Setting the query string programmatically moves the cursor to the end of the text field.
  set query(String value) {
    _queryTextController.value = TextEditingValue(
      text: value,
      selection: TextSelection.collapsed(offset: value.length),
    );
  }

  /// Transition from the suggestions returned by [buildSuggestions] to the
  /// [query] results returned by [buildResults].
  ///
  /// If the user taps on a suggestion provided by [buildSuggestions] the
  /// screen should typically transition to the page showing the search
  /// results for the suggested query. This transition can be triggered
  /// by calling this method.
  ///
  /// See also:
  ///
  ///  * [showSuggestions] to show the search suggestions again.
  void showResults(BuildContext context) {
    _focusNode?.unfocus();
    _currentBody = _SearchBody.results;
  }

  /// Transition from showing the results returned by [buildResults] to showing
  /// the suggestions returned by [buildSuggestions].
  ///
  /// Calling this method will also put the input focus back into the search
  /// field of the [AppBar].
  ///
  /// If the results are currently shown this method can be used to go back
  /// to showing the search suggestions.
  ///
  /// See also:
  ///
  ///  * [showResults] to show the search results.
  void showSuggestions(BuildContext context) {
    assert(_focusNode != null, '_focusNode must be set by route before showSuggestions is called.');
    _focusNode!.requestFocus();
    _currentBody = _SearchBody.suggestions;
  }

  /// Closes the search page and returns to the underlying route.
  ///
  /// The value provided for `result` is used as the return value of the call
  /// to [showSearch] that launched the search initially.
  void close(BuildContext context, T result) {
    _currentBody = null;
    _focusNode?.unfocus();
    Navigator.of(context)
      ..popUntil((Route<dynamic> route) => route == _route)
      ..pop(result);
  }

  /// The hint text that is shown in the search field when it is empty.
  ///
  /// If this value is set to null, the value of
  /// `MaterialLocalizations.of(context).searchFieldLabel` will be used instead.
  final String? searchFieldLabel;

  /// The style of the [searchFieldLabel].
  ///
  /// If this value is set to null, the value of the ambient [Theme]'s
  /// [InputDecorationTheme.hintStyle] will be used instead.
  ///
  /// Only one of [searchFieldStyle] or [searchFieldDecorationTheme] can
  /// be non-null.
  final TextStyle? searchFieldStyle;

  /// The [InputDecorationTheme] used to configure the search field's visuals.
  ///
  /// Only one of [searchFieldStyle] or [searchFieldDecorationTheme] can
  /// be non-null.
  final InputDecorationTheme? searchFieldDecorationTheme;

  /// The type of action button to use for the keyboard.
  ///
  /// Defaults to the default value specified in [TextField].
  final TextInputType? keyboardType;

  /// {@macro flutter.widgets.editableText.autocorrect}
  final bool autocorrect;

  /// {@macro flutter.services.TextInputConfiguration.enableSuggestions}
  final bool enableSuggestions;

  /// The text input action configuring the soft keyboard to a particular action
  /// button.
  ///
  /// Defaults to [TextInputAction.search].
  final TextInputAction textInputAction;

  /// [Animation] triggered when the search pages fades in or out.
  ///
  /// This animation is commonly used to animate [AnimatedIcon]s of
  /// [IconButton]s returned by [buildLeading] or [buildActions]. It can also be
  /// used to animate [IconButton]s contained within the route below the search
  /// page.
  Animation<double> get transitionAnimation => _proxyAnimation;

  // The focus node to use for manipulating focus on the search page. This is
  // managed, owned, and set by the _SearchPageRoute using this delegate.
  FocusNode? _focusNode;

  final TextEditingController _queryTextController = TextEditingController();

  final ProxyAnimation _proxyAnimation = ProxyAnimation(kAlwaysDismissedAnimation);

  final ValueNotifier<_SearchBody?> _currentBodyNotifier = ValueNotifier<_SearchBody?>(null);

  _SearchBody? get _currentBody => _currentBodyNotifier.value;
  set _currentBody(_SearchBody? value) {
    _currentBodyNotifier.value = value;
  }

  _SearchPageRoute<T>? _route;

  /// Releases the resources.
  @mustCallSuper
  void dispose() {
    _currentBodyNotifier.dispose();
    _focusNode?.dispose();
    _queryTextController.dispose();
    _proxyAnimation.parent = null;
  }
}

/// Describes the body that is currently shown under the [AppBar] in the
/// search page.
enum _SearchBody {
  /// Suggested queries are shown in the body.
  ///
  /// The suggested queries are generated by [SearchDelegate.buildSuggestions].
  suggestions,

  /// Search results are currently shown in the body.
  ///
  /// The search results are generated by [SearchDelegate.buildResults].
  results,
}

class _SearchPageRoute<T> extends PageRoute<T> {
  _SearchPageRoute({required this.delegate, required this.maintainState}) {
    assert(
      delegate._route == null,
      'The ${delegate.runtimeType} instance is currently used by another active '
      'search. Please close that search by calling close() on the SearchDelegate '
      'before opening another search with the same delegate instance.',
    );
    delegate._route = this;
  }

  final SearchDelegate<T> delegate;

  @override
  final bool maintainState;

  @override
  Color? get barrierColor => null;

  @override
  String? get barrierLabel => null;

  @override
  Duration get transitionDuration => const Duration(milliseconds: 300);

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return FadeTransition(opacity: animation, child: child);
  }

  @override
  Animation<double> createAnimation() {
    final Animation<double> animation = super.createAnimation();
    delegate._proxyAnimation.parent = animation;
    return animation;
  }

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return _SearchPage<T>(delegate: delegate, animation: animation);
  }

  @override
  void didComplete(T? result) {
    super.didComplete(result);
    assert(delegate._route == this);
    delegate._route = null;
    delegate._currentBody = null;
  }
}

class _SearchPage<T> extends StatefulWidget {
  const _SearchPage({required this.delegate, required this.animation});

  final SearchDelegate<T> delegate;
  final Animation<double> animation;

  @override
  State<StatefulWidget> createState() => _SearchPageState<T>();
}

class _SearchPageState<T> extends State<_SearchPage<T>> {
  // This node is owned, but not hosted by, the search page. Hosting is done by
  // the text field.
  FocusNode focusNode = FocusNode();

  @override
  void initState() {
    super.initState();
    widget.delegate._queryTextController.addListener(_onQueryChanged);
    widget.animation.addStatusListener(_onAnimationStatusChanged);
    widget.delegate._currentBodyNotifier.addListener(_onSearchBodyChanged);
    focusNode.addListener(_onFocusChanged);
    widget.delegate._focusNode = focusNode;
  }

  @override
  void dispose() {
    super.dispose();
    widget.delegate._queryTextController.removeListener(_onQueryChanged);
    widget.animation.removeStatusListener(_onAnimationStatusChanged);
    widget.delegate._currentBodyNotifier.removeListener(_onSearchBodyChanged);
    widget.delegate._focusNode = null;
    focusNode.dispose();
  }

  void _onAnimationStatusChanged(AnimationStatus status) {
    if (!status.isCompleted) {
      return;
    }
    widget.animation.removeStatusListener(_onAnimationStatusChanged);
    if (widget.delegate._currentBody == _SearchBody.suggestions) {
      focusNode.requestFocus();
    }
  }

  @override
  void didUpdateWidget(_SearchPage<T> oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.delegate != oldWidget.delegate) {
      oldWidget.delegate._queryTextController.removeListener(_onQueryChanged);
      widget.delegate._queryTextController.addListener(_onQueryChanged);
      oldWidget.delegate._currentBodyNotifier.removeListener(_onSearchBodyChanged);
      widget.delegate._currentBodyNotifier.addListener(_onSearchBodyChanged);
      oldWidget.delegate._focusNode = null;
      widget.delegate._focusNode = focusNode;
    }
  }

  void _onFocusChanged() {
    if (focusNode.hasFocus && widget.delegate._currentBody != _SearchBody.suggestions) {
      widget.delegate.showSuggestions(context);
    }
  }

  void _onQueryChanged() {
    setState(() {
      // rebuild ourselves because query changed.
    });
  }

  void _onSearchBodyChanged() {
    setState(() {
      // rebuild ourselves because search body changed.
    });
  }

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    final ThemeData theme = widget.delegate.appBarTheme(context);
    final String searchFieldLabel =
        widget.delegate.searchFieldLabel ?? MaterialLocalizations.of(context).searchFieldLabel;
    Widget? body;
    switch (widget.delegate._currentBody) {
      case _SearchBody.suggestions:
        body = KeyedSubtree(
          key: const ValueKey<_SearchBody>(_SearchBody.suggestions),
          child: widget.delegate.buildSuggestions(context),
        );
      case _SearchBody.results:
        body = KeyedSubtree(
          key: const ValueKey<_SearchBody>(_SearchBody.results),
          child: widget.delegate.buildResults(context),
        );
      case null:
        break;
    }

    late final String routeName;
    switch (theme.platform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        routeName = '';
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        routeName = searchFieldLabel;
    }

    return Semantics(
      explicitChildNodes: true,
      scopesRoute: true,
      namesRoute: true,
      label: routeName,
      child: Theme(
        data: theme,
        child: Scaffold(
          appBar: AppBar(
            leadingWidth: widget.delegate.leadingWidth,
            automaticallyImplyLeading: widget.delegate.automaticallyImplyLeading ?? true,
            leading: widget.delegate.buildLeading(context),
            title: TextField(
              controller: widget.delegate._queryTextController,
              focusNode: focusNode,
              style: widget.delegate.searchFieldStyle ?? theme.textTheme.titleLarge,
              textInputAction: widget.delegate.textInputAction,
              autocorrect: widget.delegate.autocorrect,
              enableSuggestions: widget.delegate.enableSuggestions,
              keyboardType: widget.delegate.keyboardType,
              onSubmitted: (String _) => widget.delegate.showResults(context),
              decoration: InputDecoration(hintText: searchFieldLabel),
            ),
            flexibleSpace: widget.delegate.buildFlexibleSpace(context),
            actions: widget.delegate.buildActions(context),
            bottom: widget.delegate.buildBottom(context),
          ),
          body: AnimatedSwitcher(duration: const Duration(milliseconds: 300), child: body),
        ),
      ),
    );
  }
}
