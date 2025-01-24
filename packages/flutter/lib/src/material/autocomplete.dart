// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';

import 'ink_well.dart';
import 'material.dart';
import 'text_form_field.dart';
import 'theme.dart';

/// {@macro flutter.widgets.RawAutocomplete.RawAutocomplete}
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=-Nny8kzW380}
///
/// {@tool dartpad}
/// This example shows how to create a very basic Autocomplete widget using the
/// default UI.
///
/// ** See code in examples/api/lib/material/autocomplete/autocomplete.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to create an Autocomplete widget with a custom type.
/// Try searching with text from the name or email field.
///
/// ** See code in examples/api/lib/material/autocomplete/autocomplete.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to create an Autocomplete widget whose options are
/// fetched over the network.
///
/// ** See code in examples/api/lib/material/autocomplete/autocomplete.2.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to create an Autocomplete widget whose options are
/// fetched over the network. It uses debouncing to wait to perform the network
/// request until after the user finishes typing.
///
/// ** See code in examples/api/lib/material/autocomplete/autocomplete.3.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to create an Autocomplete widget whose options are
/// fetched over the network. It includes both debouncing and error handling, so
/// that failed network requests show an error to the user and can be recovered
/// from. Try toggling the network Switch widget to simulate going offline.
///
/// ** See code in examples/api/lib/material/autocomplete/autocomplete.4.dart **
/// {@end-tool}
///
/// See also:
///
///  * [RawAutocomplete], which is what Autocomplete is built upon, and which
///    contains more detailed examples.
class Autocomplete<T extends Object> extends StatelessWidget {
  /// Creates an instance of [Autocomplete].
  const Autocomplete({
    super.key,
    required this.optionsBuilder,
    this.displayStringForOption = RawAutocomplete.defaultStringForOption,
    this.fieldViewBuilder = _defaultFieldViewBuilder,
    this.onSelected,
    this.optionsMaxHeight = 200.0,
    this.optionsViewBuilder,
    this.optionsViewOpenDirection = OptionsViewOpenDirection.down,
    this.initialValue,
  });

  /// {@macro flutter.widgets.RawAutocomplete.displayStringForOption}
  final AutocompleteOptionToString<T> displayStringForOption;

  /// {@macro flutter.widgets.RawAutocomplete.fieldViewBuilder}
  ///
  /// If not provided, will build a standard Material-style text field by
  /// default.
  final AutocompleteFieldViewBuilder fieldViewBuilder;

  /// {@macro flutter.widgets.RawAutocomplete.onSelected}
  final AutocompleteOnSelected<T>? onSelected;

  /// {@macro flutter.widgets.RawAutocomplete.optionsBuilder}
  final AutocompleteOptionsBuilder<T> optionsBuilder;

  /// {@macro flutter.widgets.RawAutocomplete.optionsViewBuilder}
  ///
  /// If not provided, will build a standard Material-style list of results by
  /// default.
  final AutocompleteOptionsViewBuilder<T>? optionsViewBuilder;

  /// {@macro flutter.widgets.RawAutocomplete.optionsViewOpenDirection}
  final OptionsViewOpenDirection optionsViewOpenDirection;

  /// The maximum height used for the default Material options list widget.
  ///
  /// When [optionsViewBuilder] is `null`, this property sets the maximum height
  /// that the options widget can occupy.
  ///
  /// The default value is set to 200.
  final double optionsMaxHeight;

  /// {@macro flutter.widgets.RawAutocomplete.initialValue}
  final TextEditingValue? initialValue;

  static Widget _defaultFieldViewBuilder(
    BuildContext context,
    TextEditingController textEditingController,
    FocusNode focusNode,
    VoidCallback onFieldSubmitted,
  ) {
    return _AutocompleteField(
      focusNode: focusNode,
      textEditingController: textEditingController,
      onFieldSubmitted: onFieldSubmitted,
    );
  }

  @override
  Widget build(BuildContext context) {
    return RawAutocomplete<T>(
      displayStringForOption: displayStringForOption,
      fieldViewBuilder: fieldViewBuilder,
      initialValue: initialValue,
      optionsBuilder: optionsBuilder,
      optionsViewOpenDirection: optionsViewOpenDirection,
      optionsViewBuilder:
          optionsViewBuilder ??
          (BuildContext context, AutocompleteOnSelected<T> onSelected, Iterable<T> options) {
            return _AutocompleteOptions<T>(
              displayStringForOption: displayStringForOption,
              onSelected: onSelected,
              options: options,
              openDirection: optionsViewOpenDirection,
              optionsMaxHeight: optionsMaxHeight,
            );
          },
      onSelected: onSelected,
    );
  }
}

// The default Material-style Autocomplete text field.
class _AutocompleteField extends StatelessWidget {
  const _AutocompleteField({
    required this.focusNode,
    required this.textEditingController,
    required this.onFieldSubmitted,
  });

  final FocusNode focusNode;

  final VoidCallback onFieldSubmitted;

  final TextEditingController textEditingController;

  @override
  Widget build(BuildContext context) {
    return TextFormField(
      controller: textEditingController,
      focusNode: focusNode,
      onFieldSubmitted: (String value) {
        onFieldSubmitted();
      },
    );
  }
}

// The default Material-style Autocomplete options.
class _AutocompleteOptions<T extends Object> extends StatelessWidget {
  const _AutocompleteOptions({
    super.key,
    required this.displayStringForOption,
    required this.onSelected,
    required this.openDirection,
    required this.options,
    required this.optionsMaxHeight,
  });

  final AutocompleteOptionToString<T> displayStringForOption;
  final AutocompleteOnSelected<T> onSelected;
  final OptionsViewOpenDirection openDirection;
  final Iterable<T> options;
  final double optionsMaxHeight;

  @override
  Widget build(BuildContext context) {
    final int highlightedIndex = AutocompleteHighlightedOption.of(context);

    final AlignmentDirectional optionsAlignment = switch (openDirection) {
      OptionsViewOpenDirection.up => AlignmentDirectional.bottomStart,
      OptionsViewOpenDirection.down => AlignmentDirectional.topStart,
    };

    return Align(
      alignment: optionsAlignment,
      child: Material(
        elevation: 4.0,
        child: ConstrainedBox(
          constraints: BoxConstraints(maxHeight: optionsMaxHeight),
          child: _AutocompleteOptionsList<T>(
            displayStringForOption: displayStringForOption,
            highlightedIndex: highlightedIndex,
            onSelected: onSelected,
            options: options,
          ),
        ),
      ),
    );
  }
}

class _AutocompleteOptionsList<T extends Object> extends StatefulWidget {
  const _AutocompleteOptionsList({
    required this.displayStringForOption,
    required this.highlightedIndex,
    required this.onSelected,
    required this.options,
  });

  final AutocompleteOptionToString<T> displayStringForOption;
  final int highlightedIndex;
  final AutocompleteOnSelected<T> onSelected;
  final Iterable<T> options;

  @override
  State<_AutocompleteOptionsList<T>> createState() => _AutocompleteOptionsListState<T>();
}

class _AutocompleteOptionsListState<T extends Object> extends State<_AutocompleteOptionsList<T>> {
  final ScrollController _scrollController = ScrollController();

  @override
  void didUpdateWidget(_AutocompleteOptionsList<T> oldWidget) {
    super.didUpdateWidget(oldWidget);

    if (widget.highlightedIndex != oldWidget.highlightedIndex) {
      SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
        if (!mounted) {
          return;
        }
        final BuildContext? highlightedContext =
            GlobalObjectKey(widget.options.elementAt(widget.highlightedIndex)).currentContext;
        if (highlightedContext == null) {
          _scrollController.jumpTo(
            widget.highlightedIndex == 0 ? 0.0 : _scrollController.position.maxScrollExtent,
          );
        } else {
          Scrollable.ensureVisible(highlightedContext, alignment: 0.5);
        }
      }, debugLabel: 'AutocompleteOptions.ensureVisible');
    }
  }

  @override
  void dispose() {
    _scrollController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final int highlightedIndex = AutocompleteHighlightedOption.of(context);

    return ListView.builder(
      padding: EdgeInsets.zero,
      shrinkWrap: true,
      controller: _scrollController,
      itemCount: widget.options.length,
      itemBuilder: (BuildContext context, int index) {
        final T option = widget.options.elementAt(index);
        return InkWell(
          key: GlobalObjectKey(option),
          onTap: () {
            widget.onSelected(option);
          },
          child: Builder(
            builder: (BuildContext context) {
              final bool highlight = highlightedIndex == index;
              return Container(
                color: highlight ? Theme.of(context).focusColor : null,
                padding: const EdgeInsets.all(16.0),
                child: Text(widget.displayStringForOption(option)),
              );
            },
          ),
        );
      },
    );
  }
}
