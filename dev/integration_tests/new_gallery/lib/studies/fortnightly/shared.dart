// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:intl/intl.dart';

import '../../data/gallery_options.dart';
import '../../gallery_localizations.dart';
import '../../layout/image_placeholder.dart';
import '../../layout/text_scale.dart';

class ArticleData {
  ArticleData({
    required this.imageUrl,
    required this.imageAspectRatio,
    required this.category,
    required this.title,
    this.snippet,
  });

  final String imageUrl;
  final double imageAspectRatio;
  final String category;
  final String title;
  final String? snippet;
}

class HorizontalArticlePreview extends StatelessWidget {
  const HorizontalArticlePreview({super.key, required this.data, this.minutes});

  final ArticleData data;
  final int? minutes;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        Expanded(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: <Widget>[
              SelectableText(data.category, style: textTheme.titleMedium),
              const SizedBox(height: 12),
              SelectableText(data.title, style: textTheme.headlineSmall!.copyWith(fontSize: 16)),
            ],
          ),
        ),
        if (minutes != null) ...<Widget>[
          SelectableText(
            GalleryLocalizations.of(context)!.craneMinutes(minutes!),
            style: textTheme.bodyLarge,
          ),
          const SizedBox(width: 8),
        ],
        FadeInImagePlaceholder(
          image: AssetImage(data.imageUrl, package: 'flutter_gallery_assets'),
          placeholder: Container(
            color: Colors.black.withOpacity(0.1),
            width: 64 / (1 / data.imageAspectRatio),
            height: 64,
          ),
          fit: BoxFit.cover,
          excludeFromSemantics: true,
        ),
      ],
    );
  }
}

class VerticalArticlePreview extends StatelessWidget {
  const VerticalArticlePreview({
    super.key,
    required this.data,
    this.width,
    this.headlineTextStyle,
    this.showSnippet = false,
  });

  final ArticleData data;
  final double? width;
  final TextStyle? headlineTextStyle;
  final bool showSnippet;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;

    return SizedBox(
      width: width ?? double.infinity,
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          SizedBox(
            width: double.infinity,
            child: FadeInImagePlaceholder(
              image: AssetImage(data.imageUrl, package: 'flutter_gallery_assets'),
              placeholder: LayoutBuilder(
                builder: (BuildContext context, BoxConstraints constraints) {
                  return Container(
                    color: Colors.black.withOpacity(0.1),
                    width: constraints.maxWidth,
                    height: constraints.maxWidth / data.imageAspectRatio,
                  );
                },
              ),
              fit: BoxFit.fitWidth,
              width: double.infinity,
              excludeFromSemantics: true,
            ),
          ),
          const SizedBox(height: 12),
          SelectableText(data.category, style: textTheme.titleMedium),
          const SizedBox(height: 12),
          SelectableText(data.title, style: headlineTextStyle ?? textTheme.headlineSmall),
          if (showSnippet) ...<Widget>[
            const SizedBox(height: 4),
            SelectableText(data.snippet!, style: textTheme.bodyMedium),
          ],
        ],
      ),
    );
  }
}

List<Widget> buildArticlePreviewItems(BuildContext context) {
  final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
  final Widget articleDivider = Container(
    margin: const EdgeInsets.symmetric(vertical: 16),
    color: Colors.black.withOpacity(0.07),
    height: 1,
  );
  final Widget sectionDivider = Container(
    margin: const EdgeInsets.symmetric(vertical: 16),
    color: Colors.black.withOpacity(0.2),
    height: 1,
  );
  final TextTheme textTheme = Theme.of(context).textTheme;

  return <Widget>[
    VerticalArticlePreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_healthcare.jpg',
        imageAspectRatio: 391 / 248,
        category: localizations.fortnightlyMenuWorld.toUpperCase(),
        title: localizations.fortnightlyHeadlineHealthcare,
      ),
      headlineTextStyle: textTheme.headlineSmall!.copyWith(fontSize: 20),
    ),
    articleDivider,
    HorizontalArticlePreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_war.png',
        imageAspectRatio: 1,
        category: localizations.fortnightlyMenuPolitics.toUpperCase(),
        title: localizations.fortnightlyHeadlineWar,
      ),
    ),
    articleDivider,
    HorizontalArticlePreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_gas.png',
        imageAspectRatio: 1,
        category: localizations.fortnightlyMenuTech.toUpperCase(),
        title: localizations.fortnightlyHeadlineGasoline,
      ),
    ),
    sectionDivider,
    SelectableText(localizations.fortnightlyLatestUpdates, style: textTheme.titleLarge),
    articleDivider,
    HorizontalArticlePreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_army.png',
        imageAspectRatio: 1,
        category: localizations.fortnightlyMenuPolitics.toUpperCase(),
        title: localizations.fortnightlyHeadlineArmy,
      ),
      minutes: 2,
    ),
    articleDivider,
    HorizontalArticlePreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_stocks.png',
        imageAspectRatio: 77 / 64,
        category: localizations.fortnightlyMenuWorld.toUpperCase(),
        title: localizations.fortnightlyHeadlineStocks,
      ),
      minutes: 5,
    ),
    articleDivider,
    HorizontalArticlePreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_fabrics.png',
        imageAspectRatio: 76 / 64,
        category: localizations.fortnightlyMenuTech.toUpperCase(),
        title: localizations.fortnightlyHeadlineFabrics,
      ),
      minutes: 4,
    ),
    articleDivider,
  ];
}

class HashtagBar extends StatelessWidget {
  const HashtagBar({super.key});

  @override
  Widget build(BuildContext context) {
    final Container verticalDivider = Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      color: Colors.black.withOpacity(0.1),
      width: 1,
    );
    final TextTheme textTheme = Theme.of(context).textTheme;
    final double height = 32 * reducedTextScale(context);

    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;

    return SizedBox(
      height: height,
      child: ListView(
        restorationId: 'hashtag_bar_list_view',
        scrollDirection: Axis.horizontal,
        children: <Widget>[
          const SizedBox(width: 16),
          Center(
            child: SelectableText(
              '#${localizations.fortnightlyTrendingTechDesign}',
              style: textTheme.titleSmall,
            ),
          ),
          verticalDivider,
          Center(
            child: SelectableText(
              '#${localizations.fortnightlyTrendingReform}',
              style: textTheme.titleSmall,
            ),
          ),
          verticalDivider,
          Center(
            child: SelectableText(
              '#${localizations.fortnightlyTrendingHealthcareRevolution}',
              style: textTheme.titleSmall,
            ),
          ),
          verticalDivider,
          Center(
            child: SelectableText(
              '#${localizations.fortnightlyTrendingGreenArmy}',
              style: textTheme.titleSmall,
            ),
          ),
          verticalDivider,
          Center(
            child: SelectableText(
              '#${localizations.fortnightlyTrendingStocks}',
              style: textTheme.titleSmall,
            ),
          ),
          verticalDivider,
        ],
      ),
    );
  }
}

class NavigationMenu extends StatelessWidget {
  const NavigationMenu({super.key, this.isCloseable = false});

  final bool isCloseable;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return ListView(
      children: <Widget>[
        if (isCloseable)
          Row(
            children: <Widget>[
              IconButton(
                key: StandardComponentType.closeButton.key,
                icon: const Icon(Icons.close),
                tooltip: MaterialLocalizations.of(context).closeButtonTooltip,
                onPressed: () => Navigator.pop(context),
              ),
              Image.asset(
                'fortnightly/fortnightly_title.png',
                package: 'flutter_gallery_assets',
                excludeFromSemantics: true,
              ),
            ],
          ),
        const SizedBox(height: 32),
        MenuItem(localizations.fortnightlyMenuFrontPage, header: true),
        MenuItem(localizations.fortnightlyMenuWorld),
        MenuItem(localizations.fortnightlyMenuUS),
        MenuItem(localizations.fortnightlyMenuPolitics),
        MenuItem(localizations.fortnightlyMenuBusiness),
        MenuItem(localizations.fortnightlyMenuTech),
        MenuItem(localizations.fortnightlyMenuScience),
        MenuItem(localizations.fortnightlyMenuSports),
        MenuItem(localizations.fortnightlyMenuTravel),
        MenuItem(localizations.fortnightlyMenuCulture),
      ],
    );
  }
}

class MenuItem extends StatelessWidget {
  const MenuItem(this.title, {super.key, this.header = false});

  final String title;
  final bool header;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: Row(
        children: <Widget>[
          Container(
            width: 32,
            alignment: Alignment.centerLeft,
            child: header ? null : const Icon(Icons.arrow_drop_down),
          ),
          Expanded(
            child: SelectableText(
              title,
              style: Theme.of(context).textTheme.titleMedium!.copyWith(
                fontWeight: header ? FontWeight.w700 : FontWeight.w600,
                fontSize: 16,
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class StockItem extends StatelessWidget {
  const StockItem({super.key, required this.ticker, required this.price, required this.percent});

  final String ticker;
  final String price;
  final double percent;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;
    final NumberFormat percentFormat = NumberFormat.decimalPercentPattern(
      locale: GalleryOptions.of(context).locale.toString(),
      decimalDigits: 2,
    );

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        SelectableText(ticker, style: textTheme.titleMedium),
        const SizedBox(height: 2),
        Row(
          children: <Widget>[
            Expanded(
              child: SelectableText(
                price,
                style: textTheme.titleSmall!.copyWith(
                  color: textTheme.titleSmall!.color!.withOpacity(0.75),
                ),
              ),
            ),
            SelectableText(
              percent > 0 ? '+' : '-',
              style: textTheme.titleSmall!.copyWith(
                fontSize: 12,
                color: percent > 0 ? const Color(0xff20CF63) : const Color(0xff661FFF),
              ),
            ),
            const SizedBox(width: 4),
            SelectableText(
              percentFormat.format(percent.abs() / 100),
              style: textTheme.bodySmall!.copyWith(
                fontSize: 12,
                color: textTheme.titleSmall!.color!.withOpacity(0.75),
              ),
            ),
          ],
        ),
      ],
    );
  }
}

List<Widget> buildStockItems(BuildContext context) {
  final Widget articleDivider = Container(
    margin: const EdgeInsets.symmetric(vertical: 16),
    color: Colors.black.withOpacity(0.07),
    height: 1,
  );
  const double imageAspectRatio = 165 / 55;

  return <Widget>[
    SizedBox(
      width: double.infinity,
      child: FadeInImagePlaceholder(
        image: const AssetImage(
          'fortnightly/fortnightly_chart.png',
          package: 'flutter_gallery_assets',
        ),
        placeholder: LayoutBuilder(
          builder: (BuildContext context, BoxConstraints constraints) {
            return Container(
              color: Colors.black.withOpacity(0.1),
              width: constraints.maxWidth,
              height: constraints.maxWidth / imageAspectRatio,
            );
          },
        ),
        width: double.infinity,
        fit: BoxFit.contain,
        excludeFromSemantics: true,
      ),
    ),
    articleDivider,
    const StockItem(ticker: 'DIJA', price: '7,031.21', percent: -0.48),
    articleDivider,
    const StockItem(ticker: 'SP', price: '1,967.84', percent: -0.23),
    articleDivider,
    const StockItem(ticker: 'Nasdaq', price: '6,211.46', percent: 0.52),
    articleDivider,
    const StockItem(ticker: 'Nikkei', price: '5,891', percent: 1.16),
    articleDivider,
    const StockItem(ticker: 'DJ Total', price: '89.02', percent: 0.80),
    articleDivider,
  ];
}

class VideoPreview extends StatelessWidget {
  const VideoPreview({super.key, required this.data, required this.time});

  final ArticleData data;
  final String time;

  @override
  Widget build(BuildContext context) {
    final TextTheme textTheme = Theme.of(context).textTheme;

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: <Widget>[
        SizedBox(
          width: double.infinity,
          child: FadeInImagePlaceholder(
            image: AssetImage(data.imageUrl, package: 'flutter_gallery_assets'),
            placeholder: LayoutBuilder(
              builder: (BuildContext context, BoxConstraints constraints) {
                return Container(
                  color: Colors.black.withOpacity(0.1),
                  width: constraints.maxWidth,
                  height: constraints.maxWidth / data.imageAspectRatio,
                );
              },
            ),
            fit: BoxFit.contain,
            width: double.infinity,
            excludeFromSemantics: true,
          ),
        ),
        const SizedBox(height: 4),
        Row(
          children: <Widget>[
            Expanded(child: SelectableText(data.category, style: textTheme.titleMedium)),
            SelectableText(time, style: textTheme.bodyLarge),
          ],
        ),
        const SizedBox(height: 4),
        SelectableText(data.title, style: textTheme.headlineSmall!.copyWith(fontSize: 16)),
      ],
    );
  }
}

List<Widget> buildVideoPreviewItems(BuildContext context) {
  final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
  return <Widget>[
    VideoPreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_feminists.jpg',
        imageAspectRatio: 148 / 88,
        category: localizations.fortnightlyMenuPolitics.toUpperCase(),
        title: localizations.fortnightlyHeadlineFeminists,
      ),
      time: '2:31',
    ),
    const SizedBox(height: 32),
    VideoPreview(
      data: ArticleData(
        imageUrl: 'fortnightly/fortnightly_bees.jpg',
        imageAspectRatio: 148 / 88,
        category: localizations.fortnightlyMenuUS.toUpperCase(),
        title: localizations.fortnightlyHeadlineBees,
      ),
      time: '1:37',
    ),
  ];
}

ThemeData buildTheme(BuildContext context) {
  final TextTheme lightTextTheme = ThemeData.light().textTheme;
  return ThemeData(
    scaffoldBackgroundColor: Colors.white,
    appBarTheme: AppBarTheme(
      color: Colors.white,
      elevation: 0,
      iconTheme: IconTheme.of(context).copyWith(color: Colors.black),
    ),
    highlightColor: Colors.transparent,
    textTheme: TextTheme(
      // preview snippet
      bodyMedium: GoogleFonts.merriweather(
        fontWeight: FontWeight.w300,
        fontSize: 16,
        textStyle: lightTextTheme.bodyMedium,
      ),
      // time in latest updates
      bodyLarge: GoogleFonts.libreFranklin(
        fontWeight: FontWeight.w500,
        fontSize: 11,
        color: Colors.black.withOpacity(0.5),
        textStyle: lightTextTheme.bodyLarge,
      ),
      // preview headlines
      headlineSmall: GoogleFonts.libreFranklin(
        fontWeight: FontWeight.w500,
        fontSize: 16,
        textStyle: lightTextTheme.headlineSmall,
      ),
      // (caption 2), preview category, stock ticker
      titleMedium: GoogleFonts.robotoCondensed(fontWeight: FontWeight.w700, fontSize: 16),
      titleSmall: GoogleFonts.libreFranklin(
        fontWeight: FontWeight.w400,
        fontSize: 14,
        textStyle: lightTextTheme.titleSmall,
      ),
      // section titles: Top Highlights, Last Updated...
      titleLarge: GoogleFonts.merriweather(
        fontWeight: FontWeight.w700,
        fontStyle: FontStyle.italic,
        fontSize: 14,
        textStyle: lightTextTheme.titleLarge,
      ),
    ),
  );
}
