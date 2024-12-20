// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class PestoDemo extends StatelessWidget {
  const PestoDemo({super.key});

  static const String routeName = '/pesto';

  @override
  Widget build(BuildContext context) => const PestoHome();
}

const String _kSmallLogoImage = 'logos/pesto/logo_small.png';
const String _kGalleryAssetsPackage = 'flutter_gallery_assets';
const double _kAppBarHeight = 128.0;
const double _kFabHalfSize = 28.0; // TODO(mpcomplete): needs to adapt to screen size
const double _kRecipePageMaxWidth = 500.0;

final Set<Recipe?> _favoriteRecipes = <Recipe?>{};

final ThemeData _kTheme = ThemeData(
  appBarTheme: const AppBarTheme(foregroundColor: Colors.white, backgroundColor: Colors.teal),
  brightness: Brightness.light,
  floatingActionButtonTheme: const FloatingActionButtonThemeData(foregroundColor: Colors.white),
);

class PestoHome extends StatelessWidget {
  const PestoHome({super.key});

  @override
  Widget build(BuildContext context) {
    return const RecipeGridPage(recipes: kPestoRecipes);
  }
}

class PestoFavorites extends StatelessWidget {
  const PestoFavorites({super.key});

  @override
  Widget build(BuildContext context) {
    return RecipeGridPage(recipes: _favoriteRecipes.toList());
  }
}

class PestoStyle extends TextStyle {
  const PestoStyle({
    double super.fontSize = 12.0,
    super.fontWeight,
    Color super.color = Colors.black87,
    super.letterSpacing,
    super.height,
  }) : super(inherit: false, fontFamily: 'Raleway', textBaseline: TextBaseline.alphabetic);
}

// Displays a grid of recipe cards.
class RecipeGridPage extends StatefulWidget {
  const RecipeGridPage({super.key, this.recipes});

  final List<Recipe?>? recipes;

  @override
  State<RecipeGridPage> createState() => _RecipeGridPageState();
}

class _RecipeGridPageState extends State<RecipeGridPage> {
  @override
  Widget build(BuildContext context) {
    final double statusBarHeight = MediaQuery.of(context).padding.top;
    return Theme(
      data: _kTheme.copyWith(platform: Theme.of(context).platform),
      child: Scaffold(
        floatingActionButton: FloatingActionButton(
          backgroundColor: Colors.redAccent,
          onPressed: () {
            ScaffoldMessenger.of(
              context,
            ).showSnackBar(const SnackBar(content: Text('Not supported.')));
          },
          child: const Icon(Icons.edit),
        ),
        body: CustomScrollView(
          semanticChildCount: widget.recipes!.length,
          slivers: <Widget>[
            _buildAppBar(context, statusBarHeight),
            _buildBody(context, statusBarHeight),
          ],
        ),
      ),
    );
  }

  Widget _buildAppBar(BuildContext context, double statusBarHeight) {
    return SliverAppBar(
      pinned: true,
      expandedHeight: _kAppBarHeight,
      actions: <Widget>[
        IconButton(
          icon: const Icon(Icons.search),
          tooltip: 'Search',
          onPressed: () {
            ScaffoldMessenger.of(
              context,
            ).showSnackBar(const SnackBar(content: Text('Not supported.')));
          },
        ),
      ],
      flexibleSpace: LayoutBuilder(
        builder: (BuildContext context, BoxConstraints constraints) {
          final Size size = constraints.biggest;
          final double appBarHeight = size.height - statusBarHeight;
          final double t = (appBarHeight - kToolbarHeight) / (_kAppBarHeight - kToolbarHeight);
          final double extraPadding = Tween<double>(begin: 10.0, end: 24.0).transform(t);
          final double logoHeight = appBarHeight - 1.5 * extraPadding;
          return Padding(
            padding: EdgeInsets.only(
              top: statusBarHeight + 0.5 * extraPadding,
              bottom: extraPadding,
            ),
            child: Center(child: PestoLogo(height: logoHeight, t: t.clamp(0.0, 1.0))),
          );
        },
      ),
    );
  }

  Widget _buildBody(BuildContext context, double statusBarHeight) {
    final EdgeInsets mediaPadding = MediaQuery.of(context).padding;
    final EdgeInsets padding = EdgeInsets.only(
      top: 8.0,
      left: 8.0 + mediaPadding.left,
      right: 8.0 + mediaPadding.right,
      bottom: 8.0,
    );
    return SliverPadding(
      padding: padding,
      sliver: SliverGrid(
        gridDelegate: const SliverGridDelegateWithMaxCrossAxisExtent(
          maxCrossAxisExtent: _kRecipePageMaxWidth,
          crossAxisSpacing: 8.0,
          mainAxisSpacing: 8.0,
        ),
        delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
          final Recipe? recipe = widget.recipes![index];
          return RecipeCard(
            recipe: recipe,
            onTap: () {
              showRecipePage(context, recipe);
            },
          );
        }, childCount: widget.recipes!.length),
      ),
    );
  }

  void showFavoritesPage(BuildContext context) {
    Navigator.push(
      context,
      MaterialPageRoute<void>(
        settings: const RouteSettings(name: '/pesto/favorites'),
        builder: (BuildContext context) => const PestoFavorites(),
      ),
    );
  }

  void showRecipePage(BuildContext context, Recipe? recipe) {
    Navigator.push(
      context,
      MaterialPageRoute<void>(
        settings: const RouteSettings(name: '/pesto/recipe'),
        builder: (BuildContext context) {
          return Theme(
            data: _kTheme.copyWith(platform: Theme.of(context).platform),
            child: RecipePage(recipe: recipe),
          );
        },
      ),
    );
  }
}

class PestoLogo extends StatefulWidget {
  const PestoLogo({super.key, this.height, this.t});

  final double? height;
  final double? t;

  @override
  State<PestoLogo> createState() => _PestoLogoState();
}

class _PestoLogoState extends State<PestoLogo> {
  // Native sizes for logo and its image/text components.
  static const double kLogoHeight = 162.0;
  static const double kLogoWidth = 220.0;
  static const double kImageHeight = 108.0;
  static const double kTextHeight = 48.0;
  final TextStyle titleStyle = const PestoStyle(
    fontSize: kTextHeight,
    fontWeight: FontWeight.w900,
    color: Colors.white,
    letterSpacing: 3.0,
  );
  final RectTween _textRectTween = RectTween(
    begin: const Rect.fromLTWH(0.0, kLogoHeight, kLogoWidth, kTextHeight),
    end: const Rect.fromLTWH(0.0, kImageHeight, kLogoWidth, kTextHeight),
  );
  final Curve _textOpacity = const Interval(0.4, 1.0, curve: Curves.easeInOut);
  final RectTween _imageRectTween = RectTween(
    begin: const Rect.fromLTWH(0.0, 0.0, kLogoWidth, kLogoHeight),
    end: const Rect.fromLTWH(0.0, 0.0, kLogoWidth, kImageHeight),
  );

  @override
  Widget build(BuildContext context) {
    return Semantics(
      namesRoute: true,
      child: Transform(
        transform: Matrix4.identity()..scale(widget.height! / kLogoHeight),
        alignment: Alignment.topCenter,
        child: SizedBox(
          width: kLogoWidth,
          child: Stack(
            clipBehavior: Clip.none,
            children: <Widget>[
              Positioned.fromRect(
                rect: _imageRectTween.lerp(widget.t!)!,
                child: Image.asset(
                  _kSmallLogoImage,
                  package: _kGalleryAssetsPackage,
                  fit: BoxFit.contain,
                ),
              ),
              Positioned.fromRect(
                rect: _textRectTween.lerp(widget.t!)!,
                child: Opacity(
                  opacity: _textOpacity.transform(widget.t!),
                  child: Text('PESTO', style: titleStyle, textAlign: TextAlign.center),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

// A card with the recipe's image, author, and title.
class RecipeCard extends StatelessWidget {
  const RecipeCard({super.key, this.recipe, this.onTap});

  final Recipe? recipe;
  final VoidCallback? onTap;

  TextStyle get titleStyle => const PestoStyle(fontSize: 24.0, fontWeight: FontWeight.w600);
  TextStyle get authorStyle => const PestoStyle(fontWeight: FontWeight.w500, color: Colors.black54);

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTap: onTap,
      child: Card(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            Hero(
              tag: 'packages/$_kGalleryAssetsPackage/${recipe!.imagePath}',
              child: AspectRatio(
                aspectRatio: 4.0 / 3.0,
                child: Image.asset(
                  recipe!.imagePath!,
                  package: recipe!.imagePackage,
                  fit: BoxFit.cover,
                  semanticLabel: recipe!.name,
                ),
              ),
            ),
            Expanded(
              child: Row(
                children: <Widget>[
                  Padding(
                    padding: const EdgeInsets.all(16.0),
                    child: Image.asset(
                      recipe!.ingredientsImagePath!,
                      package: recipe!.ingredientsImagePackage,
                      width: 48.0,
                      height: 48.0,
                    ),
                  ),
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: <Widget>[
                        Text(
                          recipe!.name!,
                          style: titleStyle,
                          softWrap: false,
                          overflow: TextOverflow.ellipsis,
                        ),
                        Text(recipe!.author!, style: authorStyle),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// Displays one recipe. Includes the recipe sheet with a background image.
class RecipePage extends StatefulWidget {
  const RecipePage({super.key, this.recipe});

  final Recipe? recipe;

  @override
  State<RecipePage> createState() => _RecipePageState();
}

class _RecipePageState extends State<RecipePage> {
  final TextStyle menuItemStyle = const PestoStyle(
    fontSize: 15.0,
    color: Colors.black54,
    height: 24.0 / 15.0,
  );

  double _getAppBarHeight(BuildContext context) => MediaQuery.of(context).size.height * 0.3;

  @override
  Widget build(BuildContext context) {
    // The full page content with the recipe's image behind it. This
    // adjusts based on the size of the screen. If the recipe sheet touches
    // the edge of the screen, use a slightly different layout.
    final double appBarHeight = _getAppBarHeight(context);
    final Size screenSize = MediaQuery.of(context).size;
    final bool fullWidth = screenSize.width < _kRecipePageMaxWidth;
    final bool isFavorite = _favoriteRecipes.contains(widget.recipe);
    return Scaffold(
      body: Stack(
        children: <Widget>[
          Positioned(
            top: 0.0,
            left: 0.0,
            right: 0.0,
            height: appBarHeight + _kFabHalfSize,
            child: Hero(
              tag: 'packages/$_kGalleryAssetsPackage/${widget.recipe!.imagePath}',
              child: Image.asset(
                widget.recipe!.imagePath!,
                package: widget.recipe!.imagePackage,
                fit: fullWidth ? BoxFit.fitWidth : BoxFit.cover,
              ),
            ),
          ),
          CustomScrollView(
            slivers: <Widget>[
              SliverAppBar(
                expandedHeight: appBarHeight - _kFabHalfSize,
                backgroundColor: Colors.transparent,
                actions: <Widget>[
                  PopupMenuButton<String>(
                    onSelected: (String item) {},
                    itemBuilder:
                        (BuildContext context) => <PopupMenuItem<String>>[
                          _buildMenuItem(Icons.share, 'Tweet recipe'),
                          _buildMenuItem(Icons.email, 'Email recipe'),
                          _buildMenuItem(Icons.message, 'Message recipe'),
                          _buildMenuItem(Icons.people, 'Share on Facebook'),
                        ],
                  ),
                ],
                flexibleSpace: const FlexibleSpaceBar(
                  background: DecoratedBox(
                    decoration: BoxDecoration(
                      gradient: LinearGradient(
                        begin: Alignment.topCenter,
                        end: Alignment(0.0, -0.2),
                        colors: <Color>[Color(0x60000000), Color(0x00000000)],
                      ),
                    ),
                  ),
                ),
              ),
              SliverToBoxAdapter(
                child: Stack(
                  children: <Widget>[
                    Container(
                      padding: const EdgeInsets.only(top: _kFabHalfSize),
                      width: fullWidth ? null : _kRecipePageMaxWidth,
                      child: RecipeSheet(recipe: widget.recipe),
                    ),
                    Positioned(
                      right: 16.0,
                      child: FloatingActionButton(
                        backgroundColor: Colors.redAccent,
                        onPressed: _toggleFavorite,
                        child: Icon(isFavorite ? Icons.favorite : Icons.favorite_border),
                      ),
                    ),
                  ],
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  PopupMenuItem<String> _buildMenuItem(IconData icon, String label) {
    return PopupMenuItem<String>(
      child: Row(
        children: <Widget>[
          Padding(
            padding: const EdgeInsets.only(right: 24.0),
            child: Icon(icon, color: Colors.black54),
          ),
          Text(label, style: menuItemStyle),
        ],
      ),
    );
  }

  void _toggleFavorite() {
    setState(() {
      if (_favoriteRecipes.contains(widget.recipe)) {
        _favoriteRecipes.remove(widget.recipe);
      } else {
        _favoriteRecipes.add(widget.recipe);
      }
    });
  }
}

/// Displays the recipe's name and instructions.
class RecipeSheet extends StatelessWidget {
  RecipeSheet({super.key, this.recipe});

  final TextStyle titleStyle = const PestoStyle(fontSize: 34.0);
  final TextStyle descriptionStyle = const PestoStyle(
    fontSize: 15.0,
    color: Colors.black54,
    height: 24.0 / 15.0,
  );
  final TextStyle itemStyle = const PestoStyle(fontSize: 15.0, height: 24.0 / 15.0);
  final TextStyle itemAmountStyle = PestoStyle(
    fontSize: 15.0,
    color: _kTheme.primaryColor,
    height: 24.0 / 15.0,
  );
  final TextStyle headingStyle = const PestoStyle(
    fontSize: 16.0,
    fontWeight: FontWeight.bold,
    height: 24.0 / 15.0,
  );

  final Recipe? recipe;

  @override
  Widget build(BuildContext context) {
    return Material(
      child: SafeArea(
        top: false,
        bottom: false,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 16.0, vertical: 40.0),
          child: Table(
            columnWidths: const <int, TableColumnWidth>{0: FixedColumnWidth(64.0)},
            children: <TableRow>[
              TableRow(
                children: <Widget>[
                  TableCell(
                    verticalAlignment: TableCellVerticalAlignment.middle,
                    child: Image.asset(
                      recipe!.ingredientsImagePath!,
                      package: recipe!.ingredientsImagePackage,
                      width: 32.0,
                      height: 32.0,
                      alignment: Alignment.centerLeft,
                      fit: BoxFit.scaleDown,
                    ),
                  ),
                  TableCell(
                    verticalAlignment: TableCellVerticalAlignment.middle,
                    child: Text(recipe!.name!, style: titleStyle),
                  ),
                ],
              ),
              TableRow(
                children: <Widget>[
                  const SizedBox(),
                  Padding(
                    padding: const EdgeInsets.only(top: 8.0, bottom: 4.0),
                    child: Text(recipe!.description!, style: descriptionStyle),
                  ),
                ],
              ),
              TableRow(
                children: <Widget>[
                  const SizedBox(),
                  Padding(
                    padding: const EdgeInsets.only(top: 24.0, bottom: 4.0),
                    child: Text('Ingredients', style: headingStyle),
                  ),
                ],
              ),
              ...recipe!.ingredients!.map<TableRow>((RecipeIngredient ingredient) {
                return _buildItemRow(ingredient.amount!, ingredient.description!);
              }),
              TableRow(
                children: <Widget>[
                  const SizedBox(),
                  Padding(
                    padding: const EdgeInsets.only(top: 24.0, bottom: 4.0),
                    child: Text('Steps', style: headingStyle),
                  ),
                ],
              ),
              ...recipe!.steps!.map<TableRow>((RecipeStep step) {
                return _buildItemRow(step.duration ?? '', step.description!);
              }),
            ],
          ),
        ),
      ),
    );
  }

  TableRow _buildItemRow(String left, String right) {
    return TableRow(
      children: <Widget>[
        Padding(
          padding: const EdgeInsets.symmetric(vertical: 4.0),
          child: Text(left, style: itemAmountStyle),
        ),
        Padding(
          padding: const EdgeInsets.symmetric(vertical: 4.0),
          child: Text(right, style: itemStyle),
        ),
      ],
    );
  }
}

class Recipe {
  const Recipe({
    this.name,
    this.author,
    this.description,
    this.imagePath,
    this.imagePackage,
    this.ingredientsImagePath,
    this.ingredientsImagePackage,
    this.ingredients,
    this.steps,
  });

  final String? name;
  final String? author;
  final String? description;
  final String? imagePath;
  final String? imagePackage;
  final String? ingredientsImagePath;
  final String? ingredientsImagePackage;
  final List<RecipeIngredient>? ingredients;
  final List<RecipeStep>? steps;
}

class RecipeIngredient {
  const RecipeIngredient({this.amount, this.description});

  final String? amount;
  final String? description;
}

class RecipeStep {
  const RecipeStep({this.duration, this.description});

  final String? duration;
  final String? description;
}

const List<Recipe> kPestoRecipes = <Recipe>[
  Recipe(
    name: 'Roasted Chicken',
    author: 'Peter Carlsson',
    ingredientsImagePath: 'food/icons/main.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        'The perfect dish to welcome your family and friends with on a crisp autumn night. Pair with roasted veggies to truly impress them.',
    imagePath: 'food/roasted_chicken.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[
      RecipeIngredient(amount: '1 whole', description: 'Chicken'),
      RecipeIngredient(amount: '1/2 cup', description: 'Butter'),
      RecipeIngredient(amount: '1 tbsp', description: 'Onion powder'),
      RecipeIngredient(amount: '1 tbsp', description: 'Freshly ground pepper'),
      RecipeIngredient(amount: '1 tsp', description: 'Salt'),
    ],
    steps: <RecipeStep>[
      RecipeStep(duration: '1 min', description: 'Put in oven'),
      RecipeStep(duration: '1hr 45 min', description: 'Cook'),
    ],
  ),
  Recipe(
    name: 'Chopped Beet Leaves',
    author: 'Trevor Hansen',
    ingredientsImagePath: 'food/icons/veggie.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        'This vegetable has more to offer than just its root. Beet greens can be tossed into a salad to add some variety or sauteed on its own with some oil and garlic.',
    imagePath: 'food/chopped_beet_leaves.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[RecipeIngredient(amount: '3 cups', description: 'Beet greens')],
    steps: <RecipeStep>[RecipeStep(duration: '5 min', description: 'Chop')],
  ),
  Recipe(
    name: 'Pesto Pasta',
    author: 'Ali Connors',
    ingredientsImagePath: 'food/icons/main.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        "With this pesto recipe, you can quickly whip up a meal to satisfy your savory needs. And if you're feeling festive, you can add bacon to taste.",
    imagePath: 'food/pesto_pasta.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[
      RecipeIngredient(amount: '1/4 cup ', description: 'Pasta'),
      RecipeIngredient(amount: '2 cups', description: 'Fresh basil leaves'),
      RecipeIngredient(amount: '1/2 cup', description: 'Parmesan cheese'),
      RecipeIngredient(amount: '1/2 cup', description: 'Extra virgin olive oil'),
      RecipeIngredient(amount: '1/3 cup', description: 'Pine nuts'),
      RecipeIngredient(amount: '1/4 cup', description: 'Lemon juice'),
      RecipeIngredient(amount: '3 cloves', description: 'Garlic'),
      RecipeIngredient(amount: '1/4 tsp', description: 'Salt'),
      RecipeIngredient(amount: '1/8 tsp', description: 'Pepper'),
      RecipeIngredient(amount: '3 lbs', description: 'Bacon'),
    ],
    steps: <RecipeStep>[RecipeStep(duration: '15 min', description: 'Blend')],
  ),
  Recipe(
    name: 'Cherry Pie',
    author: 'Sandra Adams',
    ingredientsImagePath: 'food/icons/main.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        "Sometimes when you're craving some cheer in your life you can jumpstart your day with some cherry pie. Dessert for breakfast is perfectly acceptable.",
    imagePath: 'food/cherry_pie.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[
      RecipeIngredient(amount: '1', description: 'Pie crust'),
      RecipeIngredient(amount: '4 cups', description: 'Fresh or frozen cherries'),
      RecipeIngredient(amount: '1 cup', description: 'Granulated sugar'),
      RecipeIngredient(amount: '4 tbsp', description: 'Cornstarch'),
      RecipeIngredient(amount: '1½ tbsp', description: 'Butter'),
    ],
    steps: <RecipeStep>[
      RecipeStep(duration: '15 min', description: 'Mix'),
      RecipeStep(duration: '1hr 30 min', description: 'Bake'),
    ],
  ),
  Recipe(
    name: 'Spinach Salad',
    author: 'Peter Carlsson',
    ingredientsImagePath: 'food/icons/spicy.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        "Everyone's favorite leafy green is back. Paired with fresh sliced onion, it's ready to tackle any dish, whether it be a salad or an egg scramble.",
    imagePath: 'food/spinach_onion_salad.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[
      RecipeIngredient(amount: '4 cups', description: 'Spinach'),
      RecipeIngredient(amount: '1 cup', description: 'Sliced onion'),
    ],
    steps: <RecipeStep>[RecipeStep(duration: '5 min', description: 'Mix')],
  ),
  Recipe(
    name: 'Butternut Squash Soup',
    author: 'Ali Connors',
    ingredientsImagePath: 'food/icons/healthy.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        'This creamy butternut squash soup will warm you on the chilliest of winter nights and bring a delightful pop of orange to the dinner table.',
    imagePath: 'food/butternut_squash_soup.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[
      RecipeIngredient(amount: '1', description: 'Butternut squash'),
      RecipeIngredient(amount: '4 cups', description: 'Chicken stock'),
      RecipeIngredient(amount: '2', description: 'Potatoes'),
      RecipeIngredient(amount: '1', description: 'Onion'),
      RecipeIngredient(amount: '1', description: 'Carrot'),
      RecipeIngredient(amount: '1', description: 'Celery'),
      RecipeIngredient(amount: '1 tsp', description: 'Salt'),
      RecipeIngredient(amount: '1 tsp', description: 'Pepper'),
    ],
    steps: <RecipeStep>[
      RecipeStep(duration: '10 min', description: 'Prep vegetables'),
      RecipeStep(duration: '5 min', description: 'Stir'),
      RecipeStep(duration: '1 hr 10 min', description: 'Cook'),
    ],
  ),
  Recipe(
    name: 'Spanakopita',
    author: 'Trevor Hansen',
    ingredientsImagePath: 'food/icons/quick.png',
    ingredientsImagePackage: _kGalleryAssetsPackage,
    description:
        "You 'feta' believe this is a crowd-pleaser! Flaky phyllo pastry surrounds a delicious mixture of spinach and cheeses to create the perfect appetizer.",
    imagePath: 'food/spanakopita.png',
    imagePackage: _kGalleryAssetsPackage,
    ingredients: <RecipeIngredient>[
      RecipeIngredient(amount: '1 lb', description: 'Spinach'),
      RecipeIngredient(amount: '½ cup', description: 'Feta cheese'),
      RecipeIngredient(amount: '½ cup', description: 'Cottage cheese'),
      RecipeIngredient(amount: '2', description: 'Eggs'),
      RecipeIngredient(amount: '1', description: 'Onion'),
      RecipeIngredient(amount: '½ lb', description: 'Phyllo dough'),
    ],
    steps: <RecipeStep>[
      RecipeStep(duration: '5 min', description: 'Sauté vegetables'),
      RecipeStep(duration: '3 min', description: 'Stir vegetables and other filling ingredients'),
      RecipeStep(
        duration: '10 min',
        description: 'Fill phyllo squares half-full with filling and fold.',
      ),
      RecipeStep(duration: '40 min', description: 'Bake'),
    ],
  ),
];
