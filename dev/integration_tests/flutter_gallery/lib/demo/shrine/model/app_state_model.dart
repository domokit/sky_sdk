// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:scoped_model/scoped_model.dart';

import 'product.dart';
import 'products_repository.dart' as product_repository;

double _salesTaxRate = 0.06;
double _shippingCostPerItem = 7.0;

class AppStateModel extends Model {
  // All the available products.
  List<Product>? _availableProducts;

  // The currently selected category of products.
  Category _selectedCategory = Category.all;

  // The IDs and quantities of products currently in the cart.
  final Map<int, int> _productsInCart = <int, int>{};

  Map<int, int> get productsInCart => Map<int, int>.from(_productsInCart);

  // Total number of items in the cart.
  int get totalCartQuantity => _productsInCart.values.fold(0, (int v, int e) => v + e);

  Category get selectedCategory => _selectedCategory;

  // Totaled prices of the items in the cart.
  double get subtotalCost {
    return _productsInCart.keys
        .map((int id) => _availableProducts![id].price * _productsInCart[id]!)
        .fold(0.0, (double sum, int e) => sum + e);
  }

  // Total shipping cost for the items in the cart.
  double get shippingCost {
    return _shippingCostPerItem * _productsInCart.values.fold(0.0, (num sum, int e) => sum + e);
  }

  // Sales tax for the items in the cart
  double get tax => subtotalCost * _salesTaxRate;

  // Total cost to order everything in the cart.
  double get totalCost => subtotalCost + shippingCost + tax;

  // Returns a copy of the list of available products, filtered by category.
  List<Product> getProducts() {
    if (_availableProducts == null) {
      return <Product>[];
    }

    if (_selectedCategory == Category.all) {
      return List<Product>.from(_availableProducts!);
    } else {
      return _availableProducts!.where((Product p) => p.category == _selectedCategory).toList();
    }
  }

  // Adds a product to the cart.
  void addProductToCart(int productId) {
    final int value = _productsInCart[productId] ?? 0;
    _productsInCart[productId] = value + 1;

    notifyListeners();
  }

  // Removes an item from the cart.
  void removeItemFromCart(int productId) {
    switch (_productsInCart[productId]) {
      case 1:
        _productsInCart.remove(productId);
      case final int value:
        _productsInCart[productId] = value - 1;
    }
    notifyListeners();
  }

  // Returns the Product instance matching the provided id.
  Product getProductById(int id) {
    return _availableProducts!.firstWhere((Product p) => p.id == id);
  }

  // Removes everything from the cart.
  void clearCart() {
    _productsInCart.clear();
    notifyListeners();
  }

  // Loads the list of available products from the repo.
  void loadProducts() {
    _availableProducts = product_repository.loadProducts(Category.all);
    notifyListeners();
  }

  void setCategory(Category newCategory) {
    _selectedCategory = newCategory;
    notifyListeners();
  }

  @override
  String toString() {
    return 'AppStateModel(totalCost: $totalCost)';
  }
}
