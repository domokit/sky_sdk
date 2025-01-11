import 'package:flutter/material.dart';

void main() {
  runApp(const InfiniteScrollApp());
}

class InfiniteScrollApp extends StatefulWidget {
  const InfiniteScrollApp({super.key});

  @override
  State<InfiniteScrollApp> createState() => _InfiniteScrollAppState();
}

class _InfiniteScrollAppState extends State<InfiniteScrollApp> {
  List<String> items = List.generate(20, (index) => 'dash');
  final ScrollController _scrollController = ScrollController();

  @override
  void initState() {
    super.initState();
    _scrollController.addListener(_onScroll);
  }

  @override
  void dispose() {
    _scrollController.dispose();
    super.dispose();
  }

  void _onScroll() {
    if (_isBottom) {
      _loadMore();
    }
  }

  bool get _isBottom {
    if (!_scrollController.hasClients) return false;
    final maxScroll = _scrollController.position.maxScrollExtent;
    final currentScroll = _scrollController.offset;
    return currentScroll >= (maxScroll * 0.9);
  }

  void _loadMore() {
    Future.delayed(const Duration(seconds: 1), () {
      setState(() {
        items.addAll(List.generate(20, (index) => 'dash'));
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        body: ListView.builder(
          controller: _scrollController,
          itemCount: items.length,
          itemBuilder: (context, index) {
            return Image.asset('assets/dash.jpg'); // Replace with your image path
          },
        ),
      ),
    );
  }
}