import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    final router = GoRouter(
      // Default to the nested location
      // Or for a more realistic use-case, navigate to the url /root/sub on the web
      initialLocation: '/root/sub',
      routes: [
        StatefulShellRoute.indexedStack(
            builder: (context, state, navigationShell) {
              // return Scaffold(body: const ListTile(title: Text('someText'))); // Does work
              return Scaffold(
                  body: Column(
                    children: [
                      navigationShell,
                    ],
                  )); // Does not work
            },
            branches: [
              StatefulShellBranch(routes: [
                GoRoute(
                  path: '/root',
                  builder: (context, __) {
                    final visibility = Visibility.of(context);
                    print('visibility root: $visibility');
                    // return Container(width: 111, height: 112, color: Colors.red);
                    return Ink(
                      child:
                      Container(width: 111, height: 112, color: Colors.red),
                    );
                  },
                  routes: [
                    // Must be nested, so layouts are stacked on top
                    GoRoute(
                      path: 'sub',
                      builder: (context, __) {
                        final visibility = Visibility.of(context);
                        print('visibility child: $visibility');
                        return Container(
                            width: 222, height: 223, color: Colors.blue);
                        return const Text('subA');
                      },
                    ),
                  ],
                ),
              ]),
            ]),
      ],
    );

    return MaterialApp(
      routes: {
        '/': (context) =>
            Scaffold(
              appBar: AppBar(),
              body: Ink(
                child: Container(width: 111, height: 112, color: Colors.red),
              ),
            ),
        '/sub': (context) =>
            Scaffold(
              appBar: AppBar(),
              body: Ink(
                child: Container(width: 222, height: 223, color: Colors.blue),
              ),
            ),
      },
      initialRoute: '/sub',
    );

    return MaterialApp.router(
      routerConfig: router,
    );
  }
}

class OverlayExample extends StatefulWidget {
  const OverlayExample({super.key});

  @override
  State<OverlayExample> createState() => _OverlayExampleState();
}

class _OverlayExampleState extends State<OverlayExample> {
  @override
  void initState() {
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Ink(
        child: Container(width: 222, height: 222, color: Colors.blue),
      ),
    );
  }
}
