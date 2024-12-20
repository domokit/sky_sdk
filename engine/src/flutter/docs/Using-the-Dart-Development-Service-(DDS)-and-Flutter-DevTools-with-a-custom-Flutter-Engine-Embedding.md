When attempting to connect to the VM service URI output by the Flutter engine in the context of a custom embedder, users may encounter the following error:

`This VM does not have a registered Dart Development Service (DDS) instance and is not currently serving Dart DevTools.`

This happens when a Flutter application is launched without using the `flutter` CLI tool, which typically is responsible for starting a Dart Development Service (DDS) instance. DDS is middleware for the Dart VM Service that provides additional functionality like log and event history, and can be configured to serve the DevTools developer tooling suite.

Developers working on custom embeddings of the Flutter engine can start a DDS instance in one of two ways:

1) Using the `dart development-service` command shipped with the Dart SDK (**recommended**).
2) Starting a Flutter DevTools instance using the `dart devtools` shipped with the Dart SDK, providing the VM service URI as an argument (e.g., `dart devtools http://localhost:8181`).

## Using `dart development-service`

DDS can be started using the `dart development-service` command. As of writing, the command has the following interface allowing for configuration of the service:

```
Start Dart's development service.

Usage: dart [vm-options] development-service [arguments]
-h, --help                                 Print this usage information.
    --vm-service-uri=<uri> (mandatory)     The VM service URI DDS will connect to.
    --bind-address=<address>               The address DDS should bind to.
                                           (defaults to "localhost")
    --bind-port=<port>                     The port DDS should be served on.
                                           (defaults to "0")
    --[no-]disable-service-auth-codes      Disables authentication codes.
    --[no-]serve-devtools                  If provided, DDS will serve DevTools. If not specified, "--devtools-server-address" is ignored.
    --devtools-server-address              Redirect to an existing DevTools server. Ignored if "--serve-devtools" is not specified.
    --[no-]enable-service-port-fallback    Bind to a random port if DDS fails to bind to the provided port.
    --cached-user-tags                     A set of UserTag names used to determine which CPU samples are cached by DDS.
    --google3-workspace-root               Sets the Google3 workspace root used for google3:// URI resolution.

Run "dart help" to see global options.
```

The `--vm-service-uri` option is required and specifies the URI of the Dart VM service served by the Flutter engine. If provided, the `--serve-devtools` flag will result in the DevTools instance shipped with the Dart SDK from DDS's HTTP server.

Running the command will result in JSON encoded connection information being output to `stdout`:

```bash
$ dart development-service --vm-service-uri=http://127.0.0.1:59113/BBPoXnZUWFU=/ --serve-devtools
{"state":"started","ddsUri":"http://127.0.0.1:59123/tbrR0DzW2j8=/","devToolsUri":"http://127.0.0.1:59123/tbrR0DzW2j8=/devtools?uri=ws://127.0.0.1:59123/tbrR0DzW2j8=/ws","dtd":{"uri":"ws://127.0.0.1:59122/R1LbdlhtkUygRWNA"}}
```

Once DDS has started, all VM service requests should be made through the URI provided by DDS instead of the original VM service URI. In the context of the standalone Dart VM (i.e., `dart`) and the `flutter` tool, the original VM service URI is hidden and the DDS URI is advertised as the Dart VM service URI to reduce the likelihood of developers accidentally connecting to the VM service directly. Direct connections to a VM service with an active DDS instance attached will be rejected.

The DDS instance will automatically shutdown when the target application is closed, requiring no manual lifecycle management.

## Using `dart devtools`

Developers are also able to start DevTools using the `dart devtools` command. By providing the VM service URI as a positional parameter, the served DevTools instance will automatically connect to the provided target application. However, before connecting directly to the VM service URI, the command first checks that the provided VM service URI points to a DDS instance. If it doesn't, the command will start DDS, print the DDS URI to console, and then launch a DevTools instance that connects to the DDS instance directly instead of to the provided VM service URI.

```bash
$ dart devtools http://127.0.0.1:59251/2LS6f3Kb2JI=/
Started the Dart Development Service (DDS) at http://127.0.0.1:59260/38XeuQpIHRE=/
Serving DevTools at http://127.0.0.1:9101.

          Hit ctrl-c to terminate the server.
```

As with the `dart development-service` command, the lifecycle of DDS is tied directly to the lifecycle of the application it's connected to. However, using the `dart devtools` command does not cause DevTools to be served by DDS, meaning that DevTools will not be available if the `dart devtools` process is killed, even though DDS will remain attached to the target application.