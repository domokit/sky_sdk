# Image Codecs in the Flutter Engine

Flutter has built-in support for a number of common codecs. Images in the supported formats can be decompressed by the Flutter Engine even if they are not supported by the underlying platform.

However, each codec supported out-of-the-box by the Flutter Engine adds to the binary size of the engine. Unlike AOT compiled Dart code, this binary size increase cannot be tree shaken away. Every user of the Flutter application will be forced to download the codec irrespective of whether their Flutter applications actually decompress images of that type. Flutter users and developers are extremely sensitive to increases in binary size of the application.

If the Flutter engine is unable to decompress specific images because it doesn’t support that codec, it will delegate responsibility of image decompression to the platform. This comes in handy if the platform has support for newer or less widely used codecs. The downside of course is that the application author will need to provide an alternative in case both the engine and platform don’t support a specific codec.

If the application wishes to support a codec that isn’t supported by either the engine or the platform, it will have to ship the codec along with their application and expose its functionality via a plugin. This is more cumbersome but the current recommendation for applications for whom support for specific formats is table stakes.

Speculatively adding and removing codecs into the Flutter engine may lead to fragmentation in the supported codecs across Flutter engine versions. But, if a format does become widely used, it strengthens the case for adding the codec to the engine in spite of the binary size increase.
