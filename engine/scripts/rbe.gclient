# Copy this file to the root of your flutter checkout to bootstrap gclient
# or just run gclient sync in an empty directory with this file.
solutions = [
  {
    "custom_vars": {
      "use_rbe": True,
    },
    "deps_file": "DEPS",
    "managed": False,
    "name": ".",
    "safesync_url": "",
    "url": "https://github.com/flutter/flutter.git",
  },
]