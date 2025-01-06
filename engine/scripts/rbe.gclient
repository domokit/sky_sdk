# Copy this file to the root of your flutter checkout to bootstrap gclient
# or just run gclient sync in an empty directory with this file.
solutions = [
  {
    "deps_file": "DEPS",
    "managed": False,
    "name": ".",
    "safesync_url": "",

    # If you are using SSH to connect to GitHub, change the URL to:
    # git@github.com:flutter/flutter.git
    "url": "https://github.com/flutter/flutter.git",

    "custom_vars": {
      "use_rbe": True,

      # Uncomment download_emsdk below if you plan to build the web engine.
      # "download_emsdk": True,
    },
  },
]