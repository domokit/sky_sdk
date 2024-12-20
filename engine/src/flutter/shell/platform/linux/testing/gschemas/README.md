# gsettings-desktop-schemas

This directory contains a few variants of
[gsettings-desktop-schemas](https://packages.ubuntu.com/search?keywords=gsettings-desktop-schemas)
with different schemas for testing purposes.

- [`ubuntu-20.04.compiled`](https://packages.ubuntu.com/focal/gsettings-desktop-schemas)

### Add or update schemas

```bash
# download gsettings-desktop-schemas package
wget http://archive.ubuntu.com/ubuntu/pool/main/g/gsettings-desktop-schemas/gsettings-desktop-schemas_<version>.deb

# extract schema sources (/usr/share/glib-2.0/schemas/*.gschema.xml & .override)
ar x gsettings-desktop-schemas_<version>.deb
tar xf data.tar.zst

# compile schemas (/usr/share/glib-2.0/schemas/gschemas.compiled)
glib-compile-schemas --targetdir path/to/testing/gschemas usr/share/glib-2.0/schemas/
```
