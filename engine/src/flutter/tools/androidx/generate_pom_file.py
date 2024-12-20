#!/usr/bin/env python3
#
# Copyright 2019 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import datetime
import os
import sys
import json

THIS_DIR = os.path.abspath(os.path.dirname(__file__))

# The template for the POM file.
POM_FILE_CONTENT = '''<?xml version="1.0" encoding="UTF-8"?>
<project xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 http://maven.apache.org/xsd/maven-4.0.0.xsd" xmlns="http://maven.apache.org/POM/4.0.0"
    xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <modelVersion>4.0.0</modelVersion>
  <groupId>io.flutter</groupId>
  <artifactId>{0}</artifactId>
  <version>{1}</version>
  <packaging>jar</packaging>
  <dependencies>
    {2}
  </dependencies>
</project>
'''

POM_DEPENDENCY = '''
    <dependency>
      <groupId>{0}</groupId>
      <artifactId>{1}</artifactId>
      <version>{2}</version>
      <scope>compile</scope>
    </dependency>
'''

MAVEN_METADATA_CONTENT = '''
<metadata xmlns="http://maven.apache.org/METADATA/1.1.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://maven.apache.org/METADATA/1.1.0 http://maven.apache.org/xsd/metadata-1.1.0.xsd" modelVersion="1.1.0">
  <groupId>io.flutter</groupId>
  <artifactId>{0}</artifactId>
  <version>{1}</version>
  <versioning>
    <versions>
      <version>{1}</version>
    </versions>
    <snapshot>
      <timestamp>{2}</timestamp>
      <buildNumber>0</buildNumber>
    </snapshot>
    <snapshotVersions>
      <snapshotVersion>
        <extension>jar</extension>
        <value>{1}</value>
      </snapshotVersion>
      <snapshotVersion>
        <extension>pom</extension>
        <value>{1}</value>
      </snapshotVersion>
    </snapshotVersions>
  </versioning>
</metadata>
'''


def utf8(s):
  return str(s, 'utf-8') if isinstance(s, (bytes, bytearray)) else s


def main():
  with open(os.path.join(THIS_DIR, 'files.json')) as f:
    dependencies = json.load(f)

  parser = argparse.ArgumentParser(description='Generate the POM file for the engine artifacts')
  parser.add_argument(
      '--engine-artifact-id',
      type=utf8,
      required=True,
      help='The artifact id. e.g. android_arm_release'
  )
  parser.add_argument('--engine-version', type=utf8, required=True, help='The engine commit hash')
  parser.add_argument(
      '--destination', type=utf8, required=True, help='The destination directory absolute path'
  )
  parser.add_argument(
      '--include-embedding-dependencies',
      type=bool,
      help='Include the dependencies for the embedding'
  )

  args = parser.parse_args()
  engine_artifact_id = args.engine_artifact_id
  engine_version = args.engine_version
  artifact_version = '1.0.0-' + engine_version
  out_file_name = '%s.pom' % engine_artifact_id

  pom_dependencies = ''
  if args.include_embedding_dependencies:
    for dependency in dependencies:
      if not dependency['provides']:
        # Don't include transitive dependencies since they aren't used by the embedding.
        continue
      group_id, artifact_id, version = dependency['maven_dependency'].split(':')
      pom_dependencies += POM_DEPENDENCY.format(group_id, artifact_id, version)

  # Write the POM file.
  with open(os.path.join(args.destination, out_file_name), 'w') as f:
    f.write(POM_FILE_CONTENT.format(engine_artifact_id, artifact_version, pom_dependencies))

  # Write the Maven metadata file.
  with open(os.path.join(args.destination, '%s.maven-metadata.xml' % engine_artifact_id), 'w') as f:
    timestamp = datetime.datetime.utcnow().strftime("%Y%m%d.%H%M%S")
    f.write(MAVEN_METADATA_CONTENT.format(engine_artifact_id, artifact_version, timestamp))


if __name__ == '__main__':
  sys.exit(main())
