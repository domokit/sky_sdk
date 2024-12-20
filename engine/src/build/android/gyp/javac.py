#!/usr/bin/env python3
#
# Copyright 2013 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import fnmatch
import optparse
import os
import shutil
import re
import sys
import textwrap

from util import ansi_colors
from util import build_utils
from util import md5_check

import jar

def ColorJavacOutput(output):
  fileline_prefix = r'(?P<fileline>(?P<file>[-.\w/\\]+.java):(?P<line>[0-9]+):)'
  warning_re = re.compile(
      fileline_prefix + r'(?P<full_message> warning: (?P<message>.*))$')
  error_re = re.compile(
      fileline_prefix + r'(?P<full_message> (?P<message>.*))$')
  marker_re = re.compile(r'\s*(?P<marker>\^)\s*$')

  warning_color = ['full_message', ansi_colors.FOREGROUND_YELLOW + ansi_colors.STYLE_DIM]
  error_color = ['full_message', ansi_colors.FOREGROUND_MAGENTA + ansi_colors.STYLE_BRIGHT]
  marker_color = ['marker',  ansi_colors.FOREGROUND_BLUE + ansi_colors.STYLE_BRIGHT]

  def Colorize(line, regex, color):
    match = regex.match(line)
    start = match.start(color[0])
    end = match.end(color[0])
    return (line[:start]
            + color[1] + line[start:end]
            + ansi_colors.FOREGROUND_RESET + ansi_colors.STYLE_RESET_ALL
            + line[end:])

  def ApplyColor(line):
    if warning_re.match(line):
      line = Colorize(line, warning_re, warning_color)
    elif error_re.match(line):
      line = Colorize(line, error_re, error_color)
    elif marker_re.match(line):
      line = Colorize(line, marker_re, marker_color)
    return line

  return '\n'.join(map(ApplyColor, output.split('\n')))

def DoJavac(
    bootclasspath, classpath, classes_dir, chromium_code,
    javac_bin, java_version, java_files):
  """Runs javac.

  Builds |java_files| with the provided |classpath| and puts the generated
  .class files into |classes_dir|. If |chromium_code| is true, extra lint
  checking will be enabled.
  """

  jar_inputs = []
  for path in classpath:
    if os.path.exists(path + '.TOC'):
      jar_inputs.append(path + '.TOC')
    else:
      jar_inputs.append(path)

  javac_args = [
      '-g',
      # Chromium only allows UTF8 source files.  Being explicit avoids
      # javac pulling a default encoding from the user's environment.
      '-encoding', 'UTF-8',
      '-classpath', ':'.join(classpath),
      '-d', classes_dir,
      # TODO(camsim99): Fix deprecations:
      # https://github.com/flutter/flutter/issues/98602.
      '-Xlint:-deprecation']

  if bootclasspath:
    javac_args.extend([
        '-bootclasspath', ':'.join(bootclasspath),
        '-source', java_version,
        '-target', java_version,
        ])

  if chromium_code:
    javac_args.extend(['-Xlint:unchecked'])
  else:
    # XDignore.symbol.file makes javac compile against rt.jar instead of
    # ct.sym. This means that using a java internal package/class will not
    # trigger a compile warning or error.
    javac_args.extend(['-XDignore.symbol.file'])

  javac_cmd = [javac_bin] + javac_args + java_files

  def Compile():
    build_utils.CheckOutput(
        javac_cmd,
        print_stdout=chromium_code,
        stderr_filter=ColorJavacOutput)

  record_path = os.path.join(classes_dir, 'javac.md5.stamp')
  md5_check.CallAndRecordIfStale(
      Compile,
      record_path=record_path,
      input_paths=java_files + jar_inputs,
      input_strings=javac_cmd)


_MAX_MANIFEST_LINE_LEN = 72


def CreateManifest(manifest_path, classpath, main_class=None,
                   manifest_entries=None):
  """Creates a manifest file with the given parameters.

  This generates a manifest file that compiles with the spec found at
  http://docs.oracle.com/javase/7/docs/technotes/guides/jar/jar.html#JAR_Manifest

  Args:
    manifest_path: The path to the manifest file that should be created.
    classpath: The JAR files that should be listed on the manifest file's
      classpath.
    main_class: If present, the class containing the main() function.
    manifest_entries: If present, a list of (key, value) pairs to add to
      the manifest.

  """
  output = ['Manifest-Version: 1.0']
  if main_class:
    output.append('Main-Class: %s' % main_class)
  if manifest_entries:
    for k, v in manifest_entries:
      output.append('%s: %s' % (k, v))
  if classpath:
    sanitized_paths = []
    for path in classpath:
      sanitized_paths.append(os.path.basename(path.strip('"')))
    output.append('Class-Path: %s' % ' '.join(sanitized_paths))
  output.append('Created-By: ')
  output.append('')

  wrapper = textwrap.TextWrapper(break_long_words=True,
                                 drop_whitespace=False,
                                 subsequent_indent=' ',
                                 width=_MAX_MANIFEST_LINE_LEN - 2)
  output = '\r\n'.join(w for l in output for w in wrapper.wrap(l))

  with open(manifest_path, 'w') as f:
    f.write(output)


def main(argv):
  argv = build_utils.ExpandFileArgs(argv)

  parser = optparse.OptionParser()
  build_utils.AddDepfileOption(parser)

  parser.add_option(
      '--src-gendirs',
      help='Directories containing generated java files.')
  parser.add_option(
      '--java-srcjars',
      action='append',
      default=[],
      help='List of srcjars to include in compilation.')
  parser.add_option(
      '--bootclasspath',
      action='append',
      default=[],
      help='Boot classpath for javac. If this is specified multiple times, '
      'they will all be appended to construct the classpath.')
  parser.add_option(
      '--classpath',
      action='append',
      help='Classpath for javac. If this is specified multiple times, they '
      'will all be appended to construct the classpath.')
  parser.add_option(
      '--javac-includes',
      help='A list of file patterns. If provided, only java files that match'
      'one of the patterns will be compiled.')
  parser.add_option(
      '--jar-excluded-classes',
      default='',
      help='List of .class file patterns to exclude from the jar.')

  parser.add_option(
      '--chromium-code',
      type='int',
      help='Whether code being compiled should be built with stricter '
      'warnings for chromium code.')

  parser.add_option(
      '--javac-bin',
      default='javac',
      help='The javac binary. If empty, the javac binary is resolved from PATH.')

  parser.add_option(
      '--jar-bin',
      default='jar',
      help='The jar binary. If empty, the jar binary is resolved from PATH.')

  parser.add_option(
      '--java-version',
      default='1.8',
      help='The source and target versions passed to javac.')

  parser.add_option(
      '--classes-dir',
      help='Directory for compiled .class files.')
  parser.add_option('--jar-path', help='Jar output path.')
  parser.add_option('--jar-source-path', help='Source jar output path.')
  parser.add_option(
      '--jar-source-base-dir',
      help='Base directory for the source files included in the output source jar.')
  parser.add_option(
      '--main-class',
      help='The class containing the main method.')
  parser.add_option(
      '--manifest-entry',
      action='append',
      help='Key:value pairs to add to the .jar manifest.')
  parser.add_option(
      '--additional-jar-files',
      dest='additional_jar_files',
      action='append',
      help='Additional files to package into jar. By default, only Java .class '
      'files are packaged into the jar.')

  parser.add_option('--stamp', help='Path to touch on success.')

  options, args = parser.parse_args(argv)

  if options.main_class and not options.jar_path:
    parser.error('--main-class requires --jar-path')

  bootclasspath = []
  for arg in options.bootclasspath:
    bootclasspath += build_utils.ParseGypList(arg)

  classpath = []
  for arg in options.classpath:
    classpath += build_utils.ParseGypList(arg)

  java_srcjars = []
  for arg in options.java_srcjars:
    java_srcjars += build_utils.ParseGypList(arg)

  java_files = args
  if options.src_gendirs:
    src_gendirs = build_utils.ParseGypList(options.src_gendirs)
    java_files += build_utils.FindInDirectories(src_gendirs, '*.java')

  additional_jar_files = []
  for arg in options.additional_jar_files or []:
    additional_jar_files += build_utils.ParseGypList(arg)

  input_files = bootclasspath + classpath + java_srcjars + java_files
  with build_utils.TempDir() as temp_dir:
    classes_dir = os.path.join(temp_dir, 'classes')
    os.makedirs(classes_dir)
    if java_srcjars:
      java_dir = os.path.join(temp_dir, 'java')
      os.makedirs(java_dir)
      for srcjar in java_srcjars:
        build_utils.ExtractAll(srcjar, path=java_dir, pattern='*.java')
      java_files += build_utils.FindInDirectory(java_dir, '*.java')

    if options.javac_includes:
      javac_includes = build_utils.ParseGypList(options.javac_includes)
      filtered_java_files = []
      for f in java_files:
        for include in javac_includes:
          if fnmatch.fnmatch(f, include):
            filtered_java_files.append(f)
            break
      java_files = filtered_java_files

    if len(java_files) != 0:
      DoJavac(
          bootclasspath,
          classpath,
          classes_dir,
          options.chromium_code,
          options.javac_bin,
          options.java_version,
          java_files)

    if options.jar_path:
      if options.main_class or options.manifest_entry:
        if options.manifest_entry:
          entries = [e.split(":") for e in options.manifest_entry]
        else:
          entries = []
        manifest_file = os.path.join(temp_dir, 'manifest')
        CreateManifest(manifest_file, classpath, options.main_class, entries)
      else:
        manifest_file = None

      if options.additional_jar_files:
        for f in additional_jar_files:
          # Also make the additional files available at the relative paths
          # matching their original locations.
          shutil.copyfile(f, os.path.join(
              classes_dir, os.path.relpath(f, options.jar_source_base_dir)))
        additional_jar_files = [os.path.relpath(
            f, options.jar_source_base_dir) for f in additional_jar_files]

      jar.JarDirectory(classes_dir,
                       build_utils.ParseGypList(options.jar_excluded_classes),
                       options.jar_path,
                       options.jar_bin,
                       manifest_file=manifest_file,
                       additional_jar_files=additional_jar_files)

      if options.jar_source_path:
        jar.Jar(java_files, options.jar_source_base_dir, options.jar_source_path, options.jar_bin)

    if options.classes_dir:
      # Delete the old classes directory. This ensures that all .class files in
      # the output are actually from the input .java files. For example, if a
      # .java file is deleted or an inner class is removed, the classes
      # directory should not contain the corresponding old .class file after
      # running this action.
      build_utils.DeleteDirectory(options.classes_dir)
      shutil.copytree(classes_dir, options.classes_dir)

  if options.depfile:
    build_utils.WriteDepfile(
        options.depfile,
        input_files + build_utils.GetPythonDependencies())

  if options.stamp:
    build_utils.Touch(options.stamp)


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
