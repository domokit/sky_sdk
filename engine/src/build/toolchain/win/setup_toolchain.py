# Copyright 2013 The Chromium Authors
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
# Copies the given "win tool" (which the toolchain uses to wrap compiler
# invocations) and the environment blocks for the 32-bit and 64-bit builds on
# Windows to the build directory.
#
# The arguments are the visual studio install location and the location of the
# win tool. The script assumes that the root build directory is the current dir
# and the files will be written to the current directory.


import errno
import json
import os
import re
import subprocess
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
import gn_helpers

SCRIPT_DIR = os.path.dirname(__file__)
SDK_VERSION = '10.0.22621.0'


def _ExtractImportantEnvironment(output_of_set):
  """Extracts environment variables required for the toolchain to run from
  a textual dump output by the cmd.exe 'set' command."""
  envvars_to_save = (
      'cipd_cache_dir', # needed by vpython
      'homedrive', # needed by vpython
      'homepath', # needed by vpython
      'goma_.*', # TODO(scottmg): This is ugly, but needed for goma.
      'include',
      'lib',
      'libpath',
      'luci_context', # needed by vpython
      'path',
      'pathext',
      'systemroot',
      'temp',
      'tmp',
      'userprofile', # needed by vpython
      'vpython_virtualenv_root' # needed by vpython
      )
  env = {}
  # This occasionally happens and leads to misleading SYSTEMROOT error messages
  # if not caught here.
  if output_of_set.count('=') == 0:
    raise Exception('Invalid output_of_set. Value is:\n%s' % output_of_set)
  for line in output_of_set.splitlines():
    for envvar in envvars_to_save:
      if re.match(envvar + '=', line.lower()):
        var, setting = line.split('=', 1)
        if envvar == 'path':
          # Our own rules and actions in Chromium rely on python being in the
          # path. Add the path to this python here so that if it's not in the
          # path when ninja is run later, python will still be found.
          setting = os.path.dirname(sys.executable) + os.pathsep + setting
        if envvar in ['include', 'lib']:
          # Make sure that the include and lib paths point to directories that
          # exist. This ensures a (relatively) clear error message if the
          # required SDK is not installed.
          for part in setting.split(';'):
            if not os.path.exists(part) and len(part) != 0:
              raise Exception(
                  'Path "%s" from environment variable "%s" does not exist. '
                  'Make sure the necessary SDK is installed.' % (part, envvar))
        env[var.upper()] = setting
        break
  if sys.platform in ('win32', 'cygwin'):
    for required in ('SYSTEMROOT', 'TEMP', 'TMP'):
      if required not in env:
        raise Exception('Environment variable "%s" '
                        'required to be set to valid path' % required)
  return env


def _DetectVisualStudioPath():
  """Return path to the installed Visual Studio.
  """

  # Use the code in build/vs_toolchain.py to avoid duplicating code.
  chromium_dir = os.path.abspath(os.path.join(SCRIPT_DIR, '..', '..', '..'))
  sys.path.append(os.path.join(chromium_dir, 'build'))
  import vs_toolchain
  return vs_toolchain.DetectVisualStudioPath()


def _LoadEnvFromBat(args):
  """Given a bat command, runs it and returns env vars set by it."""
  args = args[:]
  args.extend(('&&', 'set'))
  popen = subprocess.Popen(
      args, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  variables, _ = popen.communicate()
  if popen.returncode != 0:
    raise Exception('"%s" failed with error %d' % (args, popen.returncode))
  return variables.decode(errors='ignore')


def _LoadToolchainEnv(cpu, toolchain_root, sdk_dir, target_store):
  """Returns a dictionary with environment variables that must be set while
  running binaries from the toolchain (e.g. INCLUDE and PATH for cl.exe)."""
  # Check if we are running in the SDK command line environment and use
  # the setup script from the SDK if so. |cpu| should be either
  # 'x86' or 'x64' or 'arm' or 'arm64'.
  assert cpu in ('x86', 'x64', 'arm', 'arm64')
  if bool(int(os.environ.get('DEPOT_TOOLS_WIN_TOOLCHAIN', 1))) and sdk_dir:
    # Load environment from json file.
    env = os.path.normpath(os.path.join(sdk_dir, 'bin/SetEnv.%s.json' % cpu))
    env = json.load(open(env))['env']
    if env['VSINSTALLDIR'] == [["..", "..\\"]]:
      # Old-style paths were relative to the win_sdk\bin directory.
      json_relative_dir = os.path.join(sdk_dir, 'bin')
    else:
      # New-style paths are relative to the toolchain directory.
      json_relative_dir = toolchain_root
    for k in env:
      entries = [os.path.join(*([json_relative_dir] + e)) for e in env[k]]
      # clang-cl wants INCLUDE to be ;-separated even on non-Windows,
      # lld-link wants LIB to be ;-separated even on non-Windows.  Path gets :.
      # The separator for INCLUDE here must match the one used in main() below.
      sep = os.pathsep if k == 'PATH' else ';'
      env[k] = sep.join(entries)
    # PATH is a bit of a special case, it's in addition to the current PATH.
    env['PATH'] = env['PATH'] + os.pathsep + os.environ['PATH']
    # Augment with the current env to pick up TEMP and friends.
    for k in os.environ:
      if k not in env:
        env[k] = os.environ[k]

    varlines = []
    for k in sorted(env.keys()):
      varlines.append('%s=%s' % (str(k), str(env[k])))
    variables = '\n'.join(varlines)

    # Check that the json file contained the same environment as the .cmd file.
    if sys.platform in ('win32', 'cygwin'):
      script = os.path.normpath(os.path.join(sdk_dir, 'Bin/SetEnv.cmd'))
      arg = '/' + cpu
      json_env = _ExtractImportantEnvironment(variables)
      cmd_env = _ExtractImportantEnvironment(_LoadEnvFromBat([script, arg]))
      assert _LowercaseDict(json_env) == _LowercaseDict(cmd_env)
  else:
    if 'GYP_MSVS_OVERRIDE_PATH' not in os.environ:
      os.environ['GYP_MSVS_OVERRIDE_PATH'] = _DetectVisualStudioPath()
    # We only support x64-hosted tools.
    script_path = os.path.normpath(os.path.join(
                                       os.environ['GYP_MSVS_OVERRIDE_PATH'],
                                       'VC/vcvarsall.bat'))
    if not os.path.exists(script_path):
      # vcvarsall.bat for VS 2017 fails if run after running vcvarsall.bat from
      # VS 2013 or VS 2015. Fix this by clearing the vsinstalldir environment
      # variable. Since vcvarsall.bat appends to the INCLUDE, LIB, and LIBPATH
      # environment variables we need to clear those to avoid getting double
      # entries when vcvarsall.bat has been run before gn gen. vcvarsall.bat
      # also adds to PATH, but there is no clean way of clearing that and it
      # doesn't seem to cause problems.
      if 'VSINSTALLDIR' in os.environ:
        del os.environ['VSINSTALLDIR']
        if 'INCLUDE' in os.environ:
          del os.environ['INCLUDE']
        if 'LIB' in os.environ:
          del os.environ['LIB']
        if 'LIBPATH' in os.environ:
          del os.environ['LIBPATH']
      other_path = os.path.normpath(os.path.join(
                                        os.environ['GYP_MSVS_OVERRIDE_PATH'],
                                        'VC/Auxiliary/Build/vcvarsall.bat'))
      if not os.path.exists(other_path):
        raise Exception('%s is missing - make sure VC++ tools are installed.' %
                        script_path)
      script_path = other_path
    cpu_arg = "amd64"
    if (cpu != 'x64'):
      # x64 is default target CPU thus any other CPU requires a target set
      cpu_arg += '_' + cpu
    args = [script_path, cpu_arg, ]
    # Store target must come before any SDK version declaration
    if (target_store):
      args.append('store')
    # Explicitly specifying the SDK version to build with to avoid accidentally
    # building with a new and untested SDK. This should stay in sync with the
    # packaged toolchain in build/vs_toolchain.py.
    args.append(SDK_VERSION)
    variables = _LoadEnvFromBat(args)
  return _ExtractImportantEnvironment(variables)


def _FormatAsEnvironmentBlock(envvar_dict):
  """Format as an 'environment block' directly suitable for CreateProcess.
  Briefly this is a list of key=value\0, terminated by an additional \0. See
  CreateProcess documentation for more details."""
  block = ''
  nul = '\0'
  for key, value in envvar_dict.items():
    block += key + '=' + value + nul
  block += nul
  return block


def _LowercaseDict(d):
  """Returns a copy of `d` with both key and values lowercased.

  Args:
    d: dict to lowercase (e.g. {'A': 'BcD'}).

  Returns:
    A dict with both keys and values lowercased (e.g.: {'a': 'bcd'}).
  """
  return {k.lower(): d[k].lower() for k in d}


def FindFileInEnvList(env, env_name, separator, file_name, optional=False):
  parts = env[env_name].split(separator)
  for path in parts:
    if os.path.exists(os.path.join(path, file_name)):
      return os.path.realpath(path)
  assert optional, "%s is not found in %s:\n%s\nCheck if it is installed." % (
      file_name, env_name, '\n'.join(parts))
  return ''


def main():
  if len(sys.argv) != 7:
    print('Usage setup_toolchain.py '
          '<visual studio path> <win sdk path> '
          '<runtime dirs> <target_os> <target_cpu> '
          '<environment block name|none>')
    sys.exit(2)
  # toolchain_root and win_sdk_path are only read if the hermetic Windows
  # toolchain is set, that is if DEPOT_TOOLS_WIN_TOOLCHAIN is not set to 0.
  # With the hermetic Windows toolchain, the visual studio path in argv[1]
  # is the root of the Windows toolchain directory.
  toolchain_root = sys.argv[1]
  win_sdk_path = sys.argv[2]

  runtime_dirs = sys.argv[3]
  target_os = sys.argv[4]
  target_cpu = sys.argv[5]
  environment_block_name = sys.argv[6]
  if (environment_block_name == 'none'):
    environment_block_name = ''

  if (target_os == 'winuwp'):
    target_store = True
  else:
    target_store = False

  cpus = ('x86', 'x64', 'arm', 'arm64')
  assert target_cpu in cpus
  vc_bin_dir = ''
  include = ''
  lib = ''

  # TODO(scottmg|goma): Do we need an equivalent of
  # ninja_use_custom_environment_files?

  def relflag(s):  # Make s relative to builddir when cwd and sdk on same drive.
    try:
      return os.path.relpath(s).replace('\\', '/')
    except ValueError:
      return s

  def q(s):  # Quote s if it contains spaces or other weird characters.
    return s if re.match(r'^[a-zA-Z0-9._/\\:-]*$', s) else '"' + s + '"'

  for cpu in cpus:
    if cpu == target_cpu:
      # Extract environment variables for subprocesses.
      env = _LoadToolchainEnv(cpu, toolchain_root, win_sdk_path, target_store)
      env['PATH'] = runtime_dirs + os.pathsep + env['PATH']

      vc_bin_dir = FindFileInEnvList(env, 'PATH', os.pathsep, 'cl.exe')

      # The separator for INCLUDE here must match the one used in
      # _LoadToolchainEnv() above.
      include = [p.replace('"', r'\"') for p in env['INCLUDE'].split(';') if p]
      include = list(map(relflag, include))

      lib = [p.replace('"', r'\"') for p in env['LIB'].split(';') if p]
      lib = list(map(relflag, lib))

      include_I = ['/I' + i for i in include]
      include_imsvc = ['-imsvc' + i for i in include]
      libpath_flags = ['-libpath:' + i for i in lib]

      if (environment_block_name != ''):
        env_block = _FormatAsEnvironmentBlock(env)
        with open(environment_block_name, 'w', encoding='utf8') as f:
          f.write(env_block)

  def ListToArgString(x):
    return gn_helpers.ToGNString(' '.join(q(i) for i in x))

  def ListToArgList(x):
    return f'[{", ".join(gn_helpers.ToGNString(i) for i in x)}]'

  print('vc_bin_dir = ' + gn_helpers.ToGNString(vc_bin_dir))
  assert include_I
  print(f'include_flags_I = {ListToArgString(include_I)}')
  print(f'include_flags_I_list = {ListToArgList(include_I)}')
  assert include_imsvc
  if bool(int(os.environ.get('DEPOT_TOOLS_WIN_TOOLCHAIN', 1))) and win_sdk_path:
    flags = ['/winsysroot' + relflag(toolchain_root)]
    print(f'include_flags_imsvc = {ListToArgString(flags)}')
    print(f'include_flags_imsvc_list = {ListToArgList(flags)}')
  else:
    print(f'include_flags_imsvc = {ListToArgString(include_imsvc)}')
    print(f'include_flags_imsvc_list = {ListToArgList(include_imsvc)}')
  print('paths = ' + gn_helpers.ToGNString(env['PATH']))
  assert libpath_flags
  print(f'libpath_flags = {ListToArgString(libpath_flags)}')
  print(f'libpath_flags_list = {ListToArgList(libpath_flags)}')
  if bool(int(os.environ.get('DEPOT_TOOLS_WIN_TOOLCHAIN', 1))) and win_sdk_path:
    flags = ['/winsysroot:' + relflag(toolchain_root)]
    print(f'libpath_lldlink_flags = {ListToArgString(flags)}')
    print(f'libpath_lldlink_flags_list = {ListToArgList(flags)}')
  else:
    print(f'libpath_lldlink_flags = {ListToArgString(libpath_flags)}')
    print(f'libpath_lldlink_flags_list = {ListToArgList(libpath_flags)}')


if __name__ == '__main__':
  main()
