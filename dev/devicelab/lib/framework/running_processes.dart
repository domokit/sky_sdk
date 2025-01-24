// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:meta/meta.dart';
import 'package:process/process.dart';

@immutable
class RunningProcessInfo {
  const RunningProcessInfo(this.pid, this.commandLine, this.creationDate);

  final int pid;
  final String commandLine;
  final DateTime creationDate;

  @override
  bool operator ==(Object other) {
    return other is RunningProcessInfo &&
        other.pid == pid &&
        other.commandLine == commandLine &&
        other.creationDate == creationDate;
  }

  Future<bool> terminate({required ProcessManager processManager}) async {
    // This returns true when the signal is sent, not when the process goes away.
    // See also https://github.com/dart-lang/sdk/issues/40759 (killPid should wait for process to be terminated).
    if (Platform.isWindows) {
      // TODO(ianh): Move Windows to killPid once we can.
      //  - killPid on Windows has not-useful return code: https://github.com/dart-lang/sdk/issues/47675
      final ProcessResult result = await processManager.run(<String>[
        'taskkill.exe',
        '/pid',
        '$pid',
        '/f',
      ]);
      return result.exitCode == 0;
    }
    return processManager.killPid(pid, ProcessSignal.sigkill);
  }

  @override
  int get hashCode => Object.hash(pid, commandLine, creationDate);

  @override
  String toString() {
    return 'RunningProcesses(pid: $pid, commandLine: $commandLine, creationDate: $creationDate)';
  }
}

Future<Set<RunningProcessInfo>> getRunningProcesses({
  String? processName,
  required ProcessManager processManager,
}) {
  if (Platform.isWindows) {
    return windowsRunningProcesses(processName, processManager);
  }
  return posixRunningProcesses(processName, processManager);
}

@visibleForTesting
Future<Set<RunningProcessInfo>> windowsRunningProcesses(
  String? processName,
  ProcessManager processManager,
) async {
  // PowerShell script to get the command line arguments and create time of a process.
  // See: https://docs.microsoft.com/en-us/windows/desktop/cimwin32prov/win32-process
  final String script =
      processName != null
          ? '"Get-CimInstance Win32_Process -Filter \\"name=\'$processName\'\\" | Select-Object ProcessId,CreationDate,CommandLine | Format-Table -AutoSize | Out-String -Width 4096"'
          : '"Get-CimInstance Win32_Process | Select-Object ProcessId,CreationDate,CommandLine | Format-Table -AutoSize | Out-String -Width 4096"';
  // TODO(ianh): Unfortunately, there doesn't seem to be a good way to get
  // ProcessManager to run this.
  final ProcessResult result = await Process.run('powershell -command $script', <String>[]);
  if (result.exitCode != 0) {
    print('Could not list processes!');
    print(result.stderr);
    print(result.stdout);
    return <RunningProcessInfo>{};
  }
  return processPowershellOutput(result.stdout as String).toSet();
}

/// Parses the output of the PowerShell script from [windowsRunningProcesses].
///
/// E.g.:
/// ProcessId CreationDate          CommandLine
/// --------- ------------          -----------
///      2904 3/11/2019 11:01:54 AM "C:\Program Files\Android\Android Studio\jre\bin\java.exe" -Xmx1536M -Dfile.encoding=windows-1252 -Duser.country=US -Duser.language=en -Duser.variant -cp C:\Users\win1\.gradle\wrapper\dists\gradle-4.10.2-all\9fahxiiecdb76a5g3aw9oi8rv\gradle-4.10.2\lib\gradle-launcher-4.10.2.jar org.gradle.launcher.daemon.bootstrap.GradleDaemon 4.10.2
@visibleForTesting
Iterable<RunningProcessInfo> processPowershellOutput(String output) sync* {
  const int processIdHeaderSize = 'ProcessId'.length;
  const int creationDateHeaderStart = processIdHeaderSize + 1;
  late int creationDateHeaderEnd;
  late int commandLineHeaderStart;
  bool inTableBody = false;
  for (final String line in output.split('\n')) {
    if (line.startsWith('ProcessId')) {
      commandLineHeaderStart = line.indexOf('CommandLine');
      creationDateHeaderEnd = commandLineHeaderStart - 1;
    }
    if (line.startsWith('--------- ------------')) {
      inTableBody = true;
      continue;
    }
    if (!inTableBody || line.isEmpty) {
      continue;
    }
    if (line.length < commandLineHeaderStart) {
      continue;
    }

    // 3/11/2019 11:01:54 AM
    // 12/11/2019 11:01:54 AM
    String rawTime = line.substring(creationDateHeaderStart, creationDateHeaderEnd).trim();

    if (rawTime[1] == '/') {
      rawTime = '0$rawTime';
    }
    if (rawTime[4] == '/') {
      rawTime = '${rawTime.substring(0, 3)}0${rawTime.substring(3)}';
    }
    final String year = rawTime.substring(6, 10);
    final String month = rawTime.substring(3, 5);
    final String day = rawTime.substring(0, 2);
    String time = rawTime.substring(11, 19);
    if (time[7] == ' ') {
      time = '0$time'.trim();
    }
    if (rawTime.endsWith('PM')) {
      final int hours = int.parse(time.substring(0, 2));
      time = '${hours + 12}${time.substring(2)}';
    }

    final int pid = int.parse(line.substring(0, processIdHeaderSize).trim());
    final DateTime creationDate = DateTime.parse('$year-$month-${day}T$time');
    final String commandLine = line.substring(commandLineHeaderStart).trim();
    yield RunningProcessInfo(pid, commandLine, creationDate);
  }
}

@visibleForTesting
Future<Set<RunningProcessInfo>> posixRunningProcesses(
  String? processName,
  ProcessManager processManager,
) async {
  final ProcessResult result = await processManager.run(<String>[
    'ps',
    '-eo',
    'lstart,pid,command',
  ]);
  if (result.exitCode != 0) {
    print('Could not list processes!');
    print(result.stderr);
    print(result.stdout);
    return <RunningProcessInfo>{};
  }
  return processPsOutput(result.stdout as String, processName).toSet();
}

/// Parses the output of the command in [posixRunningProcesses].
///
/// E.g.:
///
/// STARTED                        PID COMMAND
/// Sat Mar  9 20:12:47 2019         1 /sbin/launchd
/// Sat Mar  9 20:13:00 2019        49 /usr/sbin/syslogd
@visibleForTesting
Iterable<RunningProcessInfo> processPsOutput(String output, String? processName) sync* {
  bool inTableBody = false;
  for (String line in output.split('\n')) {
    if (line.trim().startsWith('STARTED')) {
      inTableBody = true;
      continue;
    }
    if (!inTableBody || line.isEmpty) {
      continue;
    }

    if (processName != null && !line.contains(processName)) {
      continue;
    }
    if (line.length < 25) {
      continue;
    }

    // 'Sat Feb 16 02:29:55 2019'
    // 'Sat Mar  9 20:12:47 2019'
    const Map<String, String> months = <String, String>{
      'Jan': '01',
      'Feb': '02',
      'Mar': '03',
      'Apr': '04',
      'May': '05',
      'Jun': '06',
      'Jul': '07',
      'Aug': '08',
      'Sep': '09',
      'Oct': '10',
      'Nov': '11',
      'Dec': '12',
    };
    final String rawTime = line.substring(0, 24);

    final String year = rawTime.substring(20, 24);
    final String month = months[rawTime.substring(4, 7)]!;
    final String day = rawTime.substring(8, 10).replaceFirst(' ', '0');
    final String time = rawTime.substring(11, 19);

    final DateTime creationDate = DateTime.parse('$year-$month-${day}T$time');
    line = line.substring(24).trim();
    final int nextSpace = line.indexOf(' ');
    final int pid = int.parse(line.substring(0, nextSpace));
    final String commandLine = line.substring(nextSpace + 1);
    yield RunningProcessInfo(pid, commandLine, creationDate);
  }
}
