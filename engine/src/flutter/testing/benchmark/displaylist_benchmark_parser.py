#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import argparse
import csv
import json
import sys
import matplotlib.pyplot as plt  # pylint: disable=import-error
from matplotlib.backends.backend_pdf import PdfPages as pdfp  # pylint: disable=import-error


class BenchmarkResult:  # pylint: disable=too-many-instance-attributes

  def __init__(self, name, backend, time_unit, draw_call_count):
    self.name = name
    self.series = {}
    self.series_labels = {}
    self.backend = backend
    self.large_y_values = False
    self.y_limit = 200
    self.time_unit = time_unit
    self.draw_call_count = draw_call_count
    self.optional_values = {}

  def __repr__(self):
    return 'Name: % s\nBackend: % s\nSeries: % s\nSeriesLabels: % s\n' % (
        self.name, self.backend, self.series, self.series_labels
    )

  def add_data_point(self, family, xval, yval):
    if family not in self.series:
      self.series[family] = {'x': [], 'y': []}

    self.series[family]['x'].append(xval)
    self.series[family]['y'].append(yval)

    if yval > self.y_limit:
      self.large_y_values = True

  def add_optional_value(self, name, xval, yval):
    if name not in self.optional_values:
      self.optional_values[name] = {}

    self.optional_values[name][xval] = yval

  def set_family_label(self, family, label):
    # I'm not keying the main series dict off the family label
    # just in case we get data where the two aren't a 1:1 mapping
    if family in self.series_labels:
      assert self.series_labels[family] == label
      return

    self.series_labels[family] = label

  def plot(self):
    figures = []
    figures.append(plt.figure(dpi=1200, frameon=False, figsize=(11, 8.5)))

    for family in self.series:
      plt.plot(self.series[family]['x'], self.series[family]['y'], label=self.series_labels[family])

    plt.xlabel('Benchmark Seed')
    plt.ylabel('Time (' + self.time_unit + ')')

    title = ''
    # Crop the Y axis so that we can see what's going on at the lower end
    if self.large_y_values:
      plt.ylim((0, self.y_limit))
      title = self.name + ' ' + self.backend + ' (Cropped)'
    else:
      title = self.name + ' ' + self.backend

    if self.draw_call_count != -1:
      title += '\nDraw Call Count: ' + str(int(self.draw_call_count))

    plt.title(title)

    plt.grid(which='both', axis='both')

    plt.legend(fontsize='xx-small')
    plt.plot()

    if self.large_y_values:
      # Plot again but with the full Y axis visible
      figures.append(plt.figure(dpi=1200, frameon=False, figsize=(11, 8.5)))
      for family in self.series:
        plt.plot(
            self.series[family]['x'], self.series[family]['y'], label=self.series_labels[family]
        )

      plt.xlabel('Benchmark Seed')
      plt.ylabel('Time (' + self.time_unit + ')')
      title = self.name + ' ' + self.backend + ' (Complete)'

      if self.draw_call_count != -1:
        title += '\nDraw Call Count: ' + str(int(self.draw_call_count))

      plt.title(title)

      plt.grid(which='both', axis='both')

      plt.legend(fontsize='xx-small')
      plt.plot()

    return figures

  def write_csv(self, writer):
    # For now assume that all our series have the same x values
    # this is true for now, but may differ in the future with benchmark changes
    x_values = []
    y_values = []
    for family in self.series:
      x_values = ['x'] + self.series[family]['x']
      y_values.append([self.series_labels[family]] + self.series[family]['y'])

    for name in self.optional_values:
      column = [name]
      for key in self.optional_values[name]:
        column.append(self.optional_values[name][key])
      y_values.append(column)

    writer.writerow([self.name, self.draw_call_count])
    for line, _ in enumerate(x_values):
      row = [x_values[line]]
      for series, _ in enumerate(y_values):
        row.append(y_values[series][line])
      writer.writerow(row)


def main():
  parser = argparse.ArgumentParser()

  parser.add_argument(
      'filename', action='store', help='Path to the JSON output from Google Benchmark'
  )
  parser.add_argument(
      '-o',
      '--output-pdf',
      dest='output_pdf',
      action='store',
      default='output.pdf',
      help='Filename to output the PDF of graphs to.'
  )
  parser.add_argument(
      '-c',
      '--output-csv',
      dest='output_csv',
      action='store',
      default='output.csv',
      help='Filename to output the CSV data to.'
  )

  args = parser.parse_args()
  json_data = parse_json(args.filename)
  return process_benchmark_data(json_data, args.output_pdf, args.output_csv)


def error(message):
  print(message)
  sys.exit(1)


def extrac_attributes_label(benchmark_result):
  # Possible attribute keys are:
  #  AntiAliasing
  #  HairlineStroke
  #  StrokedStyle
  #  FilledStyle
  attributes = ['AntiAliasing', 'HairlineStroke', 'StrokedStyle', 'FilledStyle']
  label = ''

  for attr in attributes:
    try:
      if benchmark_result[attr] != 0:
        label += attr + ', '
    except KeyError:
      pass

  return label[:-2]


def process_benchmark_data(benchmark_json, output_pdf, output_csv):
  benchmark_results_data = {}

  for benchmark_result in benchmark_json:
    # Skip aggregate results
    if 'aggregate_name' in benchmark_result:
      continue

    benchmark_variant = benchmark_result['name'].split('/')
    # The final split is always `real_time` and can be discarded
    benchmark_variant.remove('real_time')

    splits = len(benchmark_variant)
    # First split is always the benchmark function name
    benchmark_name = benchmark_variant[0]
    # The last split is always the seeded value into the benchmark
    benchmark_seeded_value = benchmark_variant[splits - 1]
    # The second last split is always the backend
    benchmark_backend = benchmark_variant[splits - 2]
    # Time taken (wall clock time) for benchmark to run
    benchmark_real_time = benchmark_result['real_time']
    benchmark_unit = benchmark_result['time_unit']

    benchmark_family_index = benchmark_result['family_index']

    benchmark_family_label = ''
    if splits > 3:
      for i in range(1, splits - 2):
        benchmark_family_label += benchmark_variant[i] + ', '

    benchmark_family_attributes = extrac_attributes_label(benchmark_result)

    if benchmark_family_attributes == '':
      benchmark_family_label = benchmark_family_label[:-2]
    else:
      benchmark_family_label = benchmark_family_label + benchmark_family_attributes

    if 'DrawCallCount' in benchmark_result:
      benchmark_draw_call_count = benchmark_result['DrawCallCount']
    else:
      benchmark_draw_call_count = -1

    optional_keys = ['DrawCallCount_Varies', 'VerbCount', 'PointCount', 'VertexCount', 'GlyphCount']

    if benchmark_name not in benchmark_results_data:
      benchmark_results_data[benchmark_name] = BenchmarkResult(
          benchmark_name, benchmark_backend, benchmark_unit, benchmark_draw_call_count
      )

    for key in optional_keys:
      if key in benchmark_result:
        benchmark_results_data[benchmark_name].add_optional_value(
            key, benchmark_seeded_value, benchmark_result[key]
        )

    benchmark_results_data[benchmark_name].add_data_point(
        benchmark_family_index, benchmark_seeded_value, benchmark_real_time
    )
    benchmark_results_data[benchmark_name].set_family_label(
        benchmark_family_index, benchmark_family_label
    )

  pdf = pdfp(output_pdf)

  csv_file = open(output_csv, 'w')
  csv_writer = csv.writer(csv_file)

  for benchmark in benchmark_results_data:
    figures = benchmark_results_data[benchmark].plot()
    for fig in figures:
      pdf.savefig(fig)
    benchmark_results_data[benchmark].write_csv(csv_writer)
  pdf.close()


def parse_json(filename):
  try:
    json_file = open(filename, 'r')
  except:  # pylint: disable=bare-except
    error('Unable to load file.')

  try:
    json_data = json.load(json_file)
  except JSONDecodeError:  # pylint: disable=undefined-variable
    error('Invalid JSON. Unable to parse.')

  return json_data['benchmarks']


if __name__ == '__main__':
  sys.exit(main())
