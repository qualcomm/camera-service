################################################################################
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
################################################################################

from __future__ import print_function

import argparse
import glob
import json
import multiprocessing
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import traceback
import yaml

is_py2 = sys.version[0] == '2'

if is_py2:
    import Queue as queue
else:
    import queue as queue

def find_exp_compile_cmds(path):
  res = './'
  while not os.path.isfile(os.path.join(res, path)):
    if os.path.realpath(res) == '/':
      sys.exit(1)
    res += '../'
  return os.path.realpath(res)


def abs_path(f, directory):
  if os.path.isabs(f):
    return f
  return os.path.normpath(os.path.join(directory, f))

def get_cmd_args(f, clang_tidy_bin, checks, tmpdir, build_path,
                        header_filter, extra_arg, extra_arg_before, quiet,
                        config):
  start = [clang_tidy_bin]
  if header_filter is not None:
    start.append('-header-filter=' + header_filter)
  else:
    start.append('-header-filter=^' + build_path + '/.*')
  if checks:
    start.append('-checks=' + checks)
  if tmpdir is not None:
    start.append('-export-fixes')
    (handle, name) = tempfile.mkstemp(suffix='.yaml', dir=tmpdir)
    os.close(handle)
    start.append(name)
  for arg in extra_arg:
      start.append('-extra-arg=%s' % arg)
  for arg in extra_arg_before:
      start.append('-extra-arg-before=%s' % arg)
  start.append('-p=' + build_path)
  if quiet:
      start.append('-quiet')
  if config:
      start.append('-config=' + config)
  start.append(f)
  return start


def run(ar, tmpdir, build_path, queue, failed_files):
  while True:
    name = queue.get()
    call = get_cmd_args(name, ar.clang_tidy_binary, ar.checks,
                                     tmpdir, build_path, ar.header_filter,
                                     ar.extra_arg, ar.extra_arg_before,
                                     ar.quiet, ar.config)
    sys.stdout.write(' '.join(call) + '\n')
    return_code = subprocess.call(call)
    if return_code != 0:
      failed_files.append(name)
    queue.task_done()


def main():
  parser = argparse.ArgumentParser(description='')
  parser.add_argument('-clang-tidy-binary', metavar='PATH',
                      default='/usr/bin/clang-tidy-3.5')
  parser.add_argument('-clang-apply-replacements-binary', metavar='PATH',
                      default='clang-apply-replacements')
  parser.add_argument('-checks', default=None)
  parser.add_argument('-config', default=None)
  parser.add_argument('-header-filter', default=None)
  parser.add_argument('-j', type=int, default=0)
  parser.add_argument('files', nargs='*', default=['.*'])
  parser.add_argument('-p', dest='build_path')
  parser.add_argument('-extra-arg', dest='extra_arg',
                      action='append', default=[])
  parser.add_argument('-extra-arg-before', dest='extra_arg_before',
                      action='append', default=[])
  parser.add_argument('-quiet', action='store_true')
  ar = parser.parse_args()

  cmd_args = 'compile_commands.json'

  if ar.build_path is not None:
    build_path = ar.build_path
  else:
    build_path = find_exp_compile_cmds(cmd_args)

  try:
    call = [ar.clang_tidy_binary, '-list-checks']
    call.append('-p=' + build_path)
    if ar.checks:
      call.append('-checks=' + ar.checks)
    print(call)
    call.append('-')
    subprocess.check_call(call)
  except:
    print("Error: run clang-tidy.", file=sys.stderr)
    sys.exit(1)

  db = json.load(open(os.path.join(build_path, cmd_args)))
  files = [abs_path(entry['file'], entry['directory'])
           for entry in db]

  max_task = ar.j
  if max_task == 0:
    max_task = multiprocessing.cpu_count()

  tmpdir = None

  file_name_re = re.compile('|'.join(ar.files))

  return_code = 0
  try:
    task_queue = queue.Queue(max_task)
    failed = []
    for _ in range(max_task):
      t = threading.Thread(target=run,
                           args=(ar, tmpdir, build_path, task_queue, failed))
      t.daemon = True
      t.start()

    for name in files:
      if file_name_re.search(name):
        task_queue.put(name)

    task_queue.join()
    if len(failed):
      return_code = 1

  except KeyboardInterrupt:
    print('\nCtrl-C!')
    if tmpdir:
      shutil.rmtree(tmpdir)
    os.kill(0, 9)

  if tmpdir:
    shutil.rmtree(tmpdir)
  sys.exit(return_code)

if __name__ == '__main__':
  main()
