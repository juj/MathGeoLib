#!/usr/bin/env python

import subprocess
import sys
import re

def main():
  cmdline = sys.argv[1:]
  process = subprocess.Popen(cmdline, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  (stdout, stderr) = process.communicate()
  lines = stdout.split('\n')
  file = None
  for line in lines:
    matchObj = re.search(r'\*\*\*\*\* BEGIN FILE (.*) \*\*\*\*\*', line.strip(), re.M|re.I)
    if matchObj:
      file = open(matchObj.group(1).strip(), 'w')
    else:
      matchObj = re.search(r'\*\*\*\*\* END FILE (.*) \*\*\*\*\*', line.strip(), re.M|re.I)
      if matchObj:
        file.close()
        file = None
      elif file:
        file.write(line)
      else:
        print >> sys.stdout, line

if __name__ == '__main__':
  sys.exit(main())
