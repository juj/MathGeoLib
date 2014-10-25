import re, sys, os, fnmatch, json, subprocess

def find_files(directory, pattern):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield filename

symbols = {}

const_swap_regex = re.compile('\\b([\\w:]*)\\s+const')

def swapConsts(str):
  while True:
    ret = const_swap_regex.search(str)
    if ret:
      str = str[0:ret.start(0)] + 'const ' + ret.group(1) + str[ret.end(0):]
    else:
      return str

def canonicalizeSymbolName(symbol):
  symbol = symbol.replace('public: ', '')
  symbol = symbol.replace('protected: ', '')
  symbol = symbol.replace('private: ', '')
  symbol = symbol.replace('__cdecl ', '')
  symbol = symbol.replace('__ptr64 ', '')
  symbol = symbol.replace(' __ptr64', '')
  symbol = symbol.replace('__ptr64', '')
  symbol = symbol.replace('(void)', '()')
  symbol = symbol.replace('static ', '')
  symbol = symbol.replace('union ', '')
  symbol = symbol.replace('struct ', '')
  symbol = symbol.replace('enum ', '')
  symbol = symbol.replace('class ', '')
  symbol = symbol.replace(')const', ') const')

  # Hack: kill "math::" namespace since doxygen parsing doesn't output that!
  symbol = symbol.replace('math::', '')

  symbol = swapConsts(symbol)
  symbol = symbol.replace(' ', '')
  return symbol

def undecorateVSSymbol(symbol):
  undname = 'C:/Program Files (x86)/Microsoft Visual Studio 10.0/VC/bin/undname.exe'
  output = subprocess.check_output([undname, symbol])
  idx = output.find('is :- "')
  if idx == -1:
    print 'Parsing following output failed!'
    print output
    return symbol # Failed! Return original symbol name
  undecorated = output[idx+len('is :- "'):].strip()[:-1]
  return canonicalizeSymbolName(undecorated)

def extractCode(filename):
  code = open(filename, 'r').read()
#  block_re = re.compile('^(.*?) PROC ; (.*?)$', re.DOTALL | re.MULTILINE)
  startproc_re = re.compile('(.*)\s+PROC\s+;\s+(.*)')
#  startproc_re = re.compile('PUBLIC(.*) ; (.*)')
  endproc_re = re.compile('(.*)\s+ENDP\s+;\s+(.*)')
  pos = 0
  while pos < len(code):
    ret = startproc_re.search(code, pos)
    if ret:
      pos = ret.end(0)
      codeStart = ret.end(0)
      decoratedFuncName = ret.group(1).strip()
#      funcName = ret.group(2).strip()
      endsearch = endproc_re.search(code, pos)
      if endsearch:
#        print decoratedFuncName + ' vs ' + endsearch.group(1).strip()
        if endsearch.group(1).strip() == decoratedFuncName:
          pos = endsearch.end(0)
          funcName = undecorateVSSymbol(decoratedFuncName)
#          print decoratedFuncName + ': ' + funcName
          if not '`' in funcName and not funcName.startswith('std::') and not 'dynamic initializer for' in funcName:
            codeStr = code[codeStart:endsearch.start(0)]
            codeStrLines = codeStr.split('\n')
#            print str(codeStart) + ' ' + str(endsearch.start(0))
#            print ret.group(1) + ', ' + funcName +': ' + str(len(codeStrLines)) + ' lines.' # + ' ' + ret.group(2)
            print funcName +': ' + str(len(codeStrLines)) + ' lines.' # + ' ' + ret.group(2)
#            print str(codeStr)
            symbols[funcName] = codeStr
#            sys.exit(0)
    else:
      break

def findGccPrevFuncName(code, pos):
  while pos >= 0:
#    pos = code.rfind('****', 0, pos)
    pos = code.rfind('.globl', 0, pos)
    if pos == -1:
      return ''
    funcName = code[pos+len('.globl'):code.find('\n', pos)].strip()
    if len(funcName) > 1:
      if CLANG_FORMAT:
        undecorated = subprocess.check_output(['c++filt', '_' + funcName]).strip()
      else:
        undecorated = subprocess.check_output(['c++filt', funcName]).strip()
      undecorated = canonicalizeSymbolName(undecorated)
      return undecorated

#gcc_label_re = re.compile('\\s*\\d+\\s+([A-Z0-9]+):')
#gcc_label2_re = re.compile('\\s*\\d+\\s+\.(.*)')
gcc_label_re = re.compile('([A-Z0-9]+):')
gcc_label2_re = re.compile('\\s*\.(.*)')

def extractGccCode(filename):
#  code = subprocess.check_output(['as', '-alnd', filename])
  code = open(filename).read()
#  code = subprocess.check_output(['as', '-alhnd', filename])
#  open(filename+'.lst', 'w').write(code)
  pos = 0;
  while pos < len(code):
    pos = code.find('.cfi_startproc', pos)
    if pos == -1:
      break
    startpos = pos+len('.cfi_startproc')
    pos = code.find('.cfi_endproc', pos)
    if pos == -1:
      break
    endpos = pos
    funcName = findGccPrevFuncName(code, startpos)

    codeStr = code[startpos:endpos].strip()
    codeStr.decode('ascii', 'ignore').encode('ascii', 'ignore')
    codeStrLines = codeStr.split('\n')
    
    goodLines = []
    for line in codeStrLines:
      m = gcc_label_re.match(line)
      if m and (m.group(1).startswith('LVL') or m.group(1).startswith('LB')):
        #print "Filtered " + m.group(0)
        pass
      else:
        m = gcc_label2_re.match(line)
        if m:
          #print "Filtered " + m.group(0)
          pass
        else:
          goodLines += [line]
    funcName = funcName.decode('ascii', 'ignore').encode('ascii', 'ignore')
    symbols[funcName] = '\n'.join(goodLines)
    print funcName +': ' + str(len(goodLines)) + ' lines.' # + ' ' + ret.group(2)

platformId = sys.argv[1]

def writeJsonCodeFile(outfile):
  codefile = {}
  codefile['platform'] = platformId
  codefile['symbols'] = symbols

  data = json.dumps(codefile, sort_keys=True, indent=2, separators=(',', ': '))
  open(outfile, 'wb').write(data)

CLANG_FORMAT = '--clang' in sys.argv

GCC_FORMAT = '--gcc' in sys.argv or CLANG_FORMAT

if GCC_FORMAT:
  asmFiles = find_files(sys.argv[2], '*.obj')
else:
  asmFiles = find_files(sys.argv[2], '*.asm')

for file in asmFiles:
  if GCC_FORMAT:
    extractGccCode(file)
#    break
  else:
    extractCode(file)
#  break
print 'Finished. Found code for ' + str(len(symbols)) + ' functions.'

writeJsonCodeFile('code_'+platformId+'.json')
