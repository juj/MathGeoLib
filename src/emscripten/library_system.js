mergeInto(LibraryManager.library, {
    browser_info: function () {
        var idstr = "";
        if (typeof navigator !== "undefined") {
          if (navigator.vendor && navigator.vendor.length > 0)
              idstr += navigator.vendor + " ";
          if (navigator.platform && navigator.platform.length > 0)
              idstr += navigator.platform + " ";
          if (navigator.cpuClass && navigator.cpuClass.length > 0)
              idstr += navigator.cpuClass + " ";
          if (navigator.appName && navigator.appName.length > 0)
              idstr += navigator.appName + " ";
          if (navigator.userAgent && navigator.userAgent.length > 0)
              idstr += navigator.userAgent + " ";
          if (navigator.systemLanguage && navigator.systemLanguage.length > 0)
              idstr += navigator.systemLanguage + " ";
          else if (navigator.userLanguage && navigator.userLanguage.length > 0)
              idstr += navigator.userLanguage + " ";
          else if (navigator.language && navigator.language.length > 0)
              idstr += navigator.language + " ";
        } else {
          if (typeof module !== 'undefined' && module.exports && typeof process !== 'undefined' && typeof process.versions !== 'undefined') {
            idstr = 'Node.js ' + process.versions.node + ' v8 version ' + process.versions.v8;
          } else if (typeof isAsmJSCompilationAvailable === 'function' && isAsmJSCompilationAvailable.toString().indexOf('[native code]') > 0) {
            // Workaround a SpiderMonkey shell issue - the commands build(); and help(); do print out build information, but
            // they short-circuit to printing that information to the system console, and don't give it back as a JS string.
            // Therefore execute the JS shell via command line, and fetch the required info via a file.
            // This requires that the SPIDERMONKEY environment variable is present and identifies the currently running SpiderMonkey VM.
            try {
              if (environment['SPIDERMONKEY']) {
                system(environment['SPIDERMONKEY'] + ' --execute="build();help();">spidermonkey_version.txt');
                var version = read('spidermonkey_version.txt').split('\n').slice(0,2);
                var asmJsEnabled = isAsmJSCompilationAvailable();
                var debugStr = (typeof debug === 'function' && debug.toString().indexOf('[native code]') > 0) ? "DEBUG" : "RELEASE";
                idstr = "SpiderMonkey " + version[1] + ' ' + debugStr + ', asm.js ' + (asmJsEnabled ? 'enabled, ' : 'disabled, ') + version[0];
              } else {
                idstr = "SpiderMonkey";
              }
            } catch(e) {
              idstr = "SpiderMonkey";
            }
          } else {
            idstr = "Unknown browser environment";
          }
        }
        return allocate(intArrayFromString(idstr.trim()), 'i8', ALLOC_STACK);
    }

});
