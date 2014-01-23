mergeInto(LibraryManager.library, {
    browser_info: function () {
        idstr = "";
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
            idstr = "SpiderMonkey";
            var asmJsEnabled = isAsmJSCompilationAvailable();
            idstr += ' asm.js ' + (asmJsEnabled ? 'enabled' : 'disabled');
          } else {
            idstr = "Unknown browser environment";
          }
        }
        return allocate(intArrayFromString(idstr.trim()), 'i8', ALLOC_STACK);
    }

});
