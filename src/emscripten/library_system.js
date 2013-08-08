mergeInto(LibraryManager.library, {

    browser_info: function () {
        idstr = "";
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

        return allocate(intArrayFromString(idstr.trim()), 'i8', ALLOC_STACK);
    }

});
