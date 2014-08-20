#import <UIKit/UIDevice.h>

/// Returns a newly allocated string that represents the system information of this iOS device.
/// @note When you are done with the string, you must call free() on it.
char *IosSystemInformation()
{
	NSString *info = [NSString stringWithFormat: @"%@ %@ %@ %@", [UIDevice currentDevice].name, [UIDevice currentDevice].model, [UIDevice currentDevice].systemName, [UIDevice currentDevice].systemVersion];
	return strdup([info UTF8String]);
}
