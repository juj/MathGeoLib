#include <string>

std::string GetOSDisplayString();

unsigned long long GetTotalSystemPhysicalMemory();

/// @return A readable string of the processor as given by cpuid, like "AMD Turion(tm) X2 Dual-Core Mobile RM-74".
std::string GetProcessorBrandName();

/// @return "GenuineIntel" or "AuthenticAMD" or something similar.
std::string GetProcessorCPUIDString();

/// @return Technical information about the processor.
std::string GetProcessorExtendedCPUIDInfo();

///\todo This doesn't work on Core i3/i5/i7, so leave it out.
//int GetNumberOfLogicalCPUCores();

/// @return The clock speed of the given core, in MHz. This is nominal, not actual. It is read from registry.
unsigned long GetCPUSpeedFromRegistry(unsigned long dwCPU);
