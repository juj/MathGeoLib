#include <string>

std::string GetOSDisplayString();

/// @return The total amount of RAM on the system, in bytes.
unsigned long long GetTotalSystemPhysicalMemory();

/// @return A readable string of the processor as given by cpuid, like "AMD Turion(tm) X2 Dual-Core Mobile RM-74".
std::string GetProcessorBrandName();

/// @return "GenuineIntel" or "AuthenticAMD" or something similar.
std::string GetProcessorCPUIDString();

/// @return Technical information about the processor.
std::string GetProcessorExtendedCPUIDInfo();

/// @return The maximum number of threads the CPU can simultaneously accommodate.
int GetMaxSimultaneousThreads();

/// @return The clock speed of the given core, in MHz. This is nominal, not actual. It is read from registry.
unsigned long GetCPUSpeedFromRegistry(unsigned long dwCPU);
