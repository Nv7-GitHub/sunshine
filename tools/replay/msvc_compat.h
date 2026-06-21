/* Force-included (cl /FI) when building sunshine_core with MSVC.
 * MSVC cannot parse GCC's __attribute__((packed)); strip it. We never memcpy
 * whole structs from the .sun file (we unpack field-by-field), so in-memory
 * padding differences are harmless for replay. */
#ifndef __GNUC__
#define __attribute__(x)
#endif
