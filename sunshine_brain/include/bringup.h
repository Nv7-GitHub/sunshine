#pragma once

// BRINGUP_LEVEL controls which features are compiled in.
// 0 = production (all features), 1-4 = incremental bringup.
//
// Level 1: sensors CSV on USB serial only (no ESP-NOW, no DShot)
// Level 2: + DShot interactive test (no telemetry)
// Level 3: + ESP-NOW telemetry, no navigation filter updates (zeros dshot)
// Level 4: + full navigation (kalman, derot filter), still zeros dshot for safety
// Level 0: production — all features, full control output

#ifndef BRINGUP_LEVEL
#define BRINGUP_LEVEL 0
#endif

// Feature gates derived from level
#if BRINGUP_LEVEL == 0 || BRINGUP_LEVEL >= 2
#  define FEATURE_DSHOT     1
#else
#  define FEATURE_DSHOT     0
#endif

#if BRINGUP_LEVEL == 0 || BRINGUP_LEVEL >= 3
#  define FEATURE_TELEMETRY 1
#else
#  define FEATURE_TELEMETRY 0
#endif

// At levels 3-4 we always zero the dshot outputs for safety (no actual drive)
#if BRINGUP_LEVEL >= 3 && BRINGUP_LEVEL <= 4
#  define FORCE_DSHOT_ZERO 1
#else
#  define FORCE_DSHOT_ZERO 0
#endif
