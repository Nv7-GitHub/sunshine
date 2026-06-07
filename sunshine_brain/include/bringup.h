#pragma once

// BRINGUP_LEVEL controls which features are compiled in.
// 0 = production (all features), 1-4 = incremental bringup.
//
// Level 1: sensors CSV on USB serial only (no ESP-NOW, no DShot)
// Level 2: + DShot interactive test (no telemetry)
// Level 3: + ESP-NOW telemetry, motor outputs always zeroed (no motion)
// Level 4: + full navigation (kalman, derot filter); TANK mode drives so you can
//          spin the robot for nav tuning, DISABLED/MELTY stay zeroed
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

// Level 3: telemetry bringup — always zero the dshot outputs (no motion at all).
#if BRINGUP_LEVEL == 3
#  define FORCE_DSHOT_ZERO 1
#else
#  define FORCE_DSHOT_ZERO 0
#endif

// Level 4: navigation tuning — allow motor output only in TANK mode so you can
// spin the robot to tune the Kalman/derot filters; DISABLED and MELTY stay
// zeroed (README: "use tank mode to spin the robot. MELTY mode will still not
// be used"). Production (level 0) drives all modes.
#if BRINGUP_LEVEL == 4
#  define TANK_ONLY_OUTPUT 1
#else
#  define TANK_ONLY_OUTPUT 0
#endif
