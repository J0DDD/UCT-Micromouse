#ifndef FIRMWARE_VERSION_H
#define FIRMWARE_VERSION_H

// =========================================================================
// UCT Micromouse - Firmware Version & Binary Identification
// =========================================================================

#ifndef UCT_FW_VERSION_MAJOR
#define UCT_FW_VERSION_MAJOR 2026
#endif

#ifndef UCT_FW_VERSION_MINOR
#define UCT_FW_VERSION_MINOR 1
#endif

#ifndef UCT_FW_VERSION_PATCH
#define UCT_FW_VERSION_PATCH 1
#endif

#ifndef UCT_FW_VERSION_STR
#define UCT_FW_VERSION_STR "v2026.1.1"
#endif

#ifndef UCT_FW_GIT_HASH
#define UCT_FW_GIT_HASH "56b7cae-dirty"
#endif

#ifndef UCT_FW_BUILD_DATE
#define UCT_FW_BUILD_DATE "2026-10-22"
#endif

// Machine-scannable ASCII watermark embedded directly into flash .rodata
// Format: UCT_MM_FW:<version>:<git_hash>:<build_date>
#define UCT_FW_WATERMARK "UCT_MM_FW:" UCT_FW_VERSION_STR ":" UCT_FW_GIT_HASH ":" UCT_FW_BUILD_DATE

// Human-readable firmware label
#define UCT_FW_LABEL "UCT-Micromouse " UCT_FW_VERSION_STR " (" UCT_FW_GIT_HASH ") " UCT_FW_BUILD_DATE

#endif // FIRMWARE_VERSION_H
