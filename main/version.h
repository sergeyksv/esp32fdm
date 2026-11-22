#pragma once

#define FW_VERSION "2.6"

/* "16.11.26 14:32 deee079+": build time, commit, "+" if there were uncommitted
 * changes.  Generated on every build (tools/build_stamp.py), so every flash
 * can be told apart while FW_VERSION changes only with a release. */
extern const char *const FW_BUILD;
