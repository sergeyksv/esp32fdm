#pragma once

#include <stdint.h>

/**
 * Evict least-recently-used files from a cache directory.
 *
 * Call after writing a new entry. If the number of files in @p dir_path
 * exceeds @p max_entries, the oldest files (by mtime) are deleted until
 * the count is at or below max_entries.
 *
 * @param dir_path   Absolute path, e.g. "/cache/gcode"
 * @param max_entries  Maximum number of files to keep
 */
void cache_evict_lru(const char *dir_path, int max_entries);

/**
 * Touch a cache file's mtime to now (marks it recently used).
 *
 * @param path  Absolute file path
 */
void cache_touch(const char *path);
