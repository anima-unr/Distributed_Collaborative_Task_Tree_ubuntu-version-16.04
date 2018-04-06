/*
timeseries_recording_toolkit
Copyright (C) 2015  Luke Fraser

timeseries_recording_toolkit is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

timeseries_recording_toolkit is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with timeseries_recording_toolkit.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef INCLUDE_LOG_H_
#define INCLUDE_LOG_H_
#include <stdio.h>
#include <stdarg.h>

static inline void log_printf(const char *fmt, ...);

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)             \

#ifdef LOG_INFO_PRINTING
#define LOG_INFO(fmt, ...) do {                                                \
  log_printf("%s:%d:%s(): " fmt "\n",                                          \
                  __FILENAME__, __LINE__, __func__, ##__VA_ARGS__);              \
} while (0)

// #define LOG_INFO(fmt) do {                                                     \
//   log_printf("%s:%d:%s(): " fmt "\n",                                          \
//                   __FILENAME__, __LINE__, __func__);                           \
// } while (0)
#else
#define LOG_INFO(fmt, ...) {}
#endif

static inline void log_printf(const char *fmt, ...) {
  va_list arg;
  va_start(arg, fmt);
  vfprintf(stderr, fmt, arg);
  va_end(arg);
}
#endif  // INCLUDE_LOG_H_