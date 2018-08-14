#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Thread-safe version of strsignal(3) (except it will never return NULL).
//
// Returns a pointer to static data or a thread-local buffer.
const char *aos_strsignal(int signal);

#ifdef __cplusplus
}
#endif

