#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Return true if camera hw init OK (stub OK to always return true for now)
bool cam_init(void);

// Provide a pointer to a JPEG buffer and its size. Return true if a new frame
// is available; return false to let main use its built-in fallback JPEG.
bool cam_get_jpeg(const uint8_t **jpg, size_t *n);

#ifdef __cplusplus
}
#endif
