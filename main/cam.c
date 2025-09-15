#include "cam.h"
#include "esp_log.h"

static const char *TAG = "cam_stub";

bool cam_init(void) {
    ESP_LOGW(TAG, "OV5647 stub init (using fallback frames until real driver is wired)");
    return true;
}

bool cam_get_jpeg(const uint8_t **jpg, size_t *n) {
    // No real camera yet — tell caller to use its fallback JPEG.
    if (jpg) *jpg = NULL;
    if (n)   *n   = 0;
    return false;
}
