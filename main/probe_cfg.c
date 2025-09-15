#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "★★ P4CFG ★★";

// Hard fail build if any streaming flag is ON.
#if defined(CONFIG_ESP_HOSTED_SDIO_OPTIMIZATION_RX_STREAMING_MODE) && CONFIG_ESP_HOSTED_SDIO_OPTIMIZATION_RX_STREAMING_MODE
#  error "Streaming ON via CONFIG_ESP_HOSTED_SDIO_OPTIMIZATION_RX_STREAMING_MODE — must be OFF."
#endif
#if defined(CONFIG_ESP_SDIO_OPTIMIZATION_RX_STREAMING_MODE) && CONFIG_ESP_SDIO_OPTIMIZATION_RX_STREAMING_MODE
#  error "Streaming ON via CONFIG_ESP_SDIO_OPTIMIZATION_RX_STREAMING_MODE — must be OFF."
#endif

__attribute__((constructor(101)))
static void early_banner(void)
{
    ESP_LOGW(TAG, "==================== BUILD CONFIG (THIS BINARY) ====================");
#ifdef CONFIG_ESP_HOSTED_SDIO_CLOCK_FREQ_KHZ
    ESP_LOGW(TAG, "SDIO FREQ_KHZ   : %d", CONFIG_ESP_HOSTED_SDIO_CLOCK_FREQ_KHZ);
#else
    ESP_LOGW(TAG, "SDIO FREQ_KHZ   : <undefined>");
#endif

#if defined(CONFIG_ESP_HOSTED_SDIO_1_BIT_BUS) && CONFIG_ESP_HOSTED_SDIO_1_BIT_BUS
    ESP_LOGW(TAG, "BUS WIDTH       : 1-bit");
#elif defined(CONFIG_ESP_HOSTED_SDIO_4_BIT_BUS) && CONFIG_ESP_HOSTED_SDIO_4_BIT_BUS
    ESP_LOGW(TAG, "BUS WIDTH       : 4-bit");
#else
    ESP_LOGW(TAG, "BUS WIDTH       : <undefined>");
#endif

#ifdef CONFIG_ESP_HOSTED_SDIO_PIN_CLK
    ESP_LOGW(TAG, "Pins (CLK,CMD,D0,RST): %d, %d, %d, %d",
             CONFIG_ESP_HOSTED_SDIO_PIN_CLK,
             CONFIG_ESP_HOSTED_SDIO_PIN_CMD,
             CONFIG_ESP_HOSTED_SDIO_PIN_D0,
             CONFIG_ESP_HOSTED_SDIO_GPIO_RESET_SLAVE);
#else
    ESP_LOGW(TAG, "Pins           : <undefined>");
#endif

#if defined(CONFIG_ESP_HOSTED_SDIO_RESET_ACTIVE_HIGH) && CONFIG_ESP_HOSTED_SDIO_RESET_ACTIVE_HIGH
    ESP_LOGW(TAG, "RESET POLARITY  : ACTIVE-HIGH");
#elif defined(CONFIG_ESP_HOSTED_SDIO_RESET_ACTIVE_LOW) && CONFIG_ESP_HOSTED_SDIO_RESET_ACTIVE_LOW
    ESP_LOGW(TAG, "RESET POLARITY  : ACTIVE-LOW");
#else
    ESP_LOGW(TAG, "RESET POLARITY  : <undefined>");
#endif

#if defined(CONFIG_ESP_HOSTED_SDIO_OPTIMIZATION_RX_STREAMING_MODE)
    ESP_LOGW(TAG, "STREAMING       : %s (HOSTED macro)",
             CONFIG_ESP_HOSTED_SDIO_OPTIMIZATION_RX_STREAMING_MODE ? "ON" : "OFF");
#elif defined(CONFIG_ESP_SDIO_OPTIMIZATION_RX_STREAMING_MODE)
    ESP_LOGW(TAG, "STREAMING       : %s (ALT macro)",
             CONFIG_ESP_SDIO_OPTIMIZATION_RX_STREAMING_MODE ? "ON" : "OFF");
#else
    ESP_LOGW(TAG, "STREAMING       : <macro not present>");
#endif
    ESP_LOGW(TAG, "====================================================================");
}