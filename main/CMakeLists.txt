idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES
    esp_wifi_remote
    esp_hosted
    esp_http_server
    esp_timer
    driver
    esp_event
    nvs_flash
    esp_netif
    es8311
)

