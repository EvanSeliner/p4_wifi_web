# main/CMakeLists.txt  (single component, no duplicates)

idf_component_register(
    SRCS
        "main.c"
        "cam.c"                      # <- stub/driver for cam_init/cam_get_jpeg
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        esp_event
        esp_netif
        nvs_flash
        esp_timer
        esp_http_server
        driver
        esp_wifi
        espressif__esp_wifi_remote
        espressif__esp_video
        espressif__esp_cam_sensor
        espressif__esp_h264
    EMBED_FILES
        "index.html"
)

# If you really need private/public headers from managed components:
set(PRJ ${PROJECT_DIR})
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${PRJ}/managed_components/espressif__esp_video/private_include"
    "${PRJ}/managed_components/espressif__esp_cam_sensor/private_include"
    "${PRJ}/managed_components/espressif__esp_video/include"
    "${PRJ}/managed_components/espressif__esp_cam_sensor/include"
    "${PRJ}/managed_components/espressif__esp_h264/interface/include"
    "${PRJ}/managed_components/espressif__esp_h264/sw/include"
    "${PRJ}/managed_components/espressif__esp_h264/port/include"
    "${PRJ}/managed_components/espressif__esp_h264/hw/include"
)
