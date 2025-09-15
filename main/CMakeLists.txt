# main/CMakeLists.txt
idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
    EMBED_FILES
        "index.html"
    PRIV_REQUIRES
        # Core bits
        esp_event
        esp_netif
        nvs_flash
        esp_timer
        esp_http_server
        driver

        # Wi-Fi (C6 hosted + actual esp_wifi)
        espressif__esp_wifi_remote
        esp_wifi

        # Video pipeline
        espressif__esp_video
        espressif__esp_cam_sensor
        espressif__esp_h264
)

# Let main see private headers of these components (no edits to managed files)
set(PRJ ${PROJECT_DIR})
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${PRJ}/managed_components/espressif__esp_video/private_include"
    "${PRJ}/managed_components/espressif__esp_cam_sensor/private_include"
    # esp_h264 has no 'private_include', but it does have several include roots
)

# Also add public include dirs explicitly (some packages split public/private)
target_include_directories(${COMPONENT_LIB} PRIVATE
    "${PRJ}/managed_components/espressif__esp_video/include"
    "${PRJ}/managed_components/espressif__esp_cam_sensor/include"
    # esp_h264 public headers are spread across these directories
    "${PRJ}/managed_components/espressif__esp_h264/interface/include"
    "${PRJ}/managed_components/espressif__esp_h264/sw/include"
    "${PRJ}/managed_components/espressif__esp_h264/port/include"
    "${PRJ}/managed_components/espressif__esp_h264/hw/include"
)
