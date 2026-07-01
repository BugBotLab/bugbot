idf_component_register(
    SRCS
        # â”€â”€ Entry point â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        "main.cpp"
        "bugbot_shims.cpp"

        # â”€â”€ Config â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        "lib/config/RobotConfig.cpp"
        "lib/config/AppConfig.cpp"

        # â”€â”€ Drivers â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        "lib/drivers/I2CBus.cpp"
        "lib/drivers/BMI270Driver.cpp"
        "lib/drivers/GroundSensorsLib.cpp"
        "lib/drivers/LidarLib.cpp"
        "lib/drivers/MotionLib.cpp"
        "lib/drivers/CameraDriver.cpp"
        "lib/drivers/SDCardDriver.cpp"

        # â”€â”€ Core â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        "lib/core/OdometryLib.cpp"

        # â”€â”€ Services â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        "lib/services/OdomService.cpp"
        "lib/services/MotionService.cpp"
        "lib/services/LidarService.cpp"
        "lib/services/LidarProjector.cpp"
        "lib/services/PoseService.cpp"
        "lib/services/CalibService.cpp"
        "lib/services/ActuatorService.cpp"
        "lib/services/DroidChime.cpp"
        "lib/services/WiFiService.cpp"
        "lib/services/WiFiLib.cpp"
        "lib/services/WiFiGlue.cpp"
        "lib/services/EspNowService.cpp"
        "lib/services/ScriptService.cpp"
        "lib/services/ConfigService.cpp"
        "lib/services/ConfigPortalService.cpp"
        "lib/services/MidiService.cpp"
        "lib/services/CameraService.cpp"
        "lib/services/CameraAPI.cpp"
        "lib/services/AprilTagService.cpp"
        "lib/services/BlobService.cpp"
        "lib/services/ContourService.cpp"
        "lib/services/TinyMLService.cpp"
        "lib/services/FaceDetectService.cpp"
        "lib/services/human_face_detect.cpp"

        # â”€â”€ Net â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        "lib/net/WebWS.cpp"

    INCLUDE_DIRS
        # Current directory â€” resolves "lib/foo/bar.h" paths in main.cpp and lib sources
        "."
        # Specific subdirs â€” for sources that use short-form includes like "AppConfig.h"
        "lib/config"
        "lib/core"
        "lib/drivers"
        "lib/services"
        "lib/net"
        "lib/util"

    REQUIRES
        espressif__arduino-esp32
        espressif__esp32-camera
        ESPAsyncWebServer
        AsyncTCP
        SparkFun_VL53L5CX
        SparkFun_BMI270
        Adafruit_PWMServoDriver
        Adafruit_BusIO
        ESP32Servo
        Adafruit_NeoPixel
        OpticalFlowESP32
        ArucoLite
        AppleMIDI
        MIDI
        bugbot_micropython
        mdns
        nvs_flash
        esp_http_server
        esp_mm
        espressif__esp-tflite-micro
        espressif__esp-dl
)

# Build the LittleFS image from data/ and flash it alongside the app.
# The partition name "spiffs" must match partitions.csv.
littlefs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)
# Model location: LittleFS VFS at /littlefs/models/<name>.espdl
target_compile_definitions(__idf_main PRIVATE
    CONFIG_HUMAN_FACE_DETECT_MODEL_IN_SDCARD=1
    CONFIG_BSP_SD_MOUNT_POINT="/littlefs"
    CONFIG_HUMAN_FACE_DETECT_MODEL_SDCARD_DIR="models"
    CONFIG_DEFAULT_HUMAN_FACE_DETECT_MODEL=0
)

