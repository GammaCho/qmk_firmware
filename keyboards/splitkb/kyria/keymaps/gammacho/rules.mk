OLED_ENABLE = yes
#OLED_DRIVER = SSD1306   # Enables the use of OLED displays
OS_DETECTION_ENABLE = yes
DEFERRED_EXEC_ENABLE = yes
ENCODER_ENABLE = no       # Enables the use of one or more encoders
RGBLIGHT_ENABLE = no      # Enable keyboard RGB underglow
MOUSEKEY_ENABLE = yes     # Mouse keys
RGB_MATRIX_SUPPORTED = no
POINTING_DEVICE_ENABLE = yes
POINTING_DEVICE_DRIVER = cirque_pinnacle_i2c
TAP_DANCE_ENABLE = yes
COMBO_ENABLE = yes

#WS2812_DRIVER_REQUIRED = yes
RGBLIGHT_ENABLE = yes # Enables QMK's RGB code
WS2812_DRIVER = vendor # Use the RP2040's PIO interface

# POINTING_DEVICE_DRIVER = custom
