VIA_ENABLE = yes
VIAL_ENABLE = yes
VIAL_INSECURE = yes

POINTING_DEVICE_DRIVER = custom
SRC += azoteq_iqs9151.c trackpad_gesture_parser.c
I2C_DRIVER_REQUIRED = yes