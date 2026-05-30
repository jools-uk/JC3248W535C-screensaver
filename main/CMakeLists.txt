# Build the standalone display test instead of the fractal app:
#   idf.py build -DDISPLAY_TEST=ON
#
option(DISPLAY_TEST "Build display_test.c instead of the fractal app" OFF)

if(DISPLAY_TEST)
    idf_component_register(SRCS "display_test.c"
                            INCLUDE_DIRS "."
                            REQUIRES esp_lcd driver)
else()
    idf_component_register(SRCS "main.c" "display.c" "fractal.c" "touch.c" "ui.c"
                            INCLUDE_DIRS "."
                            REQUIRES esp_lcd driver nixy4__u8g2)
endif()
