cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(co2_low_power)


# Add your main application sources
target_sources(app PRIVATE
    src/main.c
    src/epd_3in7/epd_3in7.c
)

