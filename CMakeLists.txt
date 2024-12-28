cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr)
project(co2_low_power)


set(BOARD xiao_ble)

target_sources(app PRIVATE src/main.c)



