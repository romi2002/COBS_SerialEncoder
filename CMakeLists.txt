cmake_minimum_required(VERSION 3.10.2)
project(COBS_SerialEncoder)

set(CMAKE_CXX_STANDARD 17)

add_executable(COBS_SerialEncoder main.cpp cobs.c)