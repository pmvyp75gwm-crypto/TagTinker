# Host-side tests for the ESL module (src/modules/esl).
#
# These run on your development machine with a plain C++17 compiler -- no
# ESP32, no PlatformIO, no hardware. They cover the layers that are pure
# logic, which is everything except the RMT driver itself:
#
#   pipeline_test  test pattern -> image encoding -> protocol frames ->
#                  PP4 waveform symbols (mirrors esl_ir.cpp's builder),
#                  plus CRC, frame layouts, profiles and barcode parsing.
#   sequence_test  esl_sequence.cpp's transmit schedules, with the IR
#                  layer faked so every frame put on the wire is counted
#                  and checked against the TagTinker values it was
#                  transcribed from.
#
# Usage:
#   make            build and run both suites
#   make clean
#
# This directory is deliberately outside src/, because Bruce's
# build_src_filter compiles everything under src/ into the firmware.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O1 -g
ESL      := ../../src/modules/esl
BUILD    := build

.PHONY: all run clean
all: run

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/pipeline_test: pipeline_test.cpp $(ESL)/esl_protocol.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(ESL) $^ -o $@

# esl_sequence.cpp pulls in Arduino.h for delay(); fake/ provides a stub so
# the schedule logic can be exercised on the host.
$(BUILD)/sequence_test: sequence_test.cpp $(ESL)/esl_sequence.cpp $(ESL)/esl_protocol.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -Ifake -I$(ESL) $^ -o $@

run: $(BUILD)/pipeline_test $(BUILD)/sequence_test
	@./$(BUILD)/pipeline_test
	@./$(BUILD)/sequence_test

clean:
	@rm -rf $(BUILD)
