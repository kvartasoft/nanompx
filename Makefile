# SPDX-License-Identifier: MIT
# Convenience Makefile wrapping CMake

BUILD_DIR ?= build
CMAKE_FLAGS ?=

.PHONY: all clean test install

all:
	cmake -S . -B $(BUILD_DIR) $(CMAKE_FLAGS)
	cmake --build $(BUILD_DIR) -j

test: all
	cd $(BUILD_DIR) && ctest --output-on-failure

clean:
	rm -rf $(BUILD_DIR)

install: all
	cmake --install $(BUILD_DIR)
