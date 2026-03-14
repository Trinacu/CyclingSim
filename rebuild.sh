#!/bin/bash

# Clean the build directory and rebuild with bear for compilation database
rm -rf build
cmake -S . -B build
bear -- cmake --build build
