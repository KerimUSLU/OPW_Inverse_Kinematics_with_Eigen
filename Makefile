# Simple Makefile for building the OPW inverse kinematics demo
# Usage:
#   make        # build executable
#   make run    # build and run
#   make clean  # remove build artifacts

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I./eigen-3.4.0
LDFLAGS ?=

SRC = main.cpp
TARGET = opw_ik

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
