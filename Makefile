# eSlate Project Makefile
# Compiler and flags for ESP32-S3
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -I. -I./eslate -I./esp -I./gpio -I./spi -I./i2c -I./rtos -I./drawing -I./hwr -I./shapes -I./gui -I./storage
LDFLAGS = -lm

# Source files
SOURCES = eslate/main.c eslate/eSlate.c esp/esp_err.c gpio/gpio.c spi/spi_master.c i2c/i2c.c rtos/FreeRTOS.c drawing/drawing.c hwr/hwr.c shapes/shapes.c gui/gui.c storage/storage.c
OBJECTS = $(SOURCES:.c=.o)
EXECUTABLE = bin/main

# Build directories
BIN_DIR = bin
OBJ_DIR = obj

# Default target
all: $(BIN_DIR) $(EXECUTABLE)

# Create bin directory
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Link object files to create executable
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files to object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf $(EXECUTABLE) $(OBJECTS) $(BIN_DIR)

# Phony targets
.PHONY: all clean
