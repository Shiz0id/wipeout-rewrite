CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -O2
BUILD_DIR ?= build
ANTIGRAV = ../antigrav

.PHONY: all test clean

all: $(BUILD_DIR)/libpodracer.a

$(BUILD_DIR)/podracer.o: podracer.c podracer.h $(ANTIGRAV)/antigrav.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/libpodracer.a: $(BUILD_DIR)/podracer.o
	$(AR) rcs $@ $^

# The tests reuse antigrav's test geometry and BVH raycaster
$(BUILD_DIR)/sim_test: $(BUILD_DIR)/libpodracer.a test/sim_test.c $(ANTIGRAV)/test/test_world.c $(ANTIGRAV)/test/test_world.h $(ANTIGRAV)/antigrav_mesh.c
	$(CC) $(CFLAGS) test/sim_test.c $(ANTIGRAV)/test/test_world.c $(ANTIGRAV)/antigrav_mesh.c $(BUILD_DIR)/libpodracer.a -lm -o $@

test: $(BUILD_DIR)/sim_test
	$(BUILD_DIR)/sim_test

clean:
	$(RM) -r $(BUILD_DIR)
