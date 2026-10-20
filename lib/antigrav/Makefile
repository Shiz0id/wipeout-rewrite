CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -O2
BUILD_DIR ?= build

.PHONY: all test clean

all: $(BUILD_DIR)/libantigrav.a

$(BUILD_DIR)/antigrav.o: antigrav.c antigrav.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c antigrav.c -o $@

$(BUILD_DIR)/libantigrav.a: $(BUILD_DIR)/antigrav.o
	$(AR) rcs $@ $^

$(BUILD_DIR)/sim_test: $(BUILD_DIR)/libantigrav.a test/sim_test.c test/test_world.c test/test_world.h
	$(CC) $(CFLAGS) test/sim_test.c test/test_world.c $(BUILD_DIR)/libantigrav.a -lm -o $@

test: $(BUILD_DIR)/sim_test
	$(BUILD_DIR)/sim_test

clean:
	$(RM) -r $(BUILD_DIR)
