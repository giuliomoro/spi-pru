BELA_PATH?=/root/Bela
OUTPUT=DemoKeys

PRU_OBJS ?= spi-pru.bin
CC=g++
CFLAGS ?= -I/usr/xenomai/include -I$(BELA_PATH)/include -g
LDFLAGS ?= -L/usr/xenomai/lib -L/root/Bela/lib/
LDLIBS = -lrt -lnative -lxenomai -lprussdrv
OBJS ?= GPIOcontrol.o Keys.o Boards.o PruSpiKeysDriver.o
DEMO_OBJS ?= DemoKeys.o $(OBJS)
TEST_OBJS ?= TestKeys.o $(OBJS)
OLD_OBJS ?= main.o loader.o $(OBJS)

DEMO_DEPS := $(notdir $(DEMO_OBJS:.o=.d))
TEST_DEPS := $(notdir $(TEST_OBJS:.o=.d))
OLD_DEPS := $(notdir $(OLD_OBJS:.o=.d))

TEST_BINS ?= TestKeys
OLD = main

all: $(PRU_OBJS) $(OUTPUT)

spi-pru: $(OLD) $(PRU_OBJS)
	@#an empty recipe

$(OLD): $(OLD_OBJS)

spi-pru.bin: spi-pru.p
	pasm -b spi-pru.p > /dev/null

Keys.o: Keys.h
TestKeys.o: Keys.h
DemoKeys.o: Keys.h

$(OUTPUT): $(DEMO_OBJS)

%.o: %.cpp
	$(CXX) -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS)

clean:
	rm -rf *.o *.bin $(TEST_BINS) $(OUTPUT)

TestKeys: $(TEST_OBJS)

test: $(TEST_BINS)
	@./TestKeys && echo "All tests passed" || echo "An error occurred"

-include $(DEMO_DEPS) $(TEST_DEPS) $(OLD_DEPS)
.phony: all spi-pru
