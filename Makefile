BELA_PATH?=/root/Bela
OUTPUT=DemoKeys

PRU_OBJS ?= spi-pru.bin
CC=g++
C_OBJS ?= Keys_c.o BoardsTopology_c.o
CFLAGS ?= -I/usr/xenomai/include -I$(BELA_PATH)/include -g -fPIC
LDFLAGS ?= -L/usr/xenomai/lib -L/root/Bela/lib/
LDLIBS = -lrt -lnative -lxenomai -lprussdrv
OBJS ?= GPIOcontrol.o Keys.o Boards.o PruSpiKeysDriver.o
DEMO_OBJS ?= DemoKeys.o $(OBJS)
TEST_OBJS ?= TestKeys.o $(OBJS)
OLD_OBJS ?= main.o loader.o $(OBJS)
LIB_SO ?= libkeys.so

DEMO_DEPS := $(notdir $(DEMO_OBJS:.o=.d))
TEST_DEPS := $(notdir $(TEST_OBJS:.o=.d))
OLD_DEPS := $(notdir $(OLD_OBJS:.o=.d))

TEST_BINS ?= TestKeys
OLD = main

all: $(PRU_OBJS) $(OUTPUT)

spi-pru: $(OLD) $(PRU_OBJS)
	@#an empty recipe

$(OLD): $(OLD_OBJS) spi-pru.bin
	$(CC) $(LDFLAGS) $(OLD_OBJS) $(LDLIBS) -o $(OLD)

spi-pru.bin: spi-pru.p
	pasm -V2 -b spi-pru.p > /dev/null

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

lib: $(LIB_SO)

$(LIB_SO): $(OBJS) $(C_OBJS) $(PRU_OBJS)
	gcc -shared -Wl,-soname,$(LIB_SO) \
    -o $(LIB_SO) $(OBJS) $(C_OBJS) $(LDLIBS) $(LDFLAGS)


-include $(DEMO_DEPS) $(TEST_DEPS) $(OLD_DEPS) 
.phony: all spi-pru lib
