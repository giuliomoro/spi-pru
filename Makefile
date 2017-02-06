BELA_PATH?=/root/Bela
OUTPUT=DemoKeys
OUTPUT_LIB=libbed
OUTPUT_STATIC=DemoKeysStatic
OUTPUT_SHARED=DemoKeysShared

CC=clang
CXX=clang++
OPT_FLAGS ?= -O3 -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ftree-vectorize -DNDEBUG -Wall -U_FORTIFY_SOURCE
PRU_OBJS ?= spi-pru.bin

C_OBJS ?= Keys_c.o BoardsTopology_c.o
CFLAGS ?= -I/usr/xenomai/include -I$(BELA_PATH)/include $(OPT_FLAGS)
LDFLAGS ?= -L/usr/xenomai/lib -L/root/Bela/lib/
LDLIBS = -lrt -lnative -lxenomai -lprussdrv
OBJS ?= GPIOcontrol.o Keys.o Boards.o PruSpiKeysDriver.o
DEMO_OBJS ?= DemoKeys.o $(OBJS)
TEST_OBJS ?= TestKeys.o $(OBJS)
OLD_OBJS ?= main.o loader.o $(OBJS)
LIB_SO_OBJS = $(OBJS:.o=.fpic.o) $(C_OBJS:.o=.fpic.o)
LIB_SO ?= libkeys.so
LIB_A ?= libkeys.a

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

$(OUTPUT_LIB): lib DemoKeys.o
	$(CXX) $(LDFLAGS) -L. -o $(OUTPUT_STATIC) DemoKeys.o $(LIB_A) $(LDLIBS)
	$(CXX) $(LDFLAGS) -L. -o $(OUTPUT_SHARED) DemoKeys.o -lkeys $(LDLIBS) 

$(OUTPUT): $(DEMO_OBJS)
	$(CXX) $(LDFLAGS) -o $(OUTPUT) $(DEMO_OBJS) $(LDLIBS)

%.o: %.cpp
	$(CXX) -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS)

%.fpic.o: %.cpp
	$(CXX) -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS) -fPIC

clean:
	rm -rf *.o *.bin $(TEST_BINS) $(OUTPUT) $(LIB_SO) $(LIB_A)

TestKeys: $(TEST_OBJS)

test: $(TEST_BINS)
	@./TestKeys && echo "All tests passed" || echo "An error occurred"

test-lib: lib $(TEST_BINS)

lib: $(LIB_SO) $(LIB_A)

$(LIB_SO): $(LIB_SO_OBJS) $(PRU_OBJS)
	gcc -shared -Wl,-soname,$(LIB_SO) $(LDLIBS) \
    -o $(LIB_SO) $(LIB_SO_OBJS) $(LDFLAGS)

$(LIB_A): $(OBJS) $(C_OBJS) $(PRU_OBJS)
	ar rcs $(LIB_A) $(OBJS) $(C_OBJS)

-include $(DEMO_DEPS) $(TEST_DEPS) $(OLD_DEPS) 
.phony: all spi-pru lib
