BELA_PATH?=/root/Bela
OUTPUT_C=DemoKeys_c
OUTPUT=DemoKeys
OUTPUT_LIB=libbed
OUTPUT_STATIC=DemoKeysStatic
OUTPUT_SHARED=DemoKeysShared

#CC=clang
#CXX=clang++
OPT_FLAGS ?= -O3 -march=armv7-a -mtune=cortex-a8 -mfloat-abi=hard -mfpu=neon -ftree-vectorize -DNDEBUG -Wall -U_FORTIFY_SOURCE
PRU_OBJS ?= spi-pru.bin

C_OBJS ?= Keys_c.o BoardsTopology_c.o
CFLAGS ?= -I/usr/xenomai/include -I$(BELA_PATH)/include $(OPT_FLAGS)
CPPFLAGS ?= $(CFLAGS) -std=c++11
LDFLAGS ?= -L/usr/xenomai/lib -L/root/Bela/lib/
LDLIBS = -lrt -lnative -lxenomai -lprussdrv
OBJS ?= GPIOcontrol.o Keys.o Boards.o PruSpiKeysDriver.o Gpio.o
DEMO_OBJS ?= DemoKeys.o $(OBJS)
TEST_OBJS ?= TestKeys.o $(OBJS)
OLD_OBJS ?= main.o loader.o $(OBJS)
LIB_OBJS = $(OBJS:.o=.fpic.o) $(C_OBJS:.o=.fpic.o)
LIB_SO ?= libkeys.so
LIB_A ?= libkeys.a

DEMO_DEPS := $(notdir $(DEMO_OBJS:.o=.d))
TEST_DEPS := $(notdir $(TEST_OBJS:.o=.d))
OLD_DEPS := $(notdir $(OLD_OBJS:.o=.d))
LIB_DEPS := $(notdir $(LIB_OBJS:.o=.d))

TEST_BINS ?= TestKeys test_keys_c
OLD = main

all: $(PRU_OBJS) $(OUTPUT)

spi-pru: $(OLD) $(PRU_OBJS)
	@#an empty recipe

$(OLD): $(OLD_OBJS) spi-pru.bin
	$(CXX) $(LDFLAGS) $(OLD_OBJS) $(LDLIBS) -o $(OLD)

spi-pru.bin: spi-pru.p
	pasm -V2 -b spi-pru.p > /dev/null

Keys.o: Keys.h
TestKeys.o: Keys.h
DemoKeys.o: Keys.h

$(OUTPUT_LIB): lib DemoKeys.o
	$(CXX) $(LDFLAGS) -L. -o $(OUTPUT_STATIC) DemoKeys.o $(LIB_A) $(LDLIBS)
	$(CXX) $(LDFLAGS) -L. -o $(OUTPUT_SHARED) DemoKeys.o -lkeys $(LDLIBS) 

$(OUTPUT): $(DEMO_OBJS) $(PRU_OBJS)
	$(CXX) $(LDFLAGS) -o "$@" $(DEMO_OBJS) $(LDLIBS)

$(OUTPUT_C): $(OBJS) $(C_OBJS) DemoKeys_c.o
	$(CXX) $(LDFLAGS) -o "$@" $(OBJS) $(C_OBJS) DemoKeys_c.o $(LDLIBS)

%.o: %.cpp
	$(CXX) -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS)

%.fpic.o: %.cpp
	$(CXX) -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS) -fPIC

clean:
	rm -rf *.o *.bin $(TEST_BINS) $(OUTPUT) $(LIB_SO) $(LIB_A)

TestKeys: $(TEST_OBJS)
	$(CXX) $(LDFLAGS) -L. $(TEST_OBJS) -o "$@" $(LIB_A) $(LDLIBS)

test_keys_c: $(LIB_SO)
	$(CC) $(LDFLAGS) -L. -o test_keys_c test_keys_c.cpp $(LIB_A) $(LDLIBS) -lstdc++

test: $(TEST_BINS)
	@./TestKeys && \
	./test_keys_c && \
	echo "All tests passed" || echo "An error occurred"

test-lib: lib $(TEST_BINS)

lib: $(LIB_SO) $(LIB_A)

$(LIB_SO): $(LIB_OBJS) $(PRU_OBJS)
	gcc -shared -Wl,-soname,$(LIB_SO) $(LDLIBS) \
    -o $(LIB_SO) $(LIB_OBJS) $(LDFLAGS)

$(LIB_A): $(LIB_OBJS) $(PRU_OBJS)
	ar rcs $(LIB_A) $(LIB_OBJS)

-include $(DEMO_DEPS) $(TEST_DEPS) $(OLD_DEPS) $(LIB_DEPS)
.PHONY: all spi-pru lib
