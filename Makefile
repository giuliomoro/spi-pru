BELA_PATH?=/root/Bela
OUTPUT=DemoKeys

PRU_OBJS ?= spi-pru.bin
CC=g++
CFLAGS ?= -I/usr/xenomai/include -I$(BELA_PATH)/include -g
LDFLAGS ?= -L/usr/xenomai/lib -L/root/Bela/lib/
LDLIBS = -lrt -lnative -lxenomai -lprussdrv
OBJS ?= GPIOcontrol.o Keys.o
DEMO_OBJS ?= DemoKeys.o 
TEST_OBJS ?= TestKeys.o Keys.o GPIOcontrol.o
TEST_BINS ?= TestKeys
OLD_OBJS ?= main.o loader.o
OLD = main

all: $(PRU_OBJS) $(OUTPUT)

spi-pru: $(OLD) $(PRU_OBJS) # make sure you preserve the hard tab at the beginning of the next line
	

$(OLD): $(OLD_OBJS) $(OBJS)

spi-pru.bin: spi-pru.p
	pasm -b spi-pru.p > /dev/null

Keys.o: Keys.h
TestKeys.o: Keys.h
DemoKeys.o: Keys.h

$(OUTPUT): $(OBJS) $(DEMO_OBJS)
	$(CXX) $(DEMO_OBJS) $(OBJS) -o $(OUTPUT) $(LDLIBS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS)

clean:
	rm -rf *.o *.bin $(TEST_BINS) $(OUTPUT)

TestKeys: $(TEST_OBJS)

test: $(TEST_BINS)
	./TestKeys

.phony: all spi-pru
