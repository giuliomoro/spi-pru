BELA_PATH?=/root/Bela
OUTPUT=spi-pru

all: spi-pru.bin $(OUTPUT)
CC=g++
CFLAGS ?= -I/usr/xenomai/include -I$(BELA_PATH)/include
LDFLAGS ?= -L/usr/xenomai/lib -L/root/Bela/lib/
LDLIBS = -lrt -lnative -lxenomai -lprussdrv
OBJS ?= main.o GPIOcontrol.o loader.o Keys.o
TEST_OBJS ?= TestKeys.o Keys.o GPIOcontrol.o
TEST_BINS ?= TestKeys

spi-pru.bin: spi-pru.p
	pasm -b spi-pru.p > /dev/null

Keys.o: Keys.h
TestKeys.o: Keys.h

$(OUTPUT): $(OBJS) $(TEST_OBJS)
	g++ $(OBJS) -o spi-pru $(LDLIBS) $(LDFLAGS)

%.o: %.cpp
	g++ -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS)

clean:
	rm -rf *.o *.bin $(TEST_BINS) $(OUTPUT)

TestKeys: $(TEST_OBJS)

test: $(TEST_BINS)
	./TestKeys

.phony= all
