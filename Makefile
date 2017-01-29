BELA_PATH?=/root/Bela
OUTPUT=spi-pru

all: spi-pru.bin $(OUTPUT)
CFLAGS ?= -I/usr/xenomai/include -I $(BELA_PATH)/include -g
LDFLAGS ?= -L/usr/xenomai/lib
LDLIBS = -lrt -lnative -lxenomai

spi-pru.bin: spi-pru.p
	pasm -b spi-pru.p > /dev/null

$(OUTPUT): main.o GPIOcontrol.o loader.o
	g++ main.o GPIOcontrol.o loader.o /root/Bela/lib/libprussdrv.a -o spi-pru $(LDLIBS) $(LDFLAGS)

%.o: %.cpp
	g++ -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -c $(CFLAGS)

clean:
	rm -r *.o *.bin

.phony= all
