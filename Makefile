BELA_PATH?=/root/Bela
OUTPUT=spi-pru

all: spi-pru.bin $(OUTPUT)

spi-pru.bin: spi-pru.p
	pasm -b spi-pru.p > /dev/null

$(OUTPUT): main.o GPIOcontrol.o loader.o
	g++ main.o GPIOcontrol.o loader.o /root/Bela/lib/libprussdrv.a -o spi-pru

%.o: %.cpp
	g++ -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" "$<" -o "$@" -I $(BELA_PATH)/include -c

.phony= all
