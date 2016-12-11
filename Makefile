BELA_PATH?=/root/Bela
all: spi-pru.bin
	pasm -b spi-pru.p > /dev/null
	gcc -std=gnu11 GPIOcontrol.c loader.c $(BELA_PATH)/lib/libprussdrv.a -o spi-pru -I $(BELA_PATH)/include
#	gcc *.o /root/Bela/lib/libprussdrv.a -o spi-pru

spi-pru.bin:

