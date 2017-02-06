/******************************************************************************
* PRU_memAcc_DDR_sharedRAM.c
*
* The PRU reads three values from external DDR memory and stores these values
* in shared PRU RAM using the programmable constant table entries.  The example
* initially loads 3 values into the external DDR RAM.  The PRU configures its
* Constant Table Programmable Pointer Register 0 and 1 (CTPPR_0, 1) to point
* to appropriate locations in the DDR memory and the PRU shared RAM.  The
* values are then read from the DDR memory and stored into the PRU shared RAM
* using the values in the 28th and 31st entries of the constant table.
*
******************************************************************************/

// Standard header files
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include </root/Bela/include/Gpio.h>
#include <stdlib.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/intr.h>
#include <rtdk.h>

const uint32_t Polynomial = 0x04C11DB7;

uint32_t crc32_bitwise(const void* data, size_t length)
{
  uint32_t previousCrc32 = 0;
  uint32_t crc = ~previousCrc32;
  unsigned char* current = (unsigned char*) data;
  while (length--)
  {
	unsigned int data = *current++;
    crc ^= data;
    for (unsigned int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (-(int)(crc & 1) & Polynomial);
  }
  return ~crc; // same as crc ^ 0xFFFFFFFF
}


/* Commands to communicate with the other boards */
enum {
	kBusCommandStatus = 0x80,    /* Return device status */
	kBusCommandStartScan,     /* Start a new I2C / analog scan */
	kBusCommandStopRunning,   /* Stop devices after regular scans have been initiated */
	kBusCommandI2CSend,       /* Send a specific I2C command to a given device (or all devices) */
	kBusCommandI2CReadResponse, /* Read the response from the previous I2C command */
	kBusCommandRescanKeyboard,	/* Rescan the keys that are connected, and put all keys in centroid mode */
	kBusCommandRGBLEDSetColors = 0x90, /* Set colors for the LEDs */
	kBusCommandRGBLEDAllOff
};


// Driver header file
#include "prussdrv.h"
#include "pruss_intc_mapping.h"
#include <GPIOcontrol.h>
#include <digital_gpio_mapping.h>

short int digitalPins[NUM_DIGITALS] = {
  GPIO_NO_BIT_0,
  GPIO_NO_BIT_1,
  GPIO_NO_BIT_2,
  GPIO_NO_BIT_3,
  GPIO_NO_BIT_4,
  GPIO_NO_BIT_5,
  GPIO_NO_BIT_6,
  GPIO_NO_BIT_7,
  GPIO_NO_BIT_8,
  GPIO_NO_BIT_9,
  GPIO_NO_BIT_10,
  GPIO_NO_BIT_11,
  GPIO_NO_BIT_12,
  GPIO_NO_BIT_13,
  GPIO_NO_BIT_14,
  GPIO_NO_BIT_15,
};

#define PRU_NUM 	 1
#define ADDEND1	 	 0x98765400u
#define ADDEND2		 0x12345678u
#define ADDEND3		 0x10210210u

#define DDR_BASEADDR     0x80000000
#define OFFSET_DDR	 0x00001000
#define OFFSET_SHAREDRAM 2048		//equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4


extern int gShouldStop;

/* Pack three 12-bit values plus an index into a 6-byte data structure for transmission
 * on the bus.
 */
void rgbledPackBuffer(uint8_t *buffer, uint8_t led, uint16_t red, uint16_t green, uint16_t blue) {
	buffer[0] = led;
	buffer[1] = (red >> 4) & 0xFF;
	buffer[2] = ((red << 4) & 0xF0) | ((green >> 8) & 0x0F);
	buffer[3] = (green & 0xFF);
	buffer[4] = (blue >> 4) & 0xFF;
	buffer[5] = (blue << 4) & 0xF0;
}

/* Convert HSV to RGB (the actual mechanics here0 */
void rgbledHSVToRGB(uint8_t hue, uint8_t saturation, uint8_t value,
		uint16_t *red, uint16_t *green, uint16_t *blue) 
{
	if(saturation == 0) {
		/* Gray = all colors the same intensity; convert value from 8 to 12 bits */
		*red = *green = *blue = (uint16_t)value << 4;
	}
	else if(value == 0) {
		*red = *green = *blue = 0;
	}
	else {
		uint8_t sector = hue / 43; /* Divide 0-255 range into segments 0-5 */
		uint8_t factorial = hue - (sector * 43);

		/* Intermediate factors for HSV->RGB */
		uint32_t p = value * (255 - saturation); /* Range 0-(255*255) */
		uint32_t q = value * (255*42 - saturation * factorial); /* Range 0-(255*255*42) */
		uint32_t t = value * (255*42 - saturation * (42 - factorial)); /* Range 0-(255*255*42) */

		/* Normalize each intermediate value to 12-bit range (0-4095). All intermediate values
		 * should stay within 32-bit range. The numbers below come from reducing the fractions
		 * to keep the intermediate value as small as feasible.
		 */
		p = (p * 273) / (17*255);
		q = (q * 273) / (17*255*42);
		t = (t * 273) / (17*255*42);

		/* Safety check */
		if(p > 4095) p = 4095;
		if(q > 4095) q = 4095;
		if(t > 4095) t = 4095;

		switch(sector) {
		case 0:
			*red = (uint16_t)value << 4;
			*green = t;
			*blue = p;
			break;
		case 1:
			*red = q;
			*green = (uint16_t)value << 4;
			*blue = p;
			break;
		case 2:
			*red = p;
			*green = (uint16_t)value << 4;
			*blue = t;
			break;
		case 3:
			*red = p;
			*green = q;
			*blue = (uint16_t)value << 4;
			break;
		case 4:
			*red = t;
			*green = p;
			*blue = (uint16_t)value << 4;
			break;
		case 5:
			*red = (uint16_t)value << 4;
			*green = p;
			*blue = q;
			break;
		default:
			/* Shouldn't happen */
			*red = *green = *blue = 0;
			return;
		}
	}
}

/* Perform HSV->RGB conversion and pack the results into a 6-byte data structure for transmission
 * on the bus.
 */
void rgbledPackBufferWithHSV(uint8_t *buffer, uint8_t led, uint8_t hue, uint8_t saturation, uint8_t value) {
	uint16_t red, green, blue;

	rgbledHSVToRGB(hue, saturation, value, &red, &green, &blue);
	rgbledPackBuffer(buffer, led, red, green, blue);
}

const unsigned int kPruGPIODACSyncPin = 49;	// GPIO1(17); P9-23
const unsigned int kPruGPIOADCSyncPin = 48; // GPIO1(16); P9-15

int prepareGPIO(int include_led)
{
	if(1) {
		// Prepare DAC CS/ pin: output, high to begin
		if(gpio_export(kPruGPIODACSyncPin)) {
		}
		if(gpio_set_dir(kPruGPIODACSyncPin, OUTPUT_PIN)) {
			return -1;
		}
		if(gpio_set_value(kPruGPIODACSyncPin, HIGH)) {
			return -1;
		}

		// Prepare ADC CS/ pin: output, high to begin
		if(gpio_export(kPruGPIOADCSyncPin)) {
		}
		if(gpio_set_dir(kPruGPIOADCSyncPin, OUTPUT_PIN)) {
			return -1;
		}
		if(gpio_set_value(kPruGPIOADCSyncPin, HIGH)) {
			return -1;
		}

	}

	if(1){
		for(unsigned int i = 0; i < 16; i++){
			if(gpio_export(digitalPins[i])) {
                return -1;
			}
			if(gpio_set_dir(digitalPins[i], INPUT_PIN)) {
				return -1;
			}
		}
	}

	if(include_led) {
		// Turn off system function for LED3 so it can be reused by PRU
		led_set_trigger(3, "none");
	}

	return 0;
}
//*/

static unsigned int *sharedMem_int;

/* Query the presence and status of any connected slave devices (up to 3). Devices will respond
 * as follows:
 *
 * Byte 0: total length of response, including this byte. Must be 8.
 * Byte 1: hardware version
 * Byte 2: firmware major version
 * Byte 3: firmware minor version (all these three should be the same as our own)
 * Bytes 4-5: LSB/MSB of lower octave connected keys (uint16_t)
 * Bytes 6-7: LSB/MSB of upper octave connected keys (uint16_t)
 *
 * This function is synchronous, and updates the number of octaves and the connected keys before
 * returning.
 */

#ifdef nope
void busMasterQueryConnectedDevices(uint8_t* gBusSlaveTransmitBuffer, uint32_t* length) {
	uint16_t i;
	gBusSlaveTransmitBuffer[0] = kBusCommandStatus;
	const uint8_t expectedResponseLength = 8;
	/* First transmit a query command to all devices */
	//busModeMasterTransmitter(0, gBusSlaveTransmitBuffer, 1);

	while(gBusCommunicationMode != kBusCommunicationModeIdle) {}

	/* Let devices process, ~20us with gcc -O2 */
	volatile int j;
	for(j = 0; j < 100; j++) {}

	/* Then go through and read the status of each device in turn */
	for(i = 1; i < 4; i++) {
		gBusMasterState = kBusMasterStateReadingStatus;
		busModeMasterReceiver(i, gBusReceiveBuffer, 12);

		/* Wait for command to execute */
		while(gBusCommunicationMode != kBusCommunicationModeIdle) {}

		//printf("r %x %x %x\n\r", gBusReceiveBuffer[0], gBusReceiveBuffer[1], gBusReceiveBuffer[2]);

		/* Check for a coherent response by a matching device */
		if(gBusReceiveBuffer[0] != expectedResponseLength)
			break;
		if(gBusReceiveBuffer[1] != kHardwareVersion)
			break;
		if(gBusReceiveBuffer[2] != kFirmwareMajorVersion)
			break;
		/* Allow variation in firmware minor version */

		/* Copy connected keys to our own buffer */
		memcpy(&gConnectedKeys[i*2], &gBusReceiveBuffer[4],  4);
	}

	/* Number of octaves is twice the number of connected boards */
    gNumDevices = i;
    gNumOctaves = i*2;

	gBusMasterState = kBusMasterStateIdle;
	gBusMasterFrameToProcess = 0;
	gBusCommunicationMode = kBusCommunicationModeIdle;
}
#endif

static volatile uint8_t* data;

int16_t get_key_position_raw(unsigned int n){
    return (*(int16_t*)(&data[n * 2 + 8]));
}

float get_key_position(unsigned int n){
	float value = get_key_position_raw(n) / 4096.f;
	if (value < 0)
		return 0;
	else
		return value;
}

int spi_pru_loader (void)
{
    int ret = rt_task_shadow (NULL, "spi-pru", 40, T_FPU);
    if(ret != 0){
        fprintf(stderr, "error: %s\n", strerror);
    }
	rt_print_auto_init(1);
	Gpio testGpio;
	testGpio.open(66, 1);
	Gpio testGpio2;
	testGpio2.open(67, 1);
	Gpio testGpio3;
	testGpio3.open(69, 1);
	Gpio testGpio4;
	testGpio4.open(68, 1);

    if(prepareGPIO(1)){
	}
    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

    /* Initialize the PRU */
    prussdrv_init ();

    /* Open PRU Interrupt */
    ret = prussdrv_open(PRU_EVTOUT_0);
    if (ret)
    {
        printf("prussdrv_open open failed\n");
        return (ret);
    }

    /* Map PRU's INTC */
    prussdrv_pruintc_init(&pruss_intc_initdata);

	prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

    uint8_t *pruMem = NULL;
    prussdrv_map_prumem (PRU_NUM == 0 ? PRUSS0_PRU0_DATARAM : PRUSS0_PRU1_DATARAM, (void **)&pruMem);
    if(pruMem == NULL){
        fprintf(stderr, "prussdrv_map_prumem failed\n");
        return 1;
    }
    volatile uint32_t* t32 = (uint32_t*)pruMem;
    data = (uint8_t*)&t32[1];
    // disable transmissions before starting the PRU
    t32[0] = 0;
    uint32_t length;
    int numLeds = 25;
    int firstLed = 0;
    if(0){
        length = numLeds * 6 + 2;

        data[0] = kBusCommandRGBLEDSetColors;
        data[1] = numLeds;
        int p = 0;
        for(int n = 0; n < numLeds; ++n){
            uint8_t led = n + firstLed;
            uint16_t red = 4095;
            uint16_t green = 0;
            uint16_t blue = 0;
            rgbledPackBuffer((uint8_t*)&data[2 + p], led, red, green, blue);
            p += 6;
        }
        //memset(&data[2], 0, 10);
        t32[0] = length; 
        printf("---------\nlength: %d\n%#x %#x %#x\n-------------\n", length, 
          t32[0], t32[1], t32[2]);
    }
    /* Execute example on PRU */
    if(prussdrv_exec_program (PRU_NUM, "/root/spi-pru/spi-pru.bin")){
        fprintf(stderr, "Failed loading program\n");
    }

    if(0){
        int h = 0;
        int c[3] = {0, 1500, 3000};
        int in = 10;
        int inc[3] = {in, in, in};
		int numTimes = 100;
        for(int j = 0; j < numTimes; ++j){
            memset(pruMem, 0, 100);
            data[0] = kBusCommandRGBLEDSetColors;
            data[1] = numLeds;
            int p = 0;
            for(int n = 0; n < numLeds; ++n){
                uint8_t led = n;
                uint16_t red = c[0]; // h > 1000 ? 0 : 4095;
                uint16_t green = c[1]; // h > 2000 ? 0 : 4095;
                uint16_t blue = c[2]; //h > 3000 ? 0 : 4095;
				if(j == numTimes - 1){
					red = 0;
					green = 0;
					blue = 0;
				}
                rgbledPackBuffer((uint8_t*)&data[2 + p], led, red, green, blue);
                p += 6;
                for(int n = 0; n < 3; ++n){
                    c[n] += inc[n];
                    if(c[n] <= 0)
                        inc[n] = in;
                    if(c[n] >= 4096)
                        inc[n] = -in;
                }
                printf("r: %d, g: %d, b: %d\n", c[0], c[1], c[2]);
            }
            t32[0] = length; 
            printf("%10d---------\nlength: %d\n%#x %#x %#x\n-------------\n", j, length, 
              t32[0], t32[1], t32[2]);
        }
        while(t32[0] != 0 && !gShouldStop){
            printf("waiting for previous tasks to complete\n");
        }
    }

    printf("done\n");
    if(0){ // query connected devices
        data[0] = kBusCommandStatus;
        length = 1;
        t32[0] = length;
        do {
            printf("Delivering the kBusCommandStatus\n");
        } while(t32[0] != 0 && !gShouldStop);

        length = 8;
        t32[0] = length | (1 << 31);
        memset((void*)data, 0, 100);
        printf("---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
          t32[0], t32[1], t32[2]);
            printf("Querying status\n");
        while(t32[0] != 0 && !gShouldStop);
        printf("done---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
          t32[0], t32[1], t32[2]);
    }
    uint8_t* startScan = 0x100 + (uint8_t*)pruMem;
    uint32_t* log = (uint32_t*)( 0x800 + (uint8_t*)pruMem);
	memset(log, 0, 100);
	memset((void*)t32, 0, 0x1000);

	int dev = 2;
    uint8_t* pruBuffers[] = {(uint8_t*)t32 + 0x100 * dev, ((uint8_t*)t32) + 0x400 + 0x100 * dev};
    uint32_t* pruCommon = (uint32_t*)(((uint8_t*)t32) + 0x800);
	*pruCommon = 0; // initialize the area where common flags are written
    int lastPruBuffer = *pruCommon;
    int successful = 0;
    int corrupted = 0;
    int empty = 0;
    int outofrange = 0;
    int outoforder = 0;
    int ticks = 0;
    int verbose = 1;
	while(!gShouldStop){
        int pruBuffer = *pruCommon;
        if(lastPruBuffer == pruBuffer){
            rt_task_sleep(100000);
            continue;
        }
        lastPruBuffer = pruBuffer;
        data = pruBuffers[pruBuffer];
        if(verbose) rt_printf("b: %d, ", pruBuffer);
        uint32_t timestamp = 0;

		int devices = pruCommon[1];
		int numDevices = __builtin_popcount(devices);
		rt_printf("D: %1u %1u %1u %1u,", !!(devices & (1)), !!(devices & (1 << 1)), !!(devices & (1 << 2)), !!(devices & (1 << 3)));
		int receivedLength = data[0];
		int moreData = data[1];
		int frameType = data[2];
		int zero = data[3];
        uint32_t paddedLength = (receivedLength + 3) & ~3;
        uint32_t computedCrc = crc32_bitwise((void*)data, paddedLength >> 2);
		uint32_t receivedCrc;
		memcpy((void*)&receivedCrc, (void*)&data[paddedLength], 4);
        verbose = 1;
        if(receivedCrc != computedCrc){
            ++corrupted;
            if(verbose) rt_printf("l: %3d, wrong Crc:%#x %#x\n ", receivedLength, computedCrc, receivedCrc);
        } else {
            if(frameType == 19)
            {
                static int pastTimestamp = 0;
                timestamp = *(uint32_t*)&data[4];
                if(abs(pastTimestamp - timestamp) > 1000){ // out of order timestamp
                    outoforder++;
                    verbose = 1;
                    if(outoforder == 1){
                        //gShouldStop = 1;
                    }
                    rt_task_sleep(1000);
                } else if (get_key_position_raw(23) < 500) {
                    ++outofrange;
                } else {
                    ++successful;
                }
                pastTimestamp = timestamp;
            }
            if(frameType == 20){
                ++empty;
                if(verbose) rt_printf("empty");
            } else {
                if(verbose && 0) rt_printf("l: %3d, f: %3d, t: %#10x -- data: ", (uint32_t)receivedLength, frameType, timestamp);
                if(frameType == 19){
					for(int p = 256 * (2 - dev); p >= 0; p -= 256){
						if(!(devices & (1 << p/256))) continue;
						if(verbose) rt_printf(" || ");
						for(int n = p + 8; n < p + 58; n += 2){
							int16_t raw;
							memcpy(&raw, (void*)&(data[n]), 2);
							float value = raw/4096.f;
							value = value < 0 ? 0 : value;
							if (verbose) rt_printf("%5d ",  raw);
							//if (verbose) rt_printf("%1.0f ", value * 10);
						}
					}
                } else {
					rt_printf("ERRRPPPPPPPPPPPR: frame %d\n", frameType);
				}
            }
        }
        if (verbose) rt_printf("\n");
        ++ticks;
        if(ticks % 1000 == 0){
            if (verbose) rt_printf("\n\n------------------------------------successful: %d(%.3f%%), outofrange: %d(%.3f%%), corrupted: %d(%.3f%%), empty: %d(%.3f%%), outoforder: %d(%.3f%%), \n",
                successful, successful/(float)ticks * 100,
                outofrange, outofrange/(float)ticks * 100,
                corrupted, corrupted/(float)ticks * 100,
                empty, empty/(float)ticks * 100, 
                outoforder, outoforder/(float)ticks * 100
            );
        }
	}

    while(!gShouldStop){
        testGpio.clear();
        usleep(900);

		// startScan request
		if(ticks % 1 == 0 || 0){
			//Ask devices to start scan
			if(verbose)
				printf("Ask for a new scan\n");
			length = 5;
			memset((void*)data, 0, 400);
			data[0] = kBusCommandStartScan;
			memcpy((uint8_t*)&data[1], &ticks, 4); /* Timestamp goes in bytes 1-4 */
			t32[0] = length; 
			if(verbose)
				printf("---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
					t32[0], t32[1], t32[2]);
			while(t32[0] != 0 && !gShouldStop);
			if(verbose)
				printf("done---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
				t32[0], t32[1], t32[2]);

			usleep(0);
		}
        //retrieve results
        if(verbose)
            printf("Retrieve results\n");
        length = 255;
        memset((void*)data, 0, length);
        t32[0] =        //tell PRU to start transaction
            length |    // transaction is length bytes long
            (1 << 31) | // it is a receive
            (1 << 30);  // the first byte of the transmission contains the actual length of the transmission, so this can be safely terminated earlier if shorter than expected. A 4-byte CRC might follow actualLength bytes
        if(verbose)
            printf("---------\nlength: %d\n%#x %#x %#x\n-------------\n", length, 
              t32[0], t32[1], t32[2]);
        while(t32[0] != 0 && !gShouldStop);

        if(verbose)
            printf("done---------\nlength: %d\n%#x %#x %#x\n-------------\n", length, 
              t32[0], t32[1], t32[2]);
        if(ticks % 100 == 1)
        ;
		int kFrameTypeAnalog = 19;
		
		uint32_t receivedCrc;
		uint16_t receivedLength = data[0];
        //TODO: cap received length to requested length
		uint16_t alignedReceivedLength = (receivedLength + 3) & 0xFFFC;
		uint32_t computedCrc = crc32_bitwise((void*)data, alignedReceivedLength >> 2);
        computedCrc = 0x12345678;
		memcpy(&receivedCrc, (uint8_t*)&data[alignedReceivedLength], 4);
		int frameType = data[2];

		//static int count = 0;
		int print = 0;
        static int count = 0;
        static int successful = 0;
        static int outofrange = 0;
        static int corrupted = 0;
        static int voids = 0;

        if(count % 100 == 0 || 1) {
			printf("%3d, %d: ", (uint32_t)receivedLength, frameType);
            for(int n = 0; n < 20; ++n){
				char s;
				if(n == 0 || n == 12 || n == 24)
					s = 'C';
				else if(n == 2 || n == 14)
					s = 'D';
				else if(n == 4 || n == 16)
					s = 'E';
				else if(n == 5 || n == 17)
					s = 'F';
				else if(n == 7 || n == 19)
					s = 'G';
				else if(n == 9 || n == 21)
					s = 'A';
				else if(n == 11 || n == 23)
					s = 'B';
				else
					s = '#';
				s = '\0';
                printf("%c%5d  ", s, get_key_position_raw(n));
			}
            printf("\n");
        }
        if(count % 1000 == 0 && 0)
        {
            printf("successful: %d(%.3f%%), outofrange: %d(%.3f), corrupted: %d(%.3f%%), void: %d(%.3f%%), value: %d\n",
                successful, successful/(float)count * 100,
                outofrange, outofrange/(float)count * 100,
                corrupted, corrupted/(float)count* 100,
                voids, voids/(float)count* 100,
                get_key_position_raw(0)
            );
        }
        count++;

		if(receivedCrc != computedCrc){
			if(receivedLength != 255){
                ++corrupted;
				print = 0;
            }
            else {
                int isvoid = 1;
                for(int n = 0; n < receivedLength - 4; ++n){
                    if(data[n] != 0xff){
                        //print = 1;
                        isvoid = 0;
                        break;
                    }
                }
                if(isvoid)
                    ++voids;
                else {
                    ++corrupted;
					//print = 1;
				}
            }
		} else {
			if(get_key_position_raw(0) < 100)
				++outofrange;
			else
				++successful;
		}
        if(print)
		{
			printf("CRC, received: %#10x, computed: %#10x\n", receivedCrc, computedCrc);
        	// ++off;
            printf("Results(%d):\n", count);
            printf("Received length: %d (%d)\n", receivedLength, alignedReceivedLength);  
            printf("More data: %d\n", (uint32_t)data[1]);
            printf("Type: %#x %s\n", (uint32_t)frameType, frameType == kFrameTypeAnalog ? "correct" : "unknown");
            printf("Zero: %#x\n", (uint32_t)data[3]);
            printf("Timestamp: rec %10d, trans %10d\n", *(uint32_t*)&data[4], ticks);
            // get_key_position();
            for(int n = 8, j = 0; n < receivedLength; n += 2, j++){
                printf("[%2d]:%5d, ", j, (int)get_key_position_raw(j));
                if((n - 6) % 8 == 0)
                    printf("\n");
            }
            printf("\n\n");
			printf("Data:\n");
			for(int n = 0; n < alignedReceivedLength + 4; ++n){
				printf("%#x ", data[n]);
			}
			printf("\n\n");

            // printf("off/ticks: %.2f%\n", off/(float)ticks);
        }
	        // if(get_key_position(0) == 0){
	        // 	printf("get_key_position(0) at %d ticks\n", ticks);
	        // 	return 1;
	        // }
        ticks++;
    }


    rt_task_sleep(1000000);
    /* Disable PRU and close memory mapping*/
    prussdrv_pru_disable(PRU_NUM);
    prussdrv_exit ();
    usleep(100000);
    printf("\n------------------------------------successful: %d(%.3f%%), outofrange: %d(%.3f%%), corrupted: %d(%.3f%%), empty: %d(%.3f%%), outoforder: %d(%.3f%%), total: %d\n",
       successful, successful/(float)ticks * 100,
       outofrange, outofrange/(float)ticks * 100,
       corrupted, corrupted/(float)ticks * 100,
       empty, empty/(float)ticks * 100, 
       outoforder, outoforder/(float)ticks * 100,
       ticks
    );
    return(0);
}

