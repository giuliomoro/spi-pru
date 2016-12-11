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

// Standard header files
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>

// Driver header file
#include "prussdrv.h"
#include "pruss_intc_mapping.h"

#define PRU_NUM 	 0
#define ADDEND1	 	 0x98765400u
#define ADDEND2		 0x12345678u
#define ADDEND3		 0x10210210u

#define DDR_BASEADDR     0x80000000
#define OFFSET_DDR	 0x00001000
#define OFFSET_SHAREDRAM 2048		//equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4

#include <GPIOcontrol.h>
#include <digital_gpio_mapping.h>

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

static int mem_fd;
static void *ddrMem, *sharedMem;

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

int main (void)
{
    if(prepareGPIO(1)){
	}
	unsigned int ret;
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
    volatile uint8_t* data = (uint8_t*)&t32[1];
    // disable transmisssions before starting the PRU
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
    printf("\tINFO: Executing example.\r\n");
    if(prussdrv_exec_program (PRU_NUM, "/root/spi-pru/spi-pru.bin")){
        fprintf(stderr, "Failed loading program\n");
    }

    if(0){
        int h = 0;
        int c[3] = {0, 1500, 3000};
        float in = 10;
        int inc[3] = {in, in, in};
        for(int n = 0; n < 100; ++n){
            memset(pruMem, 0, 100);
            data[0] = kBusCommandRGBLEDSetColors;
            data[1] = numLeds;
            int p = 0;
            for(int n = 0; n < numLeds; ++n){
                uint8_t led = n;
                uint16_t red = c[0]; // h > 1000 ? 0 : 4095;
                uint16_t green = c[1]; // h > 2000 ? 0 : 4095;
                uint16_t blue = c[2]; //h > 3000 ? 0 : 4095;
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
            printf("%10d---------\nlength: %d\n%#x %#x %#x\n-------------\n", n, length, 
              t32[0], t32[1], t32[2]);
            usleep(10000);
        }
    }

    while(t32[0] != 0){
        printf("waiting for previous tasks to complete\n");
    }
    printf("done\n");
    if(0){ // query connected devices
        data[0] = kBusCommandStatus;
        length = 1;
        t32[0] = length;
        do {
            printf("Delivering the kBusCommandStatus\n");
        } while(t32[0] != 0);

        length = 8;
        t32[0] = length | (1 << 31);
        memset((void*)data, 0, 100);
        printf("---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
          t32[0], t32[1], t32[2]);
            printf("Querying status\n");
        while(t32[0] != 0);
        printf("done---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
          t32[0], t32[1], t32[2]);
    }
    uint32_t ticks = 0;
    int verbose = 0;
    while(1){
        usleep(1000);

        //Ask devices to start scan
        if(verbose)
            printf("Ask for a new scan\n");
        length = 5;
        memset((void*)data, 0, 100);
        data[0] = kBusCommandStartScan;
        memcpy((uint8_t*)&data[1], &ticks, 4); /* Timestamp goes in bytes 1-4 */
        t32[0] = length; 
        if(verbose)
            printf("---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
              t32[0], t32[1], t32[2]);
        while(t32[0] != 0);
        if(verbose)
            printf("done---------\nlength: %#x\n%#x %#x %#x\n-------------\n", length, 
              t32[0], t32[1], t32[2]);

        usleep(100);
        //retrieve results
        if(verbose)
            printf("Retrieve results\n");
        length = 56;
        memset((void*)data, 0, 100);
        t32[0] = length | (1 << 31);
        if(verbose)
            printf("---------\nlength: %#d\n%#x %#x %#x\n-------------\n", length, 
              t32[0], t32[1], t32[2]);
        while(t32[0] != 0);
        if(verbose)
            printf("done---------\nlength: %#d\n%#x %#x %#x\n-------------\n", length, 
              t32[0], t32[1], t32[2]);
        if(ticks % 100 == 1){
            printf("Results:\n");
            printf("Received length: %d\n", (uint32_t)data[0]);  
            printf("More data: %d\n", (uint32_t)data[1]);
            int kFrameTypeAnalog = 19;
            printf("Type: %#x %s\n", (uint32_t)data[2], data[2] == kFrameTypeAnalog ? "correct" : "unknown");
            printf("Zero: %#x\n", (uint32_t)data[3]);
            printf("Timestamp: %10d\n", *(uint32_t*)&data[4]);
            for(int n = 8; n < length; n += 2){
                int value = (*(int16_t*)(&data[n]));
                if(value < 0)
                    value = 0;
                printf("%6.3f ", (float)value / 4096.f);
                if((n - 6) % 8 == 0)
                    printf("\n");
            }
            printf("\n\n");
        }
        ticks++;
    }

    return 0;

    /* Disable PRU and close memory mapping*/
    prussdrv_pru_disable(PRU_NUM);
    prussdrv_exit ();
    munmap(ddrMem, 0x0FFFFFFF);
    close(mem_fd);

    return(0);
}

