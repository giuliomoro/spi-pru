.origin 0
.entrypoint START

#define PRU_NUMBER 0

#define CLOCK_BASE  0x44E00000
#define CLOCK_SPI0  0x4C
#define CLOCK_L4LS  0x60
#define SPI0_BASE   0x48030100
#define SPI_BASE    SPI0_BASE

#define SPI_SYSCONFIG 0x10
#define SPI_SYSSTATUS 0x14
#define SPI_MODULCTRL 0x28
#define SPI_CH0CONF   0x2C
#define SPI_CH0STAT   0x30
#define SPI_CH0CTRL   0x34
#define SPI_CH0TX     0x38
#define SPI_CH0RX     0x3C

#define GPIO0 0x44E07000
#define GPIO1 0x4804C000
#define GPIO2 0x481AC000
#define GPIO3 0x481AE000
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194

#define SPICH0_TRM       0       // SPI transmit and receive
#define SPICH0_WL        8       // Word length
#define SPICH0_WL_BYTES  (SPICH0_WL >> 3) // Word length in bytes
#define SPICH0_CLK_MODE  0       // SPI mode
#define SPICH0_CLK_DIV   2      // Clock divider (48MHz / 2^n)
#define SPICH0_DPE       1       // d0 = receive, d1 = transmit
#define SPICH0_CS_GPIO      GPIO1
#define SPICH0_CS_PIN    (1<<17) // GPIO1:17 = P9 pin 23
#define SPICH0_RW_GPIO   GPIO1
#define SPICH0_RW_PIN    (1<<16) // GPIO1:16 = P9 pin 15

// miso: spi0_d0 P9_21
// mosi: spi0_d1 P9_18
// clk: spi0_sclk P9_22

#define DO_SPI

#define reg_spi_addr r29
#define reg_start_scan r26
#define reg_device r25 
#define reg_num_devices r24
#define reg_scans_since_last_start_scan r23
#define reg_ticks r22
#define reg_transmission_length r21
#define reg_current_output_buffer r20
#define reg_log r19
#define reg_isvoid r18
#define reg_outofrange r17

#define reg_transmitted_words r5
#define reg_curr_word r6

#define TICKS_PER_START_SCAN 5
#define START_SCAN_POSITION 0x220
#define CYCLES_PER_TICK (200000000/1000)




#define CONST_PRUCFG          C4
#define CONST_PRUDRAM        C24
#define CONST_PRUSHAREDRAM   C28
#define CONST_DDR            C31

// Address for the Constant table Block Index Register (CTBIR)
#define CTBIR          0x22020

// Address for the Constant table Programmable Pointer Register 0(CTPPR_0)
#define CTPPR_0         0x22028

// Address for the Constant table Programmable Pointer Register 1(CTPPR_1)
#define CTPPR_1         0x2202C

.macro  LD32
.mparam dst,src
    LBBO    dst,src,#0x00,4
.endm

.macro  LD16
.mparam dst,src
    LBBO    dst,src,#0x00,2
.endm

.macro  LD8
.mparam dst,src
    LBBO    dst,src,#0x00,1
.endm

.macro ST32
.mparam src,dst
    SBBO    src,dst,#0x00,4
.endm

.macro ST16
.mparam src,dst
    SBBO    src,dst,#0x00,2
.endm

.macro ST8
.mparam src,dst
    SBBO    src,dst,#0x00,1
.endm

.macro CLEAR_GPIO
.mparam gpio_address, gpio_pins
     MOV r27, gpio_pins
     MOV r28, gpio_address + GPIO_CLEARDATAOUT
     SBBO r27, r28, 0, 4
.endm

.macro SET_GPIO
.mparam gpio_address, gpio_pins
     MOV r27, gpio_pins
     MOV r28, gpio_address + GPIO_SETDATAOUT
     SBBO r27, r28, 0, 4
.endm

// Bring CS line low to write to SPI
.macro SPICH0_CS_ASSERT
     MOV r27, SPICH0_CS_PIN
     MOV r28, SPICH0_CS_GPIO + GPIO_CLEARDATAOUT
     SBBO r27, r28, 0, 4
.endm

// Bring CS line high at end of SPICH0 transaction
.macro SPICH0_CS_UNASSERT
     MOV r27, SPICH0_CS_PIN
     MOV r28, SPICH0_CS_GPIO + GPIO_SETDATAOUT
     SBBO r27, r28, 0, 4
.endm

// Write to SPICH0 TX register
.macro SPICH0_TX
.mparam data
     SBBO data, reg_spi_addr, SPI_CH0TX, 4
.endm

// Wait for SPI to finish (uses RXS indicator)
.macro SPICH0_WAIT_FOR_FINISH
 LOOP:
     LBBO r27, reg_spi_addr, SPI_CH0STAT, 4
     QBBC LOOP, r27, 0
.endm

// Read the RX word to clear; store output
.macro SPICH0_RX
.mparam data
     LBBO data, reg_spi_addr, SPI_CH0RX, 4
.endm

// Complete SPICH0 write+read with chip select
.macro SPICH0_WRITE
.mparam in, out
     SPICH0_CS_ASSERT
     SPICH0_TX in
     SPICH0_WAIT_FOR_FINISH
     SPICH0_RX out
     SPICH0_CS_UNASSERT
.endm

// DELAY: Wait (busy loop) for a specified time
// Parameters:
//   count: how long to wait, in 10ns increments
//          this macro also adds a constant 10ns at the beginning 
// Uses registers: r27
.macro DELAY
.mparam count
    MOV r27, count
DELAY_LOOP:
    SUB r27, r27, 1
    QBNE DELAY_LOOP, r27, 0
.endm

.macro BUS_MODE_MASTER_TRANSMITTER
.mparam slaveDevice, buffer, transmitLength
    PREPARE_TRANSMIT
    MOV reg_transmitted_words, 0 // reg_transmitted_words counts how many words we transmitted
    // empty the register, so that words shorter than 32bits find it empty
    MOV reg_curr_word, 0
    // preload one word
    LBBO reg_curr_word, buffer, reg_transmitted_words, SPICH0_WL_BYTES
WRITE_BUFFER_LOOP:
#ifdef DO_SPI
    SPICH0_TX reg_curr_word
#endif
    // before waiting, we preload the next word (if any)
    QBGE WRITE_BUFFER_LOOP_PRELOAD_DONE, transmitLength, reg_transmitted_words
    ADD reg_transmitted_words, reg_transmitted_words, SPICH0_WL_BYTES
    LBBO reg_curr_word, buffer, reg_transmitted_words, SPICH0_WL_BYTES
    WRITE_BUFFER_LOOP_PRELOAD_DONE:
#ifdef DO_SPI
    SPICH0_WAIT_FOR_FINISH
    SPICH0_RX r10
#endif
    QBLT WRITE_BUFFER_LOOP, transmitLength, reg_transmitted_words
    
    FINISH_TRANSACTION
    DELAY 100
.endm

.macro BUS_MODE_MASTER_RECEIVER
.mparam slaveDevice, buffer, transmitLength, dynamicLengthLocation
    /* Set RW* line high for receive */
    SET_GPIO SPICH0_RW_GPIO, SPICH0_RW_PIN
    /* Set relevant CS/ line(s) low */
    DELAY 1000
    CLEAR_GPIO SPICH0_CS_GPIO, SPICH0_CS_PIN
    /* Short delay, ~2us, to let slave device prepare */
    DELAY 1000
    MOV reg_transmitted_words, 0 // reg_transmitted_words counts how many words we transmitted
    // empty the register, so that words shorter than 32bits find it empty
    MOV reg_curr_word, 0
    // preload one word
    LBBO reg_curr_word, buffer, reg_transmitted_words, SPICH0_WL_BYTES
WRITE_BUFFER_LOOP:
#ifdef DO_SPI
    MOV r28, 0 // we are actually receiving only, so we send out zeros.
    SPICH0_TX r28
#endif
    // before waiting, we preload the next word (if any)
    QBGE WRITE_BUFFER_LOOP_PRELOAD_DONE, transmitLength, reg_transmitted_words
    ADD reg_transmitted_words, reg_transmitted_words, SPICH0_WL_BYTES
    LBBO reg_curr_word, buffer, reg_transmitted_words, SPICH0_WL_BYTES
    WRITE_BUFFER_LOOP_PRELOAD_DONE:
#ifdef DO_SPI
    SPICH0_WAIT_FOR_FINISH
    SPICH0_RX reg_curr_word
    SUB r27, reg_transmitted_words, SPICH0_WL_BYTES
    SBBO reg_curr_word, buffer, r27, SPICH0_WL_BYTES
#endif
    QBNE CHECK_DYNAMIC_LENGTH_DONE, reg_transmitted_words, dynamicLengthLocation
    // if we are using dynamic length, then here we have just received it,
    QBEQ ABORT, reg_curr_word, 0xFF // skip if slave is inactive
    // so replace existing length with it after:
    // a) extend it to a multiple of 4 and adding + 4 for CRC,
    // that is add 7 ...
    ADD reg_curr_word, reg_curr_word, 7
    // ... and clear  lower two bits
    CLR reg_curr_word, 0
    CLR reg_curr_word, 1 
    // b) and capping it anyhow with the requested transmitLength
    MIN transmitLength, reg_curr_word, transmitLength

    QBNE CHECK_DYNAMIC_LENGTH_DONE, transmitLength, 8
    // debug here


CHECK_DYNAMIC_LENGTH_DONE:
    QBLT WRITE_BUFFER_LOOP, transmitLength, reg_transmitted_words
    QBA RECEIVE_DONE
ABORT:
    ADD reg_isvoid, reg_isvoid, 1
    FINISH_TRANSACTION
    QBA RECEIVE_VALIDATION_DONE
RECEIVE_DONE:
    FINISH_TRANSACTION
    //MOV r27, 0
    //LBBO r27, buffer, 12, 2
    //SBBO r27, reg_log, 12, 4
    //QBGT RECEIVE_VALIDATION_DONE, r27, 255
    //ADD reg_outofrange, reg_outofrange, 1
RECEIVE_VALIDATION_DONE:
    DELAY 100
.endm

.macro SIGNAL_ARM_OVER
    // reset word count to 0
    MOV r27, 0 
    SBCO r27, CONST_PRUDRAM, 0, 4
.endm

.macro GET_CYCLE_COUNTER
.mparam out
    MOV r27, 0x22000      // PRU0 control register offset
    LBBO out, r27, 0x000C, 4
.endm

.macro CLEAR_CYCLE_COUNTER
    MOV r27, 0x22000      // PRU0 control register offset
    MOV r28, 0
    SBBO r28, r27, 0x000C, 4
.endm

.macro ENABLE_CYCLE_COUNTER
    MOV r28, 0x22000      // PRU0 control register offset
    // Load content of the control register into r27
    LBBO r27, r28, 0, 4
    // Enable cycle counter
    OR r27, r27, 1 << 3
    // Store the new control register value
    SBBO r27, r28, 0, 4
.endm

.macro PREPARE_TRANSMIT
    /* Set RW* line low for transmit */
    CLEAR_GPIO SPICH0_RW_GPIO, SPICH0_RW_PIN
    /* Set relevant CS/ line(s) low */
    DELAY 1
    CLEAR_GPIO SPICH0_CS_GPIO, SPICH0_CS_PIN
    /* Short delay, ~2us, to let slave device prepare */
    DELAY 300
.endm

// This is the same for TRANSMITTER or RECEIVER
.macro FINISH_TRANSACTION
    DELAY 100
    /* Set relevant CS/ line(s) high */
    SET_GPIO SPICH0_CS_GPIO, SPICH0_CS_PIN
.endm

.macro WAIT_FOR_TICK
WAIT_FOR_TICK_LOOP:
    GET_CYCLE_COUNTER r27
    MOV r28, CYCLES_PER_TICK
    QBGE WAIT_FOR_TICK_LOOP, r27, r28
    CLEAR_CYCLE_COUNTER
.endm

.macro SPICH0_TX_BLOCK
.mparam input, output
    SPICH0_TX input
    SPICH0_WAIT_FOR_FINISH
    SPICH0_RX output
.endm

.macro START_SCAN
    // set the appropriate command in the buffer
    // TODO: do this only once at the beginning
    #define kBusCommandStartScan 0x81
    MOV r27, 0
    MOV r28, 0
    SBBO r27, reg_start_scan, 0, 8
    MOV r27, kBusCommandStartScan
    SBBO r27, reg_start_scan, 0, 1

    // store the timestamp in memory
    SBBO reg_ticks, reg_start_scan, 1, 4

    // add simple error check for the timestamp:
    // append a 32-bit aligned 32-bit word
    // obtained adding together the timestamp
    // and 0x55555555
    MOV r27, 0
    MOV r28, 0x55555555
    ADD r28, reg_ticks, r28
    SBBO reg_ticks, reg_start_scan, 1, 4
    // ensure 32-bit alignment
    SBBO r27, reg_start_scan, 5, 3
    // add the error check
    SBBO r28, reg_start_scan, 8, 4

    MOV reg_transmission_length, 12 // length

    // we tried to unroll the loop here to make it faster
    // and so we avoid to go back and forth from memory
    // in practice it is not faster, which means the preload works nicely :)
    /*
    PREPARE_TRANSMIT
    MOV r28, kBusCommandStartScan
    SPICH0_TX_BLOCK r28.b0, r27
    SPICH0_TX_BLOCK reg_ticks.b0, r27
    SPICH0_TX_BLOCK reg_ticks.b1, r27
    SPICH0_TX_BLOCK reg_ticks.b2, r27
    SPICH0_TX_BLOCK reg_ticks.b3, r27
    FINISH_TRANSACTION
    */

    BUS_MODE_MASTER_TRANSMITTER reg_device, reg_start_scan, reg_transmission_length
.endm

#define FIRST_BUFFER 0
#define SECOND_BUFFER 0x100
#define CURRENT_BUFFER_ADDRESS 0x200

.macro BUS_MASTER_GATHER_SCAN_RESULTS

    // switch buffers over 
    MOV r27, SECOND_BUFFER
    QBNE SELECT_SECOND_BUFFER, reg_current_output_buffer, r27
SELECT_FIRST_BUFFER:
    //SET_GPIO GPIO2, 1 << 2
    MOV reg_current_output_buffer, FIRST_BUFFER
    MOV r28, 1
    QBA SELECT_BUFFER_DONE
SELECT_SECOND_BUFFER:
    //CLEAR_GPIO GPIO2, 1 << 2
    MOV reg_current_output_buffer, SECOND_BUFFER
    MOV r28, 0
    QBA SELECT_BUFFER_DONE
SELECT_BUFFER_DONE:
    MOV r27, CURRENT_BUFFER_ADDRESS
    SBBO r28, r27, 0, 4 // signal ARM so that it knows where to read from

    MOV reg_transmission_length, 255
    BUS_MODE_MASTER_RECEIVER reg_device, reg_current_output_buffer, reg_transmission_length, 1
.endm

.macro MASTER_MODE
// every millisecond scan slaves
// keep internal count of ticks
// every so often hand this over to ARM
// signal ARM of buffer ready
// CRC check ? ??
    // initialize reg_start_scan with a pointer to where the start_scan command is stored
    MOV reg_start_scan, START_SCAN_POSITION
    ENABLE_CYCLE_COUNTER
    CLEAR_CYCLE_COUNTER
    MOV r0, 0
    MOV reg_scans_since_last_start_scan, TICKS_PER_START_SCAN
    MOV reg_ticks, 0
    MOV reg_isvoid, 0
    MOV reg_outofrange, 0
    MOV reg_log, 0x300
    MOV r10, 0
    // send a command to the devices to start scanning

MASTER_LOOP:

    // Check whether we have done enough scans since the last time we issued a start_scan
    QBGT START_SCAN_DONE, reg_scans_since_last_start_scan, TICKS_PER_START_SCAN
    // if we did, then issue start_scan again
    START_SCAN

    // and reset the counter
    MOV reg_scans_since_last_start_scan, 0
    DELAY 400 // wait for devices to be ready before continuing
START_SCAN_DONE:
    // ask devices to return their data
    BUS_MASTER_GATHER_SCAN_RESULTS

    ADD reg_scans_since_last_start_scan, reg_scans_since_last_start_scan, 1

    // store logging in memory
    //MOV r27, reg_ticks
    //MOV r28, reg_isvoid
    //SBBO r27, reg_log, 0, 8
    //SBBO reg_outofrange, reg_log, 8, 4

    // wait for 1ms to expire
    WAIT_FOR_TICK
    ADD reg_ticks, reg_ticks, 1
    QBA MASTER_LOOP
.endm

START:
    //start scan
    //while sleep 1
    //  gatherScanResults
    //
    
    // Find out which PRU we are running on
    // This affects the following offsets
    MOV  r0, 0x24000      // PRU1 control register offset
    //LBBO r2, reg_comm_addr, COMM_PRU_NUMBER, 4
    MOV r2, PRU_NUMBER
    QBEQ PRU_NUMBER_CHECK_DONE, r2, 1
    MOV  r0, 0x22000      // PRU0 control register offset

PRU_NUMBER_CHECK_DONE:

    // Set up c24 and c25 offsets with CTBIR register
    // Thus C24 points to start of PRU0 RAM
    OR  r3, r0, 0x20      // CTBIR0
    MOV r2, 0
    SBBO r2, r3, 0, 4

    // Enable OCP master port
    LBCO      r0, C4, 4, 4
    CLR     r0, r0, 4   // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
    SBCO      r0, C4, 4, 4

    // Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
    // field to 0x0120.  This will make C28 point to 0x00012000 (PRU shared RAM).
    MOV     r0, 0x00000120
    MOV       r1, CTPPR_0
    ST32      r0, r1

    // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
    // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
    MOV     r0, 0x00100000
    MOV       r1, CTPPR_1
    ST32      r0, r1


    MOV reg_spi_addr, SPI_BASE
    // Init SPI clock
    MOV r2, 0x02
    MOV r3, CLOCK_BASE + CLOCK_SPI0
    SBBO r2, r3, 0, 4
    // Reset SPI and wait for finish
    MOV r2, 0x02
    SBBO r2, reg_spi_addr, SPI_SYSCONFIG, 4


SPI_WAIT_RESET:
    LBBO r2, reg_spi_addr, SPI_SYSSTATUS, 4
    QBBC SPI_WAIT_RESET, r2, 0

    // Turn off SPI channel
    MOV r2, 0
    SBBO r2, reg_spi_addr, SPI_CH0CTRL, 4

    // Set to master; chip select lines enabled (CS0 used for DAC)
    MOV r2, 0x00
    SBBO r2, reg_spi_addr, SPI_MODULCTRL, 4

    // Configure CH0 for SPICH0
    MOV r2, (3 << 27) | (SPICH0_DPE << 16) | (SPICH0_TRM << 12) | ((SPICH0_WL - 1) << 7) | (SPICH0_CLK_DIV << 2) | SPICH0_CLK_MODE
    SBBO r2, reg_spi_addr, SPI_CH0CONF, 4

    // bring up the CS and RW lines
    //SET_GPIO SPICH0_RW_GPIO, SPICH0_RW_PIN
    SET_GPIO SPICH0_CS_GPIO, SPICH0_CS_PIN

    // Turn on SPI channels
    MOV r2, 0x01
    SBBO r2, reg_spi_addr, SPI_CH0CTRL, 4

    MOV reg_device, 0
    MOV reg_num_devices, 1
    MASTER_MODE
WAIT_FOR_ARM:
    // the loader will have placed the number of words
    // to transmit into CONST_PRUDRAM
    // load this in r1
    //0x0032
    LBCO r1, CONST_PRUDRAM, 0, 4
    // if there is no word to send, wait again
    QBEQ WAIT_FOR_ARM, r1, 0

    // the payload is then at address CONST_PRUDRAM + 4
    MOV r2, 0x0000000 // CONST_PRUDRAM
    ADD r2, r2, 4

    // the length will have b31 clear if it is a transmit
    // and set if it is a receive 
    //0x0036
    QBBC TRANSMIT, r1, 31
RECEIVE:
    // the length will have b30 set if the provided length
    // is an upper limit and actual length is in the first
    // received byte
    QBBC RECEIVE_FIXED_LENGTH, r1, 30
    MOV r3, 1
    QBA DO_RECEIVE
RECEIVE_FIXED_LENGTH:
    MOV r3, 0
DO_RECEIVE:
    // mask out bit flags (upper 16 bits)
    LSL r1, r1, 16
    LSR r1, r1, 16
    BUS_MODE_MASTER_RECEIVER reg_device, r2, r1, r3
    QBA COMMUNICATION_DONE
TRANSMIT: 
    BUS_MODE_MASTER_TRANSMITTER reg_device, r2, r1
COMMUNICATION_DONE:
    // either way, signal ARM that the communication is over
    SIGNAL_ARM_OVER
    QBA WAIT_FOR_ARM
