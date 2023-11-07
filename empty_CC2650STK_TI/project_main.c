/* C Standard library */
#include <stdio.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <string.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/mpu9250.h"

struct mpuMeasurement {
    float gx, gy, gz;
    float rx, ry, rz;
};

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// JTKJ: Teht�v� 3. Tilakoneen esittely
// JTKJ: Exercise 3. Definition of the state machine
enum state { WAITING=1, RECORD_MOTION, MOTION_IDENTIFIED, DATA_READY };
enum state programState = WAITING;

//global accelerometer TEMP
float ax, ay, az, gx, gy, gz;
UInt32 measurement_tick;

#define MEASUREMENTCOUNT 32

struct mpuMeasurement measurementBuffer[MEASUREMENTCOUNT];
UInt32 measurements = 0;

//handles
static PIN_Handle button1Handle;
static PIN_State button1State;
static PIN_Handle ledHandle;
static PIN_State ledState;

static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

static PIN_Config MpuPinConfig[] = {
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};



PIN_Config buttonConfig[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config ledConfig[] = {
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

//TODO button task

void buttonTaskFxn (UArg arg0, UArg arg1) {
    //setup button(s)

    while(1) {
        //reset measurement count to 0
    }
}

//TODO beep task

//TODO network task


/* Task Functions */
void uartTaskFxn(UArg arg0, UArg arg1) {
    UART_Handle uart;
    UART_Params uartParams;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode=UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    uart = UART_open(Board_UART0, &uartParams);

    char print_string[56];

    while (1) {
        if(programState == DATA_READY) {
            sprintf(print_string, "%012lu,%06.3f,%06.3f,%06.3f,%06.1f,%06.1f,%06.1f\n\r", measurement_tick, ax, ay, az, gx, gy, gz);
            //System_printf(print_string);
            UART_write(uart, print_string, 56);
            programState = WAITING;
        }
        Task_sleep(10000 / Clock_tickPeriod);
    }
}



void mpuTaskFxn(UArg arg0, UArg arg1) {
    I2C_Handle i2cMPU; // Own i2c-interface for MPU9250 sensor
    I2C_Params i2cMPUParams;

    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    //FOR TESTING
    programState = RECORD_MOTION;

    // Loop forever
    while (1) {
        if(programState == WAITING) {
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);
            measurement_tick = Clock_getTicks();

            programState = DATA_READY;
        }

        if(programState == RECORD_MOTION) {
            UInt motionIndex = wrapIndex(measurements);
            //collect motion measurement
            mpu9250_get_data(&i2cMPU,
                             &measurementBuffer[motionIndex].gx,
                             &measurementBuffer[motionIndex].gy,
                             &measurementBuffer[motionIndex].gz,
                             &measurementBuffer[motionIndex].rx,
                             &measurementBuffer[motionIndex].ry,
                             &measurementBuffer[motionIndex].rz
            );
            measurements++;

            //enough measurements gathered for detection
            if (measurements > 6) {
                //roll check
                if(measurementBuffer[motionIndex].rx > 200) {
                    Int offsetIndex = wrapIndex(motionIndex - 6);
                    if (measurementBuffer[offsetIndex].rx < -200) {
                        System_printf("Detected roll\n");
                    }
                }

                //front-back-wrist-motion check
                if(measurementBuffer[motionIndex].ry > 200) {
                    Int offsetIndex = wrapIndex(motionIndex - 6);
                    if (measurementBuffer[offsetIndex].ry < -200) {
                        System_printf("Detected forwards-wrist-motion\n");
                    }
                }

                if(measurementBuffer[motionIndex].gz > 1) {
                    Int offsetIndex = wrapIndex(motionIndex - 6);
                    if (measurementBuffer[offsetIndex].gz < -1.2) {
                        System_printf("Detected up-down\n");
                    }

                }
            }
        }

        // Wait for ???s
        Task_sleep(62500 / Clock_tickPeriod);
    }
}

//helper for index wrapping
Int wrapIndex (Int index) {
    index = index % MEASUREMENTCOUNT;
    if(index < 0) {
        index = index + MEASUREMENTCOUNT;
    }
    return index;
}


Int main(void) {

    // Task variables
    Task_Handle mpuTaskHandle;
    Task_Params mpuTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize board
    Board_initGeneral();
    Board_initI2C();
    Board_initUART();

    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
        System_abort("Pin open failed!");
    }


    /* Task initiation */
    Task_Params_init(&mpuTaskParams);
    mpuTaskParams.stackSize = STACKSIZE;
    mpuTaskParams.stack = &sensorTaskStack;
    mpuTaskParams.priority=2;
    mpuTaskHandle = Task_create(mpuTaskFxn, &mpuTaskParams, NULL);
    if (mpuTaskHandle == NULL) {
        System_abort("MPU task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
