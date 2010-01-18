/* Name: main.c
 * Project: classic2usb, a Wii Classic Controller to USB adapter
 * Author: Torsten Stremlau <torsten@stremlau.de>
 * Creation Date: 2009-04-03
 * Tabsize: 4
 * This is largely based on the HID-Mouse example, so much of the code is
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 *
 * I also used code by Raphael Assenat <raph@raphnet.net>
 *
 * License: GNU GPL v2 (see License.txt), GNU GPL v3
 */

#define TW_SCL 100000// TWI frequency in Hz

#include "twi_speed.h"

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

#include "bit_tools.h"
#include "twi_func.h"

// #include "my_timers.h"

#define SLAVE_ADDR 0x52     /* address of classic controller and nunchuck */


/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

/*
 * This USB report descriptor is taken from Gamecube/N64 to USB converter by
 * Raphael Assenat <raph@raphnet.net>
 */
/* USB report descriptor, size must match usbconfig.h */
#ifdef WITH_ANALOG_L_R
PROGMEM char usbHidReportDescriptor[53] = {
#else
PROGMEM char usbHidReportDescriptor[49] = {
#endif
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Gamepad)
    0xa1, 0x01,                    // COLLECTION (Application)

    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)

    0x09, 0x33,                    //     USAGE (Rx)
    0x09, 0x34,                     //    USAGE (Ry)
#ifdef WITH_ANALOG_L_R
    0x09, 0x35,                     //    USAGE (Rz)
    0x09, 0x36,                     //    USAGE (Slider)
#endif
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,              //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
#ifdef WITH_ANALOG_L_R
    0x95, 0x06,                    //     REPORT_COUNT (6)
#else
    0x95, 0x04,                    //     REPORT_COUNT (4)
#endif
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION (Physical)

    0x05, 0x09,                    //   USAGE_PAGE (Button)
    0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
    0x29, 0x10,                    //   USAGE_MAXIMUM (Button 14)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x10,                    //   REPORT_COUNT (15)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

    0xc0                           // END_COLLECTION (Application)
};

typedef struct {
    uchar   x;
    uchar   y;
    // uint16_t debug;
    uchar   Rx;
    uchar   Ry;
#ifdef WITH_ANALOG_L_R
    uchar   leftTrig;
    uchar   rightTrig;
#endif
    uchar   buttons[2];
} report_t;


uchar rawData[6];

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */
// static uchar    startByte = 0;

/* Calibration values for the analog sticks and triggers */

#define INITIAL_XMAX 100
#define INITIAL_XMIN -100
#define INITIAL_YMAX 100
#define INITIAL_YMIN -100
#define INITIAL_RXMAX 100
#define INITIAL_RXMIN -100
#define INITIAL_RYMAX 100
#define INITIAL_RYMIN -100

typedef struct {
    signed char xMax;
    signed char xMin;
    signed char yMax;
    signed char yMin;
    signed char RxMax;
    signed char RxMin;
    signed char RyMax;
    signed char RyMin;
} calibration_t;

static calibration_t calibration;


/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */

/* I2C initialization */
void myI2CInit(void) {
    twi_init(); // this is a macro from "twi_speed.h"
}

// initialize Wii controller
unsigned char myWiiInit(void) {
    unsigned char buf[2] = {0x40, 0x00};

    return twi_send_data(SLAVE_ADDR, buf, 2);
}


unsigned char fillReportWithWii(void) {
    uchar i;
    unsigned char buf[6];

    /* send 0x00 to the controller to tell him we want data! */
    buf[0] = 0x00;
    
    if (!twi_send_data(SLAVE_ADDR, buf, 1)) {
        goto fend;
    }

    _delay_ms(2);
    // _delay_us(100);

    // ------ now get 6 bytes of data
    
    if (!(twi_receive_data(SLAVE_ADDR, buf, 6))) {
        goto fend;
    }

    // for (i = 0; i < 6; i++) {
    for (i = 0; i < 6; i++) {
        rawData[i] = (buf[i] ^ 0x17) + 0x17; // decrypt data
    }

    // _delay_ms(1);
    
    // 128 is center ??
    
    // FIXME: Do this calibration stuff with less duplicate code...
    //        Maybe don't use float math, its slow and big
    
    // calculation for x-axis
    signed char x = (((rawData[0] & 0x3F))<<2) - 128;
    if (x > calibration.xMax) calibration.xMax = x;
    if (x < calibration.xMin) calibration.xMin = x;
    
    if (x > 0) {
        reportBuffer.x = x * (127.0 / calibration.xMax) + 128;
    } else {
        reportBuffer.x = x * (128.0 / (-calibration.xMin)) + 128;
    }
    
    // calculation for y-axis
    signed char y = 0xff - (((rawData[1] & 0x3F))<<2) - 128;
    if (y > calibration.yMax) calibration.yMax = y;
    if (y < calibration.yMin) calibration.yMin = y;
    
    if (y > 0) {
        reportBuffer.y = y * (127.0 / calibration.yMax) + 128;
    } else {
        reportBuffer.y = y * (128.0 / (-calibration.yMin)) + 128;
    }
    
    // calculation for Rx-axis
    signed char Rx = (((((rawData[0] & 0xC0) >> 3) | ((rawData[1] & 0xC0) >> 5) | ((rawData[2] & 0x80) >> 7))) << 3) - 128;
    if (Rx > calibration.RxMax) calibration.RxMax = Rx;
    if (Rx < calibration.RxMin) calibration.RxMin = Rx;
    
    if (Rx > 0) {
        reportBuffer.Rx = Rx * (127.0 / calibration.RxMax) + 128;
    } else {
        reportBuffer.Rx = Rx * (128.0 / (-calibration.RxMin)) + 128;
    }
   
    // calculation for Ry-axis
    signed char Ry = (0xff - ((((rawData[2] & 0x1F))) << 3)) - 128;
    if (Ry > calibration.RyMax) calibration.RyMax = Ry;
    if (Ry < calibration.RyMin) calibration.RyMin = Ry;
    
    if (Ry > 0) {
        reportBuffer.Ry = Ry * (127.0 / calibration.RyMax) + 128;
    } else {
        reportBuffer.Ry = Ry * (128.0 / (-calibration.RyMin)) + 128;
    }
    
#ifdef WITH_ANALOG_L_R
    reportBuffer.leftTrig = (((rawData[2] & 0x60) >> 2) | ((rawData[3] & 0xE0) >> 5)) << 3;
    reportBuffer.rightTrig = (rawData[3] & 0x1F) << 3;
#else
    // reportBuffer.leftTrig = (1<<7);
    // reportBuffer.rightTrig = (1<<7);
#endif

    // reportBuffer.buttons1 = ~rawData[4];
    // reportBuffer.buttons2 = ~rawData[5];

    // split out buttons
    #define BTN_rT      GET_BIT(~rawData[4], 1)
    #define BTN_start   GET_BIT(~rawData[4], 2)
    #define BTN_home    GET_BIT(~rawData[4], 3)
    #define BTN_select  GET_BIT(~rawData[4], 4)
    #define BTN_lT      GET_BIT(~rawData[4], 5)
    #define BTN_down    GET_BIT(~rawData[4], 6)
    #define BTN_right   GET_BIT(~rawData[4], 7)
    #define BTN_up      GET_BIT(~rawData[5], 0)
    #define BTN_left    GET_BIT(~rawData[5], 1)
    #define BTN_rZ      GET_BIT(~rawData[5], 2)
    #define BTN_x       GET_BIT(~rawData[5], 3)
    #define BTN_a       GET_BIT(~rawData[5], 4)
    #define BTN_y       GET_BIT(~rawData[5], 5)
    #define BTN_b       GET_BIT(~rawData[5], 6)
    #define BTN_lZ      GET_BIT(~rawData[5], 7)

    // button mappings (button 0..15)
    #define BUTTON_X              0
    #define BUTTON_A              1
    #define BUTTON_B              2
    #define BUTTON_Y              3
    #define BUTTON_START          4
    #define BUTTON_SELECT         5
    #define BUTTON_HOME           6
    #define BUTTON_RIGHT_TRIGGER  7
    #define BUTTON_LEFT_TRIGGER   8
    #define BUTTON_RIGHT_Z        9
    #define BUTTON_LEFT_Z        10
    #define BUTTON_UP            11
    #define BUTTON_DOWN          12
    #define BUTTON_LEFT          13
    #define BUTTON_RIGHT         14
    #define NO_BUTTON            15

    #define SET_BUTTON(BUTTON, SOURCE) SET_BIT_VALUE(reportBuffer.buttons[BUTTON/8],BUTTON%8,SOURCE)

    SET_BUTTON(BUTTON_X, BTN_x);
    SET_BUTTON(BUTTON_A, BTN_a);
    SET_BUTTON(BUTTON_B, BTN_b);
    SET_BUTTON(BUTTON_Y, BTN_y);
    SET_BUTTON(BUTTON_START, BTN_start);
    SET_BUTTON(BUTTON_SELECT, BTN_select);
    SET_BUTTON(BUTTON_HOME, BTN_home);
    SET_BUTTON(BUTTON_RIGHT_TRIGGER, BTN_rT);
    SET_BUTTON(BUTTON_LEFT_TRIGGER, BTN_lT);
    SET_BUTTON(BUTTON_RIGHT_Z, BTN_rZ);
    SET_BUTTON(BUTTON_LEFT_Z, BTN_lZ);
    SET_BUTTON(BUTTON_UP, BTN_up);
    SET_BUTTON(BUTTON_DOWN, BTN_down);
    SET_BUTTON(BUTTON_LEFT, BTN_left);
    SET_BUTTON(BUTTON_RIGHT, BTN_right);
    SET_BUTTON(NO_BUTTON, 0);

    return 1;

    fend:
    // TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    _delay_us(20);


    all_fine:
    
    twi_stop();
    return 0;
}


/* This function sets up stuff */
void myInit(void) {
    // initialize calibration values
    calibration.xMax = INITIAL_XMAX;
    calibration.xMin = INITIAL_XMIN;
    calibration.yMax = INITIAL_YMAX;
    calibration.yMin = INITIAL_YMIN;
    calibration.RxMax = INITIAL_RXMAX;
    calibration.RxMin = INITIAL_RXMIN;
    calibration.RyMax = INITIAL_RYMAX;
    calibration.RyMin = INITIAL_RYMIN;
    
    _delay_ms(300);
    // SET_BIT(PORTC,0);
    myI2CInit();
    _delay_ms(120);
    myWiiInit();
    _delay_ms(1);
    fillReportWithWii();
}


/* ------------------------------------------------------------------------- */

int main(void)
{
    uchar   i;
    start:
    cli();
    wdt_enable(WDTO_2S);
    // wdt_disable();
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */

    SET_BIT(DDRC, 0);
    // SET_BIT(PORTC,0);
    // my_timer_oneshot(500, abc, 0);
    // my_timer_abort();

    odDebugInit();
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    myInit();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */

    for(;;){                /* main event loop */
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();
        usbPoll();
        if (fillReportWithWii() == 1) {
            SET_BIT(PORTC,0);
        } else {
            CLR_BIT(PORTC,0);
        }
        // TOGGLE_BIT(PORTC,0);
        // twi_stop();
        if(usbInterruptIsReady()){
            /* called after every poll of the interrupt endpoint */
            DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));

            /* If the gamepad starts feeding us 0xff, we have to restart to recover */
            if ((rawData[0] == 0xff) && (rawData[1] == 0xff) && (rawData[2] == 0xff) && (rawData[3] == 0xff) && (rawData[4] == 0xff) && (rawData[5] == 0xff)) {
                goto start;
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
