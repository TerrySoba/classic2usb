/* Name: main.c
 * Project: classic2usb, a Wii Classic Controller to USB adapter
 * Author: Torsten Stremlau
 * Creation Date: 2009-04-03
 * Tabsize: 4
 * This is largely based on the HID-Mouse example, so much of the code is
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 *
 * I also used code by Raphael Assenat <raph@raphnet.net>
 *
 * License: GNU GPL v2 (see License.txt), GNU GPL v3
 */


#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */


#define SLAVE_ADDR 0x52     /* address of classic controller */


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
    uchar   Rx;
    uchar   Ry;
#ifdef WITH_ANALOG_L_R
    uchar   leftTrig;
    uchar   rightTrig;
#endif
    uchar   buttons1;
    uchar   buttons2;
} report_t;

uchar rawData[6];

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */
static uchar    startByte = 0;

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

#define SET_BIT(X,Y)  X |= (1 << (Y))
#define CLR_BIT(X,Y)  X &= (~(1 << (Y)))
#define GET_BIT(X,Y)  (((X) >> (Y)) & 1)
#define SET_BIT_VALUE(X,Y,V) if (V & 1) SET_BIT(X,Y); else CLR_BIT(X,Y);



/* I2C initialization */
void myI2CInit(void) {
    uchar error = 0;

    // TWBR = 255;
 
    TWBR = 72; // this equals 100kHz on I2C

    // set TWPS = 0;
    CLR_BIT(TWSR, 0);
    CLR_BIT(TWSR, 1);

}



char myWiiInit(void) {
    char i;
    // enable TWI and send start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if (((TWSR & 0xf8) != 0x08) && ((TWSR & 0xf8) != 0x10)) {
        goto fend;
    }

    TWDR = (SLAVE_ADDR<<1) + 0; // WRITE MODE
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    if ((TWSR & 0xf8) != 0x18) {
        goto fend;
    }

    //  -------------  now initialize WII Classic Controller
    TWDR = 0x40;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    if ((TWSR & 0xf8) != 0x28) {
        goto fend;
    }


    TWDR = 0x00;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    if ((TWSR & 0xf8) != 0x28) {
        goto fend;
    }

    // send stop
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

    return 0;

    fend:
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    return 1;
}


char fillReportWithWii(void) {
    uchar i;

    uchar rT, lT, rZ, lZ, up, down, left, right, start, select, home, a, b, x, y;

    // enable TWI and send start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if (((TWSR & 0xf8) != 0x08) && ((TWSR & 0xf8) != 0x10)) {
        goto fend;
    }

    TWDR = (SLAVE_ADDR<<1) + 0; // WRITE MODE
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    if ((TWSR & 0xf8) != 0x18) {
        goto fend;
    }

    // --    now tell the controller we want some data!

    TWDR = 0x00;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    if ((TWSR & 0xf8) != 0x28) {
        goto fend;
    }

    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

    _delay_ms(1);

    // ------ now get 6 bytes of data
    // enable TWI and send start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if (((TWSR & 0xf8) != 0x08) && ((TWSR & 0xf8) != 0x10)) {
        goto fend;
    }

    TWDR = (SLAVE_ADDR<<1) + 1; // READ MODE
    TWCR = (1<<TWINT) | (1<<TWEN) /*| (1<<TWEA)*/;

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if ((TWSR & 0xf8) != 0x40) {
        goto fend;
    }

    // send stop
    // TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

    // _delay_ms(1);

    for (i = 0; i < 6; i++) {

        // get data
        TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);

        // wait until it has been transmited
        while (!(TWCR & (1<<TWINT)));

        // check if data was received
        if ((TWSR & 0xf8) != 0x50) {
            goto fend;
        }

        rawData[i] = (TWDR ^ 0x17) + 0x17;
    }

    // send stop
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);

    _delay_ms(1);

    // now set structure
    reportBuffer.x = ((rawData[0] & 0x3F))<<2;
    reportBuffer.y = 0xff - (((rawData[1] & 0x3F))<<2);
    reportBuffer.Rx = ((((rawData[0] & 0xC0) >> 3) | ((rawData[1] & 0xC0) >> 5) | ((rawData[2] & 0x80) >> 7))) << 3;
    reportBuffer.Ry = 0xff - ((((rawData[2] & 0x1F))) << 3);

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
    rT       = GET_BIT(~rawData[4], 1);
    start    = GET_BIT(~rawData[4], 2);
    home     = GET_BIT(~rawData[4], 3);
    select   = GET_BIT(~rawData[4], 4);
    lT       = GET_BIT(~rawData[4], 5);
    down     = GET_BIT(~rawData[4], 6);
    right    = GET_BIT(~rawData[4], 7);
    up       = GET_BIT(~rawData[5], 0);
    left     = GET_BIT(~rawData[5], 1);
    rZ       = GET_BIT(~rawData[5], 2);
    x        = GET_BIT(~rawData[5], 3);
    a        = GET_BIT(~rawData[5], 4);
    y        = GET_BIT(~rawData[5], 5);
    b        = GET_BIT(~rawData[5], 6);
    lZ       = GET_BIT(~rawData[5], 7);

    SET_BIT_VALUE(reportBuffer.buttons1,0,x);
    SET_BIT_VALUE(reportBuffer.buttons1,1,a);
    SET_BIT_VALUE(reportBuffer.buttons1,2,b);
    SET_BIT_VALUE(reportBuffer.buttons1,3,y);
    SET_BIT_VALUE(reportBuffer.buttons1,4,start);
    SET_BIT_VALUE(reportBuffer.buttons1,5,select);
    SET_BIT_VALUE(reportBuffer.buttons1,6,home);
    SET_BIT_VALUE(reportBuffer.buttons1,7,rT);
    
    SET_BIT_VALUE(reportBuffer.buttons2,0,lT);
    SET_BIT_VALUE(reportBuffer.buttons2,1,rZ);
    SET_BIT_VALUE(reportBuffer.buttons2,2,lZ);
    SET_BIT_VALUE(reportBuffer.buttons2,3,up);
    SET_BIT_VALUE(reportBuffer.buttons2,4,down);
    SET_BIT_VALUE(reportBuffer.buttons2,5,left);
    SET_BIT_VALUE(reportBuffer.buttons2,6,right);
    SET_BIT_VALUE(reportBuffer.buttons2,7,0);

    return 0;

    fend:
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
    return 1;
}


/* This function sets up stuff */
void myInit(void) {
    int i;
    _delay_ms(300);
    SET_BIT(DDRC, 0);
    myI2CInit();
    _delay_ms(20);
    myWiiInit();
    _delay_ms(1);
    fillReportWithWii();
}


/* ------------------------------------------------------------------------- */

int main(void)
{
    uchar   i;
    start:
    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */

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
        if (fillReportWithWii() == 0) {
            SET_BIT(PORTC,0);
        } else {
            CLR_BIT(PORTC,0);
        }

        if(usbInterruptIsReady()){
            /* called after every poll of the interrupt endpoint */
            DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));

            /* If the Gamepad starts feeding us 0xff, we heve to restart to recover */
            if ((rawData[0] == 0xff) && (rawData[1] == 0xff) && (rawData[2] == 0xff) && (rawData[3] == 0xff) && (rawData[4] == 0xff) && (rawData[5] == 0xff)) {
                goto start;
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
