#include "twi_func.h"

#include <avr/io.h>

unsigned char twi_send_data(unsigned char addr, unsigned char* data, unsigned char len) {
    unsigned char i;
    
    // enable TWI and send start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if (((TWSR & 0xf8) != 0x08) && ((TWSR & 0xf8) != 0x10)) {
        goto fend;
    }

    // enable master write mode
    TWDR = (addr<<1) + 0; // WRITE MODE
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    if ((TWSR & 0xf8) != 0x18) {
        goto fend;
    }

    // now send data
    for (i = 0; i < len; i++) {
        TWDR = data[i];
        TWCR = (1<<TWINT) | (1<<TWEN);

        // wait until it has been transmited
        while (!(TWCR & (1<<TWINT)));

        // check if data was acked by slave
        if ((TWSR & 0xf8) != 0x28) {
            goto fend;
        }
    }

    // send stop
    twi_stop();

    return 1;


    fend:
    // if an error occurs send stop and return 0
    twi_stop();
    return 0;
    
}

unsigned char twi_receive_data(unsigned char addr, unsigned char* data, unsigned char len) {
    unsigned char i;
    
    // enable TWI and send start condition
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if (((TWSR & 0xf8) != 0x08) && ((TWSR & 0xf8) != 0x10)) {
        goto fend;
    }

    TWDR = (addr << 1) + 1; // READ MODE
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wait until it has been transmited
    while (!(TWCR & (1<<TWINT)));

    // check if start was sent
    if ((TWSR & 0xf8) != 0x40) {
        goto fend;
    }

    // now get data
    for (i = 0; i < 6; i++) {

        // get data
        TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);

        // wait until it has been transmited
        while (!(TWCR & (1<<TWINT)));

        // check if data was received
        if ((TWSR & 0xf8) != 0x50) {
            goto fend;
        }

        data[i] = TWDR; // get data
    }

    twi_stop();

    return 1;

    fend:

    twi_stop();
    return 0;
}


void twi_stop(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}