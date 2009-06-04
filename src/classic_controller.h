#ifndef CLASSIC_CONTROLLER_H
#define CLASSIC_CONTROLLER_H

/* !!!!!!!!!!!!!! FOR NOW THIS FILE IS USELESS !!!!!!!!!!!!!!!! */

typedef struct {
    uchar   x;
    uchar   y;
    uchar   Rx;
    uchar   Ry;
#ifdef WITH_ANALOG_L_R
    uchar   leftTrig;
    uchar   rightTrig;
#endif
    uchar   buttons[2];
} report_t;


/*
 * Description:
 *  This function uses the data to fill in the report.
 *
 * Parameters:
 *  report : pointer to the report to be filled
 *  data   : pointer to 6 bytes of data that came from the controller
 *
 * Returnvalue:
 *  0 in errorcase, else 1.
 */
unsigned char classic_controller_fill_report(report_t* report, unsigned char* data);


#endif
