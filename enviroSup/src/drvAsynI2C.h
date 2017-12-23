/* drvAsynI2C.h
 *
 * EPICS asynPortDriver support for i2c devices under Linux
 * using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include "asynPortDriver.h"

class drvAsynI2C : public asynPortDriver {

public:
    drvAsynI2C(const char* portName, int i2cPortNum, int i2cAddr, int maxAddrIn,
               int interfaceMask, int interruptMask, int asynFlags,
               int autoConnect, int priority, int stackSize);

protected:
    int fd;
    int i2cPortNum;
    unsigned short i2cAddr;

    asynStatus i2c_connect(asynUser* pasynUser);
    asynStatus i2c_disconnect(asynUser* pasynUser);
    int i2c_wr_rd(unsigned char* tx, unsigned short tx_n, unsigned char* rx,
                  unsigned short rx_n);
};
