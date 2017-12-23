/* drvAsynI2C.cpp
 *
 * EPICS asynPortDriver support for i2c devices under Linux
 * using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drvAsynI2C.h"

static const char* driverName = "drvAsynI2C";

drvAsynI2C::drvAsynI2C(const char* portName, int i2cPortNum, int i2cAddr,
                       int maxAddrIn, int interfaceMask, int interruptMask,
                       int asynFlags, int autoConnect, int priority,
                       int stackSize)
    : asynPortDriver(portName, maxAddrIn, interfaceMask, interruptMask,
                     asynFlags, autoConnect, priority, stackSize)
{
    this->i2cPortNum = i2cPortNum;
    this->i2cAddr    = (unsigned short)i2cAddr;
    this->fd         = -1;
}

asynStatus drvAsynI2C::i2c_connect(asynUser* pasynUser)
{
    const char* functionName = "connect";
    asynStatus status        = asynSuccess;

    /* Open i2c port */
    char device[16];
    sprintf(device, "/dev/i2c-%d", this->i2cPortNum);
    this->fd = open(device, O_RDWR);
    if (this->fd < 0) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: unable to open %s for R/W\n", driverName,
                  functionName, device);
        return asynDisconnected;
    }

    /* Signal that we're connected */
    pasynManager->exceptionConnect(pasynUser);

    return status;
}

asynStatus drvAsynI2C::i2c_disconnect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;

    /* Close i2c port */
    if (close(this->fd) != 0) {
        return asynError;
    }
    this->fd = -1;

    /* Signal that we're disconnected */
    pasynManager->exceptionDisconnect(pasynUser);

    return status;
}

int drvAsynI2C::i2c_wr_rd(unsigned char* tx, unsigned short tx_n,
                          unsigned char* rx, unsigned short rx_n)
{
    struct i2c_msg rdwr_msg[] = { {.addr  = this->i2cAddr,
                                   .flags = 0, /* write */
                                   .len   = tx_n,
                                   .buf   = tx },
                                  {.addr  = this->i2cAddr,
                                   .flags = I2C_M_RD, /* read */
                                   .len   = rx_n,
                                   .buf   = rx } };

    struct i2c_rdwr_ioctl_data rdwr_data;
    rdwr_data.msgs  = rdwr_msg;
    rdwr_data.nmsgs = 2;

    if (ioctl(this->fd, I2C_RDWR, &rdwr_data) < 0) {
        return -1;
    }
    return 0;
}
