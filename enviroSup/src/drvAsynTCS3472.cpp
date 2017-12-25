/* drvAsynTCS3472.cpp
 *
 * EPICS asynPortDriver for TCS3472 Color Light-to-Digital
 * Converter under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "drvAsynTCS3472.h"

static const char* driverName = "drvAsynTCS3472";

static void pollTask(void *drvPvt);

drvAsynTCS3472::drvAsynTCS3472(const char* portName, int i2cPortNum,
                               int i2cAddr)
    : drvAsynI2C(portName, i2cPortNum, i2cAddr,
		 1, /* maxAddr */
                    /* Interface mask */
                 (asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask
                  | asynDrvUserMask),
                    /* Interrupt mask */
		 (asynFloat64Mask | asynFloat64ArrayMask),
                 0, /* asynFlags (does not block and is not multi-device) */
                 1, /* Autoconnect */
                 0, /* Default priority */
                 0) /* Default stack size */
{
    const char* functionName = "drvAsynLSM303D";

    createParam(P_GainString, asynParamInt32, &P_Gain);
    createParam(P_LightString, asynParamFloat64, &P_Light);
    createParam(P_RedString, asynParamFloat64, &P_Red);
    createParam(P_GreenString, asynParamFloat64, &P_Green);
    createParam(P_BlueString, asynParamFloat64, &P_Blue);
    createParam(P_ColorString, asynParamFloat64Array, &P_Color);

    eventId_ = epicsEventCreate(epicsEventEmpty);
    if (epicsThreadCreate("drvAsynTCS3472Task", epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pollTask, this) == NULL) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    // Call our connect method to work-around the fact that autoconnect flag
    // triggers the base class to call connect before our constructor runs.
    this->connect(this->pasynUserSelf);
}

int drvAsynTCS3472::read_reg(unsigned char reg, unsigned char* value,
                             unsigned short len)
{
    unsigned char tx;

    tx = 0x80;  /* Command Register */
    tx |= 0x20; /* Auto Increment Addressing */
    tx |= reg;  /* Register address */

    return this->i2c_wr_rd(&tx, 1, value, len);
}

int drvAsynTCS3472::write_reg(unsigned char reg, unsigned char value)
{
    unsigned char tx[2];

    tx[0] = 0x80;  /* Command Register */
    tx[0] |= 0x20; /* Auto Increment Addressing */
    tx[0] |= reg;  /* Register address */

    tx[1] = value;

    return this->i2c_wr_rd(tx, 2, tx, 2);
}

asynStatus drvAsynTCS3472::connect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;

    /* Perform i2c connect */
    status = this->i2c_connect(pasynUser);
    if (status != asynSuccess) {
        return status;
    }

    /* Turn on TCS3472 conversions */
    if (this->write_reg(0x00, 0x03) != 0) {
        return asynError;
    }

    /* Set integration time to 511.2ms */
    int atime      = 256 - (int)(511.2 / 2.4);
    this->maxCount = (256 - atime) * 1024;
    if (this->maxCount > 65535) {
        this->maxCount = 65535;
    }
    if (this->write_reg(0x01, atime) != 0) {
        return asynError;
    }

    return status;
}

asynStatus drvAsynTCS3472::disconnect(asynUser* pasynUser)
{
    /* Perform i2c disconnect */
    return this->i2c_disconnect(pasynUser);
}

static void pollTask(void* drvPvt)
{
    drvAsynTCS3472* pPvt = (drvAsynTCS3472*)drvPvt;
    pPvt->pollTask();
}

void drvAsynTCS3472::pollTask(void)
{
    epicsTimeStamp now;
    epicsUInt32 delay_ns;
    int oneHz = 0;
    unsigned char vals[8];

    lock();
    while (1) {
        unlock();

        epicsTimeGetCurrent(&now);
        delay_ns = 50000000 - (now.nsec % 50000000); /* 20 Hz */
        epicsEventWaitWithTimeout(eventId_, delay_ns / 1.e9);
        oneHz = (now.nsec >= 750000000) && (now.nsec < 800000000);

        lock();

        if (oneHz) { /* 1 Hz */
            if (this->read_reg(0x14, vals, 8) == 0) {
                light = (vals[1] << 8) | vals[0];
                light = light / this->maxCount;
                setDoubleParam(P_Light, light);
                color[0] = (vals[3] << 8) | vals[2];
                color[0] = color[0] / this->maxCount;
                setDoubleParam(P_Red, color[0]);
                color[1] = (vals[5] << 8) | vals[4];
                color[1] = color[1] / this->maxCount;
                setDoubleParam(P_Green, color[1]);
                color[2] = (vals[7] << 8) | vals[6];
                color[2] = color[2] / this->maxCount;
                setDoubleParam(P_Blue, color[2]);

                updateTimeStamp();
                callParamCallbacks();
                doCallbacksFloat64Array(color, 3, P_Color, 0);
            }
        }
    }
}

asynStatus drvAsynTCS3472::writeInt32(asynUser* pasynUser, epicsInt32 value)
{
    int function      = pasynUser->reason;
    asynStatus status = asynSuccess;

    // Check if we're still connected
    if (this->fd < 0) {
        pasynManager->exceptionDisconnect(pasynUser);
        return asynDisconnected;
    }

    if (function == P_Gain) {
        unsigned char val = value & 0x03;
        if (this->write_reg(0x0f, val) != 0) {
            return asynError;
        }
    }

    return status;
}
extern "C" {

int drvAsynTCS3472Configure(const char* portName, int i2cPortNum, int i2cAddr)
{
    new drvAsynTCS3472(portName, i2cPortNum, i2cAddr);
    return (asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0          = { "portName", iocshArgString };
static const iocshArg initArg1          = { "i2c port num", iocshArgInt };
static const iocshArg initArg2          = { "i2c address", iocshArgInt };
static const iocshArg* const initArgs[] = { &initArg0, &initArg1, &initArg2 };
static const iocshFuncDef initFuncDef
    = { "drvAsynTCS3472Configure", 3, initArgs };

static void initCallFunc(const iocshArgBuf* args)
{
    drvAsynTCS3472Configure(args[0].sval, args[1].ival, args[2].ival);
}

void drvAsynTCS3472Register(void) { iocshRegister(&initFuncDef, initCallFunc); }

epicsExportRegistrar(drvAsynTCS3472Register);
}
