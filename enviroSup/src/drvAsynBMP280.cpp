/* drvAsynBMP280.cpp
 *
 * EPICS asynPortDriver for BMP280 Digital Temperature
 * and Pressure Sensor under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "drvAsynBMP280.h"

static const char* driverName = "drvAsynBMP280";

static void pollTask(void* drvPvt);

drvAsynBMP280::drvAsynBMP280(const char* portName, int i2cPortNum, int i2cAddr)
    : asynI2CDriver(portName, i2cPortNum,
                    1, /* maxAddr */
                       /* Interface mask */
                    (asynFloat64Mask | asynDrvUserMask),
                       /* Interrupt mask */
                    (asynFloat64Mask),
                    0, /* asynFlags (does not block and is not multi-device) */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size */
{
    const char* functionName = "drvAsynBMP280";

    this->i2cAddr = (unsigned short)i2cAddr;

    createParam(P_TempCString, asynParamFloat64, &P_Temperature_C);
    createParam(P_TempFString, asynParamFloat64, &P_Temperature_F);
    createParam(P_PressPaString, asynParamFloat64, &P_Pressure_Pa);
    createParam(P_PressinHgString, asynParamFloat64, &P_Pressure_inHg);

    eventId_ = epicsEventCreate(epicsEventEmpty);
    if (epicsThreadCreate("drvAsynBMP280Task", epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pollTask, this) == NULL) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    // Call our connect method to work-around the fact that autoconnect flag
    // triggers the base class to call connect before our constructor runs.
    this->connect(this->pasynUserSelf);
}

asynStatus drvAsynBMP280::connect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;
    unsigned char vals[2];

    /* Perform i2c connect */
    status = this->i2c_connect(pasynUser);
    if (status != asynSuccess) {
        return status;
    }

    /* Configure BMP280 */
    vals[0] = (0x01 << 5);  /* 1x temperature oversampling */
    vals[0] |= (0x01 << 2); /* 1x pressure oversampling */
    vals[0] |= 0x03;        /* Power Normal */
    if (this->write_reg(0xf4, vals[0]) != 0) {
        return asynError;
    }

    vals[0] = (0x05 << 5);  /* Standby 1000ms */
    vals[0] |= (0x04 << 2); /* Filter Coefficient */
    if (this->write_reg(0xf5, vals[0]) != 0) {
        return asynError;
    }

    /* Read temperature calibration data */
    if (this->read_reg(0x88, vals, 2) != 0) {
        return asynError;
    }
    this->t1 = (unsigned short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x8a, vals, 2) != 0) {
        return asynError;
    }
    this->t2 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x8c, vals, 2) != 0) {
        return asynError;
    }
    this->t3 = (short)((vals[1] << 8) | vals[0]);

    /* Read pressure calibration data */
    if (this->read_reg(0x8e, vals, 2) != 0) {
        return asynError;
    }
    this->p1 = (unsigned short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x90, vals, 2) != 0) {
        return asynError;
    }
    this->p2 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x92, vals, 2) != 0) {
        return asynError;
    }
    this->p3 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x94, vals, 2) != 0) {
        return asynError;
    }
    this->p4 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x96, vals, 2) != 0) {
        return asynError;
    }
    this->p5 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x98, vals, 2) != 0) {
        return asynError;
    }
    this->p6 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x9a, vals, 2) != 0) {
        return asynError;
    }
    this->p7 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x9c, vals, 2) != 0) {
        return asynError;
    }
    this->p8 = (short)((vals[1] << 8) | vals[0]);

    if (this->read_reg(0x9e, vals, 2) != 0) {
        return asynError;
    }
    this->p9 = (short)((vals[1] << 8) | vals[0]);

    return status;
}

asynStatus drvAsynBMP280::disconnect(asynUser* pasynUser)
{
    /* Perform i2c disconnect */
    return this->i2c_disconnect(pasynUser);
}

int drvAsynBMP280::read_reg(unsigned char reg, unsigned char* value,
                            unsigned short len)
{
    return this->i2c_wr_rd(i2cAddr, &reg, 1, value, len);
}

int drvAsynBMP280::write_reg(unsigned char reg, unsigned char value)
{
    unsigned char tx[2];
    tx[0] = reg;   /* Register address */
    tx[1] = value; /* Value */
    return this->i2c_wr_rd(i2cAddr, tx, 2, tx, 2);
}

static void pollTask(void* drvPvt)
{
    drvAsynBMP280* pPvt = (drvAsynBMP280*)drvPvt;
    pPvt->pollTask();
}

void drvAsynBMP280::pollTask(void)
{
    epicsTimeStamp now;
    epicsUInt32 delay_ns;
    int oneHz = 0;
    unsigned char vals[6];
    int raw, v1, v2, v3;
    int tfine;
    double value;

    lock();
    while (1) {
        unlock();

        epicsTimeGetCurrent(&now);
        delay_ns = 50000000 - (now.nsec % 50000000); /* 20 Hz */
        epicsEventWaitWithTimeout(eventId_, delay_ns / 1.e9);
        oneHz = (now.nsec >= 750000000) && (now.nsec < 800000000);

        lock();

        if (oneHz) {
            if (this->read_reg(0xf7, vals, 6) == 0) {
                /* Apply temperature calibration */
                raw   = (vals[3] << 12) | (vals[4] << 4) | (vals[5] >> 4);
                v1    = (raw / 16364.0 - this->t1 / 1024.0) * this->t2;
                v2    = (raw / 131072.0 - this->t1 / 8192.0);
                v2    = v2 * v2 * this->t3;
                tfine = v1 + v2;
                value = tfine / 5120.0;
                setDoubleParam(P_Temperature_C, value);
                setDoubleParam(P_Temperature_F, value * 9 / 5 + 32);

                /* Apply pressure calibration */
                raw = (vals[0] << 12) | (vals[1] << 4) | (vals[2] >> 4);
                v1  = tfine / 2.0 - 64000.0;
                v2  = v1 * v1 * this->p6 / 32768.0;
                v2  = v2 + v1 * this->p5 * 2.0;
                v2  = v2 / 4.0 + this->p4 * 65535.0;
                v1 = (this->p3 * v1 * v1 / 524288.0 + this->p2 * v1) / 524288.0;
                v1 = (1.0 + v1 / 32768.0) * this->p1;
                v3 = 1048576.0 - raw;
                v3 = (v3 - v2 / 4096.0) * 6250.0 / v1;
                v1 = this->p9 * v3 * v3 / 2147483648.0;
                v2 = v3 * this->p8 / 32768.0;
                value = v3 + (v1 + v2 + this->p7) / 16.0;
                setDoubleParam(P_Pressure_Pa, value);
                setDoubleParam(P_Pressure_inHg, value * 0.00029530);
            }

            updateTimeStamp();
            callParamCallbacks();
        }
    }
}

extern "C" {

int drvAsynBMP280Configure(const char* portName, int i2cPortNum, int i2cAddr)
{
    new drvAsynBMP280(portName, i2cPortNum, i2cAddr);
    return (asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0          = { "portName", iocshArgString };
static const iocshArg initArg1          = { "i2c port num", iocshArgInt };
static const iocshArg initArg2          = { "i2c address", iocshArgInt };
static const iocshArg* const initArgs[] = { &initArg0, &initArg1, &initArg2 };
static const iocshFuncDef initFuncDef
    = { "drvAsynBMP280Configure", 3, initArgs };

static void initCallFunc(const iocshArgBuf* args)
{
    drvAsynBMP280Configure(args[0].sval, args[1].ival, args[2].ival);
}

void drvAsynBMP280Register(void) { iocshRegister(&initFuncDef, initCallFunc); }

epicsExportRegistrar(drvAsynBMP280Register);
}
