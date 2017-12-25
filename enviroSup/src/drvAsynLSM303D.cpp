/* drvAsynLSM303D.cpp
 *
 * EPICS asynPortDriver for LSM303D 3D Accerlerometer
 * and 3D Magnetometer under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include "drvAsynLSM303D.h"

static const char* driverName = "drvAsynLSM303D";

static void pollTask(void *drvPvt);

drvAsynLSM303D::drvAsynLSM303D(const char* portName, int i2cPortNum,
                               int i2cAddr)
    : drvAsynI2C(portName, i2cPortNum, i2cAddr,
		 1, /* maxAddr */
                    /* Interface mask */
                 (asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask),
                    /* Interrupt mask */
                 (asynFloat64Mask | asynFloat64ArrayMask),
                 0, /* asynFlags (does not block and is not multi-device) */
                 1, /* Autoconnect */
                 0, /* Default priority */
                 0) /* Default stack size */
{
    const char* functionName = "drvAsynLSM303D";

    createParam(P_MagXString, asynParamFloat64, &P_Mag_X);
    createParam(P_MagYString, asynParamFloat64, &P_Mag_Y);
    createParam(P_MagZString, asynParamFloat64, &P_Mag_Z);
    createParam(P_MagString, asynParamFloat64Array, &P_Mag);
    createParam(P_AccelXString, asynParamFloat64, &P_Accel_X);
    createParam(P_AccelYString, asynParamFloat64, &P_Accel_Y);
    createParam(P_AccelZString, asynParamFloat64, &P_Accel_Z);
    createParam(P_AccelString, asynParamFloat64Array, &P_Accel);

    eventId_ = epicsEventCreate(epicsEventEmpty);
    if (epicsThreadCreate("drvAsynLSM303DTask", epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pollTask, this) == NULL) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    // Call our connect method to work-around the fact that autoconnect flag
    // triggers the base class to call connect before our constructor runs.
    this->connect(this->pasynUserSelf);
}

int drvAsynLSM303D::read_reg(unsigned char reg, unsigned char* value,
                             unsigned short len)
{
    unsigned char tx;

    tx = 0x80;  /* Auto Increment Addressing */
    tx |= reg;  /* Register address */

    return this->i2c_wr_rd(&tx, 1, value, len);
}

int drvAsynLSM303D::write_reg(unsigned char reg, unsigned char value)
{
    unsigned char tx[2];

    tx[0] = 0x80;  /* Auto Increment Addressing */
    tx[0] |= reg;  /* Register address */

    tx[1] = value;

    return this->i2c_wr_rd(tx, 2, tx, 2);
}

asynStatus drvAsynLSM303D::connect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;

    /* Perform i2c connect */
    status = this->i2c_connect(pasynUser);
    if (status != asynSuccess) {
        return status;
    }

    /* Configure LSM303D */
    if (this->write_reg(0x1f, 0x00) != 0) {
        return asynError;
    }
    if (this->write_reg(0x20, 0x57) != 0) {  /* 50Hz accel, xyz accel ena */
        return asynError;
    }
    if (this->write_reg(0x21, 0xc0) != 0) {  /* 50Hz accel LPF, +/-2g scale */
        return asynError;
    }
    if (this->write_reg(0x22, 0x00) != 0) {
        return asynError;
    }
    if (this->write_reg(0x23, 0x00) != 0) {
        return asynError;
    }
    if (this->write_reg(0x24, 0x60) != 0) {  /* Full mag resolution, 3.125Hz */
        return asynError;
    }
    if (this->write_reg(0x25, 0x20) != 0) {  /* Mag +/-4gauss scale */
        return asynError;
    }
    if (this->write_reg(0x26, 0x00) != 0) {
        return asynError;
    }

    return status;
}

asynStatus drvAsynLSM303D::disconnect(asynUser* pasynUser)
{
    /* Perform i2c disconnect */
    return this->i2c_disconnect(pasynUser);
}

static void pollTask(void* drvPvt)
{
    drvAsynLSM303D* pPvt = (drvAsynLSM303D*)drvPvt;
    pPvt->pollTask();
}

void drvAsynLSM303D::pollTask(void)
{
    epicsTimeStamp now;
    epicsUInt32 delay_ns;
    unsigned char vals[6];
    epicsFloat64 newVal;
    epicsFloat64 smoo = 0.368;
    int oneHz = 0;

    lock();
    while (1) {
        unlock();

        epicsTimeGetCurrent(&now);
        delay_ns = 50000000 - (now.nsec % 50000000);  /* 20 Hz */
        epicsEventWaitWithTimeout(eventId_, delay_ns / 1.e9);
	oneHz = (now.nsec >= 750000000) && (now.nsec < 800000000) ;

	lock();

	if (this->read_reg(0x28, vals, 6) == 0) {
	    newVal = (short)((vals[1] << 8) | vals[0]) / 32768. * 2.;
	    accel[0] = (1-smoo)*newVal + smoo*accel[0];
	    setDoubleParam(P_Accel_X, accel[0]);
	    newVal = (short)((vals[3] << 8) | vals[2]) / 32768. * 2.;
	    accel[1] = (1-smoo)*newVal + smoo*accel[1];
	    setDoubleParam(P_Accel_Y, accel[1]);
	    newVal = (short)((vals[5] << 8) | vals[4]) / 32768. * 2.;
	    accel[2] = (1-smoo)*newVal + smoo*accel[2];
	    setDoubleParam(P_Accel_Z, accel[2]);
        }

	if (oneHz) {  /* 1 Hz */
	    if (this->read_reg(0x08, vals, 6) == 0) {
	        newVal = (short)((vals[1] << 8) | vals[0]) / 32768. * 4.;
	        mag[0] = (1-smoo)*newVal + smoo*mag[0];
	        setDoubleParam(P_Mag_X, mag[0]);
	        newVal = (short)((vals[3] << 8) | vals[2]) / 32768. * 4.;
	        mag[1] = (1-smoo)*newVal + smoo*mag[1];
	        setDoubleParam(P_Mag_Y, mag[1]);
	        newVal = (short)((vals[5] << 8) | vals[4]) / 32768. * 4.;
	        mag[2] = (1-smoo)*newVal + smoo*mag[2];
	        setDoubleParam(P_Mag_Z, mag[2]);
            }
	}
	updateTimeStamp();
	callParamCallbacks();
	doCallbacksFloat64Array(accel, 3, P_Accel, 0);
	if (oneHz) {
            doCallbacksFloat64Array(mag, 3, P_Mag, 0);
	}
    }
}

extern "C" {

int drvAsynLSM303DConfigure(const char* portName, int i2cPortNum, int i2cAddr)
{
    new drvAsynLSM303D(portName, i2cPortNum, i2cAddr);
    return (asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0          = { "portName", iocshArgString };
static const iocshArg initArg1          = { "i2c port num", iocshArgInt };
static const iocshArg initArg2          = { "i2c address", iocshArgInt };
static const iocshArg* const initArgs[] = { &initArg0, &initArg1, &initArg2 };
static const iocshFuncDef initFuncDef
    = { "drvAsynLSM303DConfigure", 3, initArgs };

static void initCallFunc(const iocshArgBuf* args)
{
    drvAsynLSM303DConfigure(args[0].sval, args[1].ival, args[2].ival);
}

void drvAsynLSM303DRegister(void) { iocshRegister(&initFuncDef, initCallFunc); }

epicsExportRegistrar(drvAsynLSM303DRegister);
}
