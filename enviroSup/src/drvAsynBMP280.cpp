/* drvAsynBMP280.cpp
 *
 * EPICS asynPortDriver for BMP280 Digital Temperature
 * and Pressure Sensor under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsExport.h>
#include <iocsh.h>

#include "drvAsynBMP280.h"

// static const char *driverName = "drvAsynBMP280";

drvAsynBMP280::drvAsynBMP280(const char* portName, int i2cPortNum, int i2cAddr)
    : drvAsynI2C(portName, i2cPortNum, i2cAddr,
		 1, /* maxAddr */
                    /* Interface mask */
                 asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask
                     | asynDrvUserMask,
                 0, /* Interrupt mask */
                 0, /* asynFlags (does not block and is not multi-device) */
                 1, /* Autoconnect */
                 0, /* Default priority */
                 0) /* Default stack size */
{
    createParam(P_TempString, asynParamFloat64, &P_Temperature);
    createParam(P_TempCString, asynParamFloat64, &P_Temperature_C);
    createParam(P_TempFString, asynParamFloat64, &P_Temperature_F);
    createParam(P_PressString, asynParamFloat64, &P_Pressure);
    createParam(P_PressPaString, asynParamFloat64, &P_Pressure_Pa);
    createParam(P_PressinHgString, asynParamFloat64, &P_Pressure_inHg);

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
    return this->i2c_wr_rd(&reg, 1, value, len);
}

int drvAsynBMP280::write_reg(unsigned char reg, unsigned char value)
{
    unsigned char tx[2];
    tx[0] = reg;   /* Register address */
    tx[1] = value; /* Value */
    return this->i2c_wr_rd(tx, 2, tx, 2);
}

asynStatus drvAsynBMP280::readFloat64(asynUser* pasynUser, epicsFloat64* value)
{
    int function = pasynUser->reason;
    unsigned char vals[4];
    int raw, v1, v2, v3;

    // Check if we're still connected
    if (this->fd < 0) {
        pasynManager->exceptionDisconnect(pasynUser);
        return asynDisconnected;
    }

    if (function == P_Temperature || function == P_Temperature_C
        || function == P_Temperature_F) {

        /* Read device */
        if (this->read_reg(0xfa, vals, 3) != 0) {
            return asynError;
        }

        /* Apply calibration */
        raw         = (vals[0] << 12) | (vals[1] << 4) | (vals[2] >> 4);
        v1          = (raw / 16364.0 - this->t1 / 1024.0) * this->t2;
        v2          = (raw / 131072.0 - this->t1 / 8192.0);
        v2          = v2 * v2 * this->t3;
        this->tfine = v1 + v2;
        *value      = this->tfine / 5120.0;

        /* Apply conversion */
        if (function == P_Temperature_F) {
            *value = *value * 9 / 5 + 32;
        }

    } else if (function == P_Pressure || function == P_Pressure_Pa
               || function == P_Pressure_inHg) {

        /* Read device */
        if (this->read_reg(0xf7, vals, 3) != 0) {
            return asynError;
        }

        /* Apply calibration */
        raw    = (vals[0] << 12) | (vals[1] << 4) | (vals[2] >> 4);
        v1     = this->tfine / 2.0 - 64000.0;
        v2     = v1 * v1 * this->p6 / 32768.0;
        v2     = v2 + v1 * this->p5 * 2.0;
        v2     = v2 / 4.0 + this->p4 * 65535.0;
        v1     = (this->p3 * v1 * v1 / 524288.0 + this->p2 * v1) / 524288.0;
        v1     = (1.0 + v1 / 32768.0) * this->p1;
        v3     = 1048576.0 - raw;
        v3     = (v3 - v2 / 4096.0) * 6250.0 / v1;
        v1     = this->p9 * v3 * v3 / 2147483648.0;
        v2     = v3 * this->p8 / 32768.0;
        *value = v3 + (v1 + v2 + this->p7) / 16.0;

        /* Apply conversion */
        if (function == P_Pressure_inHg) {
            *value = *value * 0.00029530;
        }
    }

    return asynSuccess;
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
