/* drvAsynBMP280.h
 *
 * EPICS asynPortDriver for BMP280 Digital Temperature
 * and Pressure Sensor under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include "drvAsynI2C.h"

#define P_TempString "Temperature"         /* asynFloat64 */
#define P_TempCString "Temperature(C)"     /* asynFloat64 */
#define P_TempFString "Temperature(F)"     /* asynFloat64 */
#define P_PressString "Pressure"           /* asynFloat64 */
#define P_PressPaString "Pressure(Pa)"     /* asynFloat64 */
#define P_PressinHgString "Pressure(inHg)" /* asynFloat64 */

class drvAsynBMP280 : public drvAsynI2C {

public:
    drvAsynBMP280(const char* portName, int i2cPortNum, int i2cAddr);

    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);
    virtual asynStatus readFloat64(asynUser* pasynUser, epicsFloat64* value);

protected:
    int P_Temperature;
    int P_Temperature_C;
    int P_Temperature_F;
    int P_Pressure;
    int P_Pressure_Pa;
    int P_Pressure_inHg;

private:
    /* Calibration data */
    unsigned short t1;
    short t2, t3;
    unsigned short p1;
    short p2, p3, p4, p5, p6, p7, p8, p9;
    int tfine;

    int read_reg(unsigned char reg, unsigned char* value, unsigned short len);
    int write_reg(unsigned char reg, unsigned char value);
};
