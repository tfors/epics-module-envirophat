/* drvAsynTCS3472.h
 *
 * EPICS asynPortDriver for TCS3472 Color Light-to-Digital
 * Converter under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include "drvAsynI2C.h"

#define P_LightString "Light" /* asynFloat64 */
#define P_ColorString "Color" /* asynFloat64Array */
#define P_GainString "Gain"   /* asynInt32 */

class drvAsynTCS3472 : public drvAsynI2C {

public:
    drvAsynTCS3472(const char* portName, int i2cPortNum, int i2cAddr);

    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser* pasynUser, epicsFloat64* value);
    virtual asynStatus readFloat64Array(asynUser* pasynUser,
                                        epicsFloat64* value, size_t nElements,
                                        size_t* nIn);

protected:
    int P_Light;
    int P_Color;
    int P_Gain;

private:
    int maxCount;
    int read_reg(unsigned char reg, unsigned char* value, unsigned short len);
    int write_reg(unsigned char reg, unsigned char value);
};
