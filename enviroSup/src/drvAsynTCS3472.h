/* drvAsynTCS3472.h
 *
 * EPICS asynPortDriver for TCS3472 Color Light-to-Digital
 * Converter under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsEvent.h>

#include "asynI2CDriver.h"

#define P_GainString "Gain"   /* asynInt32 */
#define P_LightString "Light" /* asynFloat64 */
#define P_RedString "Red"     /* asynFloat64 */
#define P_GreenString "Green" /* asynFloat64 */
#define P_BlueString "Blue"   /* asynFloat64 */
#define P_ColorString "Color" /* asynFloat64Array */

class drvAsynTCS3472 : public asynI2CDriver {

public:
    drvAsynTCS3472(const char* portName, int i2cPortNum, int i2cAddr);

    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);

    void pollTask(void);

protected:
    int P_Gain;
    int P_Light;
    int P_Red;
    int P_Green;
    int P_Blue;
    int P_Color;

private:
    epicsEventId eventId_;
    double light;
    double color[3];
    int maxCount;

    int read_reg(unsigned char reg, unsigned char* value, unsigned short len);
    int write_reg(unsigned char reg, unsigned char value);
};
