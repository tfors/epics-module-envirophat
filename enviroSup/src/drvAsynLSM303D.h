/* drvAsynLSM303D.h
 *
 * EPICS asynPortDriver for LSM303D 3D Accerlerometer
 * and 3D Magnetometer under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsEvent.h>

#include "asynI2CDriver.h"

#define P_AccelXString "AccelX"       /* asynFloat64 */
#define P_AccelYString "AccelY"       /* asynFloat64 */
#define P_AccelZString "AccelZ"       /* asynFloat64 */
#define P_AccelString "Accel"         /* asynFloat64Array */
#define P_MagXString "MagX"           /* asynFloat64 */
#define P_MagYString "MagY"           /* asynFloat64 */
#define P_MagZString "MagZ"           /* asynFloat64 */
#define P_MagString "Mag"             /* asynFloat64Array */

class drvAsynLSM303D : public asynI2CDriver {

public:
    drvAsynLSM303D(const char* portName, int i2cPortNum, int i2cAddr);

    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);

    void pollTask(void);

protected:
    int P_Accel_X;
    int P_Accel_Y;
    int P_Accel_Z;
    int P_Accel;
    int P_Mag_X;
    int P_Mag_Y;
    int P_Mag_Z;
    int P_Mag;

private:
    epicsEventId eventId_;
    unsigned short i2cAddr;

    double accel[3];
    double mag[3];

    int read_reg(unsigned char reg, unsigned char* value, unsigned short len);
    int write_reg(unsigned char reg, unsigned char value);
};
