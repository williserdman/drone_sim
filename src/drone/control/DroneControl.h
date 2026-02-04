#include <string>
#include "../../common/GPSCoord.h"
#include "../../common/NEDMeters.h"
#include "../../common/RelativePosition.h"

class DroneControl {
    public:
    DroneControl(std::string port);
    bool gotoWaypoint(GPSCoord coord);
    bool moveRelativeSelf(NEDMeters dir);
    bool moveRelativeSelf(RelativePosition dir);
    bool simpleLand();
    bool takeoff();
    bool disarm();
    bool waitForArm();
    GPSCoord getCurrentGPS();
    ~DroneControl();
};