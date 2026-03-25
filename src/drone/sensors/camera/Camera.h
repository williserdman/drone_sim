#include "../../../common/RelativePosition.h"

class Camera {

    public:
    Camera();
    bool getVectorToMarker(int id, RelativePosition* vec) const; // update relative position if exists, return success/fail state
    ~Camera();
};