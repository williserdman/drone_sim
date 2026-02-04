#include "../../common/RelativePosition.h"


class PositionSmoothing {

    public:
    PositionSmoothing(int window);
    void add(RelativePosition pos);
    RelativePosition getEMA() const;
    RelativePosition getSMA() const;
    ~PositionSmoothing();
};
