#ifndef POSITIONSMOOTHING_H
#define POSITIONSMOOTHING_H
#include "ValueSmoothing.h"
#include "../../common/RelativePosition.h"

class PositionSmoothing {
private:
    ValueSmoothing x; // Values that needs to be smoothed
    ValueSmoothing y;
    ValueSmoothing z;
public:
    PositionSmoothing(int window);
    void add(RelativePosition pos);
    RelativePosition getEMA() const;
    RelativePosition getSMA() const;
    ~PositionSmoothing() = default; // there's literally nothing to destruct here lol
};

#endif