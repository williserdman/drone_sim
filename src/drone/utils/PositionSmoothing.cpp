/* written by Tim Ng for VTOL CWRU */

#include <iostream>
#include "PositionSmoothing.h"

PositionSmoothing::PositionSmoothing(int window) : x(window), y(window), z(window) { // constructor that defines window size

}
// class RelativePosition has float x, float y, float z
void PositionSmoothing::add(RelativePosition pos) {
    x.coord_add(pos.x); // adds respective coord values to each
    y.coord_add(pos.y);
    z.coord_add(pos.z);
}

RelativePosition PositionSmoothing::getSMA() const {
    RelativePosition out{};
    if (x.coord_get_sma(out.x) == false) return RelativePosition{}; // if false, sma is not attainable at the moment
    if (y.coord_get_sma(out.y) == false) return RelativePosition{}; // honestly this should almost never be the case
    if (z.coord_get_sma(out.z) == false) return RelativePosition{};
    return out;
}

RelativePosition PositionSmoothing::getEMA() const {
    RelativePosition out{};
    if (x.coord_get_ema(out.x) == false) return RelativePosition{}; // if false, ema is not attainable at the moment
    if (y.coord_get_ema(out.y) == false) return RelativePosition{};
    if (z.coord_get_ema(out.z) == false) return RelativePosition{};
    return out;
}