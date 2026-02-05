/* written by Tim Ng for VTOL CWRU */

#include <iostream>
#include "ValueSmoothing.h"
#include <cassert>


ValueSmoothing::ValueSmoothing(int window) : values(window) {
    assert(window > 0);
    this->window = window;
}

void ValueSmoothing::coord_add(float value) {
    float old = values[index]; // keep the last value for calculations later
    if (activePos == window) {
        sum -= old; // remove last value from sum because it's getting replaced
    } else {
        activePos++;
    }

    values[index] = value; // adding to running sum for sma calculations
    sum += value;

    index++;
    if (index == window) index = 0; // circular buffer

    if (ema_ready == false) { // ema can only be calculated if all positions are active
        ema = (sum / activePos); // use sma if not available
        ema_ready = true;
    } else {
        float smoothing_factor = 2.0f / (activePos + 1.0f); // calculations for ema
        ema = value * smoothing_factor + (ema * (1.0f - smoothing_factor));
    }
}

bool ValueSmoothing::coord_get_sma(float& out) const { // outputs sma and true/false
    if (activePos == 0) return false;

    float denom = static_cast<float>(activePos); // just in case weird things happen with float / int division
    out = sum / denom;
    return true;
}

bool ValueSmoothing::coord_get_ema(float& out) const { // outputs ema and true/false
    if (ema_ready == false) return false;
    out = ema;
    return true;
}
