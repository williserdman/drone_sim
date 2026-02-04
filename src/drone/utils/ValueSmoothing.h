/* written by Tim Ng for VTOL CWRU */

#ifndef VALUESMOOTHING_H
#define VALUESMOOTHING_H

class ValueSmoothing {
private:
    int window; // number of last positions we're using in calculations
    int index = 0; // next index to put in
    int activePos = 0; // the amount of positions that have actual values in them (notable during initial launch)
    float values[128]; // array that holds the last positions, set to 128 by default
    float sum = 0.0f; // running sum that'll allow SMA to be calculated in O(1) instead of O(n), trust the process
    float ema = 0.0f; // the last ema;
    bool ema_ready = false; // check if ema can actually be calculated based on sma
public:
    explicit ValueSmoothing(int window); // constructor
    void coord_add(float value); // add a coordinate to values array
    bool coord_get_sma(float& out); // gets the sma and sets it to the out variable (return true if it can be calculated, false if not)
    bool coord_get_ema(float& out); // gets the ema and sets it to the out variable  (return true if it can be calculated, false if not)
    ~ValueSmoothing();
};

#endif