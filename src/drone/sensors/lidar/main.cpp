/*
Donovan Crowley for vtol
dxc825@case.edu
Feb 17, 2026
*/

#include "Lidar.h"
#include <iostream>
#include <unistd.h>

int main() {
    Lidar lidar("/dev/i2c-1");
    if (!lidar.isConnected()) {
        printf("Lidar initialization failed.\n");
        return -1;
    }

    // Configure at default
    lidar.configure(0);

    while(true) {
        // Get distance in m
        float distance = lidar.getDistance();

        if(distance >= 0){
            printf("Distance: %.2f m\n", distance);
        } else{
            printf("Error reading distance.\n");
        }

        // 100ms delay
        usleep(100000); 
    }

    return 0;
}