/*
Donovan Crowley for vtol
dxc825@case.edu
Feb 17, 2026
*/

#include <linux/types.h>
#include <cstdio>
#include <iostream>
#include "Lidar.h"
using namespace std;

//Lidar myLidarLite;

int main()
{
    char* i2c_fc;
    cout << "I2C address (/dev/i2c-1): ";
    cin >> i2c_fc;
    Lidar lidar(i2c_fc);
    //if (!lidar.isConnected()) {
    //    printf("Lidar initialization failed.\n");
    //    return -1;
    //}

    __u16 distance;
    __u8  busyFlag;

    // Initialize i2c peripheral in the cpu core
    lidar.i2c_init();

    // Configure LIDAR-Lite
    lidar.configure(0);

    while(1) {
        // Check BUSY
        busyFlag = lidar.getBusyFlag();

        if (busyFlag == 0x00)
        {
            // Read distance (in cm) from data
            lidar.takeRange();
            distance = lidar.readDistance(); 

            // Convert to meters
            float distance_m = (float)distance / 100.0;

            // Print out data in meters
            printf("%.2f meter\n", distance_m);
        }
    } 
    //catch (exception& e){
    //    cerr << "Error: " << e.what() << endl;
    //}
}