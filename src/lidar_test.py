from .drone.common_types import RelativePosition
from .drone.sensors.lidar.lidar import Lidar
import time



if __name__ == "__main__":
    l = Lidar()
    s = 0
    c = 0
    try:
        # Read and print the distance in centimeters
        for _ in range(100):
            distance = l.get_distance()
            time.sleep(0.05)
            print(f"Distance: {distance} cm")
            s += distance
            c += 1

        print("average is", s/c)

    except RuntimeError as e:
        # Handle potential reading errors gracefully
        print(f"Error reading sensor: {e}")

    time.sleep(0.1)  # Add a small delay
