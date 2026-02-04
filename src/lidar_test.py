from drone.common_types import RelativePosition
from drone.sensors.lidar.lidar import Lidar
import time



if __name__ == "__main__":
    l = Lidar()
    while True:
        try:
            # Read and print the distance in centimeters
            distance = l.get_distance()
            print(f"Distance: {distance} cm")

        except RuntimeError as e:
            # Handle potential reading errors gracefully
            print(f"Error reading sensor: {e}")

        time.sleep(0.1)  # Add a small delay
