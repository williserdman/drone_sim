import time
import board  # type: ignore
import busio  # type: ignore
import adafruit_lidarlite  # type: ignore


class Lidar:
    def __init__(self):
        # Create the I2C bus object
        i2c = busio.I2C(board.SCL, board.SDA)

        # Create the LIDARLite sensor object with default configuration
        self.sensor = adafruit_lidarlite.LIDARLite(i2c)

    def get_distance(self) -> float:
        return self.sensor.distance


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
