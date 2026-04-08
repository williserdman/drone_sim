import time
import threading
import board  # type: ignore
import busio  # type: ignore
import adafruit_lidarlite  # type: ignore


class StaleSensorError(Exception):
    """Custom exception raised when sensor data is too old to be trusted."""

    pass


class Lidar:
    def __init__(self, timeout: float = 0.5):
        # Create the I2C bus object
        i2c = busio.I2C(board.SCL, board.SDA)

        # Create the LIDARLite sensor object with default configuration
        self.sensor = adafruit_lidarlite.LIDARLite(i2c)

        self._current_distance = 10.0  # Default safe altitude
        self._last_update_time = 0.0  # Track exactly when the data was last updated
        self._timeout = timeout  # Max age of data in seconds before it's "stale"
        self._running = True

        # Start a daemon thread to poll the sensor in the background
        self._thread = threading.Thread(target=self._poll_sensor, daemon=True)
        self._thread.start()

    def _poll_sensor(self):
        """Continuously polls the I2C bus in the background."""
        while self._running:
            try:
                raw_distance = self.sensor.distance
                # from mounting 27 cm offset
                self._current_distance = (raw_distance - 15) / 100

                # CRITICAL: Only update the timestamp if the read was SUCCESSFUL
                self._last_update_time = time.time()

            except RuntimeError:
                pass  # I2C glitch, ignore and try again
            except Exception:
                pass  # Catch all, ignore and try again

            time.sleep(0.05)

    def get_distance(self) -> float:
        """Returns the distance, but strictly enforces data freshness."""
        data_age = time.time() - self._last_update_time

        # If we haven't had a successful read within the timeout window, sound the alarm
        if data_age > self._timeout:
            raise StaleSensorError(
                f"CRITICAL: Lidar data is {data_age:.2f} seconds old!"
            )

        return self._current_distance

    def stop(self):
        self._running = False
