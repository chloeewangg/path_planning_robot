'''

!/usr/bin/env python3

NFC_sensor.py
Does everything related to the NFC sensor.

'''

# Imports
import board
import busio
from global_imports import *
from adafruit_pn532.i2c import PN532_I2C

class NFC:
    def __init__(self):
        # Initialize the hardware.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pn532 = PN532_I2C(self.i2c, debug=False)
        self.pn532.SAM_configuration()

        # Clear the last read data.
        self.last_read = None

        # Start the worker thread.
        print("Starting NFC thread...")
        self.reading = True
        self.thread = threading.Thread(name="NFCThread", target=self.run)
        self.thread.start()
    
    def run(self):
        '''
        Runs the NFC reader and saves the last value read.
        '''
        while self.reading:
            # Attempt an NFC read.
            uid = self.pn532.read_passive_target(timeout=0.2)
            # Only save if we have something.
            if uid is not None:
                self.last_read = hash(tuple(uid))
            # Wait 50ms before re-attempting.
            time.sleep(0.05)

    def read(self):
        '''
        Reads NFC sensor to last data read and returns that.
        '''
        # Grab and then clear the last read.
        data = self.last_read
        self.last_read = None
        # Return the data.
        return data
    
    def shutdown(self):
        '''
        Shuts down the NFC thread.
        '''
        self.reading = False
        print("Waiting for NFC thread to finish...")
        self.thread.join()
        print("NFC thread returned.")
    
#
#   Main: Prints what the NFC sensor last read
#
if __name__ == "__main__":
    ############################################################
    # Prepare the GPIO interface/connection 
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    ############################################################

    NFC_sensor = NFC()

    try:
        while True:
            print("Reading")
            print(NFC_sensor.read())
            input("Hit return to read")

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop the interface.
    io.stop()