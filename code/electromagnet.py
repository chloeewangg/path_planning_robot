'''

!/usr/bin/env python3

NFC_sensor.py
Does everything related to the NFC sensor.

'''

# Imports
from global_imports import *

MAGNET_TOGGLE = 1

class Electromagnet:
    def __init__(self, io, toggle):
        # Initialize the hardware.
        self.io = io
        self.toggle = toggle
        self.io.set_mode(toggle, pigpio.OUTPUT)
    
    def on(self):
        """
        Turns electromagnet on.
        """
        self.io.write(self.toggle, 1)

    def off(self):
        """
        Turns electromagnet off.
        """
        self.io.write(self.toggle, 0)

    
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

    # instantiate magnetometer object
    electromagnet  = Electromagnet(io, MAGNET_TOGGLE)
    
    try:

        while(True):
            electromagnet.on()
            time.sleep(5)
            electromagnet.off()
            time.sleep(5)

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop the interface.
    electromagnet.off()
    io.stop()

