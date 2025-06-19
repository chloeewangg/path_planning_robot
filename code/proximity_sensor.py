'''
!/usr/bin/env python3

proximity_sensor.py

Has class for proximity sensor, made of 3 ultrasound sensors, to monitor distance. 
'''

# Imports
from global_imports import *

class ProximitySensor:
    def __init__(self, io, s1_pintrig, s1_pinecho, s2_pintrig, s2_pinecho, s3_pintrig, s3_pinecho, only_front=True):
        '''
        Initializes a proximity sensor made of 3 ultrasound sensors.

        :param io: GPIO the raspberry pi uses
        :param s1_pintrig: Trigger pin of front sensor
        :param s1_pinecho: Echo pin of front sensor
        :param s2_pintrig: Trigger pin of right sensor
        :param s2_pinecho: Echo pin of right sensor
        :param s3_pintrig: Trigger pin of left sensor
        :param s3_pinecho: Echo pin of left sensor
        :param only_front: Whether or not to only take front sensor readings
        '''
        self.only_front = only_front

        self.front_sensor = Ultrasound(io, s1_pintrig, s1_pinecho)
        self.right_sensor = Ultrasound(io, s2_pintrig, s2_pinecho)
        self.left_sensor = Ultrasound(io, s3_pintrig, s3_pinecho)
        
        print("Starting triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run_trigger)
        self.thread.daemon = True
        self.thread.start()
        time.sleep(0.1)

    def trigger(self):
        '''
        Triggers all three sensors
        '''
        if self.only_front:
            self.front_sensor.trigger()
        else: 
            self.front_sensor.trigger()
            self.right_sensor.trigger()
            self.left_sensor.trigger()

    def run_trigger(self):
        '''
        Triggers the three sensors continuously.
        '''

        while self.triggering:
            self.trigger()
            time_sleep = random.uniform(0.04, 0.06)
            time.sleep(time_sleep)

        self.thread.join()

    def shutdown(self):
        '''
        Shuts down and joins all threads.
        '''

        self.triggering = False
    
    def read_vals(self):
        '''
        Returns the distance measured by the three sensors.

        :return: (left, front, right) distances measured
        '''
        return (self.left_sensor.read_val(), self.front_sensor.read_val(), self.right_sensor.read_val())

class Ultrasound:
    SOUND_SPEED = 344

    def __init__(self, io, pintrig, pinecho):
        '''
        Initializes ultrasound sensor.

        :param io: GPIO the raspberry pi uses
        :param pintrig: Trigger pin of the sensor
        :param pinecho: Echo pin of the sensor
        '''
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho
        self.last_call = 0
        self.risetick = 0
        self.falltick = 0
        self.distance = math.inf

        self.io.set_mode(pintrig, pigpio.OUTPUT)
        self.io.set_mode(pinecho, pigpio.INPUT)

        self.cbrise = self.io.callback(self.pinecho, pigpio.RISING_EDGE, self.rising)
        self.cbfall = self.io.callback(self.pinecho, pigpio.FALLING_EDGE, self.falling)

    def trigger(self):
        '''
        Triggers the ultrasound sensor to send out a signal using the trigger pin.
        '''

        # Only triggers after 50 ms have passed since the previous trigger
        if (time.time() - self.last_call >= 0.05):
            self.last_call = time.time()
            self.io.write(self.pintrig, 1)
            self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        '''
        Stores the ticks value when the echo pin rises.

        :param pin: Associated pin to the callback function, which is the echo pin
        :param level: New level detected (1)
        :param ticks: Integer representing time in microseconds
        '''
        self.risetick = float(ticks)

    def falling(self, pin, level, ticks):
        '''
        Stores the ticks value when the echo pin falls and calculates the distance using the difference in the rise and fall tick.

        :param pin: Associated pin to the callback function, which is the echo pin
        :param level: New level detected (0)
        :param ticks: Integer representing time in microseconds
        '''
        self.falltick = float(ticks)

        deltatick = self.falltick - self.risetick
        
        if (deltatick < 0):
            deltatick += 2 ** 32
        
        self.distance = deltatick * 100 * self.SOUND_SPEED / (2 * 1000000)
    
    def read_val(self):
        '''
        Triggers the ultrasound sensor and return the distance in cm.
        '''
        return self.distance
    
#
#   Main: Prints out three sensor distances.
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

    proximity_sensor = ProximitySensor(io, 19, 20, 26, 21, 13, 16, False)

    try:
        last_print_time = time.time() 
        while True:
            # Trigger the ultrasounds and wait 50ms.
            left, front, right = proximity_sensor.read_vals()
            
            if(time.time() - last_print_time >= 0.5):

                print(f"Front: {front}, Right: {right}, Left: {left}")
                last_print_time = time.time()

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop the interface.
    proximity_sensor.shutdown()
    io.stop()