import time

from sensors.base_sensor import BaseSensor
from helpers_classes.base_observer import BaseObserver, BaseSubject


class T265Sensor(BaseSensor, BaseObserver):
    def __init__(self):
        super(T265Sensor, self).__init__()
        self.num265 = 0

    def do_sensor_update(self):
        # print("T265Sensor", self.num265)
        self.num265 += 1
        time.sleep(1/200)

    def update(self, subject: BaseSubject) -> None:
        print('Value on t256 sensor value number is {}'.format(self.num265))

