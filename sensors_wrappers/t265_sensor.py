import time

from sensors_wrappers.base_sensor import BaseSensor
from helpers.base_observer import BaseObserver, BaseSubject


class T265Sensor(BaseSensor, BaseObserver):
    def __init__(self, is_device, source_name):
        super(T265Sensor, self).__init__(is_device, source_name)

        self.num265 = 0
        self.numbers=[]

    def do_sensor_update(self):
        # print("T265Sensor", self.num265)
        self.num265 += 1
        time.sleep(1/200)

    def parent_update(self, subject: BaseSubject) -> None:
        self.numbers.append(self.num265)
        # print('Value on t256 sensor value number is {}'.format(self.num265))

