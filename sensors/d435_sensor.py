import time
from sensors.base_sensor import BaseSensor
from helpers_classes.base_observer import BaseSubject, BaseObserver
from typing import List
from sensors.t265_sensor import T265Sensor


class D435Sensor(BaseSensor, BaseSubject):
    def __init__(self):
        super(D435Sensor, self).__init__()
        self._observers: List[BaseObserver] = [] # pattern observer in common

        # initial conditions
        self.number = 0

    def do_sensor_update(self):
        # update initial conditions (this function was wrapped in while)
        self.number += 1

        if (self.number % 30 == 0):
            print("sensor d435 Updated", self.number)
            # publish on other sensor notify
            self.notify()

        time.sleep(1/30) # as 1/30

    def attach(self, observer: BaseObserver) -> None:
        self._observers.append(observer)

    def detach(self, observer: BaseObserver) -> None:
        self._observers.remove(observer)

    def notify(self) -> None:
        for observer in self._observers:
            observer.update(self)

