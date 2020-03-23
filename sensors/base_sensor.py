from abc import ABC, abstractmethod
import threading
from helpers_classes.custom_threading import StoppableThread


class BaseSensor():
    def __init__(self, is_real=False, source_name='data.bag'):
        self.is_launched = False
        self.update_thread = None
        self.is_real = is_real
        self.source = source_name

    def start_sensor(self):
        try:
            self.update_thread = StoppableThread(target=self.thread_update_data)
            self.is_launched = True
            # self.source = 10/0
        except Exception: # too wide but I don't know any other here)
            self.is_launched = False

        if self.is_launched: # rewrite this
            self.update_thread.start()

    def stop_sensor(self):
        if self.is_launched:
            self.update_thread.stop()

    def thread_update_data(self):
        while threading.currentThread().is_executed():
            self.do_sensor_update()

    @abstractmethod
    def do_sensor_update(self):
        """
        Realisation of custom sensor update
        """
        pass
