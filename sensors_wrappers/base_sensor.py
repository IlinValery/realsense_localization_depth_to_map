from abc import abstractmethod
import threading
from helpers.custom_threading import StoppableThread
import pyrealsense2 as rs


class BaseSensor:
    def __init__(self, is_device, source_name):
        self.is_launched = False
        self.update_thread = None

        self.cfg = rs.config()
        self.pipe = rs.pipeline()
        if is_device:
            print("Configured as device with id={}".format(source_name))
            self.cfg.enable_device(source_name)
        else:
            print("Configured as file with name {}".format(source_name))
            self.cfg.enable_device_from_file(source_name)

    def start_sensor(self):
        try:
            self.update_thread = StoppableThread(target=self.thread_update)
            self.is_launched = True
            # self.source = 10/0
        except Exception: # too wide but I don't know any other here)
            self.is_launched = False

        if self.is_launched: # rewrite this
            self.update_thread.start()

    def stop_sensor(self):
        if self.is_launched:
            self.update_thread.stop()

    def thread_update(self):
        while threading.currentThread().is_execute():
            self.do_sensor_update()

    @abstractmethod
    def do_sensor_update(self):
        """
        Realisation of custom sensor update
        """
        pass
