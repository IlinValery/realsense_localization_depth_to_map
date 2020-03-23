from abc import abstractmethod
import threading
from helpers.custom_threading import StoppableThread
import pyrealsense2 as rs
import os


class BaseSensor:
    def __init__(self, is_device, source_name):
        self.is_launched = False
        self.update_thread = None
        self.frameset_number = 0

        self.cfg = rs.config()
        self.pipe = rs.pipeline()
        if is_device:
            print("Sensor configured as device with id={}".format(source_name))
            if not (source_name is None or source_name == ''):
                self.cfg.enable_device(source_name)
        else:
            print("Sensor configured as file with name {}".format(source_name))
            self.cfg.enable_device_from_file(source_name)

    def allow_writing_to_file(self, file_name, folder_name=None):
        path_to_file = './{0}'.format(file_name)
        if folder_name is not None:
            path_to_file = './{0}/{1}'.format(folder_name, file_name)
            if not os.path.exists(folder_name):
                os.mkdir(folder_name)
        self.cfg.enable_record_to_file(path_to_file)

    def start_sensor(self):
        try:
            self.update_thread = StoppableThread(target=self.thread_update)
            self.is_launched = True
            # self.source = 10/0
        except Exception: # too wide but I don't know any other here)
            self.is_launched = False

        if self.is_launched: # rewrite this
            self.pipe.start(self.cfg)
            self.update_thread.start()

    def stop_sensor(self):
        if self.is_launched:
            self.update_thread.stop()

    def thread_update(self):
        while threading.currentThread().is_execute():
            self.do_sensor_update()
            self.frameset_number += 1

    def get_frameset_number(self):
        return self.frameset_number

    @abstractmethod
    def do_sensor_update(self):
        """
        Realisation of custom sensor update
        """
        pass

    @abstractmethod
    def process_frameset(self):
        """
        Realisation of custom sensor update
        """
        pass
