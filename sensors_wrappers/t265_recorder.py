from helpers.base_observer import BaseObserver, BaseSubject


class T265subscriber(BaseObserver):
    """
    Class for saving data from T265
    """
    def __init__(self, T265):
        self.t265_publisher = T265
        self.pose_list = []
        self.time_list = []

    def on_parent_update(self, subject: BaseSubject) -> None:

        current_pose = self.t265_publisher.get_pose()
        if current_pose is not None:
            self.pose_list.append(current_pose.get_pose_data()) # -> Good
            self.time_list.append(current_pose.get_timestamp()) # -> Good
            # print('time_list[-1]', self.time_list[-1])
            # print('pose_list[-1].translation', self.pose_list[-1].translation)
            # self.some_list.append(current_pose)  # -> Bad. TODO: do copy.copy() to save the raw current_pose -> Problem with pickle

    def optimize_trajectory(self):
        pass




