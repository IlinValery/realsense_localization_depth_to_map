import cv2

from sensors_wrappers.d435_sensor import D435Sensor
from sensors_wrappers.t265_sensor import T265Sensor

is_device = False
is_write_to_bag = True
pose_number = 0
if __name__ == "__main__":

    if is_device:
        D435 = D435Sensor(is_device=True, source_name='845112070910')
        T265 = T265Sensor(is_device=True, source_name='943222110531')  # change 0000943222110531
        if is_write_to_bag:
            D435.allow_writing_to_file("D435.bag")
            T265.allow_writing_to_file("T265.bag")

    D435 = D435Sensor(is_device=False, source_name='data/435.bag')
    T265 = T265Sensor(is_device=False, source_name='data/265.bag')
    D435.attach(T265)

    D435.start_sensor()
    T265.start_sensor()

    try:
        while True:
            image = D435.get_image()


            if image is not None:
                # print(image.shape)
                pose = T265.get_coordinates()
                # pose_number += 1
                # print(D435.get_frameset_number(), T265.get_frameset_number())
                print(pose)
                cv2.imshow('D435 RGB Frame', image)
                cv2.waitKey(1000)

            # time.sleep(1)
            # actions here
            pass
    except KeyboardInterrupt:
        D435.stop_sensor()
        T265.stop_sensor()


