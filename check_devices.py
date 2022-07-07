import pyaudio
import cv2 as cv


def check_microphones():
    """ This is a utility function that detects the available microphones and prints the device indices.
    """
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id ", i, " - ",
                  p.get_device_info_by_host_api_device_index(0, i).get('name'))


def check_cameras():
    """ This is a utility function that detects the available cameras and prints the device indices.
    """
    index = 0
    arr = []
    while True:
        cap = cv.VideoCapture(index)
        if not cap.read()[0]:
            break
        else:
            arr.append(index)
        cap.release()
        index += 1

    print("List of available camera indices:")
    print("\n".join([str(i) for i in arr]))


print("---------- CHECKING MICROPHONES ----------")
check_microphones()
print("---------- CHECKING CAMERAS ----------")
check_cameras()
