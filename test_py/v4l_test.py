import pyv4l2

def list_video_devices():
    video_devices = pyv4l2.get_devices()
    for device in video_devices:
        print("Device Path: ", device.path)
        print("Device Name: ", device.name)
        print("Device Properties:")
        for prop in device.properties:
            print(prop)

list_video_devices()
