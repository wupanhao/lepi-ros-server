import evdev
from evdev import InputDevice, categorize, ecodes
import threading


class ButtonListener:
    def __init__(self, callback=None):
        self.callback = callback
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        self.dev = None
        for device in devices:
            print(device.path, device.name, device.phys)
            if device.name == 'HID 03eb:2421':
                self.dev = InputDevice(device.path)
                break
        print(self.dev)
        # device / dev/input/event1, name "Dell Dell USB Keyboard", phys "usb-0000:00:12.1-2/input0"
        if self.dev is not None:
            reader = threading.Thread(target=self.start_listen_loop)
            # reader.daemon = True
            reader.start()
        else:
            print('device not find')

    def start_listen_loop(self):
        for event in self.dev.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.value != 2 and self.callback is not None:
                    self.callback(event)
                elif self.callback is None:
                    print(event)
                    print(categorize(event))
    # pressing 'a' and holding 'space'


if __name__ == '__main__':
    import time
    listener = ButtonListener()
    time.sleep(10)
