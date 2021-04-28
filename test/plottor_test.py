import websocket
import json

from pi_driver import D51Driver
from pi_driver import I2cDriver
import time

target_pitch = 54.0

# lepi = D51Driver()
driver = I2cDriver()

try:
    import thread
except ImportError:
    import _thread as thread


def on_message(ws, message):
    print(message)


def on_error(ws, error):
    print(error)


def on_close(ws):
    print("### closed ###")


def on_open(ws):
    def run(*args):
        last_angle = 0
        while True:
            K1 = 0.03
            pose = driver.estimatePose()
            rot = driver.readGyroData()
            angle = 0.2*pose[1]+0.8*last_angle
            last_angle = angle
            angle = angle + K1*rot[0]/32.768
            ws.send(json.dumps({"type": "data", "value": {
                    "angle": angle, "raw": pose[1], "gyro": rot[0]}}))
            time.sleep(0.03)
        time.sleep(1)
        ws.close()
        print("thread terminating...")
    thread.start_new_thread(run, ())


if __name__ == "__main__":
    # websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://192.168.50.246:20110/plottor/sender",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    ws.run_forever()
