import serial


class SServo(object):
    """
    serial client
    """

    def __init__(self, port, baud_rate=1000000):
        self.port = serial.Serial(port=port, baudrate=baud_rate, bytesize=8, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=0.01)

    def send_cmd(self, cmd):
        self.port.write(cmd.encode('utf-8'))

    def read_cmd(self):
        response = self.port.readline()
        return response.decode('utf-8')

    def read_num(self, num):
        response = self.port.read(num)
        return response

    def send_hex(self, cmd):
        # print(cmd)
        self.port.write(cmd)

    def read_hex(self, n=0):
        if(n > 0):
            response = self.port.read(n)
        else:
            response = self.port.readline()
        array = []
        for i in response:
            array.append(ord(i))
        return array


if __name__ == '__main__':
    servo = SServo('/dev/ttyAMA1')
    print(servo.read_hex())
    servo.send_hex([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    print(servo.read_hex())
    servo.send_hex([0, 1, 1, 2])
    print(servo.read_hex())
