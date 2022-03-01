import time
from pyee import ExecutorEventEmitter
start = time.time()

def test():
    ee = ExecutorEventEmitter()
    @ee.on('event')
    def handler():
        print('handler', (time.time()-start)*1000)
        time.sleep(2)
        print('BANG BANG', (time.time()-start)*1000)


    print('emit', (time.time()-start)*1000)
    ee.emit('event')
    print('BING BING', (time.time()-start)*1000)
    time.sleep(3)

class TestNode(ExecutorEventEmitter):
    def __init__(self) -> None:
        super().__init__()
        self.on('event',self.handler)
        print('emit', (time.time()-start)*1000)
        self.emit('event')
        print('BING BING', (time.time()-start)*1000)
        time.sleep(3)
    def handler(self):
        print('handler', (time.time()-start)*1000)
        time.sleep(2)
        print('BANG BANG', (time.time()-start)*1000)
# test()
TestNode()