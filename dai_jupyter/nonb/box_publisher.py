import zmq
import time
import traceback
import numpy as np

class BoxPublisher:
    def __init__(self):
        context = zmq.Context()
        self.pub = context.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 0)
        self.pub.bind("tcp://*:5555")
        self.sub = context.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.bind("tcp://*:5556")
        self.active = True

    def close(self):
        self.sub.close()
        self.pub.close()
    
    def publish(self, messages, debug=False):
        if self.active:
            if debug:
                print("Sending messages " + str(messages))
            self.pub.send_multipart(messages)
            try:
                signal = self.sub.recv(flags=zmq.NOBLOCK)
                if debug:
                    print("recvd TERMINATE signal")
                self.active = False
            except zmq.error.Again:
                # signal not sent yet
                pass
            except:
                traceback.print_exc()

if __name__ == "__main__":
    pub = BoxPublisher()
    while pub.active:
        time.sleep(1)
        messages = np.array([0.1, 0.2, 0.3, 0.4])
        pub.publish(messages, True)
    print("Shutting down")
    pub.close()