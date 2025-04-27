"""
Objects for passing data between two real-time loops.
"""

import zmq
import numpy as np



class ZmqBinarySynchB:
    def __init__(self, bindport="tcp://*:5558", connectport="tcp://localhost:5557"):
        self.context = zmq.Context()
        self.socketB = self.context.socket(zmq.PUB)
        self.socketB.bind(bindport)

        self.socketA = self.context.socket(zmq.SUB)
        self.socketA.setsockopt(zmq.SUBSCRIBE, b'A')
        self.socketA.connect(connectport)

        self.my_count = 0
        self.data_out = None


    def update(self, data_in):
        """ read all messages, then send data."""
        while True:
            try: 
                message = self.socketA.recv(zmq.NOBLOCK)
                tokens = message.split(b',')
                # print("B update message = ", tokens)
                assert(tokens[0]==b'A')
                
            
                if int(tokens[1])==self.my_count:
                    self.my_count+=1
                    if self.my_count>=1000:
                        self.my_count=0
                # self.data_out = np.frombuffer(message[4:])
                self.data_out = np.array([float(x) for x in tokens[2:]])
            except zmq.error.Again:
                break
    
        data_str = ",".join([str(x) for x in data_in])
        self.socketB.send(b"B,%03d,%s"%(self.my_count, data_str.encode('utf-8')))#data_in.tobytes()))
        return self.data_out

class ZmqBinarySynchA:
    def __init__(self, bindport="tcp://*:5557", connectport="tcp://localhost:5558"):
        self.context = zmq.Context()
        self.socketA = self.context.socket(zmq.PUB)
        self.socketA.bind(bindport)

        self.socketB = self.context.socket(zmq.SUB)
        self.socketB.setsockopt(zmq.SUBSCRIBE, b'B')
        self.socketB.connect(connectport)

        self.my_count = -42
        self.data_out = None

    def update(self, data_in):
        """ read all messages, then send data. """
        while True:
            try: 
                message = self.socketB.recv(zmq.NOBLOCK)
                print(message)
                tokens = message.split(b',')
                assert(tokens[0]==b'B')
                self.my_count=int(tokens[1])
                self.data_out = np.array([float(x) for x in tokens[2:]])#np.frombuffer(message[4:])
            except zmq.error.Again:
                break

        data_str = ",".join([str(x) for x in data_in])
        self.socketA.send(b"A,%03d,%s"%(self.my_count, data_str.encode('utf-8')))

        return self.data_out


def main():
    # synchA = ZmqBinarySynchA(
    # bindport="ipc:///tmp/feeds/30",
    # connectport="ipc:///tmp/feeds/31")
    synchB = ZmqBinarySynchB(
    bindport="ipc:///tmp/feeds/31",
    connectport="ipc:///tmp/feeds/30")

    for i,t in enumerate(SoftRealtimeLoop(0.0020, report=True)):

        # x1 = synchA.update(np.array([42.1, t*1000]))
        x2 = synchB.update(np.array([0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001]))
        if i%100==0:
            print(x2)
   
if __name__ == '__main__':
    from SoftRealtimeLoop import SoftRealtimeLoop
    main()