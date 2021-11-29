from pylsl import StreamInfo, StreamOutlet
from pylsl import StreamInlet, resolve_stream
from collections import deque
from msgpack import unpackb
import zmq
import argparse

"""
Receive world camera data from Pupil using ZMQ.
Make sure the frame publisher plugin is loaded and configuredsource  to gray or rgb
"""


class LSL:

    def __init__(self, ):
        context = zmq.Context()
        # open a req port to talk to pupil
        addr = '127.0.0.1'  # remote ip or localhost
        req_port = "44805"  # same as in the pupil remote gui
        req = context.socket(zmq.REQ)
        req.connect("tcp://{}:{}".format(addr, req_port))
        # ask for the sub port
        req.send_string('SUB_PORT')
        sub_port = req.recv_string()

        # create a new stream info
        info = StreamInfo("GazePose", "NormPose2IP", 4, 100, "float32", "myuid34234")
        info.desc().append_child_value("manufacturer", "Vislab")
        self.outlet = StreamOutlet(info)

        # open a sub port to listen to pupil
        self.sub = context.socket(zmq.SUB)
        self.sub.connect("tcp://{}:{}".format(addr, sub_port))

        # set subscriptions to topics
        # recv just pupil/gaze/notifications
        self.sub.setsockopt_string(zmq.SUBSCRIBE, 'frame.')

        # construct the argument parse and parse the arguments
        ap = argparse.ArgumentParser()
        ap.add_argument("-v", "--video", help="path to the (optional) video file")
        ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
        args = vars(ap.parse_args())
        self.pts = deque(maxlen=args["buffer"])

        print("looking for an NormPose2IP stream...")
        streams = resolve_stream('name', 'NormPose2IP')
        # create an inlet to read from the stream
        self.inlet = StreamInlet(streams[0])

    def recv_from_sub(self, ):
        """
        Recv a message with topic, payload.

        Topic is a utf-8 encoded string. Returned as unicode object.
        Payload is a msgpack serialized dict. Returned as a python dict.

        Any addional message frames will be added as a list
        in the payload dict with key: '__raw_data__' .
        """

        topic = self.sub.recv_string()
        payload = unpackb(self.sub.recv())#, encoding='utf-8')
        extra_frames = []
        while self.sub.get(zmq.RCVMORE):
            extra_frames.append(self.sub.recv())
        if extra_frames:
            payload['__raw_data__'] = extra_frames

        return topic, payload






