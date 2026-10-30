#!/usr/bin/env python3
"""One diagnostic request; never commands a mission or writes to demo /objects."""
import json
import argparse
import threading
import time
import uuid

import rospy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--class-name', default='square_frame')
    parser.add_argument('--prompt', default='请在这张完整图像中定位可穿越的方形框架。只输出一个 <box>[x1,y1,x2,y2]</box>，坐标范围为0到1000。')
    args = parser.parse_args()
    rospy.init_node('cicc_jiuge_shadow_test', anonymous=True, disable_signals=True)
    request_id = 'shadow-' + uuid.uuid4().hex[:8]
    payload = {'request_id': request_id, 'class_name': args.class_name,
               'prompt': args.prompt}
    seen = threading.Event()
    result = {}

    def callback(msg):
        try:
            rec = json.loads(msg.data)
            if rec.get('request_id') == request_id:
                result.update(rec)
                seen.set()
        except ValueError:
            pass

    rospy.Subscriber('/jiuge/audit', String, callback, queue_size=5)
    pub = rospy.Publisher('/jiuge/request', String, queue_size=1, latch=True)
    until = time.monotonic() + 10
    while pub.get_num_connections() < 1 and time.monotonic() < until:
        time.sleep(0.1)
    if pub.get_num_connections() < 1:
        raise RuntimeError('vision bridge not connected')
    pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
    if not seen.wait(50):
        raise TimeoutError('no matching audit response')
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
