#!/usr/bin/env python3
"""Observe raw down-camera frame at ArUco event and independently ask FM9G4B-V for bbox.

This is an audit/display node only. It must not publish /aruco/detected_id or
vehicle commands; the classical decoder remains responsible for the marker ID.
"""
import json
import re
import threading
import time
import traceback
from pathlib import Path

import cv2
import numpy as np
import rospy
import torch
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from transformers import AutoModel, AutoProcessor

ROOT = Path('/root/gpufree-data/cicc_workspace')
MODEL = '/home/ubuntu/model/FM9G4B-V'
ADAPTERS = [ROOT / 'adapters/lora_stage14_down_v4',
            ROOT / 'adapters/lora_stage13_down_v2',
            ROOT / 'adapters/lora_stage12_reviewed_seed']
PROMPT = '请框出图中的地面二维码标识。只输出一个 <box>。'
PAT = re.compile(r'<box>\s*\[\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\]\s*</box>')


class Node:
    def __init__(self):
        rospy.init_node('jiuge_down_marker_observer')
        self.lock = threading.Lock()
        self.frame = None
        self.seq = None
        self.stamp = None
        self.busy = False
        self.last_trigger = 0.0
        self.pub = rospy.Publisher('/jiuge/down_marker_result', String,
                                   queue_size=2, latch=True)
        rospy.Subscriber('/rflysim/sensor2/img_rgb', Image, self.on_image,
                         queue_size=1, buff_size=2**24)
        rospy.Subscriber('/aruco/detected_id', Int32, self.on_marker, queue_size=2)
        rospy.loginfo('FM9G4B-V observer loading; output is audit-only, not flight control')
        self.processor = AutoProcessor.from_pretrained(MODEL, trust_remote_code=True,
                                                       local_files_only=True)
        base = AutoModel.from_pretrained(MODEL, trust_remote_code=True,
                                        local_files_only=True, attn_implementation='sdpa',
                                        torch_dtype=torch.bfloat16).eval().to('cuda')
        adapter = next((path for path in ADAPTERS if path.exists()), None)
        if adapter is not None:
            from peft import PeftModel
            self.model = PeftModel.from_pretrained(base, str(adapter),
                                                  is_trainable=False).eval().base_model.model
            self.adapter = str(adapter)
        else:
            self.model = base
            self.adapter = None
        rospy.loginfo('FM9G4B-V observer ready; adapter=%s', self.adapter)

    def on_image(self, msg):
        try:
            if msg.encoding.lower() == 'rgb8':
                rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()
            elif msg.encoding.lower() == 'bgr8':
                bgr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            else:
                return
            with self.lock:
                self.frame = rgb
                self.seq = int(msg.header.seq)
                self.stamp = msg.header.stamp.to_sec()
        except Exception as exc:
            rospy.logwarn_throttle(3, 'FM9G down frame conversion: %s', exc)

    def on_marker(self, msg):
        with self.lock:
            if self.busy or self.frame is None or time.monotonic()-self.last_trigger < 8:
                return
            frame = self.frame.copy()
            seq, stamp = self.seq, self.stamp
            self.busy = True
            self.last_trigger = time.monotonic()
        threading.Thread(target=self.infer, args=(frame, seq, stamp, int(msg.data)),
                         daemon=True).start()

    def infer(self, rgb, seq, stamp, classical_id):
        started = time.monotonic()
        # Ubuntu desktop viewer cannot traverse /root; keep viewable copies
        # under /home/ubuntu and a persistent audit JSONL on gpufree-data.
        evidence = Path('/home/ubuntu/cicc_visual_evidence')
        evidence.mkdir(parents=True, exist_ok=True)
        evidence.chmod(0o755)
        image_path = evidence / f'frame_{seq}_{int(time.time())}.png'
        PILImage.fromarray(rgb).save(image_path)
        image_path.chmod(0o644)
        result = {'source': 'FM9G4B-V independent raw-image bbox',
                  'classical_aruco_id': classical_id, 'id_source': 'OpenCV ArUco, not FM9G',
                  'frame_seq': seq, 'frame_stamp': stamp,
                  'image': str(image_path), 'prompt': PROMPT,
                  'adapter': self.adapter, 'bbox_px': None, 'valid': False}
        try:
            try:
                with torch.inference_mode():
                    raw = str(self.model.chat(image=PILImage.fromarray(rgb),
                        msgs=[{'role': 'user', 'content': PROMPT}],
                        tokenizer=self.processor.tokenizer, processor=self.processor,
                        sampling=False, num_beams=1, max_new_tokens=64,
                        max_slice_nums=4, use_image_id=True))
            except IndexError:
                # Rare processor slicing failure on one live frame.  Retry the
                # same unaltered full image using one image slice, never crop.
                with torch.inference_mode():
                    raw = str(self.model.chat(image=PILImage.fromarray(rgb),
                        msgs=[{'role': 'user', 'content': PROMPT}],
                        tokenizer=self.processor.tokenizer, processor=self.processor,
                        sampling=False, num_beams=1, max_new_tokens=64,
                        max_slice_nums=1, use_image_id=True))
                result['retry_max_slice_nums'] = 1
            match = PAT.fullmatch(raw.strip())
            vals = list(map(float, match.groups())) if match else None
            valid = bool(vals and len(vals) == 4 and 0 <= vals[0] < vals[2] <= 1000 and
                         0 <= vals[1] < vals[3] <= 1000)
            result['raw'] = raw
            result['valid'] = valid
            result['bbox_1000'] = vals if valid else None
            if valid:
                h, w = rgb.shape[:2]
                result['bbox_px'] = [round(vals[0]*w/1000, 1),
                                     round(vals[1]*h/1000, 1),
                                     round(vals[2]*w/1000, 1),
                                     round(vals[3]*h/1000, 1)]
        except Exception as exc:
            result['error'] = str(exc)
            result['traceback'] = traceback.format_exc()[-3000:]
        result['latency_sec'] = round(time.monotonic()-started, 3)
        result['wall_time'] = time.time()
        line = json.dumps(result, ensure_ascii=False)
        with (evidence / 'results.jsonl').open('a', encoding='utf-8') as f:
            f.write(line+'\n')
        persistent = ROOT / 'outputs/down_marker_evidence'
        persistent.mkdir(parents=True, exist_ok=True)
        with (persistent / 'results.jsonl').open('a', encoding='utf-8') as f:
            f.write(line+'\n')
        self.pub.publish(String(data=line))
        rospy.loginfo('FM9G down result: %s', line)
        with self.lock:
            self.busy = False


if __name__ == '__main__':
    Node()
    rospy.spin()
