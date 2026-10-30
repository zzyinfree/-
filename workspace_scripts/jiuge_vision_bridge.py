#!/usr/bin/env python3
"""FM9G4B-V raw-image bbox bridge in shadow mode; no flight commands.

Input /jiuge/request: JSON {request_id,class_name,prompt}. The prompt must be
the committee-approved single-step prompt. Output /jiuge/audit JSON and
/jiuge/objects (common_msgs/Objects). Never publishes to demo /objects.
"""
import argparse
import hashlib
import json
import queue
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
import torch
from PIL import Image as PILImage, ImageDraw
from sensor_msgs.msg import Image
from std_msgs.msg import String
from transformers import AutoModel, AutoProcessor
from common_msgs.msg import Obj, Objects

MODEL = '/home/ubuntu/model/FM9G4B-V'
BOX_RE = re.compile(r'<box>\s*\[\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*,\s*(\d+(?:\.\d+)?)\s*\]\s*</box>')
CLASSES = {'square_frame', 'red_balloon', 'blue_balloon', 'purple_balloon',
           'obstacle', 'balloon', 'landing_marker', 'takeoff_marker', 'smoke_bomb'}


def parse_box(raw, width, height):
    match = BOX_RE.fullmatch(raw.strip())
    if match is None:
        return None, 'not_exact_official_box_format'
    norm = [float(x) for x in match.groups()]
    x1,y1,x2,y2 = norm
    if not (0 <= x1 < x2 <= 1000 and 0 <= y1 < y2 <= 1000):
        return None, 'box_out_of_range'
    box = [int(round(x1*width/1000)), int(round(y1*height/1000)),
           int(round(x2*width/1000)), int(round(y2*height/1000))]
    box = [max(0,min(v,(width if i%2==0 else height)-1)) for i,v in enumerate(box)]
    if box[2] <= box[0] or box[3] <= box[1]:
        return None, 'pixel_box_degenerate'
    return box, None


def decode(msg):
    enc = msg.encoding.lower()
    if enc not in ('rgb8','bgr8'):
        raise ValueError('unsupported encoding '+msg.encoding)
    raw = np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)
    rgb = raw[:,:msg.width*3].reshape(msg.height,msg.width,3)
    if enc == 'bgr8':
        rgb = cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)
    return PILImage.fromarray(rgb.copy(),'RGB')


class Bridge:
    def __init__(self,args):
        rospy.init_node('cicc_jiuge_vision_bridge',anonymous=False)
        self.args=args
        self.audit_dir=Path(args.audit_dir)
        self.audit_dir.mkdir(parents=True,exist_ok=True)
        self.latest=None
        self.lock=threading.Lock()
        self.requests=queue.Queue(maxsize=1)
        self.audit=rospy.Publisher('/jiuge/audit',String,queue_size=10)
        self.objects=rospy.Publisher('/jiuge/objects',Objects,queue_size=10)
        rospy.Subscriber(args.image_topic,Image,self.on_image,queue_size=1,buff_size=8*1024*1024)
        rospy.Subscriber('/jiuge/request',String,self.on_request,queue_size=1)
        rospy.loginfo('Loading official FM9G4B-V from %s (shadow-only)',MODEL)
        self.processor=AutoProcessor.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True)
        base=AutoModel.from_pretrained(MODEL,trust_remote_code=True,local_files_only=True,
            torch_dtype=torch.bfloat16,attn_implementation='sdpa').eval().to('cuda')
        if args.adapter:
            from peft import PeftModel
            wrapped=PeftModel.from_pretrained(base,args.adapter,is_trainable=False).eval()
            self.model=wrapped.base_model.model.eval()
            rospy.logwarn('Research LoRA adapter loaded: %s; shadow-only, NOT flight-safe',args.adapter)
        else:
            self.model=base
        threading.Thread(target=self.worker,daemon=True).start()
        rospy.loginfo('Jiuge bridge READY; no /objects or flight command publication')

    def on_image(self,msg):
        with self.lock:
            self.latest=(msg,time.monotonic())

    def emit(self,record):
        self.audit.publish(String(data=json.dumps(record,ensure_ascii=False,separators=(',',':'))))
        rospy.loginfo('VLM audit %s',json.dumps(record,ensure_ascii=False))

    def on_request(self,msg):
        try:
            req=json.loads(msg.data)
            if not isinstance(req,dict):raise ValueError('request must be object')
            rid=str(req['request_id'])
            klass=str(req['class_name'])
            prompt=str(req['prompt'])
            if klass not in CLASSES:raise ValueError('class not allowed')
            if not rid or not prompt.strip() or len(prompt)>1000:raise ValueError('empty/long request')
            self.requests.put_nowait((rid,klass,prompt))
        except queue.Full:
            self.emit({'status':'busy','request':msg.data[:200]})
        except Exception as exc:
            self.emit({'status':'invalid_request','error':str(exc),'request':msg.data[:200]})

    def worker(self):
        while not rospy.is_shutdown():
            try:rid,klass,prompt=self.requests.get(timeout=0.2)
            except queue.Empty:continue
            started=time.monotonic()
            with self.lock:frame=self.latest
            if frame is None or started-frame[1]>self.args.max_image_age:
                self.emit({'request_id':rid,'class_name':klass,'status':'stale_or_missing_image'})
                self.requests.task_done()
                continue
            msg=frame[0]
            record={'request_id':rid,'class_name':klass,'prompt':prompt,
                    'source_topic':self.args.image_topic,'image_seq':msg.header.seq,
                    'image_stamp':msg.header.stamp.to_sec(),'width':msg.width,'height':msg.height,
                    'model':MODEL,'adapter':self.args.adapter or None,'mode':'shadow'}
            try:
                image=decode(msg)
                name=hashlib.sha256(f'{rid}:{msg.header.seq}'.encode()).hexdigest()[:20]
                image_path=self.audit_dir/f'{name}.png'
                image.save(image_path)
                record['image_path']=str(image_path)
                record['image_sha256']=hashlib.sha256(image_path.read_bytes()).hexdigest()
                with torch.inference_mode():
                    raw=str(self.model.chat(image=image,
                        msgs=[{'role':'user','content':prompt}],
                        tokenizer=self.processor.tokenizer,processor=self.processor,
                        sampling=False,num_beams=1,max_new_tokens=64,
                        max_slice_nums=4,use_image_id=True))
                box,error=parse_box(raw,msg.width,msg.height)
                record.update(raw=raw,pixel_box=box,status='valid_box' if box else error,
                              latency_sec=round(time.monotonic()-started,3))
                if box:
                    x1,y1,x2,y2=box
                    marked=image.copy()
                    ImageDraw.Draw(marked).rectangle(box,outline='red',width=3)
                    marked_path=self.audit_dir/f'{name}_bbox.png'
                    marked.save(marked_path)
                    record['bbox_overlay_path']=str(marked_path)
                    obj=Obj()
                    obj.header=msg.header
                    obj.class_name=klass
                    obj.left_top_x=x1; obj.left_top_y=y1
                    obj.right_bottom_x=x2; obj.right_bottom_y=y2
                    obj.center_x=(x1+x2)//2; obj.center_y=(y1+y2)//2
                    obj.score=0.0  # model has no calibrated confidence; never fabricate it
                    objects=Objects()
                    objects.header=msg.header
                    objects.source_id=1
                    objects.objects=[obj]
                    self.objects.publish(objects)
            except Exception as exc:
                record.update(status='inference_error',error=str(exc),
                              latency_sec=round(time.monotonic()-started,3))
            self.emit(record)
            self.requests.task_done()


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--image-topic',default='/rflysim/sensor1/img_rgb')
    ap.add_argument('--max-image-age',type=float,default=2.0)
    ap.add_argument('--audit-dir',default='/root/gpufree-data/cicc_workspace/outputs/jiuge_audit')
    ap.add_argument('--adapter',default='',help='Optional research LoRA. Output remains shadow-only.')
    args=ap.parse_args()
    Bridge(args)
    rospy.spin()


if __name__=='__main__':main()
