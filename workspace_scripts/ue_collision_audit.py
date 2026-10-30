#!/usr/bin/env python3
"""Read-only UE4 collision/pose telemetry for a simulation run.

Requests telemetry for copter 1. Does not send movement, ExtAct, simulator
timing, or scenario-modification commands. CrashType is evidence, not a score.
"""
import argparse
import json
import sys
import time
from pathlib import Path

SDK = Path('/home/ubuntu/PX4PSP/RflySimAPIs/RflySimSDK')
sys.path.insert(0, str(SDK))
from ue.UE4CtrlAPI import UE4CtrlAPI


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--duration', type=float, default=160)
    ap.add_argument('--period', type=float, default=0.1)
    ap.add_argument('--enable-telemetry', action='store_true',
                    help='Enable UE readback for this diagnostic run; restore disabled at end')
    args = ap.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ue = UE4CtrlAPI()
    ue.initUE4MsgRec()
    if args.enable_telemetry:
        ue.sendUE4Cmd(b'RflyReqVehicleData 1')
    ue.reqCopt([1])
    started = time.monotonic()
    last_key = None
    last_sample = 0.0
    samples = 0
    events = 0
    with args.output.open('w', encoding='utf-8', buffering=1) as out:
        out.write(json.dumps({'event':'start','wall_time':time.time(),
            'source':'UE4CtrlAPI reqVeCrashData, copterID=1',
            'interpretation':'raw collision telemetry, not judge score'})+'\n')
        while time.monotonic() - started < args.duration:
            raw = ue.getUE4Data(1)
            if raw:
                key = (raw.CrashType, raw.CrashedName)
                now = time.monotonic()
                if key != last_key or now-last_sample >= 1.0:
                    row = {'event':'sample','wall_time':time.time(),
                        'sim_time':raw.runnedTime,'copter_id':raw.copterID,
                        'crash_type':raw.CrashType,'crashed_name':raw.CrashedName,
                        'pos_e':list(raw.PosE),'vel_e':list(raw.VelE),
                        'crash_pos':list(raw.CrashPos),'target_pos':list(raw.targetPos)}
                    out.write(json.dumps(row, ensure_ascii=False)+'\n')
                    samples += 1
                    if raw.CrashType != 0:
                        events += 1
                        print(json.dumps(row, ensure_ascii=False), flush=True)
                    last_key = key
                    last_sample = now
            time.sleep(args.period)
        out.write(json.dumps({'event':'done','samples':samples,'nonzero_crash_samples':events})+'\n')
    if args.enable_telemetry:
        ue.sendUE4Cmd(b'RflyReqVehicleData 0')
    print(json.dumps({'output':str(args.output),'samples':samples,
                      'nonzero_crash_samples':events}),flush=True)


if __name__ == '__main__':
    main()
