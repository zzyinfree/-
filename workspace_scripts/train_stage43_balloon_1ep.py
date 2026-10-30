#!/usr/bin/env python3
"""One-epoch stage43 ablation, isolated from the three-epoch checkpoint."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage43_balanced_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage43_balloon_1ep_v2'
task.main()
