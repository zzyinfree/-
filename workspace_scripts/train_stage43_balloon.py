#!/usr/bin/env python3
"""Independent stage43 LoRA, with extra blue/purple original frames."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage43_balanced_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage43_balloon_3ep_v1'
task.main()
