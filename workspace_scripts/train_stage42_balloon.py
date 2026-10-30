#!/usr/bin/env python3
"""Independent one-epoch balloon-balanced LoRA; never swaps the flight model."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage42_balanced_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage42_balloon_v1'
task.main()
