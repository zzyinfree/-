#!/usr/bin/env python3
"""Non-deployed FM9G4B-V LoRA experiment on sortie-disjoint down-camera frames."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage14_down_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage14_down_v3'
task.main()
