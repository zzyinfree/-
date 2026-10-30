#!/usr/bin/env python3
"""Separate FM9G4B-V LoRA experiment; never overwrite the deployed seed."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage13_down_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage13_down_v2'
task.main()
