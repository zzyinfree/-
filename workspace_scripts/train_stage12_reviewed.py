#!/usr/bin/env python3
"""One-epoch BF16 LoRA baseline on stage12 manually reviewed simulator frames.

This is a data/protocol smoke run, not a competition-ready detector.
"""
from pathlib import Path
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage12_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage12_reviewed_seed'
task.main()
