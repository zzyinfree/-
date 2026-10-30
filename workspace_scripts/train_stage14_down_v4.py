#!/usr/bin/env python3
"""Second independent sortie-disjoint FM9G4B-V LoRA data experiment."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage14_down_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage14_down_v4'
task.main()
