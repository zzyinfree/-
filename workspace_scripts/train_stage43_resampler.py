#!/usr/bin/env python3
"""Controlled FM9G4B-V LoRA+Resampler visual-domain experiment."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage43_balanced_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage43_resampler_3ep_v3'
task.main()
