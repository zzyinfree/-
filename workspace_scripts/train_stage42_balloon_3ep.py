#!/usr/bin/env python3
"""Separate three-epoch candidate; one-epoch candidate remains immutable."""
import train_jiuge_provisional_v1 as task

task.DATA = task.ROOT / 'annotations/stage42_balanced_reviewed_bbox.jsonl'
task.OUT = task.ROOT / 'adapters/lora_stage42_balloon_3ep_v2'
task.main()
