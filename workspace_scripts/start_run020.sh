#!/usr/bin/env bash
set -eo pipefail
cd /root/gpufree-data/cicc_workspace
tmux new-session -d -s cicc_run020_rec \
  'bash /root/gpufree-data/cicc_workspace/run020_capture.sh > /root/gpufree-data/cicc_workspace/outputs/run020_capture.log 2>&1'
sleep 3
tmux new-session -d -s cicc_run020_trial \
  'bash /root/gpufree-data/cicc_workspace/run019_trial.sh > /root/gpufree-data/cicc_workspace/outputs/run020_blue_online.log 2>&1'
tmux list-sessions | grep cicc_run020
