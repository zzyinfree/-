#!/usr/bin/env bash
set -eo pipefail
cd /root/gpufree-data/cicc_workspace
tmux new-session -d -s cicc_run021_rec \
  'bash /root/gpufree-data/cicc_workspace/run021_capture.sh > /root/gpufree-data/cicc_workspace/outputs/run021_capture.log 2>&1'
sleep 3
tmux new-session -d -s cicc_run021_trial \
  'bash /root/gpufree-data/cicc_workspace/run019_trial.sh > /root/gpufree-data/cicc_workspace/outputs/run021_blue_online.log 2>&1'
tmux list-sessions | grep cicc_run021
