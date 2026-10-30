#!/usr/bin/env bash
set -eo pipefail
cd /home/ubuntu/PX4PSP/RflySimUE5/RflySim3D/Binaries/Linux
exec runuser -u ubuntu -- env DISPLAY=:20 ./RflySim3D -log
