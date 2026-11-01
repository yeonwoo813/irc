#!/bin/bash
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo 'Run this installer with sudo in the Jetson terminal.' >&2
    exit 1
fi
recorder_source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
recorder_user=${SUDO_USER:-${PKEXEC_UID:-root}}
recorder_group=$(id -gn "$recorder_user")

# Install a root-owned copy; the service never executes code from the workspace.
install -d -o root -g root -m 0755 /usr/local/lib/jetson-reboot-recorder
install -o root -g root -m 0644 "$recorder_source_dir/recorder.py" /usr/local/lib/jetson-reboot-recorder/recorder.py
install -d -o root -g root -m 0700 /var/lib/jetson-reboot-recorder
install -d -o root -g "$recorder_group" -m 0750 /var/log/jetson-reboots
for recorder_unit in jetson-reboot-recorder.service jetson-reboot-mark@.service; do
    sed "s/@LOG_GROUP@/$recorder_group/g" "$recorder_source_dir/$recorder_unit" > "/etc/systemd/system/$recorder_unit"
    chmod 0644 "/etc/systemd/system/$recorder_unit"
done
systemd-analyze verify /etc/systemd/system/jetson-reboot-recorder.service /etc/systemd/system/jetson-reboot-mark@reboot.service
systemctl daemon-reload
# Enable the marker instances only. Never start a shutdown target here.
systemctl enable jetson-reboot-mark@reboot.service jetson-reboot-mark@poweroff.service jetson-reboot-mark@halt.service jetson-reboot-mark@kexec.service
systemctl enable jetson-reboot-recorder.service
systemctl restart jetson-reboot-recorder.service
systemctl --no-pager --full status jetson-reboot-recorder.service
python3 - <<'PY'
import json
from pathlib import Path
state = json.loads(Path('/var/lib/jetson-reboot-recorder/state.json').read_text())
assert state['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert state['capture_mode'] == 'event_only'
print('Event-only recorder installed: no resident collector, sensor polling, or periodic writes.')
print('Reboot reports: /var/log/jetson-reboots/latest/summary.json')
print('First installation and ordinary boot after a recorded poweroff do not create a report.')
print('Power removal is indistinguishable from some resets; those are labeled unclean_restart.')
PY
