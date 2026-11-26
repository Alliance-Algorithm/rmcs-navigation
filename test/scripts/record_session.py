"""Run the real recording workflow against a recorder inheriting ignored SIGINT."""

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest


SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"


class RecordSessionTest(unittest.TestCase):
    def run_workflow(self, mode="normal", stop="stop"):
        with tempfile.TemporaryDirectory(prefix="rmcs recording shutdown ") as temporary:
            root = Path(temporary)
            scripts = root / "scripts"
            scripts.mkdir()
            shutil.copyfile(SCRIPTS / "record-session.sh", scripts / "record-session.sh")
            state = root / "state"
            state.mkdir()
            binaries = root / "bin"
            binaries.mkdir()
            setup = root / "env_setup.bash"
            setup.write_text(":\n")
            (scripts / "stop-navigation.sh").write_text(
                'printf "closed\\n" >>"$RMCS_RECORD_STATE_DIR/navigation-stops"\n'
            )
            # Isolate the shutdown/handoff boundary; no real navigation or sensor processes.
            (scripts / "record-map.sh").write_text(
                '''set -e
test -f "$RMCS_RECORD_BAG/metadata.yaml"
if kill -0 "$(cat "$RMCS_RECORD_BAG/pid")" 2>/dev/null; then
    echo 'mapper started before recorder exited' >&2
    exit 1
fi
printf 'mapped\n' >"$RMCS_RECORD_STATE_DIR/mapper-started"
printf 'success test.pcd\n' >"$RMCS_RECORD_STATE_DIR/record-map.status"
'''
            )
            recorder = binaries / "ros2"
            recorder.write_text(
                '''#!/usr/bin/python3
import json, os, pathlib, signal, sys, time
if sys.argv[1:3] == ['topic', 'list']:
    print('/livox/lidar_test\\n/livox/imu_test')
    sys.exit(0)
assert sys.argv[1:3] == ['bag', 'record'], sys.argv
bag = pathlib.Path(sys.argv[sys.argv.index('-o') + 1])
bag.mkdir()
mode = os.environ['RMCS_TEST_RECORDER_MODE']
state = pathlib.Path(os.environ['RMCS_RECORD_STATE_DIR'])
stopped = False
def terminate(*_):
    global stopped
    (state / 'term-received').write_text('SIGTERM')
    stopped = True
signal.signal(signal.SIGTERM, signal.SIG_IGN if mode == 'hang' else terminate)
(bag / 'pid').write_text(str(os.getpid()))
(bag / 'signals.json').write_text(json.dumps({'int_ignored': signal.getsignal(signal.SIGINT) == signal.SIG_IGN}))
(bag / 'data.mcap').write_bytes(b'recorded lidar and imu')
while not stopped:
    time.sleep(0.005)
if mode == 'error':
    sys.exit(7)
(bag / 'metadata.yaml').write_text('flushed: true\\n')
sys.exit(0)
'''
            )
            recorder.chmod(0o755)
            # Compress only the script's shutdown polling interval, so force-kill
            # coverage does not take 30 seconds. The recorder uses Python time.sleep.
            sleep = binaries / "sleep"
            sleep.write_text(
                '#!/bin/sh\nif [ "$1" = 0.1 ]; then exec /bin/sleep 0.002; fi\nexec /bin/sleep "$@"\n'
            )
            sleep.chmod(0o755)
            environment = dict(
                os.environ,
                PATH=str(binaries) + os.pathsep + os.environ["PATH"],
                RMCS_RECORD_STATE_DIR=str(state),
                RMCS_RECORD_ROOT=str(root / "bags"),
                RMCS_TEST_RECORDER_MODE=mode,
            )
            process = subprocess.Popen(
                ["bash", str(scripts / "record-session.sh"), str(setup), str(state)],
                env=environment,
                start_new_session=True,
            )

            def await_recording():
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    status_file = state / "record.status"
                    status = status_file.read_text() if status_file.exists() else ""
                    if status.startswith("recording "):
                        return Path((state / "last-record-bag").read_text().strip())
                    if process.poll() is not None:
                        break
                    time.sleep(0.01)
                self.fail("recorder did not become ready: " + (state / "record-session.log").read_text())

            try:
                bag = await_recording()
                signals = json.loads((bag / "signals.json").read_text())
                self.assertTrue(signals["int_ignored"], "test did not reproduce inherited ignored SIGINT")
                (state / ("record." + stop)).touch()
                result = process.wait(timeout=8)
                status = (state / "record.status").read_text().strip()
                log = (state / "record-session.log").read_text()
                self.assertTrue((bag / "data.mcap").exists(), "raw data was deleted")
                self.assertTrue((state / "navigation-stops").exists(), "shutdown cleanup was skipped")
                pid = int((bag / "pid").read_text())
                with self.assertRaises(ProcessLookupError, msg="recorder left running"):
                    os.kill(pid, 0)
                if mode == "normal":
                    self.assertEqual(result, 0, log + status)
                    self.assertTrue((state / "term-received").exists())
                    self.assertTrue((bag / "metadata.yaml").exists())
                    if stop == "stop":
                        self.assertEqual(status, "success test.pcd")
                        self.assertTrue((state / "mapper-started").exists())
                    else:
                        self.assertTrue(status.startswith("aborted "), status)
                        self.assertFalse((state / "mapper-started").exists())
                else:
                    self.assertNotEqual(result, 0)
                    self.assertTrue(status.startswith("failed:"), status)
                    self.assertFalse((state / "mapper-started").exists())
                    self.assertFalse((bag / "metadata.yaml").exists())
            finally:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()

    def test_normal_sigterm_flushes_and_starts_mapper(self):
        self.run_workflow()

    def test_abort_flushes_without_mapping(self):
        self.run_workflow(stop="abort")

    def test_recorder_error_does_not_start_mapper(self):
        self.run_workflow(mode="error")

    def test_force_kill_does_not_start_mapper(self):
        self.run_workflow(mode="hang")


if __name__ == "__main__":
    unittest.main()
