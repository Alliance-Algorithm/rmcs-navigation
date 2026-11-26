"""Exercise the real dispatcher with an isolated tmux server and a fake recorder."""

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import time


source = Path(__file__).resolve().parents[2] / "scripts/record-control.sh"
tmux = shutil.which("tmux")
if not tmux:
    raise SystemExit(77)

with tempfile.TemporaryDirectory(prefix="rmcs record dispatch ") as temporary:
    root = Path(temporary)
    scripts = root / "scripts with spaces"
    scripts.mkdir()
    state = root / "state with spaces"
    state.mkdir()
    binaries = root / "bin"
    binaries.mkdir()
    socket = str(root / "tmux.socket")
    wrapper = binaries / "tmux"
    wrapper.write_text(
        "#!/bin/sh\nexec " + shlex.quote(tmux) + " -S " + shlex.quote(socket) + ' "$@"\n'
    )
    wrapper.chmod(0o755)
    environment = dict(os.environ, PATH=str(binaries) + os.pathsep + os.environ["PATH"])
    environment.pop("TMUX", None)
    setup = root / "env with spaces.bash"
    setup.write_text("export RMCS_DISPATCH_TEST='loaded from setup'\n")
    shutil.copyfile(source, scripts / "record-control.sh")
    (scripts / "record-session.sh").write_text(
        '''#!/usr/bin/env bash
set -e
state=$2
test "$RMCS_DISPATCH_TEST" = 'loaded from setup'
printf 'launch\n' >>"$state/launches"
printf 'recording fake-bag\n' >"$state/record.status"
while true; do
    if [ -f "$state/record.abort" ]; then
        printf 'aborted\n' >"$state/record.status"
        exit 0
    fi
    if [ -f "$state/record.stop" ]; then
        printf 'success fake.pcd\n' >"$state/record.status"
        exit 0
    fi
    sleep 0.02
done
'''
    )

    def control(operation):
        return subprocess.run(
            ["bash", str(scripts / "record-control.sh"), operation, str(setup), str(state)],
            env=environment,
            capture_output=True,
            text=True,
            timeout=5,
        )

    def wait_for(predicate):
        deadline = time.monotonic() + 5
        while not predicate():
            if time.monotonic() >= deadline:
                raise AssertionError("dispatcher did not reach the expected state")
            time.sleep(0.02)

    def status():
        try:
            return (state / "record.status").read_text().strip()
        except FileNotFoundError:
            return ""

    try:
        assert control("probe").returncode != 0
        # Clear stale stop markers from the preceding workflow before launching.
        (state / "record.stop").touch()
        (state / "record.abort").touch()
        result = control("start")
        assert result.returncode == 0, result.stderr
        wait_for(lambda: status() == "recording fake-bag")
        assert control("probe").returncode == 0
        assert not (state / "record.stop").exists()
        assert not (state / "record.abort").exists()
        assert control("start").returncode == 0
        assert (state / "launches").read_text().splitlines() == ["launch"]
        assert control("stop").returncode == 0
        wait_for(lambda: status() == "success fake.pcd")
        wait_for(lambda: control("probe").returncode != 0)

        assert control("start").returncode == 0
        wait_for(lambda: status() == "recording fake-bag")
        assert control("abort").returncode == 0
        wait_for(lambda: status() == "aborted")
        wait_for(lambda: control("probe").returncode != 0)
        assert len((state / "launches").read_text().splitlines()) == 2
        setup.unlink()
        assert control("start").returncode != 0, "missing environment reported a successful launch"
        assert control("invalid-operation").returncode != 0
    finally:
        subprocess.run(
            [tmux, "-S", socket, "kill-server"], capture_output=True, check=False
        )

print("record_control: start/adopt/stop/abort and quoted paths passed with real tmux")
