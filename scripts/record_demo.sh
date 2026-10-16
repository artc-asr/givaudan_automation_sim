#!/usr/bin/env bash
# Run a planned demo in Gazebo headless and record it, then stop everything it started.
#   scripts/record_demo.sh <plan.json> <out_dir> [timeout_s]
#   e.g. scripts/record_demo.sh reachability/plans/task1_L07.json /tmp/run_task1
# Then: env -u PYTHONPATH deps/curobo-venv/bin/python tools/web_replay/build.py <out_dir>
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
plan=$(realpath "$1") out=$(realpath -m "$2") limit=${3:-600}
source "$root/scripts/env.sh" harmonic || exit 1
rm -rf "$out"
log="$out.log"
setsid ros2 launch givaudan_bringup demo.launch.py plan:="$plan" record:="$out" gui:=false ${WORLD:+world:=$WORLD} > "$log" 2>&1 &
pgid=$!
trap 'kill -INT -- -$pgid 2>/dev/null; sleep 5; kill -KILL -- -$pgid 2>/dev/null' EXIT
for ((i = 0; i < limit; i++)); do
    [ -f "$out/recording.json" ] && break
    kill -0 $pgid 2>/dev/null || break
    sleep 1
done
sleep 2
echo "== demo log (givaudan_demo):"
grep -E "\[givaudan_demo\]|record_run\]: wrote|demo_runner.py.*(Error|Traceback)" "$log" | sed -E 's/^\[[^]]*\] //; s/\[[0-9.]+\] //'
[ -f "$out/recording.json" ] && echo "recorded: $out" || { echo "no recording (log: $log)"; exit 1; }
