#!/bin/bash
# bozo_state_check.sh BINARY SAVE_AT END OUTDIR [PAD_SCRIPT]
# Checks VP2S save states on the Bozo route: one run to END saves a state at SAVE_AT, a second
# process loads it and runs to END. EE RAM at END and the last frame's capture must be identical.
# Tests take no host input and the run is deterministic per frame (VP2S_BOZO_SEED=1).
set -u
BIN=$1; AT=$2; END=$3; D=$4; PAD=${5:-tools/pad_scripts/bozo_logun_armory.txt}
ISO='Valkyrie Profile 2 - Silmeria (USA).iso'
mkdir -p "$D"; rm -f "$D"/a* "$D"/b* "$D"/s.state
export VP2S_BOZO_SEED=1 PS2X_PAD_NO_HOST=1 PS2X_WINDOW_NO_FOCUS=1
export VP2S_BOZO_CAPTURE_FROM=$((END-1)) VP2S_BOZO_CAPTURE_EVERY=1
VP2S_BOZO_END_RAM="$D/a.bin" nice "$BIN" --iso "$ISO" --frames "$END" --pad-script "$PAD" \
    --capture "$D/a.png" --save-state "$D/s.state" --save-state-at "$AT" > "$D/a.log" 2>&1
echo "save run rc=$?"; grep -a "\[state\]" "$D/a.log" | head -3
VP2S_BOZO_END_RAM="$D/b.bin" nice "$BIN" --iso "$ISO" --frames "$END" --pad-script "$PAD" \
    --capture "$D/b.png" --load-state "$D/s.state" > "$D/b.log" 2>&1
echo "load run rc=$?"; grep -a "\[state\]" "$D/b.log" | head -3
rc=0
if cmp -s "$D/a.bin" "$D/b.bin"; then echo "RAM identical"; else echo "RAM differs"; rc=1; fi
for f in "$D"/a_*.png; do
    g="$D/b_${f##*/a_}"
    if cmp -s "$f" "$g"; then echo "${f##*/} identical"; else echo "${f##*/} differs"; rc=1; fi
done
exit $rc
