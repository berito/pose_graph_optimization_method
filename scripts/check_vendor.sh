#!/usr/bin/env bash
# Is the vendored code still the vendored code?
#
# ⭐ The repo's central rule is "build ON the vendor, never IN it" — and a rule nothing
# checks is a rule that drifts. This diffs what we hold against the pinned upstream
# clone kept next door in pose_graph_optimization_experiments/code/, which is cloned at
# an exact commit and mounted read-only there.
#
# The test: does a difference change WHAT THE CODE DOES, or only WHERE IT LOOKS?
#
#   cfg/*.yaml      WHERE. Authors hard-code absolute paths from their own machines
#                   (IPC ships /home/slam-emix/Datasets/...), so they are unrunnable as
#                   shipped. Counted, not flagged.
#   CMakeLists.txt  ⚠ MOSTLY where — but a compile flag CAN change numerics
#                   (-DGTSAM_POSE3_EXPMAP picks a different retraction on SE(3)), so
#                   these must be RECORDED in vendor_patches.txt even though allowed.
#   any .cpp/.h     WHAT. Never edited in place; the fixed copy goes in vendor_patched/.
#
#   usage: scripts/check_vendor.sh          exit 0 clean · 1 drift · 2 cannot check
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UP="$ROOT/../pose_graph_optimization_experiments/code"

if [ ! -d "$UP" ]; then
  echo "⚠ cannot check: no pinned upstream at $UP"
  echo "  clone the experiments repo beside this one, then: bootstrap.py sync"
  exit 2
fi

# ours -> the pinned upstream of the same thing.
#
# ⚠ `taco/` is deliberately NOT here. It is a REIMPLEMENTATION (kBL-IPC + SR-SC built on
# ipc/), not a copy of Olivastri's TACO — its files (kbl_consensus, srsc, taco_simulation)
# do not exist upstream, and upstream's (taco.cpp, recovery.cpp, simulation.cpp) do not
# exist here. Comparing them reported 24 "drifts" that were all just two different
# codebases. Only vendored copies belong in this list.
declare -A PAIRS=( [ipc]=IPC [baselines]=RobustOptimizationSLAM )
rc=0
for ours in "${!PAIRS[@]}"; do
  theirs="$UP/${PAIRS[$ours]}"
  [ -d "$ROOT/$ours" ] || continue
  if [ ! -d "$theirs" ]; then
    echo "  ⚠ $ours — no upstream on disk ($theirs). Run bootstrap.py paper <it> next door."
    rc=2; continue
  fi
  # Code only. cfg/ and CMakeLists.txt differ by design (authors hard-code absolute paths);
  # build artifacts are not code.
  raw="$(diff -rq "$theirs" "$ROOT/$ours" 2>/dev/null \
         | grep -vE '\.git|/build/|/cfg/|CMakeLists\.txt|CMakeFiles|README|\.md$|build_command|^Only in .*: build$')"

  # Config differences are allowed, but never invisible: a compile flag can change a
  # result, so a changed CMakeLists must still be recorded.
  cfgn="$(diff -rq "$theirs" "$ROOT/$ours" 2>/dev/null | grep '^Files ' | grep -c '/cfg/' || true)"
  [ "$cfgn" -gt 0 ] && echo "  · $ours — $cfgn config file(s) differ (paths; not the algorithm)"
  for cm in $(diff -rq "$theirs" "$ROOT/$ours" 2>/dev/null | grep '^Files ' | grep 'CMakeLists\.txt' \
              | sed "s|$theirs/||; s|^Files ||; s/ and .* differ//"); do
    if grep -q "^$ours/$cm	" "$ROOT/scripts/vendor_patches.txt" 2>/dev/null; then
      echo "  · $ours/$cm — recorded build change"
    else
      echo "  ⚠ $ours/$cm — build file changed and NOT recorded."
      echo "      A compile flag can change a result. Add it to scripts/vendor_patches.txt."
      rc=1
    fi
  done

  # ⭐ Three findings, and only ONE of them breaks the rule.
  #   modified  — a vendored file we EDITED IN PLACE. This is the violation.
  #   added     — a file only we have. Allowed: a new baseline is an addition, not an edit,
  #               and it cannot change what their code does.
  #   upstream  — a file only they have. We took a subset. Not a violation either.
  modified="$(echo "$raw" | grep '^Files ' || true)"
  added="$(echo "$raw" | grep "^Only in $ROOT/$ours" || true)"
  upstream="$(echo "$raw" | grep '^Only in ' | grep -v "^Only in $ROOT/$ours" || true)"

  # ⭐ A recorded patch is not a violation. scripts/vendor_patches.txt lists every
  # in-place edit with the file that explains it; an edit missing from that list is
  # reported every run until it is moved out or written down.
  unrecorded=""
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    f="$(echo "$line" | sed "s|$theirs/||; s|$ROOT/$ours/||; s/^Files //; s/ and .* differ//")"
    if grep -q "^$ours/$f	" "$ROOT/scripts/vendor_patches.txt" 2>/dev/null; then
      echo "  · $ours/$f — recorded patch ($(grep "^$ours/$f	" "$ROOT/scripts/vendor_patches.txt" | cut -f3))"
    else
      unrecorded="$unrecorded$f"$'\n'
    fi
  done <<< "$modified"
  modified="$(echo "$unrecorded" | grep -v '^$' || true)"

  if [ -z "$modified" ]; then
    extra=""
    [ -n "$added" ] && extra=" ($(echo "$added" | grep -c .) of ours added)"
    echo "  ✅ $ours — no vendored file edited in place$extra"
  else
    echo "  ⛔ $ours — EDITED IN PLACE:"
    echo "$modified" | sed 's/^/      /'
    echo "      ⚠ not in scripts/vendor_patches.txt"
    rc=1
  fi
done

echo
case $rc in
  0) echo "  vendor clean — what we changed is all in our own directories." ;;
  1) echo "  ⛔ A vendored file was edited in place and NOT recorded. Either:"
     echo "     • move the change into our own tree — substitute it in a CMake source"
     echo "       list, or fork the file out under a new name (see CLAUDE.md), or"
     echo "     • if it is a genuine build-portability fix, explain it in the vendor's"
     echo "       own notes and add a line to scripts/vendor_patches.txt." ;;
  2) echo "  ⚠ incomplete — could not compare everything." ;;
esac
exit $rc
