#!/usr/bin/env bash
# Prints the reachable workspace roots of the repository as a JSON array.
# A manifest is a root if it declares a [workspace] table, or if it is a
# standalone package with no ancestor workspace to inherit from.
set -euo pipefail

if [ -z "${GITHUB_OUTPUT:-}" ]; then
  echo "GITHUB_OUTPUT is not set" >&2
  exit 1
fi

roots=()
while IFS= read -r manifest; do
  dir=${manifest%/*}
  dir=${dir#./}
  if grep -q '^\[workspace\]' "$manifest"; then
    roots+=("$dir")
    continue
  fi
  member=0
  ancestor=$(dirname "$dir")
  while [ "$ancestor" != "." ]; do
    if [ -f "$ancestor/Cargo.toml" ] &&
      grep -q '^\[workspace\]' "$ancestor/Cargo.toml"; then
      member=1
      break
    fi
    ancestor=$(dirname "$ancestor")
  done
  if [ "$member" -eq 0 ]; then
    roots+=("$dir")
  fi
done < <(find . -name Cargo.toml -not -path '*/target/*' | sort)

json=$(printf '%s\n' "${roots[@]}" | jq -Rsc 'split("\n") | map(select(length > 0))')
echo "workspaces=$json" >>"$GITHUB_OUTPUT"
