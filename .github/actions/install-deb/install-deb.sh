#!/usr/bin/env bash
# Downloads .deb packages and extracts them into a prefix under RUNNER_TEMP.
# Each URL is downloaded, unpacked, and merged into a shared root so several
# packages combine into one tree. URL lines starting with # are skipped, which
# allows version comments in the input. The bin directory is added to PATH.
set -euo pipefail

if [ -z "${GITHUB_PATH:-}" ]; then
  echo "GITHUB_PATH is not set" >&2
  exit 1
fi

root="$RUNNER_TEMP/deb-install"
mkdir -p "$root"

while IFS= read -r url; do
  [ -n "$url" ] || continue
  [ "${url:0:1}" = "#" ] && continue
  curl -sSfL --retry 3 -o "$RUNNER_TEMP/pkg.deb" "$url"
  tmp=$(mktemp -d)
  member=$(ar t "$RUNNER_TEMP/pkg.deb" | grep '^data\.tar\.' | head -1)
  (
    cd "$tmp"
    ar x "$RUNNER_TEMP/pkg.deb" "$member"
    tar -xf "$member" -C "$root"
  )
  rm -rf "$tmp"
done <<<"$URLS"

if [ -n "$BIN_DIR" ] && [ -d "$root/$BIN_DIR" ]; then
  echo "$root/$BIN_DIR" >>"$GITHUB_PATH"
fi
