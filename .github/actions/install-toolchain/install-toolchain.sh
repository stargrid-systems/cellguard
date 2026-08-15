#!/usr/bin/env bash
# Installs the Rust toolchain pinned in rust-toolchain.toml plus optional
# rustup components.
set -euo pipefail

toolchain=$(grep -oP 'channel = "\K[^"]+' rust-toolchain.toml)
rustup toolchain install --profile minimal "$toolchain"
if [ -n "$COMPONENTS" ]; then
  # shellcheck disable=SC2086
  rustup component add --toolchain "$toolchain" $COMPONENTS
fi
