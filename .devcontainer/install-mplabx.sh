#!/usr/bin/env bash
# Installs the MPLAB X headless debugger CLI (mdb) into /opt/microchip.
#
# Microchip's download endpoints sit behind bot detection, so the
# artifacts must be fetched once with a real browser and dropped into the
# gitignored scratch/ directory:
#
#   scratch/mplabx-linux-installer.tar.gz   MPLAB X IDE linux installer
#                                           (microchip.com -> MPLAB X IDE)
#   scratch/avr-dx-dfp.atpack               AVR-Dx DFP, needed by mdb to
#                                           open an AVR128DA64 target
#
# /opt/microchip is a named docker volume (see devcontainer.json), so the
# roughly 4 GB install survives container rebuilds. Delete the volume to
# force a clean reinstall. A missing installer is not an error: the
# container stays fully usable, only the debugger is absent.
set -euo pipefail

MPLAB_ROOT=/opt/microchip
SCRATCH=/workspaces/cellguard/scratch
INSTALLER="$SCRATCH/mplabx-linux-installer.tar.gz"
DFP="$SCRATCH/avr-dx-dfp.atpack"

# The named volume first mounts root-owned; hand it to the container user.
sudo mkdir -p "$MPLAB_ROOT"
sudo chown "$(id -u):$(id -g)" "$MPLAB_ROOT"

MDB=$(find "$MPLAB_ROOT" -name mdb -type f -path '*mplab_platform/bin*' 2>/dev/null | head -1 || true)
if [[ -n "$MDB" ]]; then
	sudo ln -sf "$MDB" /usr/local/bin/mdb
	echo "install-mplabx: mdb already present at $MDB"
	exit 0
fi

if [[ ! -f "$INSTALLER" ]]; then
	echo "install-mplabx: $INSTALLER not found." >&2
	echo "Download the MPLAB X IDE linux installer (.tar.gz) with a browser from" >&2
	echo "https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide" >&2
	echo "and save it as scratch/mplabx-linux-installer.tar.gz, then recreate the container." >&2
	exit 0
fi

# The installer ships without a JVM; mdb is a wrapper around java. The
# DFP step needs unzip regardless, and the base image may carry java
# already.
sudo apt-get update -yqq
sudo apt-get install -yqq --no-install-recommends default-jre-headless unzip

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
tar -xzf "$INSTALLER" -C "$work"

# Unattended install to the default /opt/microchip/mplabx/<version>.
sudo sh "$work"/MPLABX-*-linux-installer.sh \
	--mode unattended --ipelnet --nodisplayblank --noinstallnote \
	--notoolchainupdate --skipDocs

MDB=$(find "$MPLAB_ROOT" -name mdb -type f -path '*mplab_platform/bin*' 2>/dev/null | head -1 || true)
if [[ -z "$MDB" ]]; then
	echo "install-mplabx: install finished but no mdb binary found under $MPLAB_ROOT" >&2
	exit 1
fi
sudo ln -sf "$MDB" /usr/local/bin/mdb

# Optional: the AVR-Dx DFP carries the debug executive for the AVR128DA64.
if [[ -f "$DFP" ]]; then
	ver=$(basename "$DFP" .atpack | sed 's/^Microchip.AVR-Dx_DFP\.//')
	dest="$HOME/.mchp_packs/Microchip/AVR-Dx_DFP/$ver"
	mkdir -p "$dest"
	unzip -oq "$DFP" -d "$dest"
	echo "install-mplabx: AVR-Dx DFP $ver installed"
else
	echo "install-mplabx: $DFP not found; mdb will not open AVR-Dx targets." >&2
	echo "Fetch the AVR-Dx DFP .atpack with a browser and save it as scratch/avr-dx-dfp.atpack." >&2
fi

echo "install-mplabx: mdb at $MDB"
