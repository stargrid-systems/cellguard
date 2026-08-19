#!/usr/bin/env bash
# Gates the flash footprint of an AVR workspace.
#
# Usage: flash-size-gate.sh <workspace-dir>
#
# The workspace must hold a flash-budget file (one integer line in bytes) and
# at least one target/avr-none/release/*.elf. The gate sums the text and data
# columns of avr-size over all ELFs, reports measured use against the budget,
# and appends a markdown table to $GITHUB_STEP_SUMMARY when that is set.
# SIZE_CMD overrides the size command for local testing.

set -euo pipefail

if [[ $# -ne 1 ]]; then
	echo "usage: $0 <workspace-dir>" >&2
	exit 2
fi

workspace=$1
size_cmd=${SIZE_CMD:-avr-size}

fail() {
	echo "flash-size-gate: $1" >&2
	exit 1
}

budget_file=$workspace/flash-budget
[[ -f $budget_file ]] || fail "missing budget file: $budget_file"

budget=
while IFS= read -r line || [[ -n $line ]]; do
	line=${line%%#*}
	line=${line//[[:space:]]/}
	if [[ -n $line ]]; then
		budget=$line
		break
	fi
done <"$budget_file"
[[ $budget =~ ^[0-9]+$ ]] || fail "no integer budget line in $budget_file"

shopt -s nullglob
elfs=("$workspace"/target/avr-none/release/*.elf)
shopt -u nullglob
((${#elfs[@]} > 0)) || fail "no ELF in $workspace/target/avr-none/release"

names=()
sizes=()
total=0
for elf in "${elfs[@]}"; do
	if ! bytes=$("$size_cmd" "$elf" | awk 'NR > 1 { sum += $1 + $2 } END { print sum + 0 }'); then
		fail "$size_cmd failed on $elf"
	fi
	[[ $bytes =~ ^[0-9]+$ ]] || fail "unparsable size output for $elf"
	names+=("${elf##*/}")
	sizes+=("$bytes")
	total=$((total + bytes))
done
((total > 0)) || fail "measured 0 bytes for ${elfs[*]}"

name=$(basename "$workspace")

echo
echo "$name flash gate"
printf '  %-24s %10s\n' ELF BYTES
for i in "${!names[@]}"; do
	printf '  %-24s %10d\n' "${names[$i]}" "${sizes[$i]}"
done
printf '  %-24s %10d\n' total "$total"
printf '  %-24s %10d\n' budget "$budget"
echo

if ((total > budget)); then
	verdict="FAIL: $total bytes used, budget is $budget bytes ($((total - budget)) over)"
	status=1
else
	verdict="PASS: $((budget - total)) bytes under budget"
	status=0
fi
echo "$verdict"

if [[ -n ${GITHUB_STEP_SUMMARY:-} ]]; then
	{
		echo "### $name flash gate"
		echo
		echo "| ELF | bytes |"
		echo "| --- | ---: |"
		for i in "${!names[@]}"; do
			echo "| \`${names[$i]}\` | ${sizes[$i]} |"
		done
		echo "| total | $total |"
		echo "| budget | $budget |"
		echo
		echo "$verdict"
		echo
	} >>"$GITHUB_STEP_SUMMARY"
fi

exit "$status"
