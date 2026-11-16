#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: ./apply.sh <px4-v1.16.0-checkout>" >&2
    exit 2
fi

source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
target_dir=$(cd -- "$1" && pwd -P)
expected_commit=6ea3539157ca358c70a515878b77077af7d4611d

if [[ $(git -C "$target_dir" rev-parse HEAD) != "$expected_commit" ]]; then
    echo "The target checkout must be at the official PX4 v1.16.0 commit." >&2
    exit 1
fi

overlay="$source_dir/overlay"
patch="$source_dir/patches/px4-v1.16.0.patch"
while IFS= read -r -d '' file; do
    relative="${file#"$overlay"/}"
    if [[ -e "$target_dir/$relative" || -L "$target_dir/$relative" ]]; then
        echo "Overlay path already exists: $relative" >&2
        exit 1
    fi
done < <(find "$overlay" -type f -print0)

git -C "$target_dir" apply --check "$patch"
git -C "$target_dir" apply "$patch"
cp -a -- "$overlay/." "$target_dir/"
printf 'Applied the pitch-decoupled quadrotor project to %s\n' "$target_dir"
