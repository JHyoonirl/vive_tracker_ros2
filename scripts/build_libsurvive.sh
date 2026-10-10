#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
libsurvive_root="${repo_root}/third_party/libsurvive"

if [[ ! -f "${libsurvive_root}/Makefile" ]]; then
  echo "libsurvive submodule is missing. Run: git submodule update --init --recursive" >&2
  exit 1
fi

make -C "${libsurvive_root}" -j"$(nproc)"
echo "libsurvive built under ${libsurvive_root}/bin"
