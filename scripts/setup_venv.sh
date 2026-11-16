#!/usr/bin/env bash
set -euo pipefail

# Keep the environment inside the repository so it is easy to activate and
# remove, while .gitignore prevents it from being committed.
readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly venv_dir="${repo_root}/.venv"

if ! command -v python3 >/dev/null 2>&1; then
    printf 'Error: python3 is required. Install it with your WSL package manager.\n' >&2
    exit 1
fi

if [[ ! -x "${venv_dir}/bin/python" ]]; then
    python3 -m venv "${venv_dir}"
fi

"${venv_dir}/bin/python" -m pip install --requirement "${repo_root}/requirements.txt"
printf 'Virtual environment ready. Activate it with:\nsource .venv/bin/activate\n'
