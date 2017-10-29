#!/bin/bash

# Format files in the repository.
# Either all file, or only those changed sinced commit-ish.
# Untracked files will not be formatted, but any committed, staged
# or unstaged (but tracked) files will be.

# This directory
tools=$(readlink -f $(dirname ${BASH_SOURCE[0]}))
# Base repo directory
repo=$(readlink -f ${tools}/..)

orig_args="$@"

# Source the tools virtualenv.
source ${tools}/venv/bin/activate

# Diff against master by default
commitish=master

# Only show a diff by default, don't change the files.
inplace=

while [[ $# > 0 ]]; do
    case "$1" in
        --commitish)
            commitish="$2"
            shift  # Past argument
            ;;
        --all)
            commitish=""
            ;;
        -i|--in-place)
            inplace=1
            ;;
        *)
            echo "Usage: $0 [--all] [--commitish COMMITISH]"
            echo
            echo "Perform yapf formatting on repo files."
            echo
            echo "optional arguments:"
            echo "  --all                  Format all Python files"
            echo "  --commitish COMMITISH  Format only Python files changed since COMMITISH (default master)"
            echo "  -i, --in-place         Make changes to files in place"
            exit 1
    esac
    shift
done

if [[ ${commitish} != "" ]]; then
    # Run all files (A)dded, (C)opied, (M)odified, (R)emaned, or changed (T)
    # through the formatter.
    # git diff returns lines like 'A tools/format.sh'
    files=$(git diff --name-status --diff-filter=ACMRT ${commitish} |
            cut -f 2 |
            grep '\.py$')
else
    files=$(git ls-files |
            grep '\.py$')
fi

if [[ ${inplace} != "" ]]; then
    yapf_args="--in-place"
else
    yapf_args="--diff"
fi

echo "${files}" | xargs -i yapf ${yapf_args} "${repo}/{}"
exit_code=$?

if [[ ${exit_code} != 0 && ${inplace} == "" ]]; then
    echo -e "\nFormatting needed. Run '$0 ${orig_args} -i' to update files."
fi

exit ${exit_code}
