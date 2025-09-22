#!/bin/bash

# Get the list of files that were staged in the original commit
staged_files=$(git diff --cached --name-only)
./format_all.sh

# Re-add only the files that were originally staged AND still exist
for file in $staged_files; do
    if [ -f "$file" ]; then
        git add "$file"
    fi
done