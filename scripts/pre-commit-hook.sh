#!/bin/bash

# Get the list of files that were staged in the original commit
staged_files=$(git diff --cached --name-only)

./format_all.sh

# Re-add only the files that were originally staged
echo "$staged_files" | xargs git add 
