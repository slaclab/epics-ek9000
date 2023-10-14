#!/bin/bash 

[ -z "$CLANG_FORMAT" ] && CLANG_FORMAT=clang-format

if [[ $CI -eq 1 ]]; then
    ARGS="-Werror --dry-run"
fi

$CLANG_FORMAT $ARGS -i $(find . -iname "*.cpp" -o -iname "*.h")
