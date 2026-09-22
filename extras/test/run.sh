#!/bin/sh
# Compile Apis.cpp on the host against the stubs here and diff the output
# against baseline.txt. Usage: ./run.sh            (test)
#                             ./run.sh --record   (rewrite baseline.txt)
cd "$(dirname "$0")" || exit 1
NW_CORE="${NW_CORE:-../../../NW_Core}"   # NW_Core checkout: headers (and later sources) the library depends on
g++ -std=c++17 -Wall -Wno-unused-function -I. -I"$NW_CORE/src" -o apis_test test_output.cpp || exit 1
./apis_test > output.txt || exit 1
if [ "$1" = "--record" ]; then cp output.txt baseline.txt; echo "baseline recorded"; exit 0; fi
if diff -u baseline.txt output.txt; then echo "OK: output identical to baseline"; else echo "FAIL: output differs from baseline"; exit 1; fi
