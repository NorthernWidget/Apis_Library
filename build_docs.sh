#! /bin/sh
set -e

doxygen doxygen_NW.cfg
moxygen -l cpp -o README.md xml/
rm -rf xml
cat .README-intro.md README.md > README.tmp && mv README.tmp README.md
