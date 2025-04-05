#!/bin/bash -x

make csim && \
rm -rf ./spiff/ && \
time ./csim --seconds 200000 > ./out/csim.out && \
cat ./out/csim.out | egrep '^\[' | ../simplePost/plot.sh Tint.v Tamb.v fanPwm


