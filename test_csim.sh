#!/bin/bash

make csim && rm spiff/* && time ./csim --seconds 200000  | egrep '^\[' | ../simplePost/plot.sh Tint.v Tamb.v fanPwm


