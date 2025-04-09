#!/bin/bash
set -e
OPT=$1; if [ "$OPT" == "" ]; then OPT="-brp"; fi
set -e
if [[ $OPT == *b* ]]; then 
echo Compiling...; 
time make csim; 
fi


if [[ $OPT == *r* ]]; then
echo Running... 
rm -rf ./spiff/ && \
time ./csim --seconds 100000 > ./out/csim.out 
fi

if [[ $OPT == *p* ]]; then 
echo Plotting...
./bin/plotcat.sh ./out/csim.out
fi

#   , f u 1:5 w l title 'Dessicant Exhaust VPD'
#    , f u 1:(\$6 * 12.86/1560) w l  title '12V Battery'
  
