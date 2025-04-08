#!/bin/bash
set -e
OPT=$1; if [ "$OPT" == "" ]; then OPT="-brp"; fi
set -e
if [[ $OPT == *b* ]]; then 
echo Compiling...; 
time make -f Makefile.csim csim; 
fi


if [[ $OPT == *r* ]]; then
echo Running... 
rm -rf ./spiff/ && \
time ./csim --seconds 100000 > ./out/csim.out 
fi

if [[ $OPT == *p* ]]; then 
echo Plotting...
cat ./out/csim.out |  ./iso_date_parser.py fanPwm Tint.v Tamb.v Tex1.v bv2 >  plane.dat
gnuplot -e "set grid;
f='./plane.dat'; 
set title 'Refreshed $age minutes ago'; 
set terminal qt size 1800,800;
    unset y2tics ; set ytics nomirror ;
    set key top left; 
    set xdata time; 
    set xtics;
    set timefmt '%s'; 
    set grid; 
    set format x '%a %l:%M%p'; 
    p [*:*][0:16]  
    3.6 title 'Threshold'
    ,f u 1:4 w l title 'Ambient VPD'
    ,f u 1:3 w l title 'Cockpit Interior VPD' 
    ,f u 1:(\$2 * 6/63 + 10) w l title 'Fan Power'
    ;
    pause 111;
"
fi

#   , f u 1:5 w l title 'Dessicant Exhaust VPD'
#    , f u 1:(\$6 * 12.86/1560) w l  title '12V Battery'
  
