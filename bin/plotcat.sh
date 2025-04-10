#!/bin/bash
#
cat $1 |  ./bin/iso_date_parser.py fanPwm Tint.v Tamb.v Tex1.v bv2 >  ./out/plane.dat; gnuplot -e "set grid;
f='./out/plane.dat'; 
set title 'Refreshed $age minutes ago'; 
set terminal qt size 1800,800;
    unset y2tics ; set ytics nomirror ;
    set key top left; 
    set xdata time; 
    set xtics;
    set timefmt '%s'; 
    set grid; 
    set format x '%a %l:%M%p'; 
    p [*:*][0:30]  
    3.6 title 'Threshold'
    ,f u 1:4 w l title 'Ambient VPD'
    ,f u 1:3 w l title 'Cockpit Interior VPD' 
    ,f u 1:(\$2 * 6/63 + 10) w l title 'Fan Power'
    ;
    pause 111;
"

