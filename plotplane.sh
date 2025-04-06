#!/bin/bash
#
cd `dirname $0`
age=$(( ( $(date +%s) - $(stat --printf=%Y ./data.log) ) / 60  )) 

( grep MOF-Guest ./data.log | grep E4B323C457A8 | grep VPD |  ./iso_date_parser.py  pwm VPD    Tamb.v Tex1.v bv2; \
  grep MOF-Guest ./data.log | grep E4B323C457A8 | grep Tint  | ./iso_date_parser.py fanPwm Tint.v Tamb.v Tex1.v bv2 ) \
  | sort -n | tail -$2 > plane.dat

gnuplot -e "set grid;
	f='./plane.dat'; 
	set title 'Refreshed $age minutes ago'; 
	set terminal png size $1; set output './f/plane.png';
    unset y2tics ; set ytics nomirror ;
    set key top left; 
    set xdata time; 
    set xtics;
    set timefmt '%s'; 
    set grid; 
    set format x '%a %l:%M%p'; 
    p [*:*][0:16]  
    f u 1:3 w l title 'Cockpit Interior VPD' 
    , f u 1:(\$2 * 6/63 + 10) w l title 'Fan Power'
    , f u 1:4 w l title 'Ambient VPD'
    , f u 1:5 w l title 'Dessicant Exhaust VPD'
    , f u 1:(\$6 * 12.86/1560) w l  title '12V Battery'
    , 3.6 title 'Threshold'
    ;
"

