# SETUP -- ALL OF THE FOLLOWING COMES FROM plotfile.plt TEMPLATE 
reset
set datafile separator ","
set term pngcairo
set output fileName.'.png'
set title "Roast: " . fileName . " - " . coffeeName font "Verdana,12"
set xlabel 'Time (s)' font "Verdana,10"
set ylabel 'Temp (F)' font "Verdana,10"
set label "First Crack" at (firstCrack - 30),-150 rotate by 90 font "Verdana,6"
set label "Date: DD MMM YYYY" at 0, -300 font "Verdana,6"
set label "Green Weight: XXXg" at 0, -330 font "Verdana, 6"
set label "Mass Lost: " at 0, -360 font "Verdaba, 6"
set label "www.roasthacker.com" at (xmax - 300), -300 font "Verdana,6"
#set label moistureLoss at 500,-100 font "Verdana,10"
set arrow from firstCrack,-200 to firstCrack,700 nohead lt 0
set arrow from startCooling,-200 to startCooling,700 nohead lt 0
set arrow from 0,0 to xmax,0 nohead lt 0
set xtics font "Verdana,10"
set ytics font "Verdana,10"
set key font "Verdana,6"
set key samplen 1
set key spacing 0.5
set xrange [0:xmax]
set yrange [-200:700] 
set size ratio -1 #needed to make scale of image & plot match
set key opaque #adds white background behind legend

# PLOT
plot \
fileName using 1:2 with lines title "Inlet", \
fileName using 1:3 with lines title "Outlet", \
fileName using 1:($2-$3) with lines title "Difference", \
fileName using 1:4 with lines title "Setpoint", \
fileName using 1:9 with lines title "Bean Mass"


# CLEANUP 
unset label
unset arrow
 
