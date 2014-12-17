set terminal pdfcairo enhanced color font "Helvetica,8" dashed linewidth 4 rounded fontscale 1.0 size 3in,4in

set output "cost_ratio_dc_ts.pdf"

# set   autoscale                        # scale axes automatically
set xtics nomirror
set ytic nomirror
set tmargin 0
set lmargin 6 
set multiplot layout 3,1
set origin 0,0

set key right top
set xr[0:40]
set yr[1.1:1.45]
set ytics 0.2 rotate by 90 offset 0,0.5

set xlabel font "Helvetica,8"
set ylabel font "Helvetica,8"
set xtic font "Helvetica,8"
set ytic font "Helvetica,8"

set xlabel "Time (min)"

set size 1.0, 0.39
plot    	"log.dc.total_cost_ratio" using 1:2 title "Total Cost" w line lc rgb '#FA160E' lt 5

set origin 0.00, 0.35
set size 1.00,0.3

#unset label
#unset xtics
set title ""
set xlabel ""
# set ylabel "Cost Ratio"
unset xtics
set xr[0:40]
set yr[0.85:1.75]
set ytics 0.5 rotate by 90 offset 0,0.5
# set ytics 50

plot    	"log.dc.energy_cost_ratio" using 1:2 title "Energy Cost" w line lt 3


set origin 0.0, 0.61
set size 1.00,0.3

set title "Heuristic vs. CPLEX"
#unset label
#unset xtics
set xlabel ""
set ylabel ""
unset xtics
set xr[0:40]
set yr[1.1:1.55]
set ytics 0.2 rotate by 90 offset 0,0.5
# set ytics 50

plot    	"log.dc.transit_cost_ratio" using 1:2 title "Transit Cost" w line lc rgb '#9FBB55' lt 1

#unset label
#unset xtics




unset multiplot

