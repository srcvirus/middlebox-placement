set terminal pdfcairo enhanced color font "Helvetica,9" dashed linewidth 4 rounded fontscale 1.0 size 4in,5in

set output "cost_ratio_dc_ts.pdf"

# set   autoscale                        # scale axes automatically
set xtics nomirror
set ytic nomirror
set tmargin 0
set lmargin 6 
set multiplot layout 3,1
set origin 0,0

set key left top
set xr[0:40]
set yr[0:2.80]
set ytics 1.0 rotate by 90 offset 0,0.5

set xlabel font "Helvetica,8"
set ylabel font "Helvetica,8"
set xtic font "Helvetica,8"
set ytic font "Helvetica,8"

set size 1.0, 0.33
plot    	"log.dc.total_cost_ratio" using 1:2 title "Total Cost Ratio" w line lc rgb '#FA160E' lt 5

set origin 0.00, 0.29
set size 1.00,0.245

#unset label
#unset xtics
set title ""
set xlabel ""
# set ylabel "Cost Ratio"
unset xtics
set xr[0:40]
set yr[0.4:2.8]
# set ytics 50

plot    	"log.dc.energy_cost_ratio" using 1:2 title "Energy Cost Ratio" w line lt 3


set origin 0.0, 0.5
set size 1.00,0.28

#unset label
#unset xtics
set title "Time(min) vs Cost Ratio (Heuristic / CPLEX)" 
set xlabel ""
set ylabel ""
unset xtics
set xr[0:40]
set yr[0.4:2.8]
# set ytics 50

plot    	"log.dc.transit_cost_ratio" using 1:2 title "Transit Cost Ratio" w line lc rgb '#9FBB55' lt 1

#unset label
#unset xtics




unset multiplot

