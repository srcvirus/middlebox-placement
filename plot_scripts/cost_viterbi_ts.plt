set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style data histogram 
set style histogram rowstacked gap 0
set style fill solid border 0

set xtics nomirror 
set ytics nomirror

set output "cost_viterbi_time_series.pdf"
set xlabel "Time (s)"
set ylabel "OPEX ($)"
set key outside horizontal

#set xr[0:10000]
set yr[0:2.75]

plot 'log.viterbi.cost.ts' using ((int($0)%500)==0?0.:0/0):xtic(1) notitle, \
     '' using 4 t "Energy Cost", \
     '' using 5 t "Transit Cost", \
     '' using 6 t "SLA Violation Cost"
