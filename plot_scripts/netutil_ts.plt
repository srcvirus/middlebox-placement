set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style data histogram 
set style histogram rowstacked gap 0
set style fill solid border 0

set xtics nomirror 
set ytics nomirror

set output "netutil_time_series.pdf"
set xlabel "Time (s)"
set ylabel "Network Utilization (%)"
set key outside horizontal

#set xr[0:10000]
set yr[0:1.00]

plot 'log.viterbi.netutil.ts' using ((int($0)%500)==0?0.:0/0):xtic(1) notitle, \
     '' using ($2*100) notitle
