set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0

set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics 2000 nomirror 
set ytics nomirror

set style line 1 lt rgb "#A00000" lw 0.7 pt 1
set style line 2 lt rgb "#00A000" lw 2 pt 6

set output "netutil_time_series.pdf"
set xlabel "Time (s)"
set ylabel "Network Utilization (%)"
set key outside horizontal

set yr[0:1.00]

plot 'log.viterbi.netutil.ts' u 1:($2*100) t "Heuristic Solution" w line lt 1
