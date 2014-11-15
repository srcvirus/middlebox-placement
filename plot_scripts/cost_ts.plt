set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics nomirror
set ytics nomirror

set style line 1 lt rgb "#A00000" lw 0.7 pt 1
set style line 2 lt rgb "#00A000" lw 2 pt 6
set style line 3 lt rgb "#5060D0" lw 2 pt 2
set style line 4 lt rgb "#F25900" lw 2 pt 9

set output "cost_time_series.pdf"
set xlabel "Time (s)"
set ylabel "OPEX ($)"
set key top right

set xr[0:10000]
set yr[10000:4000000]

plot "log.cost.ts" using 1:2 title "Heuristic Solution" w line ls 1
