set terminal pdfcairo enhanced color font "Helvetica,9" dashed linewidth 4 rounded fontscale 1.0
set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics nomirror
set ytics nomirror

set style line 1 lt rgb "#A00000" lw 2 pt 1
set style line 2 lt rgb "#00A000" lw 2 pt 6
set style line 3 lt rgb "#5060D0" lw 2 pt 2
set style line 4 lt rgb "#F25900" lw 2 pt 9

set output "active_server_time_series_dc.pdf"
set xlabel "Time (min)"
set ylabel "Number of Active Servers"
set key outside horizontal

set yr[0:10]

plot "log.viterbi.dc.active_server.ts" using 1:2 title "Heuristic Solution" w line ls 1, \
     "log.cplex.dc.active_server.ts" using 1:2 title "CPLEX Solution" w line ls 2
