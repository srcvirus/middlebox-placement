set terminal pdfcairo font "Helvetica,9" dashed linewidth 4 rounded fontscale 1.0
set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 11 back linestyle 80

set xtics nomirror
set ytics 0.05 nomirror
set y2tics 1 nomirror

set style line 1 lt rgb "#A00000" lw 2 pt 1
set style line 2 lt rgb "#00A000" lw 2 pt 6
set style line 3 lt rgb "#5060D0" lw 2 pt 2
set style line 4 lt rgb "#F25900" lw 2 pt 9

set output "utilization_time_series_3967.pdf"
set xlabel "Time (min)"
set ylabel "Mean Utilization"
set y2label "Number of Active Servers"

set key outside horizontal

set yr[0.7:0.85]
set y2r[11:16]
set xr[0:1260]

plot "log.viterbi.3967.serverutil.ts" using 1:2 title "Utilization" w lp ls 1 axes x1y1, \
     "log.viterbi.3967.active_server.ts" using 1:2 title "Active Servers" w lp ls 2 axes x1y2
