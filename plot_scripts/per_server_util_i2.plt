set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style data histogram
set style histogram errorbar gap 1 lw 1

set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics rotate by 45 right 
set ytics nomirror

set style fill pattern border 1
set output "per_server_util_i2.pdf"
set xlabel "Server ID"
set ylabel "Mean Utilization"
set key outside horizontal

set yr[0:1.05]

plot 'log.viterbi.per_server_util' using 2:3:4:xtic(1) title "Heuristic Solution"  fs pattern 2, \
     'log.cplex.per_server_util' using 2:3:4:xtic(1) title "CPLEX Solution" fs  pattern 9
