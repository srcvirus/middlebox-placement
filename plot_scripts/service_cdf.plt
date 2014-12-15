set terminal pdfcairo font "Helvetica,9" linewidth 4 dashed rounded fontscale 1.0
set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics nomirror
set ytics nomirror

set style line 1 lt rgb "#A00000" lw 0.7 pt 1
set style line 2 lt rgb "#00A000" lw 0.7 pt 6
set style line 3 lt rgb "#5060D0" lw 0.7 pt 2
set style line 4 lt rgb "#F25900" lw 0.7 pt 9

set output "service_cdf.pdf"
set xlabel "Number of Service Points"
set ylabel "CDF"
set key outside horizontal

set xr[1:]
set yr[0.30:1.0]

plot "log.viterbi.service_points" using 1:2 title "Heuristic-Internet2" w linespoints ls 2, \
     "log.cplex.service_points" using 1:2 title "CPLEX-Internet2" w linespoints ls 3, \
     "log.viterbi.dc.service_points" using 1:2 title "Heuristic-DC" w  linespoints ls 4, \
     "log.cplex.dc.service_points" using 1:2 title "CPLEX-DC" w linespoints ls 1
