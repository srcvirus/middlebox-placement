set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics 1 nomirror
set ytics 0.25 nomirror

set style line 1 lt rgb "#A00000" lw 2 pt 1
set style line 2 lt rgb "#00A000" lw 2 pt 6
set style line 3 lt rgb "#5060D0" lw 2 pt 2
set style line 4 lt rgb "#F25900" lw 2 pt 9

set output "stretch_cdf.pdf"
set xlabel "Stretch"
set ylabel "CDF"
set key right bottom

set yr[0:1.05]

plot "log.viterbi.i2.stretch" u 1:2 t "Heuristic-Internet2" w lp ls 1, \
     "log.cplex.i2.stretch" u 1:2 t "CPLEX-Internet2" w lp ls 2, \
     "log.viterbi.dc.stretch" u 1:2 t "Heuristic-DC" w lp ls 3, \
     "log.cplex.dc.stretch" u 1:2 t "CPLEX-DC" w lp ls 4
