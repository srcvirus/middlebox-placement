set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style line 80 lt rgb "#070707"
set style line 81 lt 0
set style line 81 lt rgb "#070707"

set grid back linestyle 81
set border 31 back linestyle 80

set xtics 1 nomirror
set ytics 0.25 nomirror

set style line 1 lt rgb "#A00000" lw 0.7 pt 1
set style line 2 lt rgb "#00A000" lw 1.5 pt 6
set style line 3 lt rgb "#5060D0" lw 1.5 pt 2
set style line 4 lt rgb "#F25900" lw 1.5 pt 9

set output "stretch_hw_cplex_cdf.pdf"
set xlabel "Stretch"
set ylabel "CDF"
set key outside horizontal

set yr[0:1.05]

plot "log.cplex.i2.stretch" using 1:2 title "VNF-Internet2" w linespoints ls 2, \
     "log.cplex.hw.stretch" using 1:2 title "Hardware-Internet2" w linespoints ls 3
