set terminal pdfcairo font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style data histogram
set style histogram errorbar gap 1 lw 1

set style line 80 lt rgb "#808080"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 3 back linestyle 80

set xtics nomirror
set ytics nomirror

set style fill pattern border
set output "per_server_util.pdf"
set xlabel "Server ID"
set ylabel "Mean Utilization"
set key top right

set yr[0:1.25]

plot 'log.log.per_server_util' using 2:3:4:xtic(1) title col fs pattern 2
