set terminal pdfcairo monochrome font "Helvetica,9" linewidth 4 rounded fontscale 1.0
set style fill   solid 0.25 border

set style line 80 lt rgb "#000000"
set style line 81 lt 0
set style line 81 lt rgb "#808080"

set grid back linestyle 81
set border 31 back linestyle 80

set xtics 2000 nomirror
set ytics 0.25 nomirror

set style line 1 lt rgb "#A00000" lw 0.7 pt 1
set style line 2 lt rgb "#00A000" lw 2 pt 6
set style line 3 lt rgb "#5060D0" lw 2 pt 2
set style line 4 lt rgb "#F25900" lw 2 pt 9

set output "traffic_dist_i2.pdf"
set xlabel "Time (min)"
set ylabel "Fraction of Peak Traffic"
set key outside horizontal

set xr[0:10000]
set yr[0:1.05]

plot "traffic-request.i2.dist" using 1:2 notitle w boxes
