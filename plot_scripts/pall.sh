#!/bin/bash

for FILE in *.plt
do
	gnuplot $FILE
done

#for FILE in *.eps
#do
#	epstopdf $FILE
#done
