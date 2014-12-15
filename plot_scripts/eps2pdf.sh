#!/bin/bash

for FILE in *.eps
do
	epstopdf $FILE
done
