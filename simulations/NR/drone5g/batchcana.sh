#! /bin/sh

#./configure

# make MODE=release

#cd simulations/simu5gdrone

for i in 0 1 2 3 4
 do
	for j in 1 2 3 4 5
	do
		./run -u Cmdenv -f omnetpp.ini -r $i -c sigma_$j
	
		mv results/cp.csv results/cp"$i"_"$j".csv
		mv results/lq.csv results/lq"$i"_"$j".csv
		mv results/comdata.csv results/comdata"$i"_"$j".csv
		mv results/avcp.csv results/avcp"$i"_"$j".csv
		
	done
done
date

