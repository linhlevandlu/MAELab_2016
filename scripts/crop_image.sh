#!/bin/bash
SCENEJPGFOLDER="/home/linhpc/Biogical_Images/elytre/Images_without_grid_2/*"
SAVEFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i2448x2448_elytre/images/"
EXECUTE="./MAELab_CI"
jpgarray=(${SCENEJPGFOLDER})
total=${#jpgarray[@]}
rpby="."
for (( i=0; i< $total; i++))
do
	SCENEJPG="${jpgarray[$i]}"
	saveImg="$SAVEFOLDER$(basename "${jpgarray[$i]}"/)"
	saveImg2=${saveImg/./$rpby}
	echo $saveImg2
	$EXECUTE "$SCENEJPG" "$saveImg2"
done
