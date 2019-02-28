#!/bin/bash
SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i2448x2448/original/*"
SAVEFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i2400x2400/original/"
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
