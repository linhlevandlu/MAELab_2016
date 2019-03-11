#!/bin/bash
SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/mg/v1/original/*"
SAVEFOLDER="/home/linhpc/data_CNN/linhlv/mg/i192x192/original_2/"
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
