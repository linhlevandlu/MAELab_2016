#!/bin/bash
SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/md/v1/original/*"
SAVEFOLDER="/home/linhpc/data_CNN/linhlv/md/i192x192/original/"
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
