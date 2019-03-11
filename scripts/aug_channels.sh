#!/bin/bash
#SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i102x102_pronotum/original/*"
#sSAVEFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i102x102_pronotum/add_green/"

SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/mg/i192x192/original/*"
SAVEFOLDER="/home/linhpc/data_CNN/linhlv/mg/i192x192/aug_green/"

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
