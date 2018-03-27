#!/bin/bash
SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i224x224/images/*"
SAVEFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i224x224/add_blue/"
EXECUTE="./MAELab_CI"
jpgarray=(${SCENEJPGFOLDER})
total=${#jpgarray[@]}
rpby="_b."
for (( i=0; i< $total; i++))
do
	SCENEJPG="${jpgarray[$i]}"
	saveImg="$SAVEFOLDER$(basename "${jpgarray[$i]}"/)"
	saveImg2=${saveImg/./$rpby}
	echo $saveImg2
	$EXECUTE "$SCENEJPG" "$saveImg2"
done
