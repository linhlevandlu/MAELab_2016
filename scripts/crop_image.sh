#!/bin/bash
SCENEJPGFOLDER="/home/linhpc/data_CNN/Facial_keypoints/MTFL/MAFL/Img/img_align_celeba/*"
SAVEFOLDER="/home/linhpc/data_CNN/Facial_keypoints/MTFL/MAFL/Img/img_align_celeba_178x178/"
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
