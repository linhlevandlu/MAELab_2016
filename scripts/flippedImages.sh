#!/bin/bash
# Flip images and landmarks

IMGFOLDER="/home/linhpc/CNN_data/pronotum/v1_abc/original/*"
LMFOLDER="/home/linhpc/CNN_data/pronotum/v1_abc/landmarks/*"
SAVEFOLDER="/home/linhpc/CNN_data/pronotum/v1_abc/flipXY/"
images="images/"
lmarks="landmarks/"
save_images=$SAVEFOLDER$images
save_lmarks=$SAVEFOLDER$lmarks

JPGARRAY=(${IMGFOLDER})
LMARRAY=(${LMFOLDER})
TOTAL=${#JPGARRAY[@]}
echo $TOTAL
EXECUTE="./../MAELab_CI"
for((i=0; i<$TOTAL; i++)) do
	IMAGE="${JPGARRAY[$i]}"
	#echo $IMAGE
	LM="${LMARRAY[$i]}"
	#echo $LM
	SAVEIMAGE="$save_images$(basename "${JPGARRAY[$i]}")"
	#echo $SAVEIMAGE
	SAVELM="$save_lmarks$(basename "${LMARRAY[$i]}")"
	#echo $SAVELM
	$EXECUTE "$IMAGE" "$LM" "$SAVEIMAGE" "$SAVELM"
done
