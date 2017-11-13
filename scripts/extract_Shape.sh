#!/bin/bash
# SCRIPT CALCULATE THE SIFT DESCRIPTOR OF A PATCH AROUND THE LANDMARK
# THE OUTPUT IS A MATRIX 9X8 FOR EACH LANDMARK ( DEFAULT SIZE 9X9 -> 9 X(3X3))
SCENEJPGFOLDER="/home/linh/Desktop/data/pronotum_data_5/data_aug/_combine_data/original/*"
SCENETPSFOLDER="/home/linh/Desktop/data/pronotum_data_5/exlm/*"
SAVEFOLDER="/home/linh/Desktop/results/2017/pronotum/procrustes_27Oct_lm2/"
LMINDEX=1
BSIZE=7
EXECUTE="./MAELab_CI"
jpgarray=(${SCENEJPGFOLDER})
tpsarray=(${SCENETPSFOLDER})
total=${#jpgarray[@]}
rpby="_"
for (( i=0; i< $total; i++))
do
	SCENEJPG="${jpgarray[$i]}"
	SCENETPS="${tpsarray[$i]}"	
	echo $SCENETPS
	echo $SCENEJPG
	#saveTPS="$SAVEFOLDER$(basename "${tpsarray[$i]}"/)"
	#saveTPS2=${saveTPS/ /$rpby}
	$EXECUTE "$SCENEJPG" "$SCENETPS" "$LMINDEX" "$BSIZE" "$SAVEFOLDER"
done

#EXECUTE="./MAELab_CI"
#$EXECUTE

