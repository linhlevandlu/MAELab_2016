#!/bin/bash
# SCRIPT TO AUGEMTATION DATA (INCREASE 10)
SCENEJPGFOLDER="/home/linh/Desktop/data/elytre/i192x256/original/val/*"
SAVEFOLDER="/home/linh/Desktop/data/elytre/i192x256/gray_scale/val/"
EXECUTE="./MAELab_CI"
jpgarray=(${SCENEJPGFOLDER})
total=${#jpgarray[@]}
rpby="_gray."
for (( i=0; i< $total; i++))
do
	SCENEJPG="${jpgarray[$i]}"
	#img=${SCENEJPG:${#imagepath}-10}
	#SAVEJPG="$SAVEFOLDER$img"
	#echo $SCENEJPG
	#echo "${jpgarray[$i]}"
	SCENETPS="${tpsarray[$i]}"	
	#tps=${SCENETPS:${#tpspath}-10}
	#SAVETPS="$SAVEFOLDER$tps"
	saveImg="$SAVEFOLDER$(basename "${jpgarray[$i]}"/)"
	saveImg2=${saveImg/./$rpby}
	echo $saveImg2
	$EXECUTE "$SCENEJPG" "$saveImg2"
done

#EXECUTE="./MAELab_CI"
#$EXECUTE

