# Local prediction

SCENEJPGFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/original/*"
SCENETPSFOLDER="/home/linhpc/data_CNN/linhlv/tdata/i3264x2448/predicted_landmarks/*"

EXECUTE="./MAELab_CI"
jpgarray=(${SCENEJPGFOLDER})
tpsarray=(${SCENETPSFOLDER})
total=${#jpgarray[@]}
rpby="_"
for (( i=0; i< $total; i++))
do
	SCENEJPG="${jpgarray[$i]}"
	SCENETPS="${tpsarray[$i]}"
    #echo $SCENEJPG
    #echo $SCENETPS	
	$EXECUTE "$SCENEJPG" "$SCENETPS" 
done

#EXECUTE="./MAELab_CI"
#$EXECUTE

