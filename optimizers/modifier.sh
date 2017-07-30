#!/bin/bash

clear

path="$ARTISYNTH_PRO_ECLIPSE/swallowingRegistrationTool/optimizers"
cd path

echo "Current working directory:  $PWD"

fileNames=$(ls)
echo "exsisting files: "
#echo "$fileNames"

newString="artisynth.models.swallowingRegistrationTool.optimizers;"
oldString="artisynth.models.swallowingRegistrationTool;"

#sed -i ".bak" "s/$oldString/$newString/g" ObjInfo.java
#grep $oldString ./ObjInfo.java 


for name in {$fileNames}; do 
	if grep -q ".java" <<<$name; then 
		echo $name;
		sed -i "" "s/$oldString/$newString/g" $name
	fi
done
