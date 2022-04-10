#!/bin/bash
#Inject the rgb.intrinsics into ORB's intrinsics file
if [ -z "$1" ]
  then
    echo "No first argument supplied, first argument should be data directory"
    return 0;
fi

fx=$(cat $1/rgb.intrisics | awk "{if (NR==1) print \$0}" | awk {'print substr($1, 1, length($1)-1)'}) 
fy=$(cat $1/rgb.intrisics | awk "{if (NR==2) print \$0}" | awk {'print substr($2, 1, length($2)-1)'})
cx=$(cat $1/rgb.intrisics | awk "{if (NR==1) print \$0}" | awk {'print substr($3, 1, length($3)-1)'})
cy=$(cat $1/rgb.intrisics | awk "{if (NR==2) print \$0}" | awk {'print substr($3, 1, length($3)-1)'})

integer_fx=$(cat $1/rgb.intrisics | awk "{if (NR==1) print \$0}" | awk {'print int(substr($1, 1, length($1)-1))'}) 

echo "Current intrinsics: " $fx " " $fy " " $cx " " $cy

sed -i "s/Camera.fx:.*/Camera.fx: $fx/g" $1/TUM.yaml
sed -i "s/Camera.fy:.*/Camera.fy: $fy/g" $1/TUM.yaml
sed -i "s/Camera.cx:.*/Camera.cx: $cx/g" $1/TUM.yaml
sed -i "s/Camera.cy:.*/Camera.cy: $cy/g" $1/TUM.yaml

#-eq # Equal
#-ne # Not equal
#-lt # Less than
#-le # Less than or equal
#-gt # Greater than
#-ge # Greater than or equal

#TODO: Add the 30 FPS mode here
if [ ${integer_fx} -gt 400 ]; then
  echo "Greater than 400 so 848 480    60"
  sed -i "s/Camera.width:.*/Camera.width: 848/g"   $1/TUM.yaml
  sed -i "s/Camera.height:.*/Camera.height: 480/g" $1/TUM.yaml
  sed -i "s/Camera.fps:.*/Camera.fps: 60/g"       $1/TUM.yaml
else
  echo "Less than 400 so 640 360   90"
  sed -i "s/Camera.width:.*/Camera.width: 640/g"   $1/TUM.yaml
  sed -i "s/Camera.height:.*/Camera.height: 360/g" $1/TUM.yaml
  sed -i "s/Camera.fps:.*/Camera.fps: 90/g"       $1/TUM.yaml
fi
