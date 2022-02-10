
#!/bin/bash
#Execute orb slam for TUM dataset
if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data directory"
    return 0;
fi

if [ -z "$2" ]
  then
    echo "No second argument supplied, second argument should ORBSLAM2 main directory"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

source convert_RR_to_TUM.sh $1
source associate_script.sh $1
source run_orb2.sh $1 $2
source evaluate.sh $1 -sa

cd ${cwd}
