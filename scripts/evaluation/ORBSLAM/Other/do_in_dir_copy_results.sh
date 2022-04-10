#!/bin/bash
#Execute a command for all dirs
if [ -z "$1" ]
  then
    echo "No argument supplied, first argument should be data MAIN directory"
    return 0;
fi
if [ -z "$2" ]
  then
    echo "No second argument supplied, second  argument should be the directory to copy"
    return 0;
fi

# Save current working directory
cwd=$(pwd)

cd $1

for d in */ ; do
    [ -L "${d%/}" ] && continue
    echo "${PWD}/$d""Results/EVO"
    
    #Copy results to main dir
    mkdir -p "$2/$d"
    cp -r "${PWD}/$d/Results/EVO/." "$2/$d/"
done    


cd ${cwd}