cwd=$(pwd)

cd $1
python3 /files/Projects/UnderDev/RealsenseRecord/scripts/postprocess/assoc_rgbdi.py $1/depth.txt $1/rgb.txt $1/acc.txt $1/gyr.txt


cd ${cwd}
