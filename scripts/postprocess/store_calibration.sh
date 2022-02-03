# Drop the calibration information of a camera for a given resolution
# usage: ./store_calibration.sh 1280 720
# This will create a folder calibration/ and drop the calibration data
# of the device there

# Delete previous calibration data and recreate folder
rm -rf calibration
mkdir calibration

# Drop the full calibration data on the folder
rs-enumerate-devices  -c >> calibration/calibration_data.csv
rs-enumerate-devices  -c | grep "Intrinsic of \"Color\" / $1x$2" -A 10 >> calibration/camera_intrinsic_parameters.csv

# Drop the intrinsics of the RGB device
rm -f rgb.intrinsics
echo "$(cat calibration/camera_intrinsic_parameters.csv | grep Fx | awk '{print $2}'), 0.0, $(cat calibration/camera_intrinsic_parameters.csv | grep PPX | awk '{print $2}')">> rgb.intrinsics
echo "0.0, $(cat calibration/camera_intrinsic_parameters.csv | grep Fy | awk '{print $2}'), $(cat calibration/camera_intrinsic_parameters.csv | grep PPY | awk '{print $2}')" >> rgb.intrinsics
echo "0.0, 0.0, 1.0" >> rgb.intrinsics

# Drop the depth calibration parameters
rs-enumerate-devices  -c | grep "Intrinsic of \"Depth\" / $1x$2" -A 10 >> calibration/camera_intrinsic_parameters.csv


echo "Set intrinsics for mode $1x$2 to: "
cat rgb.intrinsics
