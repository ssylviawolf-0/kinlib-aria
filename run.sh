#!/bin/bash
# USAGE: sh run.sh <vrs file>

set -e

if [ -z "$1" ]; then
    echo "usage: sh run.sh <path to vrs file>"
    exit 1
fi

VRS_FILE="$1"
PREFIX=$(basename "$VRS_FILE" .vrs)
OUTPUT_FOLDER="outputs/output_${PREFIX}"

if [ ! -d "external/eigen" ]; then
    echo "Eigen is not installed. Installing rn..."
    mkdir -p external
    git clone --branch 3.4.0 https://gitlab.com/libeigen/eigen.git external/eigen
else
    echo "Eigen is there very good"
fi


echo "Running pipeline.py on vrs file"

cd scripts
uv sync
cd ..

uv run scripts/pipeline.py "$VRS_FILE" 0 1

echo "Moving CVVs into the right directories for the CPP"
DESTINATION="files/Demonstrations/project_aria/$PREFIX"
mkdir -p "$DESTINATION"


cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_OBJ/world_hand_formatted.csv" "$DESTINATION" 
cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_OBJ/world_object_formatted.csv" "$DESTINATION"

cmake .
make

echo "Running the object"
./bin/kinlib_projectaria  ${OUTPUT_FOLDER}/final_reference/APRIL_TAG_OBJ/world_object_formatted.csv

# still have to run the simulation
