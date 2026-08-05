#!/bin/bash
# USAGE: sh run.sh <vrs file>

set -e

if [ -z "$1" ]; then
    echo "usage: sh run.sh <path to vrs file>"
    exit 1
fi

VRS_FILE="$1"

echo "VRS file: $VRS_FILE"
cp "$VRS_FILE" "./scripts"

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
# uv sync
# source venv/bin/activate
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi
# source venv/bin/activate
pip install -r requirements.txt


python3 pipeline.py "$VRS_FILE" 0 1
cd ..
# uv run scripts/pipeline.py "$VRS_FILE" 0 1

echo "Moving CSVs into the right directories for the CPP"

DESTINATION="$(pwd)/files/Demonstrations/aria_project"
mkdir -p "$DESTINATION"

# cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_STATIC/world_hand_no_noise_pipeline.csv" "$DESTINATION" 
cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_STATIC/world_object_no_noise_pipeline.csv" "$DESTINATION"


cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_STATIC/object_poses_pipeline.csv" "$DESTINATION"

cmake .
make

echo "Running the object"
./bin/kinlib_projectaria \
    "${DESTINATION}/world_object_formatted.csv" \
    "${DESTINATION}/object_poses_pipeline.csv"

# still have to run the simulation

cp project_aria.csv scripts
cp simulation_gripper.csv scripts

cd scripts
python3 simulation.py project_aria.csv simulation_gripper.csv
cd ..


rm scripts/project_aria.csv
rm scripts/simulation_gripper.csv
