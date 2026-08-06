#!/bin/bash
set -e

if [ -z "$1" ]; then
    echo "Usage: sh run.sh <path to vrs file>"
    exit 1
fi

VRS_FILE="$1"
VRS_BASENAME=$(basename "$VRS_FILE")

PREFIX=$(basename "$VRS_FILE" .vrs)
OUTPUT_FOLDER="scripts/outputs/output_${PREFIX}"

echo "VRS file: $VRS_FILE"

cp "$VRS_FILE" "./scripts/$VRS_BASENAME"

CONTAINER_NAME="aria_env_cont"

cleanup() {
    echo "Cleaning up: Stopping Docker container..."
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# container runs in the background (-d) mounted to the current directory
docker run -d --rm --name "$CONTAINER_NAME" -v "$(pwd):/project" aria_env_img

echo "Running pipeline.py..."
docker exec -w /project/scripts "$CONTAINER_NAME" python3 pipeline.py "$VRS_BASENAME" 1 0

echo "Moving CSVs..." 
DESTINATION="$(pwd)/files/Demonstrations/aria_project"
mkdir -p "$DESTINATION"

cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_OBJ/world_object_formatted.csv" "$DESTINATION/"
cp "${OUTPUT_FOLDER}/final_reference/APRIL_TAG_OBJ/object_poses_pipeline.csv" "$DESTINATION/"

echo "Compiling Cpp code..."
docker exec -w /project "$CONTAINER_NAME" rm -rf CMakeCache.txt CMakeFiles/
docker exec -w /project "$CONTAINER_NAME" cmake .
docker exec -w /project "$CONTAINER_NAME" make

echo "Pray this works"
docker exec -w /project "$CONTAINER_NAME" ./bin/kinlib_projectaria \
    "files/Demonstrations/aria_project/world_object_formatted.csv" \
    "files/Demonstrations/aria_project/object_poses_pipeline.csv"

echo "Moving output files for simulation..."
cp project_aria.csv scripts/
cp simulation_gripper.csv scripts/

echo "Running simulation.py..."
docker exec -w /project/scripts "$CONTAINER_NAME" python3 simulation.py project_aria.csv simulation_gripper.csv "$VRS_FILE"

echo "Cleaning up CSVs..."
rm scripts/project_aria.csv
rm scripts/simulation_gripper.csv

echo "Done"
