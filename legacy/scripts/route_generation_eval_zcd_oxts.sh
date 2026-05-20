#!/bin/bash
#SBATCH --job-name route_generation_zcd_oxts_eval
#SBATCH --nodes 1
#SBATCH --gpus 0
#SBATCH --cpus-per-task 24
#SBATCH --time 10-00:00:00
#SBATCH --output /workspaces/%u/slurm/%j_eval_oxts.out
#SBATCH -p zprodcpu
#SBATCH --no-requeue
#

# Define the output path as a variable
OUTPUT_PATH="/staging/dl_tt/path_prediction/data/all_data/processed_osm_oxts/zcd_osm_oxts_eval"
OSM_CACHE_PATH="/staging/dl_tt/path_prediction/data/osm_cache"
# OSM_CACHE_PATH="None"
# move this into execution script, if OSM_CACHE_PATH is not None:
# -B $OSM_CACHE_PATH:$OSM_CACHE_PATH \
MAP_QUERY_MAX_L1=1000

# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_PATH

singularity exec --nv \
    -B /workspaces/$USER/trajectory_prediction_thesis/independent_route_generation:/app \
    -B /workspaces/$USER/trajectory_prediction_thesis/independent_route_generation/main.py:/app/main.py \
    -B $OUTPUT_PATH:$OUTPUT_PATH \
    -B $OSM_CACHE_PATH:$OSM_CACHE_PATH \
    -B /staging/dl_tt/path_prediction/data/all_data/dl2_hp_eval_default:/staging/dl_tt/path_prediction/data/all_data/dl2_hp_eval_default \
   --pwd /app \
    /staging/dl_tt/path_prediction/preprocessing_311_osmnx.sif \
    python3.11 main.py --input=/staging/dl_tt/path_prediction/data/all_data/dl2_hp_eval_default --map_query_max_l1=$MAP_QUERY_MAX_L1 --output=$OUTPUT_PATH --dataset=ZCD --location_source=oxts --cache_location=$OSM_CACHE_PATH --workers=24

#
#EOF