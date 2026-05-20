#!/bin/bash
#SBATCH --job-name route_generation_zcd_lcm_eval_delete
#SBATCH --nodes 1
#SBATCH --gpus 0
#SBATCH --cpus-per-task 128
#SBATCH --time 2-00:00:00
#SBATCH --output /workspaces/%u/slurm/%j.out
#SBATCH -p zprodlow
#SBATCH --no-requeue
#

# Define the output path as a variable
OUTPUT_PATH="/staging/dl_tt/path_prediction/data/all_data/processed_osm_lcm/dl2_hp_eval_default_processed_zcd_lcm_delete"

# Create the output directory if it doesn't exist
mkdir -p $OUTPUT_PATH

singularity exec --nv \
    -B /workspaces/$USER/trajectory_prediction_thesis/independent_route_generation:/app \
    -B /workspaces/$USER/trajectory_prediction_thesis/independent_route_generation/main.py:/app/main.py \
    -B $OUTPUT_PATH:$OUTPUT_PATH \
    -B /staging/dl_tt/path_prediction/data/all_data/dl2_hp_eval_default:/staging/dl_tt/path_prediction/data/all_data/dl2_hp_eval_default \
   --pwd /app \
    /staging/dl_tt/path_prediction/preprocessing_311_osmnx.sif \
    python3.11 main.py --input=/staging/dl_tt/path_prediction/data/all_data/dl2_hp_eval_default --output=$OUTPUT_PATH --dataset=ZCD --location_source=lcm --workers=127

#
#EOF