#!/usr/bin/zsh

#### MAIN SCRIPT ####

# Path to your experiments
MY_EXPERIMENTS_DIR="/home/josep-barbera/Documents/nViNL/legged_gym/envs/base"
NEXT_EXPERIMENTS_DIR="experiments/my_next_experiments"

# Base experiment folder
BASE_EXPERIMENT_FOLDER="experiments"

# General functions
format_duration() {
  local DURATION=$1
  local HOURS=$((DURATION / 3600))
  local MINUTES=$(( (DURATION % 3600) / 60 ))
  local SECONDS=$((DURATION % 60))
  printf "%02d:%02d:%02d\n" $HOURS $MINUTES $SECONDS
}

find_best_model() {
  local MODEL_DIR=$1
  local LATEST_FOLDER=$(ls -td "$MODEL_DIR"/*/ | head -1)
  if [ -z "$LATEST_FOLDER" ]; then
    echo "Error: No model folders found in $MODEL_DIR."
    exit 1
  fi

  local BEST_MODEL=$(ls "${LATEST_FOLDER}"*.pt | awk -F'_' '{print $NF}' | awk -F'.pt' '{print $1}' | sort -g | tail -n 1)

  if [ -z "$BEST_MODEL" ]; then
    echo "Error: No .pt files found in the latest folder $LATEST_FOLDER."
    exit 1
  fi

  local FULL_BEST_MODEL=$(ls "${LATEST_FOLDER}"*.pt | grep "_${BEST_MODEL}.pt$")
  local CHECKPOINT=$(basename "$FULL_BEST_MODEL" .pt | sed 's/^model_//')

  echo "$LATEST_FOLDER" "$CHECKPOINT"
}

find_best_model_across_runs() {
  local MODEL_DIR=$1

  local BEST_MODEL=$(ls "$MODEL_DIR"/*.pt | awk -F'_' '{print $NF}' | awk -F'.pt' '{print $1}' | sort -g | tail -n 1)

  if [ -z "$BEST_MODEL" ]; then
    echo "Error: No .pt files found in $MODEL_DIR."
    exit 1
  fi

  local FULL_BEST_MODEL=$(ls "$MODEL_DIR"/*.pt | grep "_${BEST_MODEL}.pt$")
  local CHECKPOINT=$(basename "$FULL_BEST_MODEL" .pt | sed 's/^model_//')

  echo "$FULL_BEST_MODEL" "$CHECKPOINT"
}

# Iterate over each .py file in the "my_next_experiments" folder
for EXPERIMENT_FILE in "$NEXT_EXPERIMENTS_DIR"/*.py; do
    # Extract the base name of the experiment file (without path and extension)
    EXPERIMENT_NAME=$(basename "$EXPERIMENT_FILE" .py)
    
    # Generate a timestamp
    TIMESTAMP=$(date +"%d%m%Y_%H%M%S")
    LOG_DATE=$(date +"%d/%m/%Y %H:%M:%S")
    
    # Create a new folder name with the format "timestamp_experiment_name"
    EXPERIMENT_FOLDER="${BASE_EXPERIMENT_FOLDER}/${TIMESTAMP}_${EXPERIMENT_NAME}"
    mkdir -p "$EXPERIMENT_FOLDER"
    
    # Log file in the new experiment folder
    LOG_FILE="${EXPERIMENT_FOLDER}/experiment.log"
    exec 3>&1 1>>"$LOG_FILE" 2>&1
    
    echo "================================================================="
    echo "  Experiment: $EXPERIMENT_NAME"
    echo "  Timestamp: $LOG_DATE"
    echo "  Experiment Folder: $EXPERIMENT_FOLDER"
    echo "================================================================="
    
    # Remove the existing legged_robot_config.py file
    TARGET_FILE="$MY_EXPERIMENTS_DIR/legged_robot_config.py"
    if [ -f "$TARGET_FILE" ]; then
        echo "  > Removing existing $TARGET_FILE"
        rm "$TARGET_FILE"
    fi
    
    # Copy the current .py file to the target location and rename it
    echo "  > Copying $EXPERIMENT_FILE to $TARGET_FILE"
    cp "$EXPERIMENT_FILE" "$TARGET_FILE"
    
    # Run the main experiment code
    #### MAIN TRAINING LOOP ####
    ROUGH_MODEL_DIR="$EXPERIMENT_FOLDER/rough_best_models"
    mkdir -p "$ROUGH_MODEL_DIR"
    
    #-----------------------
    NUM_OF_RUNS=5
    NUM_OF_ITERATIONS_ROUGH=6500
    NUM_OF_ITERATIONS_OBSTACLES=1500
    NUM_OF_IMAGES=5000
    #-----------------------
    
    for RUN in $(seq 1 $NUM_OF_RUNS); do
        echo "-------------------------------------------------------------"
        echo "  Starting Rough Terrain Training: Run $RUN"
        echo "-------------------------------------------------------------"
        
        RUN_FOLDER="$EXPERIMENT_FOLDER/rough/run_$RUN"
        mkdir -p "$RUN_FOLDER"
        
        TRAIN_LOG="$RUN_FOLDER/rough_train.log"
        PLAY_LOG="$RUN_FOLDER/rough_play.log"
        MAX_ITERATIONS=$NUM_OF_ITERATIONS_ROUGH
        
        echo "  > $MAX_ITERATIONS iterations..."
        TRAIN_START_TIME=$(date +%s)
        python legged_gym/scripts/train.py --task=aliengo_rough --headless --max_iterations=$MAX_ITERATIONS > "$TRAIN_LOG" 2>&1
        TRAIN_END_TIME=$(date +%s)
        TRAIN_DURATION=$((TRAIN_END_TIME - TRAIN_START_TIME))
        TRAIN_DURATION_FORMATTED=$(format_duration $TRAIN_DURATION)
        echo "  > Training completed in $TRAIN_DURATION_FORMATTED."
        
        echo "  > Recording Rough Terrain results..."
        MODEL_DIR="$(pwd)/logs/rough_aliengo"
        if [ -z "$(ls -A $MODEL_DIR 2>/dev/null)" ]; then
            echo "Error: No model directories found in $MODEL_DIR."
            echo "Check the training log for errors: $TRAIN_LOG"
            exit 1
        fi
        
        LOAD_RUN_CHECKPOINT=$(find_best_model "$MODEL_DIR")
        set -- $LOAD_RUN_CHECKPOINT
        LATEST_FOLDER=$1
        CHECKPOINT=$2
        if [ -z "$CHECKPOINT" ]; then
            echo "Error: Could not determine the best model checkpoint."
            exit 1
        fi
        
        echo "  > Using checkpoint=$CHECKPOINT for playing."
        python legged_gym/scripts/play.py --task=aliengo_rough --resume --load_run="$(basename $LATEST_FOLDER)" --checkpoint="$CHECKPOINT" --headless > "$PLAY_LOG" 2>&1 &
        PLAY_PID=$!
        TARGET_DIR="$(pwd)/logs/rough_aliengo/exported/frames"
        mkdir -p "$TARGET_DIR"
        MAX_IMAGES=$NUM_OF_IMAGES
        sleep 10
        SLEEP_INTERVAL=2
        
        count_png_files() {
            ls "$TARGET_DIR"/*.png 2>/dev/null | wc -l
        }
        
        while [ $(count_png_files) -lt $MAX_IMAGES ]; do
            sleep $SLEEP_INTERVAL
        done
        kill $PLAY_PID
        VIDEO_FILE="rough_run_$RUN.mp4"
        LOG_FILE="$RUN_FOLDER/rough_ffmpeg.log"
        /home/josep-barbera/Desktop/ffmpeg-7.0.1-amd64-static/ffmpeg -framerate 50 -pattern_type glob -i "${TARGET_DIR}/*.png" -frames:v 9999 -c:v libx264 -pix_fmt yuv420p "$RUN_FOLDER/$VIDEO_FILE" > "$LOG_FILE" 2>&1
        echo "  > Video created for ROUGH (Run $RUN)"
        rm -f "${TARGET_DIR}/"*.png
        echo "  > PNG files removed"
        
        cp "${LATEST_FOLDER}model_${CHECKPOINT}.pt" "$ROUGH_MODEL_DIR/run_${RUN}_model_${CHECKPOINT}.pt"
    done
    
    # BEST_ROUGH_MODEL=$(find_best_model_across_runs "$ROUGH_MODEL_DIR")
    # set -- $BEST_ROUGH_MODEL
    BEST_ROUGH_MODEL_PATH="$ROUGH_MODEL_DIR/run_${RUN}_model_${CHECKPOINT}.pt"
    # BEST_CHECKPOINT=$2
    
    echo "-------------------------------------------------------------"
    echo "  Next stage with best rough checkpoint: $BEST_CHECKPOINT"
    echo "-------------------------------------------------------------"
    
    #### OBSTACLES TRAINING LOOP ####
    for RUN in $(seq 1 $NUM_OF_RUNS); do
        echo "-------------------------------------------------------------"
        echo "  Starting Obstacle Terrain Training: Run $RUN"
        echo "-------------------------------------------------------------"
    
        RUN_FOLDER="$EXPERIMENT_FOLDER/obstacles/run_$RUN"
        mkdir -p "$RUN_FOLDER"
        
        TRAIN_LOG="$RUN_FOLDER/obs_train.log"
        PLAY_LOG="$RUN_FOLDER/obs_play.log"
        MAX_ITERATIONS=$NUM_OF_ITERATIONS_OBSTACLES
        
        echo "  > $MAX_ITERATIONS iterations..."
        TRAIN_START_TIME=$(date +%s)
        python legged_gym/scripts/train.py --task=aliengo_obs --headless --resume --alt-ckpt="$BEST_ROUGH_MODEL_PATH" --max_iterations=$MAX_ITERATIONS > "$TRAIN_LOG" 2>&1
        TRAIN_END_TIME=$(date +%s)
        TRAIN_DURATION=$((TRAIN_END_TIME - TRAIN_START_TIME))
        TRAIN_DURATION_FORMATTED=$(format_duration $TRAIN_DURATION)
        echo "  > Training completed in $TRAIN_DURATION_FORMATTED."
        
        echo "  > Recording Obstacle results..."
        MODEL_DIR="$(pwd)/logs/obs_aliengo"
        LOAD_RUN_CHECKPOINT=$(find_best_model "$MODEL_DIR")
        set -- $LOAD_RUN_CHECKPOINT
        LATEST_FOLDER=$1
        CHECKPOINT=$2
        
        echo "  > Using checkpoint=$CHECKPOINT for playing."
        python legged_gym/scripts/play.py --task=aliengo_obs --resume --load_run="$(basename $LATEST_FOLDER)" --checkpoint="$CHECKPOINT" --headless > "$PLAY_LOG" 2>&1 &
        PLAY_PID=$!
        TARGET_DIR="$(pwd)/logs/obs_aliengo/exported/frames"
        mkdir -p "$TARGET_DIR"
        MAX_IMAGES=$NUM_OF_IMAGES
        SLEEP_INTERVAL=10
        sleep 30
        
        while [ $(count_png_files) -lt $MAX_IMAGES ]; do
            sleep $SLEEP_INTERVAL
        done
        kill $PLAY_PID
        VIDEO_FILE="obstacles_run_$RUN.mp4"
        LOG_FILE="$RUN_FOLDER/obs_ffmpeg.log"
        /home/josep-barbera/Desktop/ffmpeg-7.0.1-amd64-static/ffmpeg -framerate 50 -pattern_type glob -i "${TARGET_DIR}/*.png" -frames:v 9999 -c:v libx264 -pix_fmt yuv420p "$RUN_FOLDER/$VIDEO_FILE" > "$LOG_FILE" 2>&1
        echo "  > Video created for OBSTACLES (Run $RUN)"
        rm -f "${TARGET_DIR}/"*.png
        echo "  > PNG files removed"
        
    done

    echo "================================================================="
    echo "   FINISHED EXPERIMENT FOR $EXPERIMENT_NAME"
    echo "================================================================="

done

echo "================================================================="
echo "   FINISHED ALL EXPERIMENTS"
echo "================================================================="
