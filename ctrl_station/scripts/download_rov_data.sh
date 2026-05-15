#!/bin/bash

# Configuration
REMOTE_USER="jetson"
REMOTE_HOST="192.168.2.3"
REMOTE_PATH="~/mission_logs/"
REMOTE_IMG_PATH="~/yolo/images/"
LOCAL_PATH="$HOME/rov_logs/"
LOCAL_IMG_PATH="$HOME/rov_logs/images/"

# Colors
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

log_communication() {
    local message="$1"
    local timestamp
    timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$timestamp] CONTROL_STATION: $message" >> "$COMM_LOG_FILE"
    echo -e "$message"
}

TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
TEMP_COMM_LOG="/tmp/rov_comm_log_$TIMESTAMP.log"

echo "=== ROV Control Station Communication Log ===" > "$TEMP_COMM_LOG"
echo "Session started: $(date '+%Y-%m-%d %H:%M:%S')" >> "$TEMP_COMM_LOG"
echo "Control Station: $(hostname)" >> "$TEMP_COMM_LOG"
echo "Robot Host: $REMOTE_USER@$REMOTE_HOST" >> "$TEMP_COMM_LOG"
echo "=============================================" >> "$TEMP_COMM_LOG"
echo "" >> "$TEMP_COMM_LOG"

COMM_LOG_FILE="$TEMP_COMM_LOG"

log_communication "${YELLOW}Starting ROV log sync session...${NC}"

if [ ! -d "$LOCAL_PATH" ]; then
    log_communication "${YELLOW}Creating local directory: $LOCAL_PATH${NC}"
    mkdir -p "$LOCAL_PATH"
fi

log_communication "${YELLOW}Initiating SSH connection to robot...${NC}"
log_communication "${YELLOW}Requesting directory listing from $REMOTE_PATH${NC}"

RECENT_FOLDER=$(ssh "$REMOTE_USER@$REMOTE_HOST" "ls -1t $REMOTE_PATH | head -n 1" 2>/dev/null)
if [ $? -ne 0 ]; then
    log_communication "${RED}ERROR: SSH connection failed to robot${NC}"
    echo "" >> "$COMM_LOG_FILE"
    echo "Session ended with error: $(date '+%Y-%m-%d %H:%M:%S')" >> "$COMM_LOG_FILE"
    echo "Error: SSH connection failed" >> "$COMM_LOG_FILE"
    exit 1
fi

if [ -z "$RECENT_FOLDER" ]; then
    log_communication "${RED}ERROR: No mission folders found on robot${NC}"
    echo "" >> "$COMM_LOG_FILE"
    echo "Session ended with error: $(date '+%Y-%m-%d %H:%M:%S')" >> "$COMM_LOG_FILE"
    echo "Error: No folders found in remote path" >> "$COMM_LOG_FILE"
    exit 1
fi

log_communication "${GREEN}Most recent mission folder is '$RECENT_FOLDER'${NC}"

if [ -d "$LOCAL_PATH$RECENT_FOLDER" ]; then
    log_communication "${YELLOW}Mission folder already exists locally - updating${NC}"
else
    log_communication "${YELLOW}New mission folder detected - starting transfer${NC}"
fi

log_communication "${YELLOW}Starting file transfer...${NC}"
TRANSFER_START=$(date +%s)
scp -r "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH$RECENT_FOLDER" "$LOCAL_PATH"
if [ $? -ne 0 ]; then
    log_communication "${RED}ERROR: File transfer failed${NC}"
    exit 1
fi
TRANSFER_END=$(date +%s)
TRANSFER_TIME=$((TRANSFER_END - TRANSFER_START))
log_communication "${GREEN}Transfer completed in ${TRANSFER_TIME} seconds${NC}"

# Check if remote images directory exists and copy it
log_communication "${YELLOW}Checking for images directory on robot...${NC}"
REMOTE_IMG_EXISTS=$(ssh "$REMOTE_USER@$REMOTE_HOST" "[ -d $REMOTE_IMG_PATH ] && echo 'exists'" 2>/dev/null)
if [ "$REMOTE_IMG_EXISTS" = "exists" ]; then
    log_communication "${GREEN}Images directory found on robot${NC}"
    
    # Create local images directory if it doesn't exist
    if [ ! -d "$LOCAL_IMG_PATH" ]; then
        log_communication "${YELLOW}Creating local images directory: $LOCAL_IMG_PATH${NC}"
        mkdir -p "$LOCAL_IMG_PATH"
    fi
    
    log_communication "${YELLOW}Starting images transfer...${NC}"
    IMG_TRANSFER_START=$(date +%s)
    scp -r "$REMOTE_USER@$REMOTE_HOST:$REMOTE_IMG_PATH"* "$LOCAL_IMG_PATH"
    if [ $? -ne 0 ]; then
        log_communication "${RED}ERROR: Images transfer failed${NC}"
        exit 1
    fi
    IMG_TRANSFER_END=$(date +%s)
    IMG_TRANSFER_TIME=$((IMG_TRANSFER_END - IMG_TRANSFER_START))
    log_communication "${GREEN}Images transfer completed in ${IMG_TRANSFER_TIME} seconds${NC}"
else
    log_communication "${YELLOW}Images directory not found on robot - skipping${NC}"
fi

MISSION_FOLDER="$LOCAL_PATH$RECENT_FOLDER"
ROS2_LOG="$HOME/robot_ctrlstation_communication.log"
FINAL_LOG="$MISSION_FOLDER/robot_ctrlstation_communication.log"

# Check if ROS2 log file exists, create it if it doesn't
if [ ! -f "$ROS2_LOG" ]; then
    log_communication "${YELLOW}ROS2 log file not found - creating new file: $ROS2_LOG${NC}"
    
    # Create the log file with initial header
    cat > "$ROS2_LOG" << EOF
=== Robot Control Station Communication Log ===
Log file created: $(date '+%Y-%m-%d %H:%M:%S')
Control Station: $(hostname)
================================================

EOF
    log_communication "${GREEN}✓ Created new ROS2 log file${NC}"
fi

{
    cat "$ROS2_LOG"
    echo ""
    echo "===== SSH SYNC SESSION LOG - $(date '+%Y-%m-%d %H:%M:%S') ====="
    cat "$COMM_LOG_FILE"
} > "$FINAL_LOG"

log_communication "${GREEN}✓ Combined log saved to mission folder: $FINAL_LOG${NC}"

rm -f "$TEMP_COMM_LOG"

echo "$RECENT_FOLDER" > "$LOCAL_PATH/.recent_folder"

exit 0

