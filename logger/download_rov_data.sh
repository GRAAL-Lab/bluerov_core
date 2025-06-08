#!/bin/bash

# Configuration
REMOTE_USER="paolo"                                 # Username on the BlueRov2
REMOTE_HOST="192.168.1.112"                         # IP address of the BlueRov2 - shuld be statically assigned
REMOTE_PATH="/home/paolo/mission_logs/"             # Path on the BlueRov2 where mission logs are stored
LOCAL_PATH="/home/paolo/Desktop/RAMI/rov_logs/"     # Local path on the control station where logs will be saved

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to log communication with timestamp
log_communication() {
    local message="$1"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$timestamp] CONTROL_STATION: $message" >> "$COMM_LOG_FILE"
    echo -e "$message"
}

# Initialize communication log
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
TEMP_COMM_LOG="/tmp/rov_comm_log_$TIMESTAMP.log"

# Start communication logging
echo "=== ROV Control Station Communication Log ===" > "$TEMP_COMM_LOG"
echo "Session started: $(date '+%Y-%m-%d %H:%M:%S')" >> "$TEMP_COMM_LOG"
echo "Control Station: $(hostname)" >> "$TEMP_COMM_LOG"
echo "Robot Host: $REMOTE_USER@$REMOTE_HOST" >> "$TEMP_COMM_LOG"
echo "=============================================" >> "$TEMP_COMM_LOG"
echo "" >> "$TEMP_COMM_LOG"

COMM_LOG_FILE="$TEMP_COMM_LOG"

log_communication "${YELLOW}Starting ROV log sync session...${NC}"

# Check if local directory exists, create if not
if [ ! -d "$LOCAL_PATH" ]; then
    log_communication "${YELLOW}Creating local directory: $LOCAL_PATH${NC}"
    mkdir -p "$LOCAL_PATH"
fi

# Get the most recent folder from remote machine
log_communication "${YELLOW}Initiating SSH connection to robot...${NC}"
log_communication "${YELLOW}Requesting directory listing from $REMOTE_PATH${NC}"

RECENT_FOLDER=$(ssh "$REMOTE_USER@$REMOTE_HOST" "ls -1t $REMOTE_PATH | head -n 1" 2>/dev/null)

# Check if SSH connection was successful
if [ $? -ne 0 ]; then
    log_communication "${RED}ERROR: SSH connection failed to robot${NC}"
    log_communication "${RED}Connection attempt failed at $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${RED}Please check:${NC}"
    echo -e "${RED}  - Network connectivity${NC}"
    echo -e "${RED}  - SSH credentials${NC}"
    echo -e "${RED}  - Remote host is accessible${NC}"
    
    # Finalize communication log even on error
    echo "" >> "$COMM_LOG_FILE"
    echo "Session ended with error: $(date '+%Y-%m-%d %H:%M:%S')" >> "$COMM_LOG_FILE"
    echo "Error: SSH connection failed" >> "$COMM_LOG_FILE"
    
    exit 1
fi

log_communication "${GREEN}SSH connection established successfully${NC}"

# Check if we found a folder
if [ -z "$RECENT_FOLDER" ]; then
    log_communication "${RED}ERROR: No mission folders found on robot${NC}"
    echo "" >> "$COMM_LOG_FILE"
    echo "Session ended with error: $(date '+%Y-%m-%d %H:%M:%S')" >> "$COMM_LOG_FILE"
    echo "Error: No folders found in remote path" >> "$COMM_LOG_FILE"
    exit 1
fi

log_communication "${GREEN}Robot responded: Most recent mission folder is '$RECENT_FOLDER'${NC}"

# Check if folder already exists locally
if [ -d "$LOCAL_PATH$RECENT_FOLDER" ]; then
    log_communication "${YELLOW}Mission folder already exists locally - performing update${NC}"
else
    log_communication "${YELLOW}New mission folder detected - initiating transfer${NC}"
fi

# Copy the folder using scp with recursive flag
log_communication "${YELLOW}Starting file transfer from robot to control station${NC}"
log_communication "${YELLOW}Source: $REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH$RECENT_FOLDER${NC}"
log_communication "${YELLOW}Destination: $LOCAL_PATH${NC}"

# Get start time for transfer
TRANSFER_START=$(date +%s)

scp -r "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH$RECENT_FOLDER" "$LOCAL_PATH"

# Calculate transfer time
TRANSFER_END=$(date +%s)
TRANSFER_TIME=$((TRANSFER_END - TRANSFER_START))

# Check if copy was successful
if [ $? -eq 0 ]; then
    log_communication "${GREEN}File transfer completed successfully${NC}"
    log_communication "${GREEN}Transfer time: ${TRANSFER_TIME} seconds${NC}"
    
    # Show some stats
    FOLDER_SIZE=$(du -sh "$LOCAL_PATH$RECENT_FOLDER" 2>/dev/null | cut -f1)
    log_communication "${GREEN}Mission data size: $FOLDER_SIZE${NC}"
    
    # Move communication log to the copied folder
    FINAL_COMM_LOG="$LOCAL_PATH$RECENT_FOLDER/control_station_communication.log"
    
    # Add session summary to log
    echo "" >> "$COMM_LOG_FILE"
    echo "=== SESSION SUMMARY ===" >> "$COMM_LOG_FILE"
    echo "Mission folder: $RECENT_FOLDER" >> "$COMM_LOG_FILE"
    echo "Data size: $FOLDER_SIZE" >> "$COMM_LOG_FILE"
    echo "Transfer time: ${TRANSFER_TIME} seconds" >> "$COMM_LOG_FILE"
    echo "Status: SUCCESS" >> "$COMM_LOG_FILE"
    echo "Session ended: $(date '+%Y-%m-%d %H:%M:%S')" >> "$COMM_LOG_FILE"
    
    # Copy communication log to mission folder
    cp "$COMM_LOG_FILE" "$FINAL_COMM_LOG"
    
    log_communication "${GREEN}✓ Communication log saved to: $FINAL_COMM_LOG${NC}"
    log_communication "${GREEN}✓ Mission sync completed successfully!${NC}"
    
    # Clean up temporary log
    rm -f "$TEMP_COMM_LOG"
    
else
    log_communication "${RED}ERROR: File transfer failed${NC}"
    
    # Add error to log
    echo "" >> "$COMM_LOG_FILE"
    echo "Session ended with error: $(date '+%Y-%m-%d %H:%M:%S')" >> "$COMM_LOG_FILE"
    echo "Error: File transfer failed" >> "$COMM_LOG_FILE"
    
    # Still try to save the communication log somewhere
    ERROR_LOG="$LOCAL_PATH/failed_sync_communication_$TIMESTAMP.log"
    cp "$COMM_LOG_FILE" "$ERROR_LOG" 2>/dev/null
    
    echo -e "${RED}Communication log saved to: $ERROR_LOG${NC}"
    rm -f "$TEMP_COMM_LOG"
    
    exit 1
fi
