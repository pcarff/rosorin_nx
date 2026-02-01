#!/bin/bash
# deploy_to_robot.sh
# Usage: ./deploy_to_robot.sh

ROBOT_IP="192.168.8.162"
USER="ubuntu"
REMOTE_WS="/home/ubuntu/ros2_ws"

echo "Deploying src/app to $USER@$ROBOT_IP:$REMOTE_WS/src/app..."

# 1. Sync the app package
rsync -avz --progress \
    -e "ssh -o StrictHostKeyChecking=no" \
    src/app/ \
    $USER@$ROBOT_IP:$REMOTE_WS/src/app/

echo "Deployment complete. You may need to run 'colcon build --packages-select app' on the robot."
echo "Login command: ssh $USER@$ROBOT_IP"
