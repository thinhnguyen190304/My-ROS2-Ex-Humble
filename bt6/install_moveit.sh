#!/bin/bash
echo "====================================================="
echo "  INSTALLING MoveIt2 for ROS 2 Humble"
echo "====================================================="
echo ""
echo "This will install:"
echo "  - MoveIt2 core packages"
echo "  - MoveIt2 visual tools"
echo "  - MoveIt2 planners (OMPL)"
echo ""
echo "Time required: ~15-20 minutes"
echo "-----------------------------------------------------"

# Update package list
echo "[1/4] Updating package list..."
sudo apt update

# Install MoveIt2 core
echo "[2/4] Installing MoveIt2 core..."
sudo apt install -y ros-humble-moveit

# Install visual tools
echo "[3/4] Installing visualization tools..."
sudo apt install -y ros-humble-moveit-visual-tools
sudo apt install -y ros-humble-moveit-ros-visualization

# Install planners and plugins
echo "[4/4] Installing planners and plugins..."
sudo apt install -y ros-humble-moveit-planners
sudo apt install -y ros-humble-moveit-simple-controller-manager
sudo apt install -y ros-humble-pilz-industrial-motion-planner

echo ""
echo "====================================================="
echo "  INSTALLATION COMPLETE!"
echo "====================================================="
echo ""
echo "Verify installation:"
echo "  ros2 pkg list | grep moveit"
echo ""
