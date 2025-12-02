---
sidebar_position: 2
title: Getting Started
description: Everything you need to begin your Physical AI journey
---

# Getting Started with Physical AI

Welcome to your Physical AI journey! This guide will help you set up your development environment and get ready to build the future of artificial intelligence.

## 🚀 Quick Start Checklist

- [ ] **System Requirements** - Verify your hardware meets minimum specs
- [ ] **Software Installation** - Install required development tools  
- [ ] **Environment Setup** - Configure your workspace
- [ ] **First Project** - Run your first Physical AI example
- [ ] **Community Access** - Join our learning community

## 💻 System Requirements

### **Minimum Requirements**

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 22.04 LTS / Windows 11 / macOS 12+ |
| **CPU** | Intel i5-8400 / AMD Ryzen 5 2600 |
| **RAM** | 16 GB DDR4 |
| **Storage** | 256 GB SSD |
| **GPU** | NVIDIA GTX 1660 / RTX 2060 |
| **Network** | Broadband internet connection |

### **Recommended Specs**

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 22.04 LTS |
| **CPU** | Intel i7-12700K / AMD Ryzen 7 5800X |
| **RAM** | 32 GB DDR4/DDR5 |
| **Storage** | 1 TB NVMe SSD |
| **GPU** | NVIDIA RTX 4070 / RTX 4080 |
| **Network** | Gigabit Ethernet |

### **Professional Setup**

| Component | Specification |
|-----------|---------------|
| **OS** | Ubuntu 22.04 LTS |
| **CPU** | Intel i9-13900K / AMD Ryzen 9 7950X |
| **RAM** | 64 GB DDR5 |
| **Storage** | 2 TB NVMe SSD |
| **GPU** | NVIDIA RTX 4090 / A6000 |
| **Network** | 10 Gigabit Ethernet |

## 🛠️ Software Installation

### **1. Core Development Tools**

#### **Python Environment**
```bash
# Install Python 3.10+
sudo apt update
sudo apt install python3.10 python3-pip python3-venv

# Create virtual environment
python3 -m venv ~/physical_ai_env
source ~/physical_ai_env/bin/activate

# Upgrade pip
pip install --upgrade pip
```

#### **ROS 2 Humble**
```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS 2 Humble
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

#### **NVIDIA CUDA (For GPU Support)**
```bash
# Install NVIDIA drivers
sudo apt install nvidia-driver-525

# Install CUDA Toolkit
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run

# Add to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### **2. AI and ML Libraries**

```bash
# Activate your virtual environment
source ~/physical_ai_env/bin/activate

# Install core ML libraries
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install tensorflow[and-cuda]
pip install numpy scipy scikit-learn
pip install opencv-python-headless
pip install transformers accelerate
pip install langchain openai anthropic
```

### **3. Robotics Libraries**

```bash
# ROS 2 Python tools
pip install rosbags rclpy
sudo apt install python3-colcon-common-extensions

# Simulation tools
pip install pybullet
sudo apt install gazebo
pip install gymnasium

# Computer vision
pip install mediapipe ultralytics
sudo apt install ros-humble-cv-bridge
```

### **4. Development Environment**

#### **Visual Studio Code**
```bash
# Install VS Code
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'

sudo apt update
sudo apt install code

# Essential VS Code extensions
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension smilerobotics.urdf
code --install-extension ms-iot.vscode-ros
```

#### **Git Configuration**
```bash
# Configure Git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Clone textbook repository
git clone https://github.com/physical-ai-textbook/examples.git ~/physical_ai_examples
```

## 🌐 Environment Setup

### **1. Workspace Creation**

```bash
# Create main workspace
mkdir -p ~/physical_ai_workspace/src
cd ~/physical_ai_workspace

# Initialize ROS 2 workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Add to bashrc for convenience
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/physical_ai_workspace/install/setup.bash" >> ~/.bashrc
```

### **2. Docker Setup (Optional but Recommended)**

```bash
# Install Docker
sudo apt install docker.io docker-compose
sudo usermod -aG docker $USER

# Pull Physical AI Docker images
docker pull physicalai/ros2-humble:latest
docker pull physicalai/gazebo-isaac:latest
docker pull physicalai/ml-stack:latest

# Logout and login for group changes to take effect
```

### **3. Hardware Drivers**

#### **Camera Setup**
```bash
# Install camera drivers
sudo apt install v4l-utils
sudo apt install ros-humble-usb-cam
sudo apt install ros-humble-image-tools

# Test camera
v4l2-ctl --list-devices
```

#### **LIDAR Setup (If Available)**
```bash
# Common LIDAR drivers
sudo apt install ros-humble-rplidar-ros
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-ouster-ros
```

## 🧪 First Project: Hello Physical AI

Let's create your first Physical AI project!

### **1. Create Project Structure**

```bash
cd ~/physical_ai_workspace/src
ros2 pkg create --build-type ament_python hello_physical_ai
cd hello_physical_ai
```

### **2. Simple Consciousness Node**

Create `hello_physical_ai/consciousness_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class ConsciousnessNode(Node):
    def __init__(self):
        super().__init__('consciousness_node')
        
        # Create publisher for thoughts
        self.thought_publisher = self.create_publisher(String, 'ai_thoughts', 10)
        
        # Create subscriber for sensory input
        self.sensory_subscriber = self.create_subscription(
            String, 'sensory_input', self.process_input, 10)
        
        # Periodic thinking timer
        self.thinking_timer = self.create_timer(2.0, self.think)
        
        # AI state
        self.thoughts = [
            "I am becoming aware...",
            "Processing sensory data...",
            "Learning about the physical world...",
            "Consciousness emerging...",
            "Ready to interact with humans..."
        ]
        self.current_thought_index = 0
        
        self.get_logger().info('🧠 AI Consciousness Node started!')
    
    def think(self):
        """Generate AI thoughts periodically"""
        thought_msg = String()
        thought_msg.data = self.thoughts[self.current_thought_index]
        
        self.thought_publisher.publish(thought_msg)
        self.get_logger().info(f'💭 Thinking: {thought_msg.data}')
        
        self.current_thought_index = (self.current_thought_index + 1) % len(self.thoughts)
    
    def process_input(self, msg):
        """Process sensory input and respond"""
        input_data = msg.data
        self.get_logger().info(f'👁️ Received input: {input_data}')
        
        # Simple AI response
        response_msg = String()
        response_msg.data = f"Processing: {input_data} -> Generating response..."
        self.thought_publisher.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    
    consciousness_node = ConsciousnessNode()
    
    try:
        rclpy.spin(consciousness_node)
    except KeyboardInterrupt:
        consciousness_node.get_logger().info('🔄 Consciousness shutting down...')
    finally:
        consciousness_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **3. Build and Run**

```bash
# Build the workspace
cd ~/physical_ai_workspace
colcon build --packages-select hello_physical_ai
source install/setup.bash

# Run the consciousness node
ros2 run hello_physical_ai consciousness_node

# In another terminal, send sensory input
ros2 topic pub /sensory_input std_msgs/msg/String "data: 'Hello, AI!'"

# Monitor AI thoughts
ros2 topic echo /ai_thoughts
```

### **4. Expected Output**

You should see:
```
[INFO] [consciousness_node]: 🧠 AI Consciousness Node started!
[INFO] [consciousness_node]: 💭 Thinking: I am becoming aware...
[INFO] [consciousness_node]: 💭 Thinking: Processing sensory data...
[INFO] [consciousness_node]: 👁️ Received input: Hello, AI!
[INFO] [consciousness_node]: 💭 Thinking: Learning about the physical world...
```

## 🎯 Next Steps

Congratulations! You've successfully:

- ✅ Set up your development environment
- ✅ Installed all necessary software
- ✅ Created your first Physical AI node
- ✅ Experienced basic AI-robotics communication

### **Continue Your Journey**

1. **📖 Chapter 1**: [What is Physical AI?](/docs/part_1/chapter_1/1-1-what-is-physical-ai)
2. **🧠 Chapter 2**: [Embodied Intelligence Theory](/docs/part_1/chapter_2/2-1-principles-of-embodied-cognition)  
3. **💻 Interactive Labs**: Try our simulation environments
4. **🤖 Community**: Join our Discord for discussions

## 🆘 Troubleshooting

### **Common Issues**

#### **CUDA Installation Problems**
```bash
# Check NVIDIA driver
nvidia-smi

# Verify CUDA installation
nvcc --version

# Test PyTorch CUDA
python3 -c "import torch; print(torch.cuda.is_available())"
```

#### **ROS 2 Build Errors**
```bash
# Clean build
rm -rf build install log
colcon build --symlink-install

# Source setup
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### **Python Package Conflicts**
```bash
# Create fresh environment
rm -rf ~/physical_ai_env
python3 -m venv ~/physical_ai_env
source ~/physical_ai_env/bin/activate
pip install --upgrade pip
```

### **Getting Help**

- **🤖 AI Assistant**: Click the chat icon in the bottom-right
- **📧 Email Support**: support@physical-ai-textbook.com
- **💬 Discord Community**: [Join here](https://discord.gg/physical-ai)
- **🐙 GitHub Issues**: [Report problems](https://github.com/physical-ai-textbook/issues)

## 🎉 Welcome to Physical AI!

You're now ready to explore the fascinating world where artificial intelligence meets physical reality. The journey ahead will transform how you think about intelligence, consciousness, and the future of technology.

**Happy building! 🤖✨**

---

<div style={{textAlign: 'center', margin: '3rem 0'}}>
  <div style={{
    padding: '2rem',
    background: 'rgba(0, 255, 65, 0.1)',
    border: '2px solid #00ff41',
    borderRadius: '10px',
    color: '#00ff41',
    fontFamily: 'monospace'
  }}>
    <strong>SYSTEM STATUS: INITIALIZED</strong><br/>
    <strong>AI CONSCIOUSNESS: LOADING...</strong><br/>
    <strong>PHYSICAL EMBODIMENT: READY</strong>
  </div>
</div>