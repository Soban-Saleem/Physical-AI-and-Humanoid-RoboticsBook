---
sidebar_position: 3
title: Interactive Tutorials
description: Hands-on tutorials and guided projects for Physical AI learning
---

# Interactive Tutorials & Labs

Welcome to the hands-on learning experience! These interactive tutorials will guide you through building real Physical AI systems step by step.

## 🎯 Tutorial Categories

### 🟢 **Beginner Tutorials**
*Perfect for getting started with Physical AI concepts*

### 🟡 **Intermediate Projects** 
*Build practical robotic systems*

### 🔴 **Advanced Challenges**
*Push the boundaries of Physical AI*

---

## 🟢 Beginner Tutorials

### **Tutorial 1: Your First AI Robot** ⭐
**Duration**: 30 minutes | **Difficulty**: Beginner

Create a simple autonomous robot that can see and respond to its environment.

**What You'll Learn:**
- Basic ROS 2 node creation
- Camera input processing
- Simple decision making
- Motor control basics

**Prerequisites:** 
- Completed [Getting Started](/docs/getting-started)
- Basic Python knowledge

<details>
<summary><strong>📋 Tutorial Steps</strong></summary>

#### Step 1: Setup Your Robot Brain
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class BasicAIRobot(Node):
    def __init__(self):
        super().__init__('basic_ai_robot')
        
        # Image processing
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # Robot movement
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('🤖 Basic AI Robot started!')
    
    def image_callback(self, msg):
        """Process camera input and make decisions"""
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Simple AI: Detect red objects
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        
        # Calculate red object position
        moments = cv2.moments(red_mask)
        if moments['m00'] > 0:
            # Red object detected - move towards it
            cx = int(moments['m10'] / moments['m00'])
            image_center = cv_image.shape[1] // 2
            
            twist = Twist()
            # Turn towards red object
            twist.angular.z = (image_center - cx) * 0.01
            twist.linear.x = 0.3  # Move forward
            self.movement_publisher.publish(twist)
            
            self.get_logger().info(f'🔴 Red object detected at x={cx}')
        else:
            # No red object - search
            twist = Twist()
            twist.angular.z = 0.5  # Rotate to search
            self.movement_publisher.publish(twist)
```

#### Step 2: Launch Your Robot
```bash
# Terminal 1: Start simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run your AI robot
ros2 run my_ai_robot basic_ai_robot

# Terminal 3: Add red objects to track
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: "red_ball", xml: "<sdf version=\"1.6\">...</sdf>"}'
```

#### Step 3: Test and Improve
- Watch your robot search for and follow red objects
- Try adding different colored objects
- Experiment with movement speeds
- Add obstacle avoidance

</details>

**🏆 Success Criteria:**
- [ ] Robot starts and shows camera feed
- [ ] Detects red objects in environment  
- [ ] Moves toward detected objects
- [ ] Rotates to search when no objects found

---

### **Tutorial 2: Voice-Controlled Robot Assistant** 🗣️
**Duration**: 45 minutes | **Difficulty**: Beginner

Build a robot that understands and responds to voice commands.

**What You'll Learn:**
- Speech recognition integration
- Natural language processing
- Text-to-speech synthesis
- Command execution

<details>
<summary><strong>📋 Tutorial Steps</strong></summary>

#### Step 1: Voice Recognition Setup
```python
import speech_recognition as sr
import pyttsx3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_robot')
        
        # Speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Text-to-speech
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        
        # ROS publishers
        self.command_publisher = self.create_publisher(String, '/robot_commands', 10)
        
        # Listen for commands
        self.listening_timer = self.create_timer(2.0, self.listen_for_commands)
        
        self.speak("Voice controlled robot ready. Say hello!")
    
    def speak(self, text):
        """Convert text to speech"""
        self.get_logger().info(f'🗣️ Speaking: {text}')
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()
    
    def listen_for_commands(self):
        """Listen for voice commands"""
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=3)
            
            # Recognize speech
            command = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f'👂 Heard: {command}')
            
            # Process command
            self.process_voice_command(command)
            
        except sr.WaitTimeoutError:
            pass  # No speech detected
        except sr.UnknownValueError:
            pass  # Could not understand
        except sr.RequestError:
            self.get_logger().error("Speech recognition service error")
    
    def process_voice_command(self, command):
        """Process and execute voice commands"""
        if 'hello' in command:
            self.speak("Hello! I am your robot assistant. How can I help you?")
        elif 'move forward' in command:
            self.execute_movement('forward')
            self.speak("Moving forward!")
        elif 'turn left' in command:
            self.execute_movement('left')
            self.speak("Turning left!")
        elif 'turn right' in command:
            self.execute_movement('right') 
            self.speak("Turning right!")
        elif 'stop' in command:
            self.execute_movement('stop')
            self.speak("Stopping now!")
        else:
            self.speak("Sorry, I didn't understand that command.")
    
    def execute_movement(self, direction):
        """Send movement commands to robot"""
        cmd_msg = String()
        cmd_msg.data = direction
        self.command_publisher.publish(cmd_msg)
```

</details>

---

## 🟡 Intermediate Projects

### **Project 1: Intelligent Home Assistant Robot** 🏠
**Duration**: 2-3 hours | **Difficulty**: Intermediate

Build a robot that can navigate your home, recognize objects, and perform tasks.

**Features:**
- SLAM mapping and navigation
- Object recognition and tracking
- Task scheduling and execution
- Natural language interaction

### **Project 2: Collaborative Manufacturing Robot** 🏭
**Duration**: 3-4 hours | **Difficulty**: Intermediate  

Create a robot that works alongside humans in manufacturing.

**Features:**
- Human pose estimation
- Safety zone monitoring
- Collaborative task execution
- Quality control with computer vision

### **Project 3: Educational Robot Companion** 📚
**Duration**: 2-3 hours | **Difficulty**: Intermediate

Develop a robot tutor that adapts to individual learning styles.

**Features:**
- Emotion recognition
- Adaptive teaching strategies
- Interactive quizzes and games
- Progress tracking

---

## 🔴 Advanced Challenges

### **Challenge 1: Autonomous Research Assistant** 🔬
**Duration**: 1-2 weeks | **Difficulty**: Advanced

Build a robot that can assist in laboratory research.

**Advanced Features:**
- Scientific instrument control
- Data collection and analysis
- Hypothesis generation
- Experimental design

### **Challenge 2: Search and Rescue Robot** 🚁
**Duration**: 1-2 weeks | **Difficulty**: Advanced

Create a robot for emergency response scenarios.

**Advanced Features:**
- Multi-sensor SLAM in GPS-denied environments
- Human detection and tracking
- Autonomous path planning through rubble
- Communication relay capabilities

### **Challenge 3: Swarm Robotics Coordination** 🐝
**Duration**: 2-3 weeks | **Difficulty**: Expert

Develop coordinated behavior for multiple robots.

**Advanced Features:**
- Distributed consensus algorithms
- Emergent behavior patterns
- Fault-tolerant coordination
- Scalable communication protocols

---

## 🛠️ Tools & Resources

### **Simulation Environments**
- **Gazebo**: Physics-based robot simulation
- **Isaac Sim**: Photorealistic NVIDIA simulation
- **Unity**: Game engine for robotics
- **PyBullet**: Python physics simulation

### **Hardware Platforms**
- **TurtleBot3**: Educational robot platform
- **Universal Robots**: Industrial robot arms  
- **Boston Dynamics Spot**: Quadruped robot
- **Custom builds**: DIY robot construction

### **AI Models & APIs**
- **OpenAI GPT-4**: Advanced language understanding
- **Google Gemini**: Multimodal AI capabilities
- **Anthropic Claude**: Safe AI assistance
- **Local models**: Privacy-focused alternatives

---

## 📊 Progress Tracking

### **Tutorial Completion Badges**

| Badge | Tutorial | Status |
|-------|----------|--------|
| 🥇 | First AI Robot | ⏳ Not Started |
| 🗣️ | Voice Control | ⏳ Not Started |
| 🏠 | Home Assistant | ⏳ Not Started |
| 🏭 | Manufacturing | ⏳ Not Started |
| 📚 | Robot Tutor | ⏳ Not Started |
| 🔬 | Research Assistant | ⏳ Not Started |
| 🚁 | Search & Rescue | ⏳ Not Started |
| 🐝 | Swarm Robotics | ⏳ Not Started |

### **Skill Levels**

Track your expertise across key areas:

- **🤖 ROS 2 Development**: ⭐⭐⭐⭐⭐
- **🧠 AI Integration**: ⭐⭐⭐⭐⭐  
- **👁️ Computer Vision**: ⭐⭐⭐⭐⭐
- **🗣️ Speech Processing**: ⭐⭐⭐⭐⭐
- **🎮 Simulation**: ⭐⭐⭐⭐⭐
- **🔧 Hardware Control**: ⭐⭐⭐⭐⭐

---

## 🆘 Need Help?

### **Tutorial Support**
- **💬 Community Discord**: Get help from other learners
- **🤖 AI Assistant**: Available 24/7 in the bottom corner
- **📧 Email Support**: tutorials@physical-ai-textbook.com
- **🎥 Video Walkthroughs**: Step-by-step video guides

### **Common Issues**

<details>
<summary><strong>🔧 Troubleshooting Guide</strong></summary>

#### Robot Not Responding
```bash
# Check ROS 2 nodes
ros2 node list
ros2 topic list

# Verify connections
ros2 topic echo /cmd_vel
ros2 topic echo /camera/image_raw
```

#### Speech Recognition Issues
```bash
# Test microphone
arecord -l
aplay -l

# Check permissions
sudo usermod -a -G audio $USER
```

#### Simulation Problems
```bash
# Reset Gazebo
killall gzserver gzclient
ros2 launch gazebo_ros gazebo.launch.py
```

</details>

---

## 🚀 Ready to Start Building?

Choose your adventure and start creating the future of Physical AI!

<div style={{display: 'flex', gap: '1rem', margin: '2rem 0', flexWrap: 'wrap'}}>
  <button style={{
    padding: '1rem 2rem',
    background: 'linear-gradient(45deg, rgba(0, 255, 65, 0.1), rgba(0, 255, 65, 0.2))',
    border: '2px solid #00ff41',
    color: '#00ff41',
    borderRadius: '5px',
    fontWeight: 'bold',
    cursor: 'pointer'
  }}>
    🟢 Start Beginner Tutorial
  </button>
  
  <button style={{
    padding: '1rem 2rem', 
    background: 'transparent',
    border: '2px solid #00d4ff',
    color: '#00d4ff',
    borderRadius: '5px',
    fontWeight: 'bold',
    cursor: 'pointer'
  }}>
    🟡 Try Intermediate Project
  </button>
  
  <button style={{
    padding: '1rem 2rem',
    background: 'transparent', 
    border: '2px solid #ff6b35',
    color: '#ff6b35',
    borderRadius: '5px',
    fontWeight: 'bold',
    cursor: 'pointer'
  }}>
    🔴 Take Advanced Challenge
  </button>
</div>

**Remember**: The best way to learn Physical AI is by building! Start with simple projects and gradually increase complexity as your skills grow.

Happy building! 🤖✨