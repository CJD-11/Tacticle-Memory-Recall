# Tacticle-Memory-Recall

A Haptic Interface for Embodied Spatial Recall
Tactile Memory Replay moves beyond traditional visual and auditory aids to leverage proprioceptive and haptic channels for spatial memory formation. This system investigates how finger pose recognition combined with haptic feedback can create persistent spatial memories, enabling users to "feel" previously visited locations through embodied interaction.
Research Context

This project builds on embodied cognition principles, creating haptic memory residencies where spatial relationships are encoded through natural hand movements and retrieved through proprioceptive reproduction. Instead of abstract digital interfaces, the system grounds memory formation in the body's natural sensorimotor coupling with the environment.

Quick Start

Prerequisites
Arduino IDE 1.8.0 or newer
Required libraries (see Installation)
Hardware components (see Bill of Materials)
Installation

Clone the repository:
git clone https://github.com/CJD-11/Tacticle-Memory-Recall
Install required Arduino libraries:
MPU6050 library by Electronic Cats
TCA9548A library
Upload the code:


Open src/tactile_memory_recall.ino in Arduino IDE
Select Arduino Mega 2560 as your board
Upload to your device
🔧 Hardware Overview
System Architecture
┌─────────────────────────────────────────────────────────────┐
│                    TACTILE MEMORY SYSTEM                   │
├─────────────────────────────────────────────────────────────┤
│  INPUT LAYER                                                │
│  ├── Multi-IMU Pose Detection (3× MPU6050)                 │
│  ├── Force Sensing Array (3× FSR402)                       │
│  └── I2C Multiplexer (TCA9548A)                           │
├─────────────────────────────────────────────────────────────┤
│  PROCESSING LAYER                                           │
│  ├── Arduino Mega 2560 (Main Controller)                   │
│  ├── Sensor Fusion Algorithms                              │
│  ├── Memory Management                                      │
│  └── Pattern Matching Engine                               │
├─────────────────────────────────────────────────────────────┤
│  OUTPUT LAYER                                               │
│  ├── Haptic Feedback Array (3× Motors)                     │
│  ├── Serial Communication Interface                        │
│  └── User Command Interface                                │
└─────────────────────────────────────────────────────────────┘

Bill of Materials
Component
Model
Quantity
Cost
Purpose
Microcontroller
Arduino Mega 2560
1
$45
Main processing unit
I2C Multiplexer
TCA9548A
1
$8
Multi-IMU communication
IMU Sensors
MPU6050
3
$15
Finger orientation tracking
Force Sensors
FSR 402
3
$36
Contact detection
Haptic Motors
10mm Coin Motors
3
$12
Tactile feedback
MOSFETs
2N7000
3
$1.50
Motor control
Resistors
1kΩ, 10kΩ
6
$2
Pull-ups and biasing
Breadboard
Half-size
1
$5
Circuit assembly
Jumper Wires
Various
30
$12
Connections
Total $137


System Operation
Core Functionality
MAP Mode - Spatial Memory Encoding
User positions fingers → Apply force → System records pose → Memory stored

REPLAY Mode - Embodied Retrieval
Finger movement → Pose comparison → Distance calculation → Haptic activation

Command Interface
Command
Function
MAP
Enter mapping mode to record new spatial memory
REPLAY
Enter replay mode to trigger haptic feedback
CALIBRATE
Real-time sensor monitoring and debugging
STATUS
Display system information and stored memories
RESET
Clear all stored spatial memories
TESTMOTOR
Verify haptic motor functionality
HELP
Display available commands

Technical Specifications
Specification
Value
Notes
Pose Accuracy
±10°
3-axis gyroscope precision
Response Latency
<50ms
Real-time haptic feedback
Memory Capacity
5 locations
Current EEPROM implementation
Power Consumption
~500mA @ 5V
Estimated total system draw
Tolerance Zone
300° threshold
Generous triggering for usability

Repository Structure
Arduino-Haptic-Glove/
├── README.md                    # This file
├── src/
│   ├── tactile_memory_recall.ino    # Main Arduino sketch
│   ├── sensor_calibration.ino       # Calibration utilities
│   └── haptic_test.ino             # Motor testing utilities
├── hardware/
│   ├── circuit_diagram.png         # Complete wiring diagram
│   ├── breadboard_layout.png       # Physical layout reference
│   └── component_photos/           # Assembly reference images
├── docs/
│   ├── assembly_guide.md           # Step-by-step build instructions
│   ├── user_manual.md              # Operation instructions
│   ├── troubleshooting.md          # Common issues and solutions
│   └── research_paper.md           # Complete research documentation
├── examples/
│   ├── basic_operation/            # Simple usage examples
│   └── advanced_features/          # Extended functionality demos
└── tests/
    ├── unit_tests/                 # Component testing
    └── integration_tests/          # System-level testing

Research Applications

Immediate Applications
Assistive Technology: Navigation aids for visually impaired individuals
Educational Enhancement: Kinesthetic learning tools for spatial concepts
Memory Augmentation: Tactile anchoring for improved recall performance

Research Opportunities
Empirical validation of embodied cognition principles
Parameter optimization studies for haptic feedback
Individual differences in spatial memory formation
Therapeutic applications for cognitive rehabilitation

Getting Started with Development

Fork the repository
Create a feature branch (git checkout -b feature/amazing-feature)
Commit your changes (git commit -m 'Add amazing feature')
Push to the branch (git push origin feature/amazing-feature)
Open a Pull Request


Documentation

Assembly Guide: Complete build instructions
User Manual: Operation and usage guidelines
Troubleshooting: Common issues and solutions
Research Paper: Complete technical documentation

Contact
Corey Dziadzio
📧 Email: coreydziadzio@c11visualarts.com
🌐 GitHub: @CJD-11
🔗 Project Link: https://github.com/CJD-11/Arduino-Haptic-Glove


Project Status
✅ Hardware Prototype: Complete and functional
✅ Software Implementation: Core functionality operational
✅ Documentation: Comprehensive research and technical docs
🔄 User Testing: In progress with target populations
📋 Future Work: Wireless integration and miniaturization
