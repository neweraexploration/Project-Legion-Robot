# **Project Legion: A Comprehensive Technical Documentation for a 12-DOF Bio-Mimetic Hybrid Robot**

## **Table of Contents**
1. [Introduction](#1-introduction)
2. [Concepts and Design Philosophy](#2-concepts-and-design-philosophy)
3. [Hardware Architecture](#3-hardware-architecture)
4. [Software Architecture](#4-software-architecture)
5. [Circuit Design and Electronics](#5-circuit-design-and-electronics)
6. [3D Design and Mechanical Structure](#6-3d-design-and-mechanical-structure)
7. [Code Implementation](#7-code-implementation)
8. [Algorithms and Kinematics](#8-algorithms-and-kinematics)
9. [Control Systems](#9-control-systems)
10. [Future Enhancements](#10-future-enhancements)

---

## **1. Introduction**

Project Legion represents a sophisticated 12-DOF (Degrees of Freedom) bio-mimetic hybrid quadruped robot that combines the precision of legged locomotion with the efficiency of wheeled mobility. This project demonstrates advanced robotics principles including inverse kinematics, gait generation, and multi-platform control systems.

### **1.1 Project Overview**

The robot features 12 servo motors (3 per leg) enabling complex walking patterns, with an optional wheeled locomotion system for hybrid operation. The project supports multiple control platforms including Arduino, ESP32, and Raspberry Pi, making it adaptable to various applications from basic robotics education to advanced AI/ML research.

### **1.2 Key Features**

- **12-DOF Quadruped Design**: Three degrees of freedom per leg for complex motion
- **Hybrid Locomotion**: Seamless switching between walking and wheeled modes
- **Multiple Control Interfaces**: Serial, RC receiver, Web-based, and Python API
- **Real-time Vision**: ESP32-CAM integration for live video streaming
- **Inverse Kinematics**: Smooth, natural movement patterns
- **Modular Codebase**: Support for multiple hardware configurations

---

## **2. Concepts and Design Philosophy**

### **2.1 Bio-Mimetic Design**

The robot's design is inspired by quadruped animals, particularly dogs and cats, which exhibit efficient and stable locomotion patterns:

- **Four-Legged Stability**: Provides a stable base with multiple support points
- **Alternating Leg Pairs**: Diagonal leg pairs move together (trot gait) for balance
- **Dynamic Balance**: Body shifts maintain center of mass over support polygon
- **Natural Gait Patterns**: Mimics biological movement for smooth operation

### **2.2 Hybrid Locomotion**

The hybrid design combines two locomotion methods:

- **Walking Mode**: Precise navigation over obstacles and uneven terrain
- **Drive Mode**: Efficient rolling on flat surfaces using wheels
- **Mode Switching**: Seamless transition between modes based on terrain

### **2.3 Degrees of Freedom Distribution**

Each leg possesses 3 DOF:
- **Alpha (α)**: Shoulder rotation (yaw) - controls leg direction
- **Beta (β)**: Shoulder pitch - controls leg lift/lower
- **Gamma (γ)**: Knee pitch - controls leg extension/retraction

**Total System DOF**: 4 legs × 3 DOF = 12 DOF

### **2.4 Coordinate System**

The robot uses a Cartesian coordinate system:
- **Origin**: Center of robot body
- **X-axis**: Forward/backward direction
- **Y-axis**: Left/right direction  
- **Z-axis**: Up/down direction (negative values indicate below body)

---

## **3. Hardware Architecture**

### **3.1 Core Components**

#### **3.1.1 Microcontrollers**

**Arduino Nano**
- **Processor**: ATmega328P
- **Clock Speed**: 16 MHz
- **Digital I/O**: 14 pins
- **Analog Inputs**: 6 pins
- **Memory**: 32KB Flash, 2KB SRAM
- **Use Case**: Basic control, servo management, RC receiver interface

**ESP32-CAM**
- **Processor**: Dual-core 240 MHz Tensilica LX6
- **WiFi**: 802.11 b/g/n
- **Bluetooth**: BLE 4.2
- **Camera**: OV2640 sensor
- **Memory**: 520KB SRAM, external PSRAM support
- **Use Case**: Wireless control, live video streaming, web server

**Raspberry Pi 4**
- **Processor**: Quad-core ARM Cortex-A72 @ 1.5GHz
- **RAM**: 4GB/8GB options
- **GPIO**: 40-pin header
- **Connectivity**: WiFi, Bluetooth, Ethernet, USB 3.0
- **Use Case**: AI/ML processing, advanced control algorithms, web server with Flask

#### **3.1.2 Actuators**

**Servo Motors (12x)**
- **Type**: Standard servo motors (e.g., SG90, MG996R)
- **Operating Range**: 0-180 degrees
- **Torque**: 1.8-10 kg/cm (depending on model)
- **Control Signal**: PWM (50Hz, 1-2ms pulse width)
- **Power**: 4.8V-6V, ~100-200mA per servo under load

**DC Motors (Mark 2)**
- **Type**: N20 gear motors
- **Voltage**: 3-6V DC
- **Speed**: Variable via PWM control
- **Control**: H-bridge motor driver
- **Use**: Wheeled locomotion in drive mode

#### **3.1.3 Servo Driver**

**PCA9685 16-Channel PWM Driver**
- **Interface**: I2C (SDA/SCL)
- **Channels**: 16 independent PWM outputs
- **Resolution**: 12-bit (4096 steps)
- **Frequency**: 50-1000 Hz (typically 50-60 Hz for servos)
- **Address**: 0x40 (default, configurable)
- **Benefits**: 
  - Reduces pin usage on microcontroller
  - Centralized servo control
  - Precise timing synchronization
  - Expandable to 16 servos

#### **3.1.4 Power System**

**Power Requirements:**
- **Main Power**: 5V for servos and microcontroller
- **Current Draw**: ~2-3A total (all servos active)
- **Battery**: LiPo 7.4V (2S) or 11.1V (3S) with voltage regulator
- **Mark 2**: Separate power system for DC motors

**Power Distribution:**
- Voltage regulator: LM2596 or similar (5V/3A)
- Servo power: Direct from regulator
- Logic power: 3.3V/5V (depending on MCU)

#### **3.1.5 Sensors and Communication**

**RC Receiver (Optional)**
- **Type**: 6-channel PWM receiver
- **Channels**: Connected to A1-A6 analog pins
- **Pulse Width**: 1000-2000μs (1-2ms)
- **Use Case**: Remote control via RC transmitter

**ESP32-CAM**
- **Camera Sensor**: OV2640
- **Resolution**: VGA (640x480) or SVGA (800x600)
- **Streaming**: MJPEG over HTTP
- **Frame Rate**: Up to 30 FPS

### **3.2 Mechanical Structure**

#### **3.2.1 Leg Dimensions**

```
length_a = 55mm    (Upper leg/femur)
length_b = 77.5mm  (Lower leg/tibia)
length_c = 27.5mm  (Shoulder offset)
length_side = 71mm (Mark 1) / 113mm (Mark 2) (Body width)
```

#### **3.2.2 3D Printed Components**

**Mark 1 Components:**
- Body top and bottom
- Forward/backward shoulders
- Forward/backward legs
- Femur segments
- Hinges (optional version)

**Mark 2 Components:**
- Enhanced body design
- Legs with N20 motor holders
- Improved hinge system
- Optimized weight distribution

**Printing Specifications:**
- **Material**: PLA or PETG recommended
- **Layer Height**: 0.2-0.3mm
- **Infill**: 20-30%
- **Supports**: Minimal (depends on design)

---

## **4. Software Architecture**

### **4.1 Code Organization**

```
Project-Legion-Robot/
├── Codes/
│   ├── ARDUINO/
│   │   ├── Arduino Without Servo Driver/
│   │   ├── Arduino With Servo Driver/
│   │   └── Arduino With ESPCam/
│   ├── ESP32/
│   ├── Legion Mark 2/
│   └── RPI/
├── 3D Files/
├── Circuit Diagrams/
└── Documents/
```

### **4.2 Programming Languages**

- **C++**: Arduino/ESP32 firmware
- **Python**: Raspberry Pi control and AI/ML
- **HTML/JavaScript**: Web interfaces

### **4.3 Key Libraries**

**Arduino Libraries:**
- `Servo.h` - Servo motor control
- `FlexiTimer2.h` - Timer interrupt management
- `Wire.h` - I2C communication
- `Adafruit_PWMServoDriver.h` - PCA9685 driver

**Python Libraries:**
- `adafruit_servokit` - Servo control via PCA9685
- `flask` - Web server framework
- `threading` - Concurrent execution
- `math` - Mathematical calculations for kinematics

**ESP32 Libraries:**
- `esp_camera.h` - Camera interface
- `WiFi.h` - WiFi connectivity
- `esp_http_server.h` - HTTP server

---

## **5. Circuit Design and Electronics**

### **5.1 Basic Arduino Configuration (Without Servo Driver)**

**Servo Pin Mapping:**
```cpp
const int servo_pin[4][3] = {
  {2, 3, 4},    // Leg 0: Alpha, Beta, Gamma
  {5, 6, 7},    // Leg 1: Alpha, Beta, Gamma
  {8, 9, 10},   // Leg 2: Alpha, Beta, Gamma
  {11, 12, 13}  // Leg 3: Alpha, Beta, Gamma
};
```

**Power Distribution:**
- VCC: 5V rail (regulated)
- GND: Common ground
- Servo power: Separate 5V supply (2-3A capacity)

### **5.2 PCA9685 Servo Driver Configuration**

**I2C Connections:**
```
Arduino Nano:
- SDA → A4
- SCL → A5
- VCC → 5V
- GND → GND
```

**Servo Channel Mapping:**
```cpp
const int servo_channels[4][3] = {
  {0, 1, 2},      // Leg 0
  {4, 5, 6},      // Leg 1
  {8, 9, 10},     // Leg 2
  {12, 13, 14}    // Leg 3
};
```

**PWM Configuration:**
- Frequency: 60 Hz
- Pulse range: 150-600 (0.5ms-2ms equivalent)
- Resolution: 12-bit (4096 steps)

### **5.3 Mark 2 Motor Driver Circuit**

**DC Motor Control:**
```cpp
// Motor A (Left)
motorPinA1 = 2  // Direction control 1
motorPinA2 = 3  // Direction control 2
enA = 6         // PWM speed control

// Motor B (Right)
motorPinB1 = 4  // Direction control 1
motorPinB2 = 5  // Direction control 2
enB = 9         // PWM speed control
```

**H-Bridge Logic:**
- **Forward**: A1=HIGH, A2=LOW / B1=LOW, B2=HIGH
- **Backward**: A1=LOW, A2=HIGH / B1=HIGH, B2=LOW
- **Left**: A1=HIGH, A2=LOW / B1=HIGH, B2=LOW
- **Right**: A1=LOW, A2=HIGH / B1=LOW, B2=HIGH
- **Stop**: All LOW

### **5.4 ESP32-CAM Circuit**

**Camera Pin Configuration:**
```cpp
// AI-Thinker ESP32-CAM
PWDN_GPIO_NUM = 32
XCLK_GPIO_NUM = 0
SIOD_GPIO_NUM = 26
SIOC_GPIO_NUM = 27
Y9_GPIO_NUM = 35
Y8_GPIO_NUM = 34
Y7_GPIO_NUM = 39
Y6_GPIO_NUM = 36
Y5_GPIO_NUM = 21
Y4_GPIO_NUM = 19
Y3_GPIO_NUM = 18
Y2_GPIO_NUM = 5
VSYNC_GPIO_NUM = 25
HREF_GPIO_NUM = 23
PCLK_GPIO_NUM = 22
```

**Serial Communication:**
- ESP32-CAM TX → Arduino RX
- ESP32-CAM RX → Arduino TX
- GND → Common ground

### **5.5 Power Management**

**Voltage Regulation:**
- Input: 7.4V-11.1V LiPo battery
- Regulator: 5V/3A (LM2596 or similar)
- Servo power: Direct from regulator
- Logic power: 3.3V/5V (depending on MCU)

**Current Requirements:**
- Arduino: ~50mA
- Servos (idle): ~10mA each
- Servos (active): ~100-200mA each
- Total peak: ~2-3A

---

## **6. 3D Design and Mechanical Structure**

### **6.1 Design Philosophy**

The mechanical design emphasizes:
- **Modularity**: Interchangeable parts for easy maintenance
- **Weight Optimization**: Minimal material usage while maintaining strength
- **Strength**: Critical joint reinforcement
- **Aesthetics**: Clean, functional design

### **6.2 Component Breakdown**

#### **6.2.1 Body Structure**

**Body Top:**
- Houses electronics and control board
- Mounting points for servos
- Cable management channels
- Ventilation holes for heat dissipation

**Body Bottom:**
- Battery compartment
- Weight distribution optimization
- Mounting points for legs

#### **6.2.2 Leg Assembly**

**Shoulder Joint:**
- Forward/backward variants for different leg orientations
- Servo mounting with proper clearance
- Range of motion: ±90 degrees

**Femur (Upper Leg):**
- Length: 55mm
- Lightweight design
- Servo attachment points

**Tibia (Lower Leg):**
- Length: 77.5mm
- End effector attachment
- Optional wheel mounting (Mark 2)

#### **6.2.3 Hinge System**

**Purpose:**
- Allows leg folding for compact storage
- Reduces collision risk during movement
- Improves stability

**Design:**
- Pivot point at shoulder
- Limited rotation range
- Spring-loaded (optional)

### **6.3 Assembly Process**

1. **3D Print** all structural components
2. **Attach servos** to structure (without servo horns)
3. **Calibrate servos** to 90° using `Servo_Config` code
4. **Attach servo horns** maintaining initial position (see `Initial Position.jpeg`)
5. **Complete circuit** connections
6. **Upload main code** to microcontroller
7. **Test and fine-tune** parameters

### **6.4 Initial Position**

**Critical Setup:**
- All servos at 90° (neutral position)
- Legs positioned according to `Initial Position.jpeg`
- Body level and balanced
- All joints properly lubricated

---

## **7. Code Implementation**

### **7.1 Core Classes and Functions**

#### **7.1.1 Legion Class (C++)**

**Header Structure:**
```cpp
class Legion {
public:
  // Basic movements
  void stand();
  void sit();
  void step_forward(unsigned int step);
  void step_back(unsigned int step);
  void turn_left(unsigned int step);
  void turn_right(unsigned int step);
  
  // Body movements
  void body_left(int i);
  void body_right(int i);
  void head_up(int i);
  void head_down(int i);
  
  // Gestures
  void hand_wave(int i);
  void hand_shake(int i);
  void body_dance(int i);
  
  // Wheel control (Mark 2)
  void MF();  // Move Forward
  void MB();  // Move Backward
  void ML();  // Move Left
  void MR();  // Move Right
  void MS();  // Move Stop
  void drivemode();
  
  // Kinematics
  void set_site(int leg, float x, float y, float z);
  void cartesian_to_polar(float &alpha, float &beta, float &gamma, 
                          float x, float y, float z);
  void polar_to_servo(int leg, float alpha, float beta, float gamma);
  
  // Utility
  void wait_all_reach();
  void wait_reach(int leg);
};
```

#### **7.1.2 Configuration Constants**

```cpp
// Robot dimensions
const float length_a = 55.0;      // Upper leg
const float length_b = 77.5;      // Lower leg
const float length_c = 27.5;      // Shoulder offset
const float length_side = 71.0;   // Body width

// Movement parameters
const float z_default = -50.0;    // Standing height
const float z_up = -30.0;         // Leg lift height
const float z_boot = -28.0;       // Sitting height
const float x_default = 62.0;     // Default X position
const float y_start = 0.0;         // Y start position
const float y_step = 40.0;        // Step distance

// Speed parameters
const float spot_turn_speed = 4.0;
const float leg_move_speed = 8.0;
const float body_move_speed = 3.0;
const float stand_seat_speed = 1.0;
```

### **7.2 Key Algorithms**

#### **7.2.1 Servo Service (Timer Interrupt)**

**Function:** Updates servo positions at 50Hz (20ms intervals)

```cpp
void servo_service(void) {
  sei();  // Enable interrupts
  static float alpha, beta, gamma;
  
  for (int i = 0; i < 4; i++) {
    // Update current position towards expected
    for (int j = 0; j < 3; j++) {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }
    
    // Convert to servo angles
    cartesian_to_polar(alpha, beta, gamma, 
                       site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }
  
  rest_counter++;
}
```

#### **7.2.2 Forward Kinematics**

**Cartesian to Polar Conversion:**

```cpp
void cartesian_to_polar(float &alpha, float &beta, float &gamma,
                        float x, float y, float z) {
  // Calculate w-z plane
  float v, w;
  w = (x >= 0 ? 1 : -1) * sqrt(pow(x, 2) + pow(y, 2));
  v = w - length_c;
  
  // Calculate alpha (shoulder pitch)
  alpha = atan2(z, v) + 
          acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) /
               (2 * length_a * sqrt(pow(v, 2) + pow(z, 2))));
  
  // Calculate beta (knee angle)
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) /
              (2 * length_a * length_b));
  
  // Calculate gamma (shoulder yaw)
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  
  // Convert to degrees
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}
```

#### **7.2.3 Servo Angle Mapping**

**Leg-Specific Transformations:**

```cpp
void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  if (leg == 0) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else if (leg == 1) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 2) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 3) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }
  
  // Set servo angles
  servo[leg][0].write(alpha);  // Alpha servo
  servo[leg][1].write(beta);   // Beta servo
  servo[leg][2].write(gamma);  // Gamma servo
}
```

### **7.3 Gait Patterns**

#### **7.3.1 Forward Walking**

**Trot Gait (Alternating Diagonal Pairs):**

```cpp
void step_forward(unsigned int step) {
  move_speed = leg_move_speed;
  while (step-- > 0) {
    if (site_now[2][1] == y_start) {
      // Legs 2 & 1 move
      // 1. Lift leg 2
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      
      // 2. Move leg 2 forward
      set_site(2, x_default + x_offset, y_start + 2*y_step, z_up);
      wait_all_reach();
      
      // 3. Lower leg 2
      set_site(2, x_default + x_offset, y_start + 2*y_step, z_default);
      wait_all_reach();
      
      // 4. Shift body forward
      move_speed = body_move_speed;
      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2*y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      
      // 5. Lift and move leg 1
      move_speed = leg_move_speed;
      set_site(1, x_default + x_offset, y_start + 2*y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      // Legs 0 & 3 move (similar pattern)
      // ... (mirror of above)
    }
  }
}
```

#### **7.3.2 Turning**

**Spot Turn Algorithm:**

```cpp
void turn_left(unsigned int step) {
  move_speed = spot_turn_speed;
  while (step-- > 0) {
    // Lift one leg, rotate body, place leg
    // Alternate between leg pairs
    // Uses turn_x0, turn_y0, turn_x1, turn_y1 coordinates
  }
}
```

**Turn Coordinate Calculation:**

```cpp
// Pre-calculated turn positions
const float temp_a = sqrt(pow(2*x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2*(y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2*x_default + length_side, 2) + 
                          pow(2*y_start + y_step + length_side, 2));
const float temp_alpha = acos((temp_a*temp_a + temp_b*temp_b - temp_c*temp_c) /
                              (2*temp_a*temp_b));

const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;
```

### **7.4 Control Interfaces**

#### **7.4.1 Serial Control**

**Command Dictionary:**
```cpp
#define FORWARD 'f'
#define BACKWARD 'b'
#define LEFT 'l'
#define RIGHT 'r'
#define STAND 's'
#define SIT 'i'
#define HELLO 'h'
#define HANDSHAKE 'm'
#define DANCE 'd'
#define HEADUP 'c'
#define HEADDOWN 'z'
#define BODYRIGHT 'e'
#define BODYLEFT 'p'
```

**Command Processing:**
```cpp
void serialEvent() {
  char tmp = -1;
  boolean taken = false;
  
  while (Serial.available()) {
    state = !state;
    tmp = Serial.read();
    taken = gaits(tmp);
    
    if (taken)
      cmd = tmp;
  }
}
```

#### **7.4.2 RC Control**

**Pulse Width Measurement:**
```cpp
void RC_Control() {
  pulseWidths[0] = pulseIn(channelPins[0], HIGH, TIME_INTERVAL);
  pulseWidths[1] = pulseIn(channelPins[1], HIGH, TIME_INTERVAL);
  // ... more channels
  
  // Convert to command
  channelValues[0] = (pulseWidths[0] > 1500) ? 1 : 
                     (pulseWidths[0] < 1100) ? -1 : 0;
  
  // Map to movement commands
  if (channelValues[0] != 0) {
    if (channelValues[0] == -1) tmp2 = LEFT;
    else if (channelValues[0] == 1) tmp2 = RIGHT;
  }
  // ... similar for other channels
}
```

#### **7.4.3 Web Interface (ESP32-CAM)**

**HTTP Server Setup:**
```cpp
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  
  // Register URI handlers
  httpd_register_uri_handler(camera_httpd, &index_uri);
  httpd_register_uri_handler(camera_httpd, &cmd_uri);
  httpd_register_uri_handler(stream_httpd, &stream_uri);
}
```

**Command Handler:**
```cpp
static esp_err_t cmd_handler(httpd_req_t *req) {
  // Parse URL parameters
  // Extract action and value
  // Send command via Serial to Arduino
  Serial.println(value);  // Forward to main controller
}
```

#### **7.4.4 Python API (Raspberry Pi)**

**Flask Web Server:**
```python
@app.route('/command', methods=['POST'])
def handle_command():
    data = request.json
    command = data.get('action', '')
    
    if command == "forward":
        robot.step_forward(1)
    elif command == "backward":
        robot.step_back(1)
    # ... more commands
    
    return jsonify({"status": "success"}), 200
```

**Kinematics Class:**
```python
class RobotKinematics:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.setup_constants()
        self.setup_servos()
        threading.Thread(target=self.servo_service, daemon=True).start()
    
    def servo_service(self):
        while True:
            for leg in range(4):
                # Update positions
                # Convert to angles
                # Set servo positions
            time.sleep(0.02)  # 50Hz
```

---

## **8. Algorithms and Kinematics**

### **8.1 Inverse Kinematics**

**Problem:** Given desired foot position (x, y, z), calculate servo angles (α, β, γ)

**Solution:** Geometric approach using trigonometry

**Mathematical Model:**

1. **Project to W-Z Plane:**
   ```
   w = sign(x) * √(x² + y²)
   v = w - length_c
   ```

2. **Calculate Alpha (Shoulder Pitch):**
   ```
   α = atan2(z, v) + acos((a² - b² + v² + z²) / (2a√(v² + z²)))
   ```

3. **Calculate Beta (Knee Angle):**
   ```
   β = acos((a² + b² - v² - z²) / (2ab))
   ```

4. **Calculate Gamma (Shoulder Yaw):**
   ```
   γ = atan2(y, x)  [if w ≥ 0]
   γ = atan2(-y, -x) [if w < 0]
   ```

**Geometric Interpretation:**
- Forms two triangles: shoulder-to-foot and shoulder-knee-foot
- Uses law of cosines for angle calculation
- Accounts for shoulder offset (length_c)

### **8.2 Forward Kinematics**

**Problem:** Given servo angles, calculate foot position

**Not directly implemented** (not needed for control), but can be derived:

```
x = (length_a*cos(α) + length_b*cos(α+β)) * cos(γ) + length_c*cos(γ)
y = (length_a*cos(α) + length_b*cos(α+β)) * sin(γ) + length_c*sin(γ)
z = length_a*sin(α) + length_b*sin(α+β)
```

### **8.3 Trajectory Planning**

**Linear Interpolation:**

```cpp
void set_site(int leg, float x, float y, float z) {
  // Calculate distance
  float length_x = (x != KEEP) ? x - site_now[leg][0] : 0;
  float length_y = (y != KEEP) ? y - site_now[leg][1] : 0;
  float length_z = (z != KEEP) ? z - site_now[leg][2] : 0;
  
  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));
  
  // Calculate speed vector
  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;
  
  // Set target
  if (x != KEEP) site_expect[leg][0] = x;
  if (y != KEEP) site_expect[leg][1] = y;
  if (z != KEEP) site_expect[leg][2] = z;
}
```

**Benefits:**
- Smooth motion
- Speed control
- Non-blocking execution
- Straight-line paths

### **8.4 Gait Generation**

#### **8.4.1 Trot Gait**

**Characteristics:**
- Diagonal leg pairs move together
- 50% duty cycle
- Stable and efficient

**Phase Sequence:**
1. Lift diagonal pair (e.g., legs 2 & 1)
2. Move forward
3. Lower to ground
4. Shift body forward
5. Repeat with opposite pair

#### **8.4.2 Walk Gait**

**Characteristics:**
- One leg at a time
- 75% duty cycle
- Maximum stability

**Implementation:** Similar to trot but sequential leg movement

#### **8.4.3 Turn Gait**

**Spot Turn:**
- Lift one leg
- Rotate body around support triangle
- Place leg in new position
- Repeat with other legs

**Radius Turn:**
- Outer legs move faster
- Inner legs move slower
- Creates curved path

### **8.5 Balance and Stability**

**Center of Mass Management:**
- Body shifts during leg lift
- Maintains support polygon
- Prevents tipping

**Support Polygon:**
- Triangle formed by three supporting legs
- Center of mass must remain within polygon
- Dynamic adjustment during movement

---

## **9. Control Systems**

### **9.1 Servo Control Loop**

**Architecture:**
```
Timer Interrupt (50Hz)
    ↓
Servo Service Function
    ↓
Update site_now towards site_expect
    ↓
Cartesian to Polar Conversion
    ↓
Servo Angle Mapping
    ↓
PWM Output
```

**Timing:**
- Interrupt frequency: 50Hz (20ms)
- Servo update: Every interrupt
- Smooth motion: Continuous interpolation

### **9.2 Multi-Platform Control**

#### **9.2.1 Arduino Standalone**

**Features:**
- Direct servo control
- Serial command interface
- RC receiver support
- Real-time execution

**Limitations:**
- Limited processing power
- No vision capabilities
- Basic control only

#### **9.2.2 ESP32-CAM Integration**

**Architecture:**
```
ESP32-CAM (Vision + Web Server)
    ↓ Serial Communication
Arduino Nano (Motion Control)
    ↓ PWM Signals
Servos
```

**Features:**
- Live video streaming
- Web-based control interface
- Wireless operation
- Remote monitoring

#### **9.2.3 Raspberry Pi Integration**

**Architecture:**
```
Raspberry Pi (AI/ML Processing)
    ↓ I2C/GPIO
PCA9685 Servo Driver
    ↓ PWM Signals
Servos
```

**Features:**
- Advanced AI/ML capabilities
- Object detection
- Path planning
- Web server with Flask
- Python API

### **9.3 Hybrid Locomotion Control**

**Mode Switching:**

```cpp
void drivemode() {
  // Position legs for wheel contact
  set_site(0, x_default, y_start, z_default);
  set_site(1, x_default, y_start, z_default);
  set_site(2, x_default, y_start, z_default);
  set_site(3, x_default, y_start, z_default);
  wait_all_reach();
  
  // Enable wheel motors
  // Disable leg movement
}
```

**Wheel Control:**

```cpp
void MF() {  // Move Forward with wheels
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinB2, HIGH);
}
```

### **9.4 Parameter Tuning**

**Adjustable Parameters:**

```cpp
// Height adjustment
void adjustheight(bool flag) {
  if (flag) {
    z_default -= 5;
    x_default -= 3;
    y_step += 1;
  } else {
    z_default += 5;
    x_default += 3;
    y_step -= 1;
  }
  // Update all legs
}

// Speed adjustment
void adjust(char axis, int val) {
  switch(axis) {
    case 'x': x_default = val; break;
    case 'y': y_default = val; break;
    case 'z': z_default = val; break;
    case 'l': leg_move_speed = val; break;
  }
}
```

---

## **10. Future Enhancements**

### **10.1 Planned Features**

1. **IMU Integration:**
   - Gyroscope for balance control
   - Accelerometer for orientation sensing
   - Magnetometer for heading

2. **Force Sensing:**
   - Pressure sensors on feet
   - Adaptive gait based on terrain
   - Load distribution optimization

3. **Advanced AI:**
   - Reinforcement learning for gait optimization
   - Object recognition and avoidance
   - Autonomous navigation

4. **Enhanced Vision:**
   - Depth sensing (stereo vision)
   - SLAM (Simultaneous Localization and Mapping)
   - Gesture recognition

5. **Communication:**
   - ROS (Robot Operating System) integration
   - MQTT for IoT connectivity
   - Cloud-based control

### **10.2 Performance Optimizations**

1. **Code Optimization:**
   - Reduce computational overhead
   - Optimize servo update frequency
   - Improve trajectory planning

2. **Hardware Upgrades:**
   - Higher torque servos
   - Better power management
   - Lighter materials

3. **Mechanical Improvements:**
   - Better joint design
   - Reduced backlash
   - Improved weight distribution

---

## **Conclusion**

Project Legion demonstrates a complete 12-DOF bio-mimetic hybrid robot system, combining mechanical design, electronics, software, and control algorithms into a functional platform suitable for research, education, and development.

The modular architecture supports multiple platforms and control methods, making it adaptable to various applications. The hybrid locomotion system provides both precision (walking) and efficiency (wheels), making it versatile for different environments.

With ongoing development and community contributions, Project Legion continues to evolve, incorporating new features and improvements to push the boundaries of quadruped robotics.

---

## **References and Credits**

- **Original Logic:** Sunfounder Robot (panerqiang@sunfounder.com)
- **Project Repository:** GitHub - Project-Legion-Robot
- **Video Demonstration:** [Legion MARK 2 YouTube Video](https://www.youtube.com/watch?v=k1hns6NKTCY)

---

**Document Version:** 1.0  
**Last Updated:** 2024  
**Author:** Project Legion Development Team

---

*This technical article provides a comprehensive overview of Project Legion. For specific implementation details, refer to the source code and documentation in the project repository.*

