// MIT Haptic Glove Final Version with Enhanced Interactive Feedback
#include <Wire.h>
#include <MPU6050.h>
#include <TCA9548A.h>
#include <EEPROM.h>

#define MAX_IMUS 8
#define MAX_WALLS 5
#define MAPPING_DURATION 3000
#define FORCE_THRESHOLD 150
#define GYRO_TOLERANCE 300.0
#define REPLAY_TIMEOUT 60000

const int hapticPins[3] = {51, 44, 46};
const int fsrPins[3] = {A3, A6, A7};

TCA9548A tca;
MPU6050 imus[MAX_IMUS];
uint8_t activeChannels[MAX_IMUS];
uint8_t imuAddresses[MAX_IMUS];
bool imuInitialized[MAX_IMUS] = {false};
unsigned long imuLastActive[MAX_IMUS] = {0};
int imuCount = 0;

int currentFSR[3] = {0};
float currentGyro[3] = {0};

String mode = "IDLE";
String previousMode = "IDLE";
bool hapticsActive = false;
int mappedObjectCount = 0;
unsigned long lastReplayMatch = 0;
unsigned long modeStartTime = 0;
unsigned long lastStatusUpdate = 0;
bool waitingForForce = false;

struct ObjectPose {
  float gyro[3];
  bool valid;
};
ObjectPose objects[MAX_WALLS];

struct ChannelMapEntry {
  uint8_t channel;
  uint8_t address;
};
ChannelMapEntry channelMap[3];

float poseDistance(float a[], float b[]) {
  float d = 0;
  for (int i = 0; i < 3; i++) d += sq(a[i] - b[i]);
  return sqrt(d);
}

int smoothFSR(int raw, int prev, float alpha = 0.2) {
  return (int)((1.0 - alpha) * prev + alpha * raw);
}

void printDivider() {
  Serial.println("========================================");
}

void printHeader(String title) {
  printDivider();
  Serial.println("  " + title);
  printDivider();
}

void printModeChange(String newMode) {
  printHeader("MODE CHANGE: " + newMode);
  
  if (newMode == "MAP") {
    Serial.println("OBJECT MAPPING MODE ACTIVATED");
    Serial.println("- Ready to map object #" + String(mappedObjectCount + 1) + " of " + String(MAX_WALLS));
    Serial.println("- Touch any object surface with force to begin mapping");
    Serial.println("- Force threshold: " + String(FORCE_THRESHOLD) + " units");
    Serial.println("- Mapping duration: " + String(MAPPING_DURATION/1000.0) + " seconds");
    Serial.println("");
    Serial.println("Instructions:");
    Serial.println("1. Position your fingers against any object surface");
    Serial.println("2. Apply steady pressure with your fingertips");
    Serial.println("3. Hold finger position during mapping countdown");
    Serial.println("4. IMU sensors will record thumb, index, and middle finger orientations");
    waitingForForce = true;
  }
  else if (newMode == "REPLAY") {
    Serial.println("REPLAY MODE ACTIVATED");
    Serial.println("- Searching for matching object poses...");
    Serial.println("- " + String(mappedObjectCount) + " objects available for matching");
    Serial.println("- Gyro tolerance: " + String(GYRO_TOLERANCE) + " degrees (VERY GENEROUS)");
    Serial.println("- Auto-exit timeout: " + String(REPLAY_TIMEOUT/1000) + " seconds");
    Serial.println("");
    Serial.println("Move your fingers to different positions to feel mapped objects");
    Serial.println("Haptics will trigger based on finger orientations (no force needed)");
    Serial.println("🔥 HAPTICS SHOULD TRIGGER EASILY - LARGE DETECTION ZONE!");
  }
  else if (newMode == "CALIBRATE") {
    Serial.println("CALIBRATION MODE ACTIVATED");
    Serial.println("- Real-time sensor monitoring enabled");
    Serial.println("- All sensor values displayed continuously");
    Serial.println("- Use this mode to test sensor functionality");
    Serial.println("");
    Serial.println("Sensor Layout:");
    Serial.println("- IMU 0: Thumb sensor (back of thumb)");
    Serial.println("- IMU 1: Index finger sensor (back of index finger)");  
    Serial.println("- IMU 2: Middle finger sensor (back of middle finger)");
    Serial.println("- Gyro[0]: X-axis rotation (roll)");
    Serial.println("- Gyro[1]: Y-axis rotation (pitch)");  
    Serial.println("- Gyro[2]: Z-axis rotation (yaw)");
    Serial.println("- FSR[0]: Thumb pressure sensor");
    Serial.println("- FSR[1]: Index finger pressure sensor");
    Serial.println("- FSR[2]: Middle finger pressure sensor");
    Serial.println("");
    Serial.println("Expected ranges:");
    Serial.println("- Gyro: -32768 to +32767 (raw values)");
    Serial.println("- FSR: 0-1023 (higher = more pressure)");
  }
  else if (newMode == "IDLE") {
    Serial.println("IDLE MODE - System Ready");
    Serial.println("Available commands: MAP | REPLAY | CALIBRATE | STATUS | RESCAN");
  }
  
  printDivider();
}

void scanIMUs() {
  printHeader("SCANNING FOR IMU SENSORS");
  Serial.println("Initializing I2C multiplexer...");
  
  imuCount = 0;
  uint8_t addresses[] = {0x68, 0x69};

  for (int i = 0; i < 3; i++) {
    channelMap[i].channel = i;
    channelMap[i].address = 0;
  }

  Serial.println("Scanning channels 0-2 for IMU devices...");
  
  for (uint8_t ch = 0; ch <= 2; ch++) {
    Serial.print("Channel " + String(ch) + ": ");
    tca.closeAll(); delay(5);
    tca.openChannel(ch); delay(5);

    bool found = false;
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t addr = addresses[i];
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        channelMap[ch].address = addr;
        Serial.println("IMU found at 0x" + String(addr, HEX));
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.println("No IMU detected");
    }
  }

  EEPROM.put(100, channelMap);
  Serial.println("\nInitializing detected IMUs...");

  for (uint8_t ch = 0; ch <= 2; ch++) {
    if (channelMap[ch].address == 0) continue;

    uint8_t addr = channelMap[ch].address;
    MPU6050 temp(addr);

    tca.closeAll(); delay(5);
    tca.openChannel(ch); delay(5);
    temp.initialize(); delay(5);

    if (temp.testConnection()) {
      imus[imuCount] = temp;
      activeChannels[imuCount] = ch;
      imuAddresses[imuCount] = addr;
      imuInitialized[imuCount] = true;
      imuLastActive[imuCount] = millis();
      Serial.println("✓ IMU #" + String(imuCount) + " initialized (Ch:" + String(ch) + ", Addr:0x" + String(addr, HEX) + ")");
      imuCount++;
    } else {
      Serial.println("✗ IMU initialization failed on channel " + String(ch));
    }
  }
  
  Serial.println("\nScan complete: " + String(imuCount) + " IMU(s) active");
  printDivider();
}

bool readGyro(float data[]) {
  for (int i = 0; i < imuCount; i++) {
    if (!imuInitialized[i]) continue;
    tca.closeAll(); tca.openChannel(activeChannels[i]);

    if (imus[i].testConnection()) {
      data[0] = imus[i].getRotationX();
      data[1] = imus[i].getRotationY();
      data[2] = imus[i].getRotationZ();
      imuLastActive[i] = millis();
      return true;
    } else if (millis() - imuLastActive[i] > 5000) {
      Serial.println("⚠ IMU #" + String(i) + " disconnected after 5s timeout");
      imuInitialized[i] = false;
    }
  }
  return false;
}

void readFSRs(int data[]) {
  for (int i = 0; i < 3; i++) data[i] = analogRead(fsrPins[i]);
}

void replayHaptics(float matchScore) {
  // ALWAYS maximum intensity - no scaling
  Serial.println("🔥 MAXIMUM HAPTIC FEEDBACK ACTIVATED! (score: " + String(matchScore, 1) + "°)");

  // Always use maximum PWM regardless of distance
  int finalPWM = 255; // ALWAYS maximum intensity
  
  // Apply maximum PWM to all three motors
  for (int i = 0; i < 3; i++) {
    analogWrite(hapticPins[i], finalPWM);
  }
  
  Serial.println("  🚀 ALL MOTORS AT MAXIMUM PWM: 255 | Distance: " + String(matchScore, 1) + "° from target");
  hapticsActive = true;
}

void stopHaptics() {
  if (!hapticsActive) return;
  Serial.println("🔇 Haptics stopped - motors OFF");
  for (int i = 0; i < 3; i++) analogWrite(hapticPins[i], 0);
  hapticsActive = false;
}

void averageGyro(float avg[], unsigned long duration) {
  float sum[3] = {0};
  int samples = 0;
  unsigned long start = millis();
  float temp[3];

  Serial.println("📍 OBJECT MAPPING IN PROGRESS...");
  Serial.println("Hold your finger position steady! Recording detailed orientation data...");
  Serial.println("Duration: " + String(duration/1000.0) + " seconds");
  Serial.println("");
  
  int progressBars = 25; // More detailed progress bar
  unsigned long progressInterval = duration / progressBars;
  int lastProgress = 0;
  
  // Show initial finger position
  if (readGyro(temp)) {
    Serial.println("📊 Starting finger position:");
    Serial.println("  X-axis (Curl): " + String(temp[0], 1) + "°");
    Serial.println("  Y-axis (Bend): " + String(temp[1], 1) + "°");
    Serial.println("  Z-axis (Spread): " + String(temp[2], 1) + "°");
    Serial.println("");
  }

  while (millis() - start < duration) {
    if (readGyro(temp)) {
      for (int j = 0; j < 3; j++) sum[j] += temp[j];
      samples++;
      
      // Show live readings every 0.5 seconds
      static unsigned long lastReading = 0;
      if (millis() - lastReading > 500) {
        Serial.println("📈 Live: X=" + String(temp[0], 1) + "° Y=" + String(temp[1], 1) + "° Z=" + String(temp[2], 1) + "° (Sample #" + String(samples) + ")");
        lastReading = millis();
      }
    }
    
    // Show progress bar
    unsigned long elapsed = millis() - start;
    int currentProgress = (elapsed * progressBars) / duration;
    if (currentProgress > lastProgress) {
      Serial.print("\rProgress: [");
      for (int i = 0; i < progressBars; i++) {
        Serial.print(i < currentProgress ? "█" : "░");
      }
      Serial.print("] " + String((elapsed * 100) / duration) + "% (" + String((duration - elapsed)/1000.0, 1) + "s left)");
      lastProgress = currentProgress;
    }
    
    delay(20);
  }
  
  Serial.println("\n\n✓ Mapping complete!");

  if (samples > 0) {
    for (int j = 0; j < 3; j++) avg[j] = sum[j] / samples;
    Serial.println("📊 FINAL RECORDED POSE:");
    Serial.println("  X-axis (Curl): " + String(avg[0], 1) + "°");
    Serial.println("  Y-axis (Bend): " + String(avg[1], 1) + "°");
    Serial.println("  Z-axis (Spread): " + String(avg[2], 1) + "°");
    Serial.println("📈 Total samples collected: " + String(samples));
    Serial.println("📍 This finger pose is now saved as Object #" + String(mappedObjectCount + 1));
  } else {
    Serial.println("⚠ Warning: No gyro samples collected!");
  }
}

String getMovementInstruction(float currentPose[], float targetPose[]) {
  String instruction = "Move your fingers: ";
  float diffX = targetPose[0] - currentPose[0];
  float diffY = targetPose[1] - currentPose[1]; 
  float diffZ = targetPose[2] - currentPose[2];
  
  // Very low threshold for finger movements (super sensitive)
  if (abs(diffX) > 100) {
    if (diffX > 0) {
      instruction += "curl fingers MORE";
    } else {
      instruction += "straighten fingers MORE";
    }
    instruction += " | ";
  }
  
  if (abs(diffY) > 100) {
    if (diffY > 0) {
      instruction += "bend fingers UP";
    } else {
      instruction += "bend fingers DOWN";
    }
    instruction += " | ";
  }
  
  if (abs(diffZ) > 100) {
    if (diffZ > 0) {
      instruction += "spread fingers APART";
    } else {
      instruction += "bring fingers TOGETHER";
    }
    instruction += " | ";
  }
  
  // Remove trailing " | "
  if (instruction.endsWith(" | ")) {
    instruction = instruction.substring(0, instruction.length() - 3);
  }
  
  if (instruction == "Move your fingers: ") {
    instruction = "🎯 PERFECT! Hold this exact finger position!";
  }
  
  return instruction;
}

void printSensorValues() {
  static unsigned long calibrateStartTime = 0;
  static bool calibrateTimerStarted = false;
  
  // Start timer on first call
  if (!calibrateTimerStarted) {
    calibrateStartTime = millis();
    calibrateTimerStarted = true;
  }
  
  // Auto-exit after 30 seconds
  if (millis() - calibrateStartTime > 30000) {
    Serial.println("\n🔄 Calibration auto-timeout (30s) - returning to IDLE mode");
    mode = "IDLE";
    calibrateTimerStarted = false;
    printDivider();
    return;
  }
  
  // Clear screen effect for calibration mode with auto-scroll
  if (millis() - lastStatusUpdate > 200) { // Faster updates
    // Auto-scroll effect - print separator lines
    Serial.println("════════════════════════════════════════");
    Serial.println("           CALIBRATION MODE             ");
    Serial.println("════════════════════════════════════════");
    
    // Show remaining time
    int timeLeft = 30 - ((millis() - calibrateStartTime) / 1000);
    Serial.println("⏱️  Auto-exit in " + String(timeLeft) + " seconds (or type any command)");
    Serial.println("");
    
    // IMU Status
    Serial.println("🔧 IMU SENSORS (Finger-mounted):");
    String fingerNames[] = {"Thumb", "Index", "Middle"};
    for (int i = 0; i < imuCount && i < 3; i++) {
      String status = imuInitialized[i] ? "✓ ACTIVE" : "✗ OFFLINE";
      Serial.println("   " + fingerNames[i] + " IMU (Ch" + String(activeChannels[i]) + "): " + status);
    }
    if (imuCount == 0) {
      Serial.println("   No IMUs detected");
    }
    
    Serial.println("");
    Serial.println("📊 FINGER ORIENTATIONS (combined):");
    Serial.println("   X-axis (Curl):   " + String(currentGyro[0], 2) + "°");
    Serial.println("   Y-axis (Bend):   " + String(currentGyro[1], 2) + "°");
    Serial.println("   Z-axis (Spread): " + String(currentGyro[2], 2) + "°");
    
    Serial.println("");
    Serial.println("👆 FORCE SENSORS (0-1023):");
    
    for (int i = 0; i < 3; i++) {
      String fingerName[] = {"Thumb", "Index", "Middle"};
      String forceBar = "";
      int barLength = map(currentFSR[i], 0, 1023, 0, 15);
      for (int j = 0; j < 15; j++) {
        forceBar += (j < barLength) ? "█" : "░";
      }
      
      Serial.println("   " + fingerName[i] + ": " + String(currentFSR[i]) + " [" + forceBar + "]");
    }
    
    int totalForce = currentFSR[0] + currentFSR[1] + currentFSR[2];
    String forceStatus = (totalForce > FORCE_THRESHOLD) ? "MAPPING READY ✓" : "Below threshold";
    Serial.println("   Total: " + String(totalForce) + " (" + forceStatus + ")");
    
    Serial.println("");
    Serial.println("💡 Type any command to exit calibration mode");
    Serial.println("");
    
    lastStatusUpdate = millis();
  }
}

void handleSerialInput() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toUpperCase();

    previousMode = mode;

    if (cmd == "MAP") {
      if (mappedObjectCount >= MAX_WALLS) {
        Serial.println("❌ Cannot map more objects - maximum " + String(MAX_WALLS) + " objects reached");
        Serial.println("💡 Use 'REPLAY' to test existing objects or 'RESET' to clear memory");
        mode = "IDLE";
      } else {
        mode = "MAP";
        modeStartTime = millis();
      }
    } else if (cmd == "REPLAY") {
      if (mappedObjectCount == 0) {
        Serial.println("❌ No objects mapped yet - use 'MAP' command first");
        mode = "IDLE";
      } else {
        mode = "REPLAY";
        lastReplayMatch = millis();
        modeStartTime = millis();
      }
    } else if (cmd == "CALIBRATE") {
      mode = "CALIBRATE";
      modeStartTime = millis();
    } else if (cmd == "STATUS") {
      printHeader("SYSTEM STATUS");
      Serial.println("🔧 Hardware Status:");
      Serial.println("  IMUs detected: " + String(imuCount) + "/3 possible");
      Serial.println("  Haptic motors: 3 (pins " + String(hapticPins[0]) + ", " + String(hapticPins[1]) + ", " + String(hapticPins[2]) + ")");
      Serial.println("  Force sensors: 3 (pins A3, A6, A7)");
      
      Serial.println("\n📍 Object Mapping:");
      Serial.println("  Mapped objects: " + String(mappedObjectCount) + "/" + String(MAX_WALLS));
      for (int i = 0; i < mappedObjectCount; i++) {
        if (objects[i].valid) {
          Serial.println("    Object #" + String(i+1) + ": X=" + String(objects[i].gyro[0], 1) + 
                        "° Y=" + String(objects[i].gyro[1], 1) + "° Z=" + String(objects[i].gyro[2], 1) + "°");
        }
      }
      
      Serial.println("⚙️ Current Settings:");
      Serial.println("  Force threshold: " + String(FORCE_THRESHOLD) + " units (mapping only)");
      Serial.println("  Gyro tolerance: " + String(GYRO_TOLERANCE) + "° (replay matching) - VERY GENEROUS!");
      Serial.println("  Mapping duration: " + String(MAPPING_DURATION/1000.0) + "s");
      Serial.println("  Replay timeout: " + String(REPLAY_TIMEOUT/1000) + "s");
      
      Serial.println("\n🎮 Current Mode: " + mode);
      if (hapticsActive) {
        Serial.println("  Haptics: ACTIVE");
      } else {
        Serial.println("  Haptics: INACTIVE");
      }
      
      printDivider();
    } else if (cmd == "RESCAN") {
      Serial.println("🔄 Rescanning I2C bus for IMU sensors...");
      scanIMUs();
    } else if (cmd == "RESET") {
      Serial.println("🗑️ Clearing all mapped objects...");
      mappedObjectCount = 0;
      for (int i = 0; i < MAX_WALLS; i++) {
        objects[i].valid = false;
        objects[i].gyro[0] = 0;
        objects[i].gyro[1] = 0;
        objects[i].gyro[2] = 0;
      }
      EEPROM.put(0, objects);
      Serial.println("✓ Memory cleared - ready for new object mapping");
      mode = "IDLE";
    } else if (cmd == "TESTMOTOR") {
      Serial.println("🔧 Testing haptic motors at MAXIMUM intensity...");
      for (int motor = 0; motor < 3; motor++) {
        Serial.println("Testing motor " + String(motor + 1) + "/3 at PWM 255...");
        analogWrite(hapticPins[motor], 255); // Maximum test pulse
        delay(1000);
        analogWrite(hapticPins[motor], 0);
        delay(200);
      }
      Serial.println("✓ Motor test complete - all motors at maximum intensity");
    } else if (cmd == "FORCEHAPTIC") {
      Serial.println("🔥 FORCE HAPTIC TEST - Activating all motors at maximum!");
      for (int i = 0; i < 3; i++) {
        analogWrite(hapticPins[i], 255);
      }
      hapticsActive = true;
      Serial.println("💡 Motors should be vibrating at maximum intensity now");
      Serial.println("💡 Type any other command to stop haptics");
    } else if (cmd == "STOPHAPTIC") {
      Serial.println("🔇 Stopping all haptic motors...");
      stopHaptics();
    } else if (cmd == "HELP") {
      printHeader("COMMAND REFERENCE");
      Serial.println("MAP       - Enter mapping mode to record a new object");
      Serial.println("REPLAY    - Enter replay mode to feel mapped objects");
      Serial.println("CALIBRATE - Real-time sensor monitoring");
      Serial.println("STATUS    - Show detailed system information");
      Serial.println("RESCAN    - Reinitialize IMU sensors");
      Serial.println("RESET     - Clear all mapped object data");
      Serial.println("TESTMOTOR - Test all haptic motors");
      Serial.println("FORCEHAPTIC - Force haptics ON at maximum intensity");
      Serial.println("STOPHAPTIC - Stop all haptic motors");
      Serial.println("HELP      - Show this command reference");
      printDivider();
    } else {
      Serial.println("❌ Unknown command: '" + cmd + "'");
      Serial.println("💡 Type 'HELP' for available commands");
    }

    // Print mode change if mode actually changed
    if (mode != previousMode) {
      printModeChange(mode);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  tca.begin(Wire);

  // Startup banner
  delay(1000);
  Serial.println("\n\n");
  Serial.println("╔══════════════════════════════════════════════════╗");
  Serial.println("║              MIT HAPTIC GLOVE v2.0               ║");
  Serial.println("║            Enhanced Interactive System           ║");
  Serial.println("╚══════════════════════════════════════════════════╝");
  Serial.println("");

  Serial.println("🔧 Initializing hardware...");

  for (int i = 0; i < 3; i++) {
    pinMode(hapticPins[i], OUTPUT);
    analogWrite(hapticPins[i], 0);
    pinMode(fsrPins[i], INPUT);
  }

  scanIMUs();
  
  Serial.println("💾 Checking EEPROM for saved objects...");
  
  // First, completely clear all objects to start fresh
  Serial.println("🔄 Clearing EEPROM to start fresh...");
  for (int i = 0; i < MAX_WALLS; i++) {
    objects[i].valid = false;
    objects[i].gyro[0] = 0;
    objects[i].gyro[1] = 0;
    objects[i].gyro[2] = 0;
  }
  mappedObjectCount = 0;
  
  // Save the cleared state to EEPROM
  EEPROM.put(0, objects);
  
  Serial.println("✓ EEPROM cleared - starting with fresh memory");
  Serial.println("ℹ Ready for first object mapping");

  Serial.println("🚀 System ready!");
  printDivider();
  Serial.println("Available commands: MAP | REPLAY | CALIBRATE | STATUS | RESCAN | RESET | TESTMOTOR | FORCEHAPTIC | STOPHAPTIC | HELP");
  Serial.println("Type a command to begin...");
  printDivider();
}

void loop() {
  handleSerialInput();
  readFSRs(currentFSR);
  bool gyroRead = readGyro(currentGyro);

  if (mode == "CALIBRATE") {
    printSensorValues();
    delay(50); // Faster update for calibration
    return;
  }

  if (mode == "MAP" && mappedObjectCount < MAX_WALLS) {
    int totalForce = currentFSR[0] + currentFSR[1] + currentFSR[2];
    
    if (waitingForForce && totalForce <= FORCE_THRESHOLD) {
      // Show waiting status every 2 seconds
      static unsigned long lastWaitMsg = 0;
      if (millis() - lastWaitMsg > 2000) {
        Serial.println("⏳ Waiting for force... Current: " + String(totalForce) + 
                      "/" + String(FORCE_THRESHOLD) + " (apply pressure to fingers)");
        lastWaitMsg = millis();
      }
    }
    
    if (totalForce > FORCE_THRESHOLD) {
      if (waitingForForce) {
        Serial.println("✓ Force detected! Beginning object mapping sequence...");
        waitingForForce = false;
      }
      
      averageGyro(objects[mappedObjectCount].gyro, MAPPING_DURATION);
      objects[mappedObjectCount].valid = true;
      EEPROM.put(mappedObjectCount * sizeof(ObjectPose), objects[mappedObjectCount]);
      
      Serial.println("💾 Object #" + String(mappedObjectCount + 1) + " saved to memory");
      mappedObjectCount++;
      
      if (mappedObjectCount >= MAX_WALLS) {
        Serial.println("🎉 Maximum objects mapped! Switching to IDLE mode");
        Serial.println("💡 Use 'REPLAY' to test your mapped objects");
      } else {
        Serial.println("✨ Ready to map another object (use 'MAP' command)");
      }
      
      mode = "IDLE";
      printDivider();
    }
  }

  if (mode == "REPLAY") {
    float bestScore = 9999;
    int bestIndex = -1;
    static int lastMatchedObject = -1;
    static unsigned long lastReplayStatus = 0;

    // Find the best matching object
    for (int i = 0; i < mappedObjectCount; i++) {
      if (!objects[i].valid) continue;
      float score = poseDistance(currentGyro, objects[i].gyro);
      if (score < bestScore) {
        bestScore = score;
        bestIndex = i;
      }
    }

    // MUCH more generous triggering - haptics activate easily
    if (bestIndex >= 0 && bestScore < GYRO_TOLERANCE) {
      if (bestIndex != lastMatchedObject) {
        Serial.println("🎯 Object #" + String(bestIndex + 1) + " detected! (match score: " + 
                      String(bestScore, 1) + "°)");
        Serial.println("📍 Target fingers: X=" + String(objects[bestIndex].gyro[0], 0) + 
                      "° Y=" + String(objects[bestIndex].gyro[1], 0) + 
                      "° Z=" + String(objects[bestIndex].gyro[2], 0) + "°");
        Serial.println("📍 Current fingers: X=" + String(currentGyro[0], 0) + 
                      "° Y=" + String(currentGyro[1], 0) + 
                      "° Z=" + String(currentGyro[2], 0) + "°");
        Serial.println("🔥 MAXIMUM HAPTICS ACTIVATED! (300° tolerance zone)");
        lastMatchedObject = bestIndex;
      }
      
      // ALWAYS trigger haptics when in range - maximum intensity
      replayHaptics(bestScore);
      lastReplayMatch = millis();
    } else {
      if (lastMatchedObject != -1) {
        Serial.println("📤 Left haptic zone - motors stopped");
        lastMatchedObject = -1;
      }
      stopHaptics();
      
      // Show search status every 2 seconds (frequent updates)
      if (millis() - lastReplayStatus > 2000) {
        Serial.println("🔍 Searching for finger patterns... (tolerance: " + String(GYRO_TOLERANCE) + "° - HUGE ZONE!)");
        Serial.println("📍 Current fingers: X=" + String(currentGyro[0], 0) + 
                      "° Y=" + String(currentGyro[1], 0) + "° Z=" + String(currentGyro[2], 0) + "°");
        Serial.println("");
        
        // Show distances and movement instructions for all mapped objects
        int closestObject = -1;
        float closestDistance = 9999;
        
        for (int i = 0; i < mappedObjectCount; i++) {
          if (objects[i].valid) {
            float dist = poseDistance(currentGyro, objects[i].gyro);
            Serial.println("📊 Object #" + String(i+1) + " distance: " + String(dist, 1) + "°");
            Serial.println("   Target: X=" + String(objects[i].gyro[0], 0) + 
                          "° Y=" + String(objects[i].gyro[1], 0) + "° Z=" + String(objects[i].gyro[2], 0) + "°");
            
            if (dist < closestDistance) {
              closestDistance = dist;
              closestObject = i;
            }
          }
        }
        
        if (closestObject >= 0) {
          Serial.println("");
          Serial.println("🎯 CLOSEST OBJECT #" + String(closestObject + 1) + " (" + String(closestDistance, 1) + "° away):");
          if (closestDistance < 350) {
            Serial.println("🟢 SHOULD BE TRIGGERING! Very close to 300° threshold!");
          } else if (closestDistance < 500) {
            Serial.println("🟡 GETTING VERY CLOSE! Almost in trigger zone!");
          } else if (closestDistance < 800) {
            Serial.println("🟠 GETTING CLOSER!");
          } else {
            Serial.println("🔴 Still searching...");
          }
          Serial.println("💡 " + getMovementInstruction(currentGyro, objects[closestObject].gyro));
        }
        
        Serial.println("");
        lastReplayStatus = millis();
      }
      
      if (millis() - lastReplayMatch > REPLAY_TIMEOUT) {
        Serial.println("⏰ Replay timeout (" + String(REPLAY_TIMEOUT/1000) + "s) - returning to IDLE mode");
        mode = "IDLE";
        printDivider();
      }
    }
  } else {
    stopHaptics();
  }

  delay(50);
}
