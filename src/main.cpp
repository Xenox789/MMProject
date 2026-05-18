// ============================================================
//  Micromouse - Flood Fill Maze Solver
//  
//  Controls:
//    - Press button once: Start EXPLORATION (find the center)
//    - Press button again: Start SPEED RUN (optimal path)
//    - Hold button 2s: Sensor calibration mode
//  
//  LED Status:
//    - LED1 blinks slowly: IDLE, waiting for button
//    - LED1 solid: EXPLORING maze
//    - LED2 solid: SPEED RUN
//    - Both LEDs blink fast: ERROR (fault/low battery)
// ============================================================

#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "motors.h"
#include "sensors.h"
#include "maze.h"

// ---- State Machine ----
enum State {
  STATE_IDLE,          // Waiting for button press
  STATE_EXPLORING,     // Flood-fill exploration to center
  STATE_RETURNING,     // Return to start after reaching center
  STATE_READY,         // At start, ready for speed run
  STATE_SPEED_RUN,     // Fast run to center
  STATE_CALIBRATE,     // Sensor calibration mode
  STATE_ERROR          // Error state (fault, low battery)
};

State currentState = STATE_IDLE;

// ---- Mouse position and heading ----
int mouseX = 0;
int mouseY = 0;
Direction mouseHeading = NORTH;

// ---- Button ----
bool buttonPressed();
unsigned long lastButtonTime = 0;

// ---- LED helpers ----
void setLED1(bool on) { digitalWrite(PIN::DEBUG_LED_1, on ? HIGH : LOW); }
void setLED2(bool on) { digitalWrite(PIN::DEBUG_LED_2, on ? HIGH : LOW); }

// ---- Forward declarations ----
void executeMove(Direction targetDir, int speed);
void exploreStep();
void speedRunStep();
void calibrateMode();

// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("================================");
  Serial.println("  Micromouse v1.0 - Flood Fill");
  Serial.println("================================");
  
  // Button
  pinMode(PIN::BTN, INPUT_PULLUP);
  
  // Debug LEDs
  pinMode(PIN::DEBUG_LED_1, OUTPUT);
  pinMode(PIN::DEBUG_LED_2, OUTPUT);
  setLED1(false);
  setLED2(false);
  
  // Initialize subsystems
  Motors::init();
  Sensors::init();
  Maze::init();
  
  // Enable motors
  Motors::enable();
  
  // Check for faults immediately
  if (Motors::isFault()) {
    Serial.println("!!! MOTOR DRIVER FAULT at startup !!!");
    currentState = STATE_ERROR;
  }
  
  // Check battery
  if (Sensors::isBatteryLow()) {
    Serial.println("!!! LOW BATTERY !!!");
    currentState = STATE_ERROR;
  }
  
  Serial.println("Ready. Press button to start exploration.");
  Serial.print("Battery: "); Serial.println(Sensors::readBatteryRaw());
}

// ============================================================
void loop() {
  switch (currentState) {
    
    // ---- IDLE: Blink LED, wait for button ----
    case STATE_IDLE: {
      // Slow blink LED1
      setLED1((millis() / 500) % 2);
      setLED2(false);
      
      if (buttonPressed()) {
        // Check for long press (calibrate mode)
        unsigned long pressStart = millis();
        while (digitalRead(PIN::BTN) == LOW) {
          if (millis() - pressStart > 2000) {
            currentState = STATE_CALIBRATE;
            Serial.println(">> Entering CALIBRATION mode");
            return;
          }
        }
        
        Serial.println(">> Starting EXPLORATION");
        Serial.println("   Starting in 1 second...");
        setLED1(true);
        delay(CONFIG::STARTUP_DELAY_MS);
        
        // Reset position
        mouseX = 0;
        mouseY = 0;
        mouseHeading = NORTH;
        
        // Initialize maze (clear old data)
        Maze::init();
        
        currentState = STATE_EXPLORING;
      }
      break;
    }
    
    // ---- EXPLORING: Navigate to center using flood-fill ----
    case STATE_EXPLORING: {
      setLED1(true);
      setLED2(false);
      
      // Check for errors
      if (Motors::isFault()) {
        Serial.println("FAULT during exploration!");
        Motors::stop();
        currentState = STATE_ERROR;
        break;
      }
      
      if (Sensors::isBatteryLow()) {
        Serial.println("Low battery during exploration!");
        Motors::stop();
        currentState = STATE_ERROR;
        break;
      }
      
      // Execute one exploration step
      exploreStep();
      
      // Check if we reached the goal
      if (Maze::isGoal(mouseX, mouseY)) {
        Motors::stop();
        Serial.println("=============================");
        Serial.println("  GOAL REACHED!");
        Serial.println("=============================");
        Maze::printMaze();
        
        Serial.println(">> Returning to start...");
        delay(500);
        currentState = STATE_RETURNING;
      }
      break;
    }
    
    // ---- RETURNING: Navigate back to (0,0) ----
    case STATE_RETURNING: {
      setLED1((millis() / 200) % 2);  // Fast blink
      setLED2(false);
      
      if (Motors::isFault() || Sensors::isBatteryLow()) {
        Motors::stop();
        currentState = STATE_ERROR;
        break;
      }
      
      // Flood fill targeting start
      Maze::floodFillToStart();
      
      // Read walls and update maze
      WallInfo wi = Sensors::detectWalls();
      Maze::updateWalls(mouseX, mouseY, mouseHeading, wi);
      
      // Find best direction
      Direction targetDir = Maze::bestDirection(mouseX, mouseY, mouseHeading);
      
      // Execute the move
      executeMove(targetDir, CONFIG::EXPLORE_SPEED);
      
      // Check if we're back at start
      if (mouseX == 0 && mouseY == 0) {
        Motors::stop();
        Serial.println("=============================");
        Serial.println("  BACK AT START!");
        Serial.println("  Press button for SPEED RUN");
        Serial.println("=============================");
        currentState = STATE_READY;
      }
      break;
    }
    
    // ---- READY: At start, waiting for speed run ----
    case STATE_READY: {
      setLED1(false);
      setLED2((millis() / 500) % 2);  // Blink LED2
      
      if (buttonPressed()) {
        Serial.println(">> Starting SPEED RUN");
        Serial.println("   Starting in 1 second...");
        setLED2(true);
        delay(CONFIG::STARTUP_DELAY_MS);
        
        mouseX = 0;
        mouseY = 0;
        mouseHeading = NORTH;
        
        // Flood fill to goal with all discovered walls
        Maze::floodFill(CONFIG::GOAL_X1, CONFIG::GOAL_Y1,
                        CONFIG::GOAL_X2, CONFIG::GOAL_Y2);
        
        currentState = STATE_SPEED_RUN;
      }
      break;
    }
    
    // ---- SPEED RUN: Fast navigation on known path ----
    case STATE_SPEED_RUN: {
      setLED1(false);
      setLED2(true);
      
      if (Motors::isFault() || Sensors::isBatteryLow()) {
        Motors::stop();
        currentState = STATE_ERROR;
        break;
      }
      
      speedRunStep();
      
      if (Maze::isGoal(mouseX, mouseY)) {
        Motors::stop();
        Serial.println("=============================");
        Serial.println("  SPEED RUN COMPLETE!");
        Serial.println("=============================");
        currentState = STATE_IDLE;
      }
      break;
    }
    
    // ---- CALIBRATE: Print sensor values for tuning ----
    case STATE_CALIBRATE: {
      setLED1(true);
      setLED2(true);
      
      Sensors::printValues();
      delay(200);
      
      if (buttonPressed()) {
        Serial.println(">> Exiting calibration");
        currentState = STATE_IDLE;
      }
      break;
    }
    
    // ---- ERROR: Flash both LEDs ----
    case STATE_ERROR: {
      Motors::stop();
      bool on = (millis() / 150) % 2;
      setLED1(on);
      setLED2(on);
      
      // Print error info periodically
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        Serial.print("ERROR - Fault: "); Serial.print(Motors::isFault());
        Serial.print(" Battery: ");      Serial.println(Sensors::readBatteryRaw());
      }
      
      // Button press to return to idle
      if (buttonPressed()) {
        currentState = STATE_IDLE;
      }
      break;
    }
  }
}

// ============================================================
//  EXPLORATION STEP
//  - Read walls at current position
//  - Update maze
//  - Run flood fill
//  - Move to the cell with the lowest flood value
// ============================================================
void exploreStep() {
  // 1) Read walls
  WallInfo wi = Sensors::detectWalls();
  
  // 2) Update maze with discovered walls
  Maze::updateWalls(mouseX, mouseY, mouseHeading, wi);
  
  Serial.print("Pos: ("); Serial.print(mouseX);
  Serial.print(",");      Serial.print(mouseY);
  Serial.print(") Heading: "); Serial.print(mouseHeading);
  Serial.print(" Walls F:"); Serial.print(wi.front);
  Serial.print(" L:");       Serial.print(wi.left);
  Serial.print(" R:");       Serial.println(wi.right);
  
  // 3) Run flood fill to the goal
  Maze::floodFill(CONFIG::GOAL_X1, CONFIG::GOAL_Y1,
                  CONFIG::GOAL_X2, CONFIG::GOAL_Y2);
  
  // 4) Find best direction
  Direction targetDir = Maze::bestDirection(mouseX, mouseY, mouseHeading);
  
  Serial.print("  Flood value: "); Serial.print(Maze::getFloodValue(mouseX, mouseY));
  Serial.print(" -> Moving: ");    Serial.println(targetDir);
  
  // 5) Execute the move
  executeMove(targetDir, CONFIG::EXPLORE_SPEED);
}

// ============================================================
//  SPEED RUN STEP
//  Uses pre-computed flood values (no re-computation each cell)
// ============================================================
void speedRunStep() {
  // We already have the flood map calculated
  Direction targetDir = Maze::bestDirection(mouseX, mouseY, mouseHeading);
  executeMove(targetDir, CONFIG::SPEED_RUN_SPEED);
}

// ============================================================
//  EXECUTE MOVE
//  Turn to face the target direction, then move forward one cell
// ============================================================
void executeMove(Direction targetDir, int speed) {
  RelativeDir turn = Maze::getRelativeTurn(mouseHeading, targetDir);
  
  // Execute the required turn
  switch (turn) {
    case REL_FORWARD:
      // No turn needed
      break;
    case REL_RIGHT:
      Motors::turnRight90(CONFIG::TURN_SPEED);
      break;
    case REL_LEFT:
      Motors::turnLeft90(CONFIG::TURN_SPEED);
      break;
    case REL_BACK:
      Motors::turnAround(CONFIG::TURN_SPEED);
      break;
  }
  
  // Update heading
  mouseHeading = targetDir;
  
  // Move forward one cell
  Motors::moveForwardCells(1, speed);
  
  // Update position
  int nx, ny;
  Maze::getNextCell(mouseX, mouseY, mouseHeading, nx, ny);
  
  // Sanity check (should never happen if maze is correct)
  if (nx < 0 || nx >= CONFIG::MAZE_SIZE || ny < 0 || ny >= CONFIG::MAZE_SIZE) {
    Serial.println("ERROR: Moved out of bounds!");
    currentState = STATE_ERROR;
    return;
  }
  
  mouseX = nx;
  mouseY = ny;
}

// ============================================================
//  BUTTON HELPER
// ============================================================
bool buttonPressed() {
  if (digitalRead(PIN::BTN) == LOW) {
    if (millis() - lastButtonTime > CONFIG::BUTTON_DEBOUNCE_MS) {
      lastButtonTime = millis();
      return true;
    }
  }
  return false;
}
