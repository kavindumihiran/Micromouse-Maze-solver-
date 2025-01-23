#include <Arduino.h>

#define trigPinLeft 2
#define echoPinLeft 3

#define trigPinRight 11
#define echoPinRight 10

#define trigPinFront 4
#define echoPinFront 8

#define ENA 5 // pins for left motor
#define IN1 6
#define IN2 7

#define ENB 9 // pins for Right motor
#define IN3 12
#define IN4 13

enum Direction {SOUTH = 0, WEST = 1, EAST = 2, NORTH = 3};
Direction currentDirection = NORTH;
Direction targetDirection;

Direction direction_order[] = {SOUTH, WEST, EAST, NORTH};
 
bool leftWall;                   
bool rightWall;                                 
bool frontWall;

bool northWall;
bool southWall;
bool westWall;
bool eastWall;
                                               
long distanceFront;                            
long distanceLeft;
long distanceRight;

int baseSpeed = 150;

double dt, last_time = 0;
double integral, previous, output = 0;

struct Cell {
  int x; // X-coordinate
  int y; // Y-coordinate
};

struct Queue{
  Cell data[9*9];
  int top = -1;

  void push(Cell cell){
    data[++top] = cell;
  }

  Cell pop(){
    return data[top--];
  }

  bool isEmpty(){
    return top == -1;
  }

  int size() {
    return top + 1;
  }
};

bool cell_wall[4];
Queue Frontier;
Queue Explored;
Cell child_cell;
int Explored_size;

long readUltrasonic(int trigPin, int echoPin) {// Function to read distance from an ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;  // Calculate distance in cm
}

void DirChange_right() { // Moves the direction counterclockwise. Adding 3 achieves a backward rotation in modular arithmetic: NORTH -> EAST EAST -> SOUTH SOUTH -> WEST WEST -> NORTH
  currentDirection = static_cast<Direction>((currentDirection + 1) % 4);
}

void DirChange_left() { //  Moves the direction counterclockwise. Adding 3 achieves a backward rotation in modular arithmetic: NORTH -> WEST WEST -> SOUTH SOUTH -> EAST EAST -> NORTH
  currentDirection = static_cast<Direction>((currentDirection + 3) % 4);
}

void DirChange_180() {  // Reverse the current direction. Example: NORTH -> SOUTH
  currentDirection = static_cast<Direction>((currentDirection + 2) % 4);
}

void getSensorState(){ // Get the current readings from the sensor and determine what is the action to do now
  distanceFront = readUltrasonic(trigPinFront, echoPinFront);
  distanceLeft = readUltrasonic(trigPinLeft, echoPinLeft);
  distanceRight = readUltrasonic(trigPinRight, echoPinRight); 

  if(distanceLeft <= 10){
    leftWall = true;
  }else{
    leftWall = false;
  }

  if(distanceRight <= 10){
    rightWall = true;
  }else{
    rightWall = false;
  }

  if(distanceFront <= 10){
    frontWall = true;
  }else{
    frontWall = false;
  }
}

void interpretSensorReadings() {
  getSensorState();

  switch (currentDirection) {
    case NORTH:
      northWall = frontWall;
      westWall = leftWall;
      eastWall = rightWall;
      southWall = false;
      break;
    case EAST:
      eastWall = frontWall;
      northWall = leftWall;
      southWall = rightWall;
      westWall = false;
      break;
    case SOUTH:
      southWall = frontWall;
      eastWall = leftWall;
      westWall = rightWall;
      northWall = false;
      break;
    case WEST:
      westWall = frontWall;
      southWall = leftWall;
      northWall = rightWall;
      eastWall = false;
      break;
  }

  cell_wall[0] = southWall;
  cell_wall[1] = westWall;
  cell_wall[2] = eastWall;
  cell_wall[3] = northWall;
}

bool IsInExplored(Cell child){
  Explored_size = Explored.size();

  for (int i = 0; i < Explored_size; i++) {
      if (Explored.data[i].x == child.x && Explored.data[i].y == child.y) {
          return true;  // Cell found in the explored list
      }
  }
  return false;
}

void turnRight_motorCommands(){ // Command the motors to turn Right
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 170);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 170);
}

void turnLeft_motorCommands(){ // Command the motors to turn Left
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 170);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 170);
}

void stopMotor_motorCommands() { // Completely stop the motors 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void RotateRight(){ // Turn the robot 90 degrees to the right
  if ((readUltrasonic(trigPinFront, echoPinFront)) < 15){ // check whether a forward wall is available 
    stopMotor_motorCommands();
    delay(250);
    turnRight_motorCommands();
    while(readUltrasonic(trigPinFront, echoPinFront) <= 25){ // Turn right untill the front sensor will not detect a wall.
      delay(10);
    }
    delay(70);
    stopMotor_motorCommands();
    delay(250);
  }
  else{
    stopMotor_motorCommands();
    delay(250);
    turnRight_motorCommands();
    delay(300);
    stopMotor_motorCommands();
    delay(250);
  }
}

void RotateLeft (){ //Turn the robot 90 degrees to the right
  if ((readUltrasonic(trigPinFront, echoPinFront)) < 15){ // check whether a forward wall is available 
    stopMotor_motorCommands();
    delay(250);
    turnLeft_motorCommands();
    while(readUltrasonic(trigPinFront, echoPinFront) <= 25){ // Turn right untill the front sensor will not detect a wall.
      delay(10);
    }
    delay(70);
    stopMotor_motorCommands();
    delay(250);
  }
  else{
    stopMotor_motorCommands();
    delay(250);
    turnLeft_motorCommands();
    delay(300);
    stopMotor_motorCommands();
    delay(250);
  }
}

void Rotate180(){ //Turn the robot 180 degrees
  stopMotor_motorCommands();
  delay(250);
  turnRight_motorCommands();
  while(readUltrasonic(trigPinFront, echoPinFront) <= 25){
    delay(50);
  }
  stopMotor_motorCommands();
  delay(250);
}

void leftwallForward(){
  double dt, last_time = millis();

  int kp = 1.5;  // Proportional gain
  int kd = 0.5 ;  // Derivative gain

  double now = millis();
  dt = (now - last_time)/1000;
  last_time = now;

  int targetDistance = 8;

  // Calculate the error between left and right distances
  double errorLeft = targetDistance - readUltrasonic(trigPinLeft, echoPinLeft);

  // Calculate the derivative (rate of change of error)
  double derivative = (errorLeft - previous) / dt;

  // Apply PD control: combine proportional and derivative terms
  int correction = (kp * errorLeft) + (kd * derivative);

  // Update the previous error for the next loop
  previous = errorLeft;

  // Adjust motor speeds with correction
  int speedA = baseSpeed + correction;
  int speedB = baseSpeed - correction;

  // Ensure the speeds are within the valid range
  speedA = constrain(speedA, 0, 170);
  speedB = constrain(speedB, 0, 170);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedA+5);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedB);
}

void rightwallForward(){
  double previous = 0, correction = 0;
  double dt, last_time = millis();

  int kp = 1;  // Proportional gain
  int kd = 0.5 ;  // Derivative gain

  double now = millis();
  dt = (now - last_time)/1000;
  last_time = now;

  int targetDistance = 5;

  // Calculate the error between left and right distances
  double errorRight = targetDistance - readUltrasonic(trigPinRight, echoPinRight);

  // Calculate the derivative (rate of change of error)
  double derivative = (errorRight - previous) / dt;

  // Apply PD control: combine proportional and derivative terms
  correction = (kp * errorRight) + (kd * derivative);

  // Update the previous error for the next loop
  previous = errorRight;

  // Adjust motor speeds with correction
  int speedA = baseSpeed - correction;
  int speedB = baseSpeed + correction;

  // Ensure the speeds are within the valid range
  speedA = constrain(speedA, 0, 170);
  speedB = constrain(speedB, 0, 170);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, (speedA+5));
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedB);
}

void moveForward2(){ // Move forward without following a wall

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 160);
  analogWrite(ENB, 160);

}

void moveStep(){ // Go forward for one step (one tile)

  double startTime = millis(); // Record the start time
  double moveDuration = 830; 

  while ((millis() - startTime) < moveDuration) {
    if (readUltrasonic(trigPinLeft, echoPinLeft)<3) {
      leftwallForward();
    }
    else if (readUltrasonic(trigPinRight, echoPinRight)<3) {
      rightwallForward();
    }else{
      moveForward2();
    } 
  }

  // Stop the motors after reaching the tile
  stopMotor_motorCommands();
  delay(250);  // Pause briefly
}

void moveRobot(Direction target, Direction current) {
    // Determine the rotation needed
    int rotation = (target - current + 4) % 4;

    // Rotate the robot based on the difference
    switch (rotation) {
        case 0: // No rotation needed
            break;
        case 1: // Rotate 90 degrees right
            RotateRight();
            DirChange_right();
            break;
        case 2: // Rotate 180 degrees
            Rotate180();
            DirChange_180();
            break;
        case 3: // Rotate 90 degrees left
            RotateLeft();
            DirChange_left();
            break;
    }

    // Move forward one tile
    moveStep();

    // Update current direction
    currentDirection = targetDirection;
}

void DFS(Cell start){
  Frontier.push(start);
  Explored.push(start);
  Cell previous_cell = start;

  while (!Frontier.isEmpty()){
    Cell current_cell = Frontier.pop();
    // IR sensor reading is white{  that means the goal is reached
    // break
    // }

    // move the robot to current_cell 
    if ( (current_cell.x == start.x) && (current_cell.y == start.y)){ // because at start we place the robot in that cell 
      previous_cell = current_cell;
      continue;
    }

    // Determine the direction to move based on the difference between cells
    int deltaX = current_cell.x - previous_cell.x;
    int deltaY = current_cell.y - previous_cell.y;

  
    if (deltaX == 0 && deltaY == 1) {
      // Move North
      targetDirection = NORTH;

    } else if (deltaX == 0 && deltaY == -1) {
      // Move South
      targetDirection = SOUTH;

    } else if (deltaX == 1 && deltaY == 0) {
      // Move East
      targetDirection = EAST;

    } else if (deltaX == -1 && deltaY == 0) {
      // Move West
      targetDirection = WEST;
    }

    moveRobot(targetDirection, currentDirection);

    previous_cell = current_cell; // keep track of the previous cell 

    for (int i = 0; i < 4; i++){
      Direction d = direction_order[i];

      if (cell_wall[i] == false){
        if (d == NORTH){
          child_cell.x = current_cell.x; 
          child_cell.y = current_cell.y + 1;
        }

        else if (d == EAST){
          child_cell.x = current_cell.x; 
          child_cell.y = current_cell.y - 1;
        } 

        else if (d == WEST){
          child_cell.x = current_cell.x - 1; 
          child_cell.y = current_cell.y;
        } 

        else if (d == SOUTH){
          child_cell.x = current_cell.x + 1; 
          child_cell.y = current_cell.y;
        } 

        if (IsInExplored(child_cell)) {
          continue;
        }
        Explored.push(child_cell);
        Frontier.push(child_cell);
      }
    }
  }
}

void setup() {
  pinMode(ENA, OUTPUT);   // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);

  pinMode(trigPinLeft, OUTPUT);  //Ultrasonic sensor pins setup
  pinMode(echoPinLeft, INPUT);

  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Cell start;
  // start.x = 0;
  // start.y = 0;
  // DFS(start);

  // Explored.push(start);
  // delay(1000);
  // Serial.println(Explored.size());

  Serial.println(readUltrasonic(trigPinFront, echoPinFront));
}
