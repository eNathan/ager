/**
  * Nathan Linebarger
  * CS 362
  * Autonomous Robot Project
**/

#include <Servo.h> 

Servo servo;

const int DIR_BACKWARDS = LOW;
const int DIR_FORWARD = HIGH;

const int DIR_LEFT = 0;
const int DIR_RIGHT = 1;

const int BRAKE_ON = HIGH;
const int BRAKE_OFF = LOW;

const int LEFT = 0;
const int RIGHT = 1;

// Pin Definitions

// Left Motor
const int LEFT_DIR = 13; // Digital
const int LEFT_BRAKE = 8; // Digital
const int LEFT_SPEED = 11; // Analog

// Right Motor
const int RIGHT_DIR = 12; // Digital
const int RIGHT_BRAKE = 9; // Digital
const int RIGHT_SPEED = 3; // Analog

// Servo
const int SERVO = 40;

// Front Distance Sensors
const int FRONT_DISTANCE_TRIGGER = 44;
const int FRONT_DISTANCE_ECHO = 45;

// Rear Distance Sensors
const int REAR_DISTANCE_TRIGGER = 48;
const int REAR_DISTANCE_ECHO = 49;

// The front servo's position that ends up pointing north. We need this as a reference for all computations on angles
const int NORTH = 55;

// If we let East be angle 0 (degrees) in the real world, and if we set the front servo's angle to 0, what real-world angle is produced?
const int SERVO_FRONT_CALIBRATION_ANGLE = 90 - NORTH - 90 + 360;
// If we let East be angle 0 (degrees) in the real world, and if we set the rear servo's angle to 0, what real-world angle is produced?
const int SERVO_REAR_CALIBRATION_ANGLE = SERVO_FRONT_CALIBRATION_ANGLE + 180;

// Misc
const double SPEED_ENCODER_RESOLUTION = 5;

// Motor calibration numbers for the function fps(voltage)
// The first dimension is for which motor (there's 2 motors)
// The second dimension is for which direction (there's 2 directions)
// The third dimension contains 3 values: the slope of the best-fitting line, the y-intercept of the best-fitting line,
// the minimum voltage required to overcome static friction, and the number of ticks that it takes to slow down the motor from 255 volts to 0 volts
// The values below are based on pre-calibration in case no calibration is performed upon startup
double MOTOR_CALIBRATION[2][2][4] = {
	{
		{0.007090, -0.410714, 153, 0},
		{0.006016, 0.263636, 102, 0},
	},
	{
		{0.008009, -0.446429, 153, 0},
		{0.000171, 0.063953, 170, 0},
	}
};

//const double PI = 3.1415926535;

// Wheel Data

// Wheel Size (in centimeters)
const double WHEEL_DIAMETER = 6.5;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;
const double DISTANCE_BETWEEN_WHEELS = 15;

const double DISTANCE_PER_REVOLUTION = WHEEL_CIRCUMFERENCE;
const double DISTANCE_PER_MAGNET_TICK = WHEEL_CIRCUMFERENCE / SPEED_ENCODER_RESOLUTION;

// Wheel revolution counter
int leftMagnetCount = 0;
int rightMagnetCount = 0;

String inData; // Read Buffer

/**************************** CALIBRATION FUNCTIONS ****************************/

// Compute the sum of a set of data
double summation(double *data, int size){
	double result = 0;
	for(int i = 0; i < size; i++){
		result += data[i];
	}
	return result;
}

double average(double *data, int size){
	return summation(data, size) / (double)size;
}

double lineFitSlope(double *x, double *y, int size){
	double xAvg = average(x, size);
	double yAvg = average(y, size);
	double a = 0;
	double b = 0;

	for(int i = 0; i < size; i++){
		a += (x[i] - xAvg) * (y[i] - yAvg);
		b += sq(x[i] - xAvg);
	}

	return a / b;
}

double lineFitYIntercept(double *x, double *y, int size){
	double slope = lineFitSlope(x, y, size);
	double xAvg = average(x, size);
	double yAvg = average(y, size);
	return yAvg - (slope * xAvg);
}

/**
  * Based on the calibration numbers, what is the predicted voltage we need to go a certain rpm?
**/
int voltageFromRPM(int rpm){

}

void setup() {
	// Set up all the pins we need

	// Left Wheel
	pinMode(13, OUTPUT);
	pinMode(8, OUTPUT);

	// Right Wheel
	pinMode(12, OUTPUT);
	pinMode(9, OUTPUT);

	// Servo
	servo.attach(SERVO);

	// Front Distance Sensors
	pinMode(FRONT_DISTANCE_TRIGGER, OUTPUT);
	pinMode(FRONT_DISTANCE_ECHO, INPUT);

	// REAR Distance Sensors
	pinMode(REAR_DISTANCE_TRIGGER, OUTPUT);
	pinMode(REAR_DISTANCE_ECHO, INPUT);

	// Set up interrupts on for Hall Effect (magnet) Sensors
	attachInterrupt(2, onLeftMagnetSensorDetect, FALLING); // Pin 21
	attachInterrupt(0, onRightMagnetSensorDetect, FALLING); // Pin 2

	// USB Debugging
	Serial.begin(9600);

	Serial.println("# AGER has come to life. You've been warned.");
}

void onLeftMagnetSensorDetect(){
	leftMagnetCount++;
}

void onRightMagnetSensorDetect(){
	rightMagnetCount++;
	//Serial.println("Right Tick");
}

/**************************** PROCESS REMOTE COMMANDS ****************************/



void processGoCommand(String command){
	command.replace("go ", "");	
	int direction;
	int distance;
	if(command.startsWith("forward ")){
		direction = DIR_FORWARD;
		Serial.println("# Dir = FORWARD");
		command.replace("forward ", "");
	} else if(command.startsWith("backward ")){
		direction = DIR_BACKWARDS;
		Serial.println("# Dir = BACK");
		command.replace("backward ", "");
	}
	distance = command.toInt();
	Serial.print("# Distance = ");
	Serial.println(distance);
	travelDistance(distance, direction);
}

void processTurnCommand(String command){
	command.replace("turn ", "");	
	int direction;
	int angle;
	if(command.startsWith("left ") || command.startsWith("right ")){
		if(command.startsWith("left ")){
			direction = DIR_LEFT;
			command.replace("left ", "");
		} else if(command.startsWith("right ")){
			direction = DIR_RIGHT;
			command.replace("right ", "");
		}
		angle = command.toInt();
	} else {
		angle = command.toInt();
		if(angle >= 0){
			direction = LEFT;
		} else {
			angle = angle * -1;
			direction = RIGHT;
		}
	}
	travelToAngle(angle, direction);
}

void processCommand(String command){
	if(command.startsWith("go ")){
		processGoCommand(command);
	} else if(command.startsWith("turn ")){
		processTurnCommand(command);
	} else if(command.startsWith("scan")){
		scanAndSend();
	} else if(command.startsWith("calibrate")){
		calibrateMotors();
	}
}

void loop(){
	String command;
	if(Serial.available()){
	    while (Serial.available() > 0)
	    {
	        char recieved = Serial.read();
	        inData += recieved; 
	        if (recieved == '\n'){ // We have a full line
	            Serial.print("# AGER Received Command: "); // Echo back to Python Debugger
	            Serial.println(inData);
	            processCommand(inData);
	            inData = ""; // Clear the buffer
	        }
	    }
		//scanAndSend();
	}
	
	//Serial.println("# Ping from AGER");
	delay(1);
}

void scanAndSend(){
	long scan[360];
	scanEnvironment(scan);
	printScan(scan);
}


/**************************** HIGH LEVEL DISTANCE SENSOR FUNCTIONS ****************************/

void printScan(long scanArray[360]){
	Serial.println("# Done with scan");
	Serial.print("scan ");
	for(int i = 0; i < 360; i++){
		Serial.print(scanArray[i]);
		if(i != 359){
			Serial.print(",");
		}
	}
	Serial.println();
}

// Take in a servo angle and convert it to a "real world" angle where 0 degrees is East
int frontSensorAngle(int servoAngle){
	return (SERVO_FRONT_CALIBRATION_ANGLE + servoAngle) % 360;
}

// Take in a servo angle and convert it to a "real world" angle where 0 degrees is East
int rearSensorAngle(int servoAngle){
	return (SERVO_REAR_CALIBRATION_ANGLE + servoAngle) % 360;
}

/**
  * Use the distance sensors and servo to perform a full 360-degree scan of all objects surrounding the robot
  * Return the result
**/
void scanEnvironment(long result[360]){
	int angle;
	int frontDistance;
	int rearDistance;

	// Reset the servo's position
	servo.write(0);
	delay(10);

	// Do a full scan
	for(angle = 0; angle < 180 - 5; angle++){
		servo.write(angle);
		(result)[frontSensorAngle(angle)] = getFrontDistance();
		(result)[rearSensorAngle(angle)] = getRearDistance();
		delay(15);
	}
}

/**************************** LOW LEVEL DISTANCE SENSOR FUNCTIONS ****************************/

/**
  * The distance sensor sends us a PWM that we recieve as the number of microseconds for which the 
  * pulse was active. This is the conversion to corrolate that with a distance (in inches, because 'merica)
**/
long microsecondsToInches(long microseconds){
	return microseconds / 74 / 2;
}

/**
  * Same conversion as above, but in centimeters
**/
long microsecondsToCentimeters(long microseconds){
	return microseconds / 29 / 2;
}

/**
  * Get the distance (in centimeters) to the object in front of the front distance sensor
**/
long getFrontDistance(){
	// establish variables for duration of the ping, 
	// and the distance result in inches and centimeters:
	long duration;

	// The sensor is "triggered" by a pulse on the trigger pin for a duration >= 10 microseconds
	digitalWrite(FRONT_DISTANCE_TRIGGER, LOW);
	delayMicroseconds(2);
	digitalWrite(FRONT_DISTANCE_TRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(FRONT_DISTANCE_TRIGGER, LOW);

	// After the sensor is triggered, it sends us a HIGH pulse on the echo pin
	// that is equal to the number of microseconds it took for the sound echo to
	// strike the object in front of it and then return back to the sensor
	return microsecondsToCentimeters(pulseIn(FRONT_DISTANCE_ECHO, HIGH));
}

/**
  * Get the distance (in centimeters) to the object in front of the front distance sensor
**/
long getRearDistance(){
	// establish variables for duration of the ping, 
	// and the distance result in inches and centimeters:
	long duration;

	// The sensor is "triggered" by a pulse on the trigger pin for a duration >= 10 microseconds
	digitalWrite(REAR_DISTANCE_TRIGGER, LOW);
	delayMicroseconds(2);
	digitalWrite(REAR_DISTANCE_TRIGGER, HIGH);
	delayMicroseconds(10);
	digitalWrite(REAR_DISTANCE_TRIGGER, LOW);

	// After the sensor is triggered, it sends us a HIGH pulse on the echo pin
	// that is equal to the number of microseconds it took for the sound echo to
	// strike the object in front of it and then return back to the sensor
	return microsecondsToCentimeters(pulseIn(REAR_DISTANCE_ECHO, HIGH));
}

/**************************** SERVO FUNCTIONS ****************************/





/**************************** MOTOR FUNCTIONS ****************************/


// Wait to return until a certain number of ticks have occured on a motor
void waitForTickCount(int motor, int ticks){
	// Set a timeout amount
	int timeout = 2000; // 2 seconds will cause a timeout
	int elapsedTime = 0;
	int initialTickCount = getMagnetCount(motor);
	while((getMagnetCount(motor) - initialTickCount < ticks)){
		delay(10);
		elapsedTime += 10;
		if(elapsedTime > timeout){
			Serial.println("# AGER is stuck while turning");
			return; // force a return
		}
	}
	return;
}

int ticksRequiredForAngle(double angle){
	return round((angle - 2.10938) / 19.2919) + 1;
}

/**
  * Let zhe bot
**/
void travelToAngle(double angle, int direction){
	int motor;
	int initialTickCount;
	int tickCount = 0;
	int ticksRequired = ticksRequiredForAngle(angle);

	if(direction == LEFT){
		motor = RIGHT;
	} else if(direction == RIGHT){
		motor = LEFT;
	}

	initialTickCount = getMagnetCount(motor);

	Serial.print("# Traveling to angle: ");
	Serial.println(angle);
	Serial.println(ticksRequiredForAngle(angle));

	setMotorParameters(motor, DIR_FORWARD, BRAKE_OFF, 255);
	waitForTickCount(motor, ticksRequiredForAngle(angle));

	setMotorParameters(motor, DIR_FORWARD, BRAKE_ON, 0);
}

void travelToAngle(double angle){
	if(angle >= 0){
		travelToAngle(angle, LEFT);
	} else if(angle < 0){
		travelToAngle(-1 * angle, RIGHT);
	}
}




/**
  * Go forward distance number of centimeters
**/
void travelDistance(int desiredDistance, int direction){
	int initialLeftMagnetCount = getMagnetCount(LEFT);
	int initialRightMagnetCount = getMagnetCount(RIGHT);
	int totalLeftTicks = 0;
	int totalRightTicks = 0;
	int totalDistance = 0;
	int leftVoltage = 237;
	int rightVoltage = 255;
	int deltaResolution = 10;
	int diff;
	int elapsedTime = 0; // in ms
	int ticksInLastSecond = 0;
	int oppositeDirection;
	if(direction == DIR_FORWARD){
		Serial.print("# Going forward ");
		Serial.println(desiredDistance);
		oppositeDirection = DIR_BACKWARDS;
	} else {
		Serial.print("# Going backward ");
		Serial.println(desiredDistance);
		oppositeDirection = DIR_FORWARD;
	}
	Serial.println(desiredDistance);
	while(totalDistance < desiredDistance){
		Serial.print(leftVoltage);
		Serial.print(rightVoltage);
		Serial.print(totalLeftTicks);
		Serial.println(totalRightTicks);
		setMotorParameters(LEFT, direction, BRAKE_OFF, leftVoltage);
		setMotorParameters(RIGHT, direction, BRAKE_OFF, rightVoltage);
		delay(100);
		if(elapsedTime % 1000 == 0){
			ticksInLastSecond = 0;
		} else {
			ticksInLastSecond += totalLeftTicks - (getMagnetCount(LEFT) - initialLeftMagnetCount);
		}
		totalLeftTicks = getMagnetCount(LEFT) - initialLeftMagnetCount;
		totalRightTicks = getMagnetCount(RIGHT) - initialRightMagnetCount;
		diff = abs(totalLeftTicks - totalRightTicks);
/*
		if(totalLeftTicks > totalRightTicks){
			if(leftVoltage >= 130){
				leftVoltage -= deltaResolution * diff;
			} else if(rightVoltage <= (255 - deltaResolution)){
				rightVoltage += deltaResolution * diff;
			}
		} else if (totalRightTicks > totalLeftTicks){
			if(rightVoltage >= 130){
				rightVoltage -= deltaResolution * diff;
			} else if(leftVoltage <= (255 - deltaResolution)){
				leftVoltage += deltaResolution * diff;
			}
		}
*/
		totalDistance = (double)(totalLeftTicks + totalRightTicks) / (double)2.0 * DISTANCE_PER_MAGNET_TICK;
		elapsedTime += 100;
		if(((elapsedTime + 100) % 1000 == 0) && (ticksInLastSecond == 0)){
			Serial.println("# Ager is stuck while going forward");
			// Attempt to get un-stick by going back a bit and turning right
			travelDistance(20, oppositeDirection);
			travelToAngle(90);
		}
	}
	setMotorParameters(LEFT, direction, BRAKE_ON, 0);
	setMotorParameters(RIGHT, direction, BRAKE_ON, 0);
}

void setMotorCalibrationData(int motor, int direction, double slope, double intercept, double minimumVoltage){
	if(motor == LEFT){
		motor = 0;
	} else if(motor == RIGHT){
		motor = 1;
	}
	if(direction == DIR_FORWARD){
		direction = 0;
	} else if(direction == DIR_BACKWARDS){
		direction = 1;
	}
	MOTOR_CALIBRATION[motor][direction][0] = slope;
	MOTOR_CALIBRATION[motor][direction][1] = intercept;	
	MOTOR_CALIBRATION[motor][direction][2] = minimumVoltage;
}

void getMotorCalibrationData(int motor, int direction, double *slope, double *intercept, double *minimumVoltage){
	if(motor == LEFT){
		motor = 0;
	} else if(motor == RIGHT){
		motor = 1;
	}
	if(direction == DIR_FORWARD){
		direction = 0;
	} else if(direction == DIR_BACKWARDS){
		direction = 1;
	}
	*slope = MOTOR_CALIBRATION[motor][direction][0];
	*intercept = MOTOR_CALIBRATION[motor][direction][1];
	*minimumVoltage = MOTOR_CALIBRATION[motor][direction][2];
}

int indexToDirection(int index){
	if(index == 0){
		return DIR_FORWARD;
	} else if(index == 1){
		return DIR_BACKWARDS;
	}
}

void printDirection(int direction){
	if(direction == DIR_FORWARD){
		Serial.print("Forward ");
	} else if(direction == DIR_BACKWARDS){
		Serial.print("Backward ");
	}
}

void printMotor(int motor){
	if(motor == LEFT){
		Serial.print("Left ");
	} else if(motor == RIGHT){
		Serial.print("Right ");
	}
}

int getMagnetCount(int motor){
	if(motor == LEFT){
		return leftMagnetCount;
	} else if(motor == RIGHT){
		return rightMagnetCount;
	}
}

// Take in 2 arrays and print them in the form (x1, y1), (x2, y2), ...
void printTuples(double *x, double *y, int size){
	for(int i = 0; i < size; i++){
		Serial.print("(");
		Serial.print(x[i], 3);
		Serial.print(",");
		Serial.print(y[i], 3);
		Serial.print("),");
	}
	Serial.println();
}

/**
  * Figure out the correlation between motor voltage and speed as measured emperically by the magnet sensors
  * I'm not going to downplay the fact that this function is a bit of a hack
**/
void calibrateMotors(){
	int initialMagnetCount;
	int finalMagnetCount;
	int postBrakeTickCount;
	double rps;
	int sampleSize = (255 - (17 * 6)) / 17 + 1;
	double x[sampleSize];
	double y[sampleSize];
	int i;
	double slope;
	double intercept;
	int voltage;
	int minimumVoltage;
	int tickCountAfterBrake;

	int MOTORS[2] = {LEFT, RIGHT};
	int DIRECTIONS[2] = {DIR_FORWARD, DIR_BACKWARDS};

	for(int motor = 0; motor < 2; motor++){
		for(int direction = 0; direction < 2; direction++){
			Serial.print("# Calibrating ");
			printDirection(indexToDirection(direction));
			printMotor(motor);
			Serial.println(":");
			i = 0;
			for(voltage = 17 * 15; voltage < 256; voltage += 17){ // change to 17 * 6
				Serial.print("# Voltage at ");
				Serial.print(voltage);
				Serial.print(": ");
				initialMagnetCount = getMagnetCount(motor);
				setMotorParameters(motor, indexToDirection(direction), BRAKE_OFF, voltage);
				delay(2 * 1000);
				finalMagnetCount = getMagnetCount(motor);
				setMotorParameters(motor, indexToDirection(direction), BRAKE_ON, 0);
				delay(1000);
				postBrakeTickCount = getMagnetCount(motor) - finalMagnetCount;
				rps = ((float)(finalMagnetCount - initialMagnetCount) / (double)SPEED_ENCODER_RESOLUTION) / 2.0;
				Serial.print("# Revs per Second = ");
				Serial.println(rps);
				if(rps > 0){
					if(i == 0){
						minimumVoltage = voltage;
					}
					x[i] = voltage;
					y[i] = rps;
					i++;
				}
			}
			slope = lineFitSlope(x, y, i);
			intercept = lineFitYIntercept(x, y, i);
			setMotorCalibrationData(
				motor,
				indexToDirection(direction),
				slope,
				intercept,
				minimumVoltage
			);
			Serial.print("# Slope = ");
			Serial.println(slope, 6);
			Serial.print("# Intercept = ");
			Serial.println(intercept, 6);
			Serial.print("# Minimum Voltage = ");
			Serial.println(minimumVoltage);
			Serial.print("# Post-Brake Tick Count: ");
			Serial.println(postBrakeTickCount);
			Serial.println("# Full Cartesian Data Points:");
			Serial.print("# ");
			printTuples(x, y, i);
		}
	}
}

	void printCalibrationData(){
		//for(int motor = 0; motor < 2; motor)
	}


void setWheelRPM(int wheel, int rpm){

}

/**
  * Set all of the parameters for the left motor channel all at once
**/
void setMotorParameters(int motor, int direction, int brake, int voltage){
	if(motor == RIGHT){
		digitalWrite(RIGHT_DIR, direction);
		digitalWrite(RIGHT_BRAKE, brake);
		analogWrite(RIGHT_SPEED, voltage);
	} else if (motor == LEFT){
		digitalWrite(LEFT_DIR, direction);
		digitalWrite(LEFT_BRAKE, brake);
		analogWrite(LEFT_SPEED, voltage);
	}
}
