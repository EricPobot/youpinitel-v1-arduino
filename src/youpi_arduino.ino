/*
    Arduino interface for Youpi robotic arm.

    Mimics AX12 servos protocol for controlling a Youpi robotic arm over
    a USB serial link.

    Authors:
        Damien Profeta and Eric Pascual from POBOT (http://www.pobot.org)

    Version:
        october 2015

    Visit:
        - for details on Youpi robotic arm :
            http://youpi.forler.ch/
        - for the project using this interface :
            http://www.pobot.org/Le-mariage-des-annees-80-et-2010.html
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <limits.h>

#define WAITINGFIRSTBYTE 0
#define WAITINGSECONDBYTE 1
#define WAITINGID 2
#define WAITINGLENGTH 3
#define WAITINGINSTRUCTION 4
#define WAITINGPARAMETERS 5
#define WAITINGCHECKSUM 6

#define PING 1
#define READ_DATA 2
#define WRITE_DATA 3
#define REG_WRITE 4
#define ACTION 5
#define RESET 6
#define SYNC_WRITE 0x83

#define ModelNumber 0
#define FirmwareVersion 2
#define Id 3
#define BaudRate 4
#define ReturnDelay 5
#define CWAngleLimit 6
#define CCWAngleLimit 8
#define TemperatureLimit 11
#define LowVoltageLimit 12
#define HighVoltageLimit 13
#define MaxTorque 14
#define StatusReturnLevel 16
#define AlarmLED 17
#define AlarmShutdown 18
#define DownCalibration 20
#define UpCalibration 22
#define TorqueEnable 24
#define LED 25
#define CWComplianceMargin 26
#define CCWComplianceMargin 27
#define CWComplianceSlope 28
#define CCWComplianceSlope 29
#define GoalPosition 30
#define MovingSpeed 32
#define TorqueLimit 34
#define CurrentPosition 36
#define CurrentSpeed 38
#define CurrentLoad 40
#define CurrentVoltage 42
#define CurrentTemperature 43
#define RegisteredInstruction 44
#define Moving 46
#define Lock 47
#define Punch 48

#define MoveToRefPos  49
#define Custom  50

#define MOTOR_BASE      0
#define MOTOR_SHOULDER  1
#define MOTOR_ELBOW     2
#define MOTOR_WRIST     3
#define MOTOR_WRIST_ROT 4
#define MOTOR_GRIPPER   5

#define MOTOR_COUNT     (MOTOR_GRIPPER + 1)

#define SENSOR_BASE      0
#define SENSOR_SHOULDER  1
#define SENSOR_ELBOW     2
#define SENSOR_WRIST     3
#define SENSOR_WRIST_ROT 4
#define SENSOR_GRIPPER   5

#define SENSOR_COUNT    (SENSOR_GRIPPER + 1)

const int D[MOTOR_COUNT + 2] =  {13, 12, 11, 10, 9, 8, 7, 6};
const int I[SENSOR_COUNT] =  {14, 19, 18, 17, 16, 15};

//unsigned long scheduleAt = 70000;
unsigned long long time = 0;
long timerstep = 1;

//Command parameters and state machine
unsigned char CommandReceptionState = WAITINGFIRSTBYTE;
unsigned char currentCommandId = 0;
unsigned char currentCommandLength = 0;
unsigned char currentCommandInstruction = 0;
unsigned char parametersStillToReceive = 0;
unsigned char currentParameters[10];
unsigned char currentParameterToFill = 0;

//to check -> more than 256 in value command
//less than 70000 to start the command

unsigned char regCache[MOTOR_COUNT][52] = {
    {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,0},
    {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,0},
    {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,0},
    {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,0},
    {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,0},
    {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0,0,0}
};

unsigned short getRegShort(unsigned char motor, unsigned char parameter) {
    return (((short)regCache[motor][parameter]) | ((short)(regCache[motor][parameter + 1]) << 8));
}

void setRegShort(unsigned char motor, unsigned char parameter, short Position) {
    regCache[motor][parameter] = (char)(Position & 0xFF);
    regCache[motor][parameter + 1] = (char)(Position >> 8);
}

unsigned short getMotorPosition(unsigned char motor) {
    return getRegShort(motor, CurrentPosition);
}

void setMotorPosition(unsigned char motor, long Position) {
    setRegShort(motor, CurrentPosition, Position/10);
}

long getMotorSpeed(unsigned char motor) {
    static long vitesse = 7000;
    //  return vitesse;
    int control = getRegShort(motor, MovingSpeed);
    if (control == 0)
        return vitesse;
    else
        return (vitesse*1023)/control;
}

unsigned short getMotorGoal(unsigned char motor) {
    return getRegShort(motor, GoalPosition);
}

void setMotorGoal(unsigned char motor, short Position) {
    setRegShort(motor, GoalPosition, Position);
}

// Replicate the state of Youpi arm for the position of the step motor
// and the direction of moving for each motor
long motor_position[MOTOR_COUNT] = {0,0,0,0,0,0};
signed char motor_direction[MOTOR_COUNT] = {0,0,0,0,0,0};

struct Command {
    unsigned long long scheduleAt;
    unsigned char order;    //which command to execute
    unsigned char value;    //parameter of the command if needed
    unsigned char id;       //id of the motor
    bool toStart;
};

struct RegisteredCommand {
    unsigned char id;
    unsigned char instruction;
    unsigned char numberOfParameters;
    unsigned char parameters[10];
};

//To be executed at the next action order
RegisteredCommand registeredCommand[6];
unsigned char numberOfRegisteredCommand = 0;

//Current running command
Command command[6];

unsigned long baudRate = 10000;


void parallelOutput(int number) {
    for( int i= 0;i<8; ++i) {
        digitalWrite(D[i], number%2);
        number = number >> 1;
    }
}

void all_motors_ccw() {
    parallelOutput(0xBF); delayMicroseconds(250);
    parallelOutput(0x3F); delayMicroseconds(1000);
}

void all_motors_cw() {
    parallelOutput(0x80); delayMicroseconds(250);
    parallelOutput(0x00); delayMicroseconds(1000);
}

void step_motor(unsigned char motor) {
    parallelOutput(0x40 + motor); delayMicroseconds(250);
    parallelOutput(0x00 + motor); delayMicroseconds(1000);
}

#define GRIPPER_OPEN_STEPS_MAX  7000

unsigned int gripper_open_steps = GRIPPER_OPEN_STEPS_MAX;

void open_gripper() {
    all_motors_cw();
    for (int i = 0; i < gripper_open_steps; i++) {
        step_motor(MOTOR_GRIPPER);
    }
}

void close_gripper() {
    unsigned char input_num = I[SENSOR_GRIPPER];

    gripper_open_steps = 0;

    all_motors_ccw();
    while (!digitalRead(input_num)) {
        step_motor(MOTOR_GRIPPER);
        gripper_open_steps++;
    }
}

void calibrate_joint(int motor, int inverted) {
    unsigned char input_num = I[motor];
    unsigned char start_bw = digitalRead(input_num);
    if (start_bw) {
        if (inverted) {
            all_motors_cw();
        } else {
            all_motors_ccw();
        }
    } else {
        if (inverted) {
            all_motors_ccw();
        } else {
            all_motors_cw();
        }
    }

    while (digitalRead(input_num) == start_bw) {
        step_motor(motor);
    }

    setRegShort(motor, CurrentPosition,0x1FF);
    setMotorGoal(motor,0x1FF);
    motor_position[motor]=5110;
}


void calibrate_gripper() {
    close_gripper();

    gripper_open_steps = GRIPPER_OPEN_STEPS_MAX;
    open_gripper();
}

void calibrate() {
    calibrate_joint(MOTOR_BASE, false);
    calibrate_joint(MOTOR_SHOULDER, false);
    calibrate_joint(MOTOR_ELBOW, true);
    calibrate_joint(MOTOR_WRIST, false);
    calibrate_joint(MOTOR_WRIST_ROT, false);

    calibrate_gripper();
    Serial.write("OK\n");
}

void setup() {
    for (int i = 0; i < 6; ++i) {
        command[i].scheduleAt = 9223372036854775807LL;
        command[i].toStart = true;
    }
    // set the digital pin as output:
    for (int i = 0; i< 8; ++i) {
        pinMode(D[i], OUTPUT);
    }
    // set the digital pin as output:
    for (int i = 0; i< 6; ++i) {
        pinMode(I[i], INPUT);
    }
    parallelOutput(0x47);
    parallelOutput(0x00);

    // initialize timer1
    cli();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 65535;;            // compare match register max
    TCCR1B |= (1 << CS10);    // no prescaler
    TCCR1B |= (1 << WGM12);   // clear on compare match
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    sei();
    Serial.begin(9600);

    calibrate();
}

char *REPLY_HEADER = "\xff\xff";

void processCommand() {
    //ugly hack -> we diminish the currentCommandId to have the id started at 0 but then the broadcast is not FE but FD…
    if (currentCommandId < 6 || currentCommandId == 0xFD) {
        unsigned char checksum ;
        bool willMove;
        switch(currentCommandInstruction) {
            case PING: 
                Serial.write(REPLY_HEADER);
                Serial.write(currentCommandId + 1);
                Serial.write(0x02); Serial.write(0);
                checksum = ~(currentCommandId + 1 + 2);
                Serial.write(checksum);
                break;

            case READ_DATA: 
                Serial.write(REPLY_HEADER);
                Serial.write(currentCommandId + 1);
                checksum = currentCommandId + 1;
                Serial.write(currentParameters[1] + 2);
                checksum += currentParameters[1] + 2;
                Serial.write(0x00);
                for (int i = currentParameters[0]; i<(currentParameters[0] + currentParameters[1]); ++i) {
                    Serial.write(regCache[currentCommandId][i]);
                    checksum += regCache[currentCommandId][i];
                }
                checksum = ~checksum;
                Serial.write(checksum);
                break;

            case WRITE_DATA: 
                willMove = false;
                for (int i = 0; i< currentCommandLength -3 ; ++i) {
                    int controlId = currentParameters[0] + i;
                    if (controlId == GoalPosition || controlId == GoalPosition + 1) {
                        willMove = true;
                    }
                    regCache[currentCommandId][controlId] = currentParameters[i + 1 ];
                }
                if (willMove) {
                    if ((getMotorGoal(currentCommandId) > getMotorPosition(currentCommandId)) && (motor_direction[currentCommandId] != 1)) {
                        command[currentCommandId].order = 0;
                        command[currentCommandId].value = 1;
                    } else if ((getMotorGoal(currentCommandId) < getMotorPosition(currentCommandId)) && (motor_direction[currentCommandId] != -1)) {
                        command[currentCommandId].order = 0;
                        command[currentCommandId].value = -1;
                    } else {
                        command[currentCommandId].order = 1;
                    }
                    command[currentCommandId].id = currentCommandId;
                    command[currentCommandId].scheduleAt = time + getMotorSpeed(command[currentCommandId].id);
                    command[currentCommandId].toStart = true;
                }
                Serial.write(REPLY_HEADER);
                Serial.write(currentCommandId + 1);
                Serial.write(0x02); Serial.write(0);
                checksum = ~(currentCommandId + 1 + 2);
                Serial.write(checksum);
                break;

            case REG_WRITE:          //Register the command
                registeredCommand[numberOfRegisteredCommand].id = currentCommandId;
                registeredCommand[numberOfRegisteredCommand].instruction = WRITE_DATA;
                registeredCommand[numberOfRegisteredCommand].numberOfParameters = currentCommandLength - 2;
                for (int i = 0; i< currentCommandLength -1 ; ++i) {
                    registeredCommand[numberOfRegisteredCommand].parameters[i] = currentParameters[i];
                }
                numberOfRegisteredCommand +=1;

                Serial.write(REPLY_HEADER);
                Serial.write(currentCommandId + 1);
                Serial.write(0x02); Serial.write(0);
                checksum = ~(currentCommandId + 1 + 2);
                Serial.write(checksum);
                break;

            case ACTION: 
                for (int commandIter = 0; commandIter< numberOfRegisteredCommand; ++commandIter) {
                    RegisteredCommand* aCommand = &registeredCommand[commandIter];
                    willMove = false;
                    for (int i = 0; i< aCommand->numberOfParameters - 1; ++i) {
                        int controlId = aCommand->parameters[0] + i;
                        if (controlId == 30 || controlId == 31) {
                            willMove = true;
                        }
                        regCache[aCommand->id][controlId] = aCommand->parameters[i + 1 ];
                    }
                    if (willMove) {
                        int commandId = aCommand->id;
                        if ((getMotorGoal(commandId) > getMotorPosition(commandId)) && (motor_direction[commandId] != 1)) {
                            command[commandId].order = 0;
                            command[commandId].value = 1;
                        } else if ((getMotorGoal(commandId) < getMotorPosition(commandId)) && (motor_direction[commandId] != -1)) {
                            command[commandId].order = 0;
                            command[commandId].value = -1;
                        } else {
                            command[commandId].order = 1;
                        }
                        command[commandId].id = commandId;
                        command[commandId].scheduleAt = time + getMotorSpeed(command[commandId].id);
                        command[commandId].toStart = true;
                    }
                }
                numberOfRegisteredCommand = 0;
                break;
        }
    }
}

void loop() {
    int inByte;
    // if we get a valid byte, read analog ins:
    if (Serial.available() > 0) {
        // get incoming byte:
        inByte = Serial.read();

        switch (CommandReceptionState) {
            case WAITINGFIRSTBYTE:
            case WAITINGSECONDBYTE: 
                if (inByte == 0xFF) {
                    CommandReceptionState += 1;
                } else {
                    CommandReceptionState = 0;
                }
                break;

            case WAITINGID: 
                currentCommandId = inByte - 1; // diminish the Id to have starting from 0
                CommandReceptionState += 1;
                break;

            case WAITINGLENGTH: 
                currentCommandLength = inByte;
                parametersStillToReceive = currentCommandLength -2;
                CommandReceptionState += 1;
                break;

            case WAITINGINSTRUCTION: 
                currentCommandInstruction = inByte;
                CommandReceptionState += 1;
                if (parametersStillToReceive == 0) {
                    CommandReceptionState += 1;
                }
                break;

            case WAITINGPARAMETERS: 
                currentParameters[currentParameterToFill] = inByte;
                currentParameterToFill += 1;
                parametersStillToReceive -=1;
                if ( parametersStillToReceive == 0) {
                    CommandReceptionState += 1;
                }
                break;

            case WAITINGCHECKSUM: 
                processCommand();
                CommandReceptionState = 0;
                currentParameterToFill = 0;
                break;

        }
    }
}

void executeCommand(unsigned char commandId) {
    if (command[commandId].order == 0) {
        if (command[commandId].toStart == true) {
            motor_direction[command[commandId].id] = command[commandId].value;
            int i;
            unsigned char output = 0x80;
            for (i = 0; i < 6; ++i) {
                if (motor_direction[i] == 1) {
                    output |= 1 << (i);
                }
            }
            parallelOutput(output);
            command[commandId].toStart = false;
            command[commandId].scheduleAt  = time + baudRate;
        } else {
            int i;
            unsigned char output = 0x00;
            for (i = 0; i < 6; ++i) {
                if (motor_direction[i] == 1) {
                    output |= 1 << (i);
                }
            }
            parallelOutput(output);
            command[commandId].toStart = true;

            command[commandId].order = 1;
            command[commandId].scheduleAt = time + getMotorSpeed(command[commandId].id);

        }
    } else if (command[commandId].order == 1) {
        if (command[commandId].toStart == true) {
            parallelOutput(0x40+command[commandId].id); //moteur id start at 1
            command[commandId].toStart = false;
            command[commandId].scheduleAt  = time + baudRate;
        } else {
            parallelOutput(0x00+command[commandId].id); //moteur id start at 1
            command[commandId].toStart = true;
            motor_position[command[commandId].id] += motor_direction[command[commandId].id];
            setMotorPosition(command[commandId].id, motor_position[command[commandId].id]);

            if (getMotorPosition(command[commandId].id) == getMotorGoal(command[commandId].id)) {
                command[commandId].scheduleAt  = 9223372036854775807LL;
            } else {
                command[commandId].scheduleAt = time + getMotorSpeed(command[commandId].id);
            }

        }
    }
}


// timer compare interrupt service routine 
ISR(TIMER1_COMPA_vect)          {
    cli();
    time += OCR1A;
    bool commanddone = false;
    for (int i=0;i < 6; ++i) {
        if (command[i].toStart == false) {
            executeCommand(i);
            commanddone = true;
            break;
        }
    }
    if (commanddone == false) {
        unsigned long long nexttime = command[0].scheduleAt;
        int nextcommand = 0;
        for (int i=1;i < 6; ++i) {
            if ( command[i].scheduleAt< nexttime) {
                nexttime = command[i].scheduleAt;
                nextcommand = i;
            }
        }

        if (command[nextcommand].scheduleAt < time) {
            executeCommand(nextcommand);
        }
    }

    unsigned long long nexttime = command[0].scheduleAt;
    for (int i=1;i < 6; ++i) {
        if ( command[i].scheduleAt< nexttime) {
            nexttime = command[i].scheduleAt;
        }
    }
    unsigned int nextInterruptTime = 65535;
    if (nexttime < (time + baudRate))
        nextInterruptTime = baudRate;
    else if (nexttime < (time + nextInterruptTime))
        nextInterruptTime = (nexttime - time);

    OCR1A = nextInterruptTime;

    sei();
}
