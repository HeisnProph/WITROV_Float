#ifndef FLOAT_WEBSERVER_H
#define FLOAT_WEBSERVER_H


void setup_float();
void loop_float();

void handleSensor();
void handleUltrasonic();
void handleStateMachine();
void handleMissionSubstates();
void handlePID();
void handleMotor();
void handleDataLogging();
String processor(const String& var);
void stepperTask(void *pvParameters);


#endif