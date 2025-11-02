#pragma once

#include "Arduino.h"
#include "main.h"

// Serial console task for FreeRTOS
void taskSerialConsole(void *pvParameters);

// Command handlers
void handleHelpCommand();
void handleStatusCommand();
void handleDeviceStatusCommand();
void handleReadAllInputsCommand();
void handleReadInputCommand(const char* args);
void handleReadAllOutputsCommand();
void handleSetOutputCommand(const char* args);
void handleRelayCommand(const char* args);
void handleTestSequenceCommand();
void handleMonitorInputsCommand();
void handleDiagnosticsCommand();

// Helper functions
void printInputStatus(uint8_t channel);
void printAllInputs();
void printAllOutputs();
void printRelayStatus(const char* name, uint8_t device, OutputChannel channel);
void parseCommand(const char* command);
