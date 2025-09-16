#ifndef FACTORY_RESET_H
#define FACTORY_RESET_H

// Function declarations
void setResetComplete();
void getFactoryResetStatusFromNVS();
void setFactoryResetStatusInNVS(bool status);
void factoryResetDevice();

#endif