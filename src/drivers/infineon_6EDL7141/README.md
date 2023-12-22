
# Infineon 6EDL7141 driver

The 6EDL7141 is an integrated FET, integrated current sensing 3-phase BLDC driver IC from Infineon, including all protections and many cool configuration optons. See https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/ for more information.

This driver includes a 6EDL7141 SPI driver, and specific subclasses for SimpleFOCs BLDCDriver3PWM and BLDCDriver6PWM generic drivers. The code is designed to be "Ardunio compatible" and should work with any of the hardware architectures supported by SimpleFOC.

## Hardware setup

To use the 6EDL7141 you will have to connect the following:

- GND
- SPI MOSI
- SPI MISO
- SPI CLK
- SPI nCS
- INHA - connect to motor PWM pin
- INHB - connect to motor PWM pin
- INHC - connect to motor PWM pin
- INLA - connect to motor PWM pin for 6-PWM, or to digital out (or pull up to VCC) for 3-PWM operation
- INLB - connect to motor PWM pin for 6-PWM, or to digital out (or pull up to VCC) for 3-PWM operation
- INLC - connect to motor PWM pin for 6-PWM, or to digital out (or pull up to VCC) for 3-PWM operation
- EN_DRV - enable gate drivers

Optionally, but probably useful:

- NFAULT - digital in, active low
- VSENSE/NBRAKE - initially controls 3.3v/5V output on LDO then allows MCU to brake
For current sensting:

Note that the Current Sense Gain can be configured analog through resistors (on CS_GAIN pin), or through SPI

- CSOA - connect to analog in
- CSOB - connect to analog in
- CSOC - connect to analog in

## Usage

Usage is quite easy, especially if you already know SimpleFOC. See also the [examples](https://github.com/simplefoc/Arduino-FOC-drivers/examples/drivers/drv8316/)

```c++
#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>
#include "SimpleFOCDrivers.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"

BLDCMotor motor = BLDCMotor(11);
Infineon6EDL7141Driver6PWM driver = Infineon6EDL7141Driver6PWM(A3,0,A4,1,2,6,7,false);          // MKR1010 6-PWM

//... normal simpleFOC init code...
```

Or, for 3-PWM:

```c++
#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>
#include "SimpleFOCDrivers.h"
#include "drivers/infineon_6EDL7141/infineon_6EDL7141.h"

BLDCMotor motor = BLDCMotor(11);
Infineon6EDL7141Driver3PWM driver = Infineon6EDL7141Driver3PWM(A3,A4,2,7,false);          	  // MKR1010 3-PWM
// these are examples, for 3-PWM you could use any output pins as the enable pins.
#define ENABLE_A 0
#define ENABLE_B 1
#define ENABLE_C 6

void setup() {


	pinMode(ENABLE_A, OUTPUT);
	digitalWrite(ENABLE_A, 1); // enable
	pinMode(ENABLE_B, OUTPUT);
	digitalWrite(ENABLE_B, 1); // enable
	pinMode(ENABLE_C, OUTPUT);
	digitalWrite(ENABLE_C, 1); // enable


//... normal simpleFOC init code...


}

```

You can use the driver's features. In general you can do this at any time, but certain features only make sense at setup-time (e.g. setting the PWM mode, which is handled automatically by the Infineon6EDL7141Driver3PWM class, or setting the current limit, which you generally want to get done before applying power to the motor).

Driver usage, there are 6 status registers and they are returned as struts which have helper functions:

```c++
	void printFaultStatus()
{
  FaultStatus status = driver.readFaultStatus();
  Serial.println("#### readFaultStatus");
  Serial.print("isFault: ");
  Serial.println(status.isFault());
  Serial.print("isCurrentSenseOverCurrent: ");
  Serial.println(status.isCurrentSenseOverCurrent());
  Serial.print("isCurrentSenseOverCurrentPhaseA: ");
  Serial.println(status.isCurrentSenseOverCurrentPhaseA());
  Serial.print("isCurrentSenseOverCurrentPhaseB: ");
  Serial.println(status.isCurrentSenseOverCurrentPhaseB());
  Serial.print("isCurrentSenseOverCurrentPhaseC: ");
  Serial.println(status.isCurrentSenseOverCurrentPhaseC());
  Serial.print("isChargePump: ");
  Serial.println(status.isChargePump());
  Serial.print("isDVDDOverCurrent: ");
  Serial.println(status.isDVDDOverCurrent());
  Serial.print("isDVDDUnderVoltage: ");
  Serial.println(status.isDVDDUnderVoltage());
  Serial.print("isDVDDOverVoltage: ");
  Serial.println(status.isDVDDOverVoltage());
  Serial.print("isBuckOverCurrentProtection: ");
  Serial.println(status.isBuckOverCurrentProtection());
  Serial.print("isOverTemperatureShutdown: ");
  Serial.println(status.isOverTemperatureShutdown());
  Serial.print("isOverTemperatureWarning: ");
  Serial.println(status.isOverTemperatureWarning());
  Serial.print("isLockedRotor: ");
  Serial.println(status.isLockedRotor());
  Serial.print("isWatchdog: ");
  Serial.println(status.isWatchdog());
  Serial.print("isOTPMemory: ");
  Serial.println(status.isOTPMemory());
}

void printTemperatureStatus()
{
  TempStatus status = driver.readTemperatureStatus();
  Serial.println("#### readTemperatureStatus");
  Serial.print("getTemperatureInCelsius: ");
  Serial.println(status.getTemperatureInCelsius());
}

void printVoltageSupplyStatus()
{
  SupplyStatus status = driver.readVoltateSupplyStatus();
  Serial.println("#### readVoltateSupplyStatus");
  Serial.print("isChargePumpLowSideUnderVoltage: ");
  Serial.println(status.isChargePumpLowSideUnderVoltage());
  Serial.print("isChargePumpHighSideUnderVoltage: ");
  Serial.println(status.isChargePumpHighSideUnderVoltage());
  Serial.print("isDVDDUnderVoltage: ");
  Serial.println(status.isDVDDUnderVoltage());
  Serial.print("isDVDDOverVoltage: ");
  Serial.println(status.isDVDDOverVoltage());
  Serial.print("isVDDBUnderVoltage: ");
  Serial.println(status.isVDDBUnderVoltage());
  Serial.print("isVDDBOverVoltage: ");
  Serial.println(status.isVDDBOverVoltage());
  Serial.print("getPVDDVoltage: ");
  Serial.println(status.getPVDDVoltage());
}

void printFunctionStatus()
{
  FunctStatus status = driver.readFunctionStatus();
  Serial.println("#### readFunctionStatus");
  Serial.print("getHallStatePhaseA: ");
  Serial.println(status.getHallStatePhaseA());
  Serial.print("getHallStatePhaseB: ");
  Serial.println(status.getHallStatePhaseB());
  Serial.print("getHallStatePhaseC: ");
  Serial.println(status.getHallStatePhaseC());
  Serial.print("isHallPolarityEqual: ");
  Serial.println(status.isHallPolarityEqual());
  Serial.print("isDVDDSetPoint5V: ");
  Serial.println(status.isDVDDSetPoint5V());
  Serial.print("getCurrentSenseGain: ");
  Serial.println(status.getCurrentSenseGain());
}

void printOneTimeProgramStatus()
{
  OTPStatus status = driver.readOneTimeProgramStatus();
  Serial.println("#### readOneTimeProgramStatus");
  Serial.print("isOneTimeProgramUsed: ");
  Serial.println(status.isOneTimeProgramUsed());
  Serial.print("isOneTimeProgramPassed: ");
  Serial.println(status.isOneTimeProgramPassed());
  Serial.print("isOneTimeProgramBlocked: ");
  Serial.println(status.isOneTimeProgramBlocked());
  Serial.print("isOneTimeProgramFailed: ");
  Serial.println(status.isOneTimeProgramFailed());
};

void printADCStatus()
{
  ADCStatus status = driver.readADCStatus();
  Serial.println("#### readADCStatus");
  Serial.print("isReady:");
  Serial.println(status.isReady());
  Serial.print("getValue:");
  Serial.println(status.getValue());
}

void printCPStatus()
{
  CPStatus status = driver.readChargePumpStatus();
  Serial.println("#### readChargePumpStatus");
  Serial.print("getChargePumpHighSideVoltage: ");
  Serial.println(status.getChargePumpHighSideVoltage());
  Serial.print("getChargePumpLowSideVoltage: ");
  Serial.println(status.getChargePumpLowSideVoltage());
}

```

There are 16 control registers.  It is common to read the register, set one or more bitfields, then write the register.

As an example:
```c++
void updateDeadTime()
{
  DTCfgRegister reg = driver.readDeadTimeConfigRegister();
  reg.setDeadTimeRise(1000);
  reg.setDeadTimeFall(1000);
  driver.writeDeadTimeConfigRegister(reg);
}
```

Setting options can be conveniently done via the provided setter methods. All documented registers and options are available via the driver, and the option values can be accessed via enums.

It is possible to overwrite the defaults for most registers using OTP.  Be cafeful - read the docs as it is possible to brick your driver if done wrong.

Leave at least 400ns delay between reading and/or writing options to ensure you don't talk to the DRV8316 too quickly:


### Current sensing

TODO...

### Current limiting

TODO...

