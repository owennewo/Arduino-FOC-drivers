

#ifndef SIMPLEFOC_INFINEON_6EDL7141
#define SIMPLEFOC_INFINEON_6EDL7141

#include "Arduino.h"
#include <SPI.h>
#include <drivers/BLDCDriver3PWM.h>
#include <drivers/BLDCDriver6PWM.h>

#include "./infineon_6EDL7141_registers.h"

class Infineon6EDL7141Driver
{

public:
	Infineon6EDL7141Driver(int cs, bool currentLimit = false, int nFault = NOT_SET) : currentLimit(currentLimit), cs(cs), nFault(nFault), spi(&SPI), settings(1000000, MSBFIRST, SPI_MODE1){};
	virtual ~Infineon6EDL7141Driver(){};

	virtual void init(SPIClass *_spi = &SPI, uint32_t clock = 1000000);

	void clearFaults(bool nonLatched, bool latched);

	// status registers
	FaultAndWarningStatus readFaultAndWarningStatus();
	TemperatureStatus readTemperatureStatus();
	PowerSupplyStatus readPowerSupplyStatus();
	FunctionalStatus readFunctionalStatus();
	OTPStatus readOTPStatus();
	ADCStatus readADCStatus();
	ChargePumpsStatus readChargePumpsStatus();
	DeviceID readDeviceID();

	// control registers
	void writeFaultClear(FaultsClear clear);

	PowerSupplyConfiguration readPowerSupplyConfiguration();
	void writePowerSupplyConfiguration(PowerSupplyConfiguration configuration);

	ADCConfiguration readADCConfiguration();
	void writeADCConfiguration(ADCConfiguration configuration);

	PWMConfiguration readPWMConfiguration();
	void writePWMConfiguration(PWMConfiguration configuration);

	SensorConfiguration readSensorConfiguration();
	void writeSensorConfiguration(SensorConfiguration configuration);

	WatchDogConfiguration readWatchDogConfiguration();
	void writeWatchDogConfiguration(WatchDogConfiguration configuration);

	WatchDogConfiguration2 readWatchDogConfiguration2();
	void writeWatchDogConfiguration2(WatchDogConfiguration2 configuration);

	GateDriverCurrentConfiguration readGateDriverCurrentConfiguration();
	void writeGateDriverCurrentConfiguration(GateDriverCurrentConfiguration configuration);

	PreChargeGateDriverCurrentConfiguration readPreChargeGateDriverCurrentConfiguration();
	void writePreChargeGateDriverCurrentConfiguration(PreChargeGateDriverCurrentConfiguration configuration);

	GateDriverSourcingTimingConfiguration readGateDriverSourcingTimingConfiguration();
	void writeGateDriverSourcingTimingConfiguration(GateDriverSourcingTimingConfiguration configuration);

	GateDriverSinkingTimingConfiguration readGateDriverSinkingTimingConfiguration();
	void writeGateDriverSinkingTimingConfiguration(GateDriverSinkingTimingConfiguration configuration);

	DeadTimeConfiguration readDeadTimeConfiguration();
	void writeDeadTimeConfiguration(DeadTimeConfiguration configuration);

	ChargePumpConfiguration readChargePumpConfiguration();
	void writeChargePumpConfiguration(ChargePumpConfiguration configuration);

	CurrentSenseAmplifierConfiguration readCurrentSenseAmplifierConfiguration();
	void writeCurrentSenseAmplifierConfiguration(CurrentSenseAmplifierConfiguration configuration);

	CurrentSenseAmplifierConfiguration2 readCurrentSenseAmplifierConfiguration2();
	void writeCurrentSenseAmplifierConfiguration2(CurrentSenseAmplifierConfiguration2 configuration);

	OTPProgram readOTPProgram();
	void writeOTPProgram(OTPProgram otpProg);

	void clearFault(); // TODO check for fault condition methods

private:
	uint16_t readSPI(uint8_t addr);
	uint16_t writeSPI(uint8_t addr, uint16_t data);
	bool getParity(uint16_t data);

	bool currentLimit;
	int cs;
	int nFault;
	SPIClass *spi;
	SPISettings settings;
};

class Infineon6EDL7141Driver3PWM : public Infineon6EDL7141Driver, public BLDCDriver3PWM
{

public:
	Infineon6EDL7141Driver3PWM(int phA, int phB, int phC, int cs, bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET) : Infineon6EDL7141Driver(cs, currentLimit, nFault), BLDCDriver3PWM(phA, phB, phC, en) { enable_active_high = true; };
	virtual ~Infineon6EDL7141Driver3PWM(){};

	virtual void init(SPIClass *_spi = &SPI, uint32_t clock = 1000000) override;
};

class Infineon6EDL7141Driver6PWM : public Infineon6EDL7141Driver, public BLDCDriver6PWM
{

public:
	Infineon6EDL7141Driver6PWM(int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int cs, bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET) : Infineon6EDL7141Driver(cs, currentLimit, nFault), BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, en) { enable_active_high = true; };
	virtual ~Infineon6EDL7141Driver6PWM(){};

	virtual void init(SPIClass *_spi = &SPI, uint32_t clock = 1000000) override;
};

#endif
