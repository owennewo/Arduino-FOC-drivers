

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

	virtual void init(SPIClass *_spi = &SPI, uint32_t clock);

	void clearFaults(bool nonLatched, bool latched);

	// status registers
	FaultStatus readFaultStatus();
	TempStatus readTemperatureStatus();
	SupplyStatus readVoltateSupplyStatus();
	FunctStatus readFunctionStatus();
	OTPStatus readOneTimeProgramStatus();
	ADCStatus readADCStatus();
	CPStatus readChargePumpStatus();
	DeviceID readDeviceID();

	// control registers
	void writeFaultClearRegister(FaultsClrRegister faultsClr);

	SupplyCfgRegister readVoltageSupplyConfigRegister();
	void writeVoltageSupplyConfigRegister(SupplyCfgRegister supplyCfg);

	ADCCfgRegister readADCConfigRegister();
	void writeADCConfigRegister(ADCCfgRegister adcCfg);

	PWMCfgRegister readPWMConfigRegister();
	void writePWMConfigRegister(PWMCfgRegister pwmCfg);

	SensorCfgRegister readSensorConfigRegister();
	void writeSensorConfigRegister(SensorCfgRegister sensorCfg);

	WDCfgRegister readWatchDogConfigRegister();
	void writeWatchDogConfigRegister(WDCfgRegister wdCfg);

	WDCfg2Register readWatchDogConfig2Register();
	void writeWatchDogConfig2Register(WDCfg2Register wdCfg2);

	IDriveCfgRegister readIDriveConfigRegister();
	void writeIDriveConfigRegister(IDriveCfgRegister idriveCfg);

	IDrivePreCfgRegister readIDrivePreConfigRegister();
	void writeIDrivePreConfigRegister(IDrivePreCfgRegister idrivePreCfg);

	TDriveSrcCfgRegister readTDriveSourceConfigRegister();
	void writeTDriveSourceConfigRegister(TDriveSrcCfgRegister tdriveSrcCfg);

	TDriveSinkCfgRegister readTDriveSinkConfigRegister();
	void writeTDriveSinkConfigRegister(TDriveSinkCfgRegister tdriveSinkCfg);

	DTCfgRegister readDeadTimeConfigRegister();
	void writeDeadTimeConfigRegister(DTCfgRegister dtCfg);

	CPCfgRegister readChargePumpConfigRegister();
	void writeChargePumpConfigRegister(CPCfgRegister cpCfg);

	CSAmpCfgRegister readCurrentSenseAmplifierConfigRegister();
	void writeCurrentSenseAmplifierConfigRegister(CSAmpCfgRegister csAmpCfg);

	CSAmpCfg2Register readCurrentSenseAmplifierConfig2Register();
	void writeCurrentSenseAmplifierConfig2Register(CSAmpCfg2Register csAmpCfg2);

	OTPProgRegister readOneTimeProgramRegister();
	void writeOneTimeProgramRegister(OTPProgRegister otpProg);

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
	Infineon6EDL7141Driver3PWM(int phA, int phB, int phC, int cs, bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET) : Infineon6EDL7141Driver(cs, currentLimit, nFault), BLDCDriver3PWM(phA, phB, phC, en) { enable_active_high = false; };
	virtual ~Infineon6EDL7141Driver3PWM(){};

	virtual void init(SPIClass *_spi = &SPI) override;
};

class Infineon6EDL7141Driver6PWM : public Infineon6EDL7141Driver, public BLDCDriver6PWM
{

public:
	Infineon6EDL7141Driver6PWM(int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int cs, bool currentLimit = false, int en = NOT_SET, int nFault = NOT_SET) : Infineon6EDL7141Driver(cs, currentLimit, nFault), BLDCDriver6PWM(phA_h, phA_l, phB_h, phB_l, phC_h, phC_l, en) { enable_active_high = false; };
	virtual ~Infineon6EDL7141Driver6PWM(){};

	virtual void init(SPIClass *_spi = &SPI) override;
};

#endif
