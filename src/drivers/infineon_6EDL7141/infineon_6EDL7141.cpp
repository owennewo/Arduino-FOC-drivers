

#include "./infineon_6EDL7141.h"

void Infineon6EDL7141Driver3PWM::init(SPIClass *_spi, uint32_t clock)
{
	Infineon6EDL7141Driver::init(_spi, clock);
	delayMicroseconds(1);
	PWMCfgRegister pwmConfig = Infineon6EDL7141Driver::readPWMConfigRegister();
	pwmConfig.setPWMMode(PWMMode::PWM3_Mode);
	Infineon6EDL7141Driver::writePWMConfigRegister(pwmConfig);
	BLDCDriver3PWM::init();
};

void Infineon6EDL7141Driver6PWM::init(SPIClass *_spi, uint32_t clock)
{
	Infineon6EDL7141Driver::init(_spi, clock);
	delayMicroseconds(1);
	PWMCfgRegister pwmConfig = Infineon6EDL7141Driver::readPWMConfigRegister();
	pwmConfig.setPWMMode(PWMMode::PWM6_Mode);
	Infineon6EDL7141Driver::writePWMConfigRegister(pwmConfig);
	BLDCDriver6PWM::init();
};

/*
 * SPI setup:
 *
 *  capture on falling, propagate on rising =
 *  MSB first. clock idels low therefore this is a MODE1 SPI
 *
 * 	24 bit words
 * 	 outgoing: R/W:1 addr:7 parity:0 data:16
 * 	 incoming: status:8 data:16
 *
 * 	 on reads, incoming data is content of register being read
 * 	 on writes, incomnig data is content of register being written
 *
 *
 */

void handleInterrupt()
{
}

void Infineon6EDL7141Driver::init(SPIClass *_spi, uint32_t clock)
{
	spi = _spi;
	settings = SPISettings(clock, MSBFIRST, SPI_MODE1);

	// setup pins
	pinMode(cs, OUTPUT);
	digitalWrite(cs, HIGH); // switch off

	// SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();

	if (_isset(nFault))
	{
		pinMode(nFault, INPUT);
		// TODO add interrupt handler on the nFault pin if configured
		// add configuration for how to handle faults... idea: interrupt handler calls a callback, depending on the type of fault
		// consider what would be a useful configuration in practice? What do we want to do on a fault, e.g. over-temperature for example?

		// attachInterrupt(digitalPinToInterrupt(nFault), handleInterrupt, PinStatus::FALLING);
	}
};

uint16_t Infineon6EDL7141Driver::readSPI(uint8_t addr)
{
	spi->beginTransaction(settings);
	spi->transfer(addr);
	uint16_t result = spi->transfer16(0x0);
	spi->endTransaction();
	return result;
}

uint16_t Infineon6EDL7141Driver::writeSPI(uint8_t addr, uint16_t value)
{
	spi->beginTransaction(settings);
	spi->transfer(addr | 0b10000000); // add WRITE bit
	uint16_t result = spi->transfer16(value);
	spi->endTransaction();
	return result;
}

FaultStatus Infineon6EDL7141Driver::readFaultStatus()
{
	FaultStatus data;
	uint16_t result = readSPI(FAULT_ST_ADDR);
	data.reg = result;
	return data;
}

TempStatus Infineon6EDL7141Driver::readTemperatureStatus()
{
	TempStatus data;
	uint16_t result = readSPI(TEMP_ST_ADDR);
	data.reg = result;
	return data;
}

SupplyStatus Infineon6EDL7141Driver::readVoltateSupplyStatus()
{
	SupplyStatus data;
	uint16_t result = readSPI(SUPPLY_ST_ADDR);
	data.reg = result;
	return data;
}

FunctStatus Infineon6EDL7141Driver::readFunctionStatus()
{
	FunctStatus data;
	uint16_t result = readSPI(FUNCT_ST_ADDR);
	data.reg = result;
	return data;
}

OTPStatus Infineon6EDL7141Driver::readOneTimeProgramStatus()
{
	OTPStatus data;
	uint16_t result = readSPI(OTP_ST_ADDR);
	data.reg = result;
	return data;
}

ADCStatus Infineon6EDL7141Driver::readADCStatus()
{
	ADCStatus data;
	uint16_t result = readSPI(ADC_ST_ADDR);
	data.reg = result;
	return data;
}

CPStatus Infineon6EDL7141Driver::readChargePumpStatus()
{
	CPStatus data;
	uint16_t result = readSPI(CP_ST_ADDR);
	data.reg = result;
	return data;
}

DeviceID Infineon6EDL7141Driver::readDeviceID()
{
	DeviceID data;
	uint16_t result = readSPI(DEVICE_ID_ADDR);
	data.reg = result;
	return data;
}

/**
 * Control Registers
 */

void Infineon6EDL7141Driver::writeFaultClearRegister(FaultsClrRegister faultsClr)
{
	uint16_t result = writeSPI(FAULTS_CLR_ADDR, faultsClr.reg);
}

void Infineon6EDL7141Driver::clearFaults(bool nonLatched = true, bool latched = true)
{
	FaultsClrRegister data;
	data.reg = 0;
	data.setClearNonLatchedFaults(nonLatched);
	data.setClearLatchedFaults(latched);
	writeFaultClearRegister(data);
}

SupplyCfgRegister Infineon6EDL7141Driver::readVoltageSupplyConfigRegister()
{
	SupplyCfgRegister data;
	uint16_t result = readSPI(SUPPLY_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeVoltageSupplyConfigRegister(SupplyCfgRegister supplyCfg)
{
	uint16_t result = writeSPI(SUPPLY_CFG_ADDR, supplyCfg.reg);
}

ADCCfgRegister Infineon6EDL7141Driver::readADCConfigRegister()
{
	ADCCfgRegister data;
	uint16_t result = readSPI(ADC_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeADCConfigRegister(ADCCfgRegister adcCfg)
{
	uint16_t result = writeSPI(ADC_CFG_ADDR, adcCfg.reg);
}

PWMCfgRegister Infineon6EDL7141Driver::readPWMConfigRegister()
{
	PWMCfgRegister data;
	uint16_t result = readSPI(PWM_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writePWMConfigRegister(PWMCfgRegister pwmCfg)
{
	uint16_t result = writeSPI(PWM_CFG_ADDR, pwmCfg.reg);
}

SensorCfgRegister Infineon6EDL7141Driver::readSensorConfigRegister()
{
	SensorCfgRegister data;
	uint16_t result = readSPI(SENSOR_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeSensorConfigRegister(SensorCfgRegister sensorCfg)
{
	uint16_t result = writeSPI(SENSOR_CFG_ADDR, sensorCfg.reg);
}

WDCfgRegister Infineon6EDL7141Driver::readWatchDogConfigRegister()
{
	WDCfgRegister data;
	uint16_t result = readSPI(WD_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeWatchDogConfigRegister(WDCfgRegister wdCfg)
{
	uint16_t result = writeSPI(WD_CFG_ADDR, wdCfg.reg);
}

WDCfg2Register Infineon6EDL7141Driver::readWatchDogConfig2Register()
{
	WDCfg2Register data;
	uint16_t result = readSPI(WD_CFG2_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeWatchDogConfig2Register(WDCfg2Register wdCfg2)
{
	uint16_t result = writeSPI(WD_CFG2_ADDR, wdCfg2.reg);
}

IDriveCfgRegister Infineon6EDL7141Driver::readIDriveConfigRegister()
{
	IDriveCfgRegister data;
	uint16_t result = readSPI(IDRIVE_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeIDriveConfigRegister(IDriveCfgRegister idriveCfg)
{
	uint16_t result = writeSPI(IDRIVE_CFG_ADDR, idriveCfg.reg);
}

IDrivePreCfgRegister Infineon6EDL7141Driver::readIDrivePreConfigRegister()
{
	IDrivePreCfgRegister data;
	uint16_t result = readSPI(IDRIVE_PRE_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeIDrivePreConfigRegister(IDrivePreCfgRegister idrivePreCfg)
{
	uint16_t result = writeSPI(IDRIVE_PRE_CFG_ADDR, idrivePreCfg.reg);
}

TDriveSrcCfgRegister Infineon6EDL7141Driver::readTDriveSourceConfigRegister()
{
	TDriveSrcCfgRegister data;
	uint16_t result = readSPI(TDRIVE_SRC_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeTDriveSourceConfigRegister(TDriveSrcCfgRegister tdriveSrcCfg)
{
	uint16_t result = writeSPI(TDRIVE_SRC_CFG_ADDR, tdriveSrcCfg.reg);
}

TDriveSinkCfgRegister Infineon6EDL7141Driver::readTDriveSinkConfigRegister()
{
	TDriveSinkCfgRegister data;
	uint16_t result = readSPI(TDRIVE_SINK_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeTDriveSinkConfigRegister(TDriveSinkCfgRegister tdriveSinkCfg)
{
	uint16_t result = writeSPI(TDRIVE_SINK_CFG_ADDR, tdriveSinkCfg.reg);
}

DTCfgRegister Infineon6EDL7141Driver::readDeadTimeConfigRegister()
{
	DTCfgRegister data;
	uint16_t result = readSPI(DT_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeDeadTimeConfigRegister(DTCfgRegister dtCfg)
{
	uint16_t result = writeSPI(DT_CFG_ADDR, dtCfg.reg);
}

CPCfgRegister Infineon6EDL7141Driver::readChargePumpConfigRegister()
{
	CPCfgRegister data;
	uint16_t result = readSPI(CP_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeChargePumpConfigRegister(CPCfgRegister cpCfg)
{
	uint16_t result = writeSPI(CP_CFG_ADDR, cpCfg.reg);
}

CSAmpCfgRegister Infineon6EDL7141Driver::readCurrentSenseAmplifierConfigRegister()
{
	CSAmpCfgRegister data;
	uint16_t result = readSPI(CSAMP_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeCurrentSenseAmplifierConfigRegister(CSAmpCfgRegister csAmpCfg)
{
	uint16_t result = writeSPI(CSAMP_CFG_ADDR, csAmpCfg.reg);
}

CSAmpCfg2Register Infineon6EDL7141Driver::readCurrentSenseAmplifierConfig2Register()
{
	CSAmpCfg2Register data;
	uint16_t result = readSPI(CSAMP_CFG2_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeCurrentSenseAmplifierConfig2Register(CSAmpCfg2Register csAmpCfg2)
{
	uint16_t result = writeSPI(CSAMP_CFG2_ADDR, csAmpCfg2.reg);
}

OTPProgRegister Infineon6EDL7141Driver::readOneTimeProgramRegister()
{
	OTPProgRegister data;
	uint16_t result = readSPI(OTP_PROG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeOneTimeProgramRegister(OTPProgRegister otpProg)
{
	uint16_t result = writeSPI(OTP_PROG_ADDR, otpProg.reg);
}
