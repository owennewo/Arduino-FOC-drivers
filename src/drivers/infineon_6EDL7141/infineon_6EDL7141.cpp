

#include "./infineon_6EDL7141.h"

void Infineon6EDL7141Driver3PWM::init(SPIClass *_spi, uint32_t clock)
{
	Infineon6EDL7141Driver::init(_spi, clock);
	delayMicroseconds(1);
	PWMConfiguration pwmConfig = Infineon6EDL7141Driver::readPWMConfiguration();
	pwmConfig.setPWMMode(PWMMode::PWM3_Mode);
	Infineon6EDL7141Driver::writePWMConfiguration(pwmConfig);
	BLDCDriver3PWM::init();
};

void Infineon6EDL7141Driver6PWM::init(SPIClass *_spi, uint32_t clock)
{
	Infineon6EDL7141Driver::init(_spi, clock);
	delayMicroseconds(1);
	PWMConfiguration pwmConfig = Infineon6EDL7141Driver::readPWMConfiguration();
	pwmConfig.setPWMMode(PWMMode::PWM6_Mode);
	Infineon6EDL7141Driver::writePWMConfiguration(pwmConfig);
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

FaultAndWarningStatus Infineon6EDL7141Driver::readFaultAndWarningStatus()
{
	FaultAndWarningStatus data;
	uint16_t result = readSPI(FAULT_ST_ADDR);
	data.reg = result;
	return data;
}

TemperatureStatus Infineon6EDL7141Driver::readTemperatureStatus()
{
	TemperatureStatus data;
	uint16_t result = readSPI(TEMP_ST_ADDR);
	data.reg = result;
	return data;
}

PowerSupplyStatus Infineon6EDL7141Driver::readPowerSupplyStatus()
{
	PowerSupplyStatus data;
	uint16_t result = readSPI(SUPPLY_ST_ADDR);
	data.reg = result;
	return data;
}

FunctionalStatus Infineon6EDL7141Driver::readFunctionalStatus()
{
	FunctionalStatus data;
	uint16_t result = readSPI(FUNCT_ST_ADDR);
	data.reg = result;
	return data;
}

OTPStatus Infineon6EDL7141Driver::readOTPStatus()
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

ChargePumpsStatus Infineon6EDL7141Driver::readChargePumpsStatus()
{
	ChargePumpsStatus data;
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

void Infineon6EDL7141Driver::writeFaultClear(FaultsClear clear)
{
	uint16_t result = writeSPI(FAULTS_CLR_ADDR, clear.reg);
}

void Infineon6EDL7141Driver::clearFaults(bool nonLatched = true, bool latched = true)
{
	FaultsClear data;
	data.reg = 0;
	data.setClearNonLatchedFaults(nonLatched);
	data.setClearLatchedFaults(latched);
	writeFaultClear(data);
}

PowerSupplyConfiguration Infineon6EDL7141Driver::readPowerSupplyConfiguration()
{
	PowerSupplyConfiguration data;
	uint16_t result = readSPI(SUPPLY_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writePowerSupplyConfiguration(PowerSupplyConfiguration supplyCfg)
{
	uint16_t result = writeSPI(SUPPLY_CFG_ADDR, supplyCfg.reg);
}

ADCConfiguration Infineon6EDL7141Driver::readADCConfiguration()
{
	ADCConfiguration data;
	uint16_t result = readSPI(ADC_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeADCConfiguration(ADCConfiguration adcCfg)
{
	uint16_t result = writeSPI(ADC_CFG_ADDR, adcCfg.reg);
}

PWMConfiguration Infineon6EDL7141Driver::readPWMConfiguration()
{
	PWMConfiguration data;
	uint16_t result = readSPI(PWM_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writePWMConfiguration(PWMConfiguration pwmCfg)
{
	uint16_t result = writeSPI(PWM_CFG_ADDR, pwmCfg.reg);
}

SensorConfiguration Infineon6EDL7141Driver::readSensorConfiguration()
{
	SensorConfiguration data;
	uint16_t result = readSPI(SENSOR_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeSensorConfiguration(SensorConfiguration sensorCfg)
{
	uint16_t result = writeSPI(SENSOR_CFG_ADDR, sensorCfg.reg);
}

WatchDogConfiguration Infineon6EDL7141Driver::readWatchDogConfiguration()
{
	WatchDogConfiguration data;
	uint16_t result = readSPI(WD_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeWatchDogConfiguration(WatchDogConfiguration wdCfg)
{
	uint16_t result = writeSPI(WD_CFG_ADDR, wdCfg.reg);
}

WatchDogConfiguration2 Infineon6EDL7141Driver::readWatchDogConfiguration2()
{
	WatchDogConfiguration2 data;
	uint16_t result = readSPI(WD_CFG2_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeWatchDogConfiguration2(WatchDogConfiguration2 wdCfg2)
{
	uint16_t result = writeSPI(WD_CFG2_ADDR, wdCfg2.reg);
}

GateDriverCurrentConfiguration Infineon6EDL7141Driver::readGateDriverCurrentConfiguration()
{
	GateDriverCurrentConfiguration data;
	uint16_t result = readSPI(IDRIVE_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeGateDriverCurrentConfiguration(GateDriverCurrentConfiguration idriveCfg)
{
	uint16_t result = writeSPI(IDRIVE_CFG_ADDR, idriveCfg.reg);
}

PreChargeGateDriverCurrentConfiguration Infineon6EDL7141Driver::readPreChargeGateDriverCurrentConfiguration()
{
	PreChargeGateDriverCurrentConfiguration data;
	uint16_t result = readSPI(IDRIVE_PRE_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writePreChargeGateDriverCurrentConfiguration(PreChargeGateDriverCurrentConfiguration idrivePreCfg)
{
	uint16_t result = writeSPI(IDRIVE_PRE_CFG_ADDR, idrivePreCfg.reg);
}

GateDriverSourcingTimingConfiguration Infineon6EDL7141Driver::readGateDriverSourcingTimingConfiguration()
{
	GateDriverSourcingTimingConfiguration data;
	uint16_t result = readSPI(TDRIVE_SRC_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeGateDriverSourcingTimingConfiguration(GateDriverSourcingTimingConfiguration tdriveSrcCfg)
{
	uint16_t result = writeSPI(TDRIVE_SRC_CFG_ADDR, tdriveSrcCfg.reg);
}

GateDriverSinkingTimingConfiguration Infineon6EDL7141Driver::readGateDriverSinkingTimingConfiguration()
{
	GateDriverSinkingTimingConfiguration data;
	uint16_t result = readSPI(TDRIVE_SINK_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeGateDriverSinkingTimingConfiguration(GateDriverSinkingTimingConfiguration tdriveSinkCfg)
{
	uint16_t result = writeSPI(TDRIVE_SINK_CFG_ADDR, tdriveSinkCfg.reg);
}

DeadTimeConfiguration Infineon6EDL7141Driver::readDeadTimeConfiguration()
{
	DeadTimeConfiguration data;
	uint16_t result = readSPI(DT_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeDeadTimeConfiguration(DeadTimeConfiguration dtCfg)
{
	uint16_t result = writeSPI(DT_CFG_ADDR, dtCfg.reg);
}

ChargePumpConfiguration Infineon6EDL7141Driver::readChargePumpConfiguration()
{
	ChargePumpConfiguration data;
	uint16_t result = readSPI(CP_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeChargePumpConfiguration(ChargePumpConfiguration cpCfg)
{
	uint16_t result = writeSPI(CP_CFG_ADDR, cpCfg.reg);
}

CurrentSenseAmplifierConfiguration Infineon6EDL7141Driver::readCurrentSenseAmplifierConfiguration()
{
	CurrentSenseAmplifierConfiguration data;
	uint16_t result = readSPI(CSAMP_CFG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeCurrentSenseAmplifierConfiguration(CurrentSenseAmplifierConfiguration csAmpCfg)
{
	uint16_t result = writeSPI(CSAMP_CFG_ADDR, csAmpCfg.reg);
}

CurrentSenseAmplifierConfiguration2 Infineon6EDL7141Driver::readCurrentSenseAmplifierConfiguration2()
{
	CurrentSenseAmplifierConfiguration2 data;
	uint16_t result = readSPI(CSAMP_CFG2_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeCurrentSenseAmplifierConfiguration2(CurrentSenseAmplifierConfiguration2 csAmpCfg2)
{
	uint16_t result = writeSPI(CSAMP_CFG2_ADDR, csAmpCfg2.reg);
}

OTPProgram Infineon6EDL7141Driver::readOTPProgram()
{
	OTPProgram data;
	uint16_t result = readSPI(OTP_PROG_ADDR);
	data.reg = result;
	return data;
}

void Infineon6EDL7141Driver::writeOTPProgram(OTPProgram otpProg)
{
	uint16_t result = writeSPI(OTP_PROG_ADDR, otpProg.reg);
}
