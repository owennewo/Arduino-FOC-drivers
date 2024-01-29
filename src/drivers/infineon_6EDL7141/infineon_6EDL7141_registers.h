

#ifndef SIMPLEFOC_INFINEON_6EDL7141_REGISTERS
#define SIMPLEFOC_INFINEON_6EDL7141_REGISTERS

// from https://www.infineon.com/dgdl/Infineon-6EDL7141-DataSheet-v01_08-EN.pdf
// Status Registers - there are 8 status registers, each 16 bits wide
#define FAULT_ST_ADDR 0x00	// Fault and warning status
#define TEMP_ST_ADDR 0x01	// Temperature status
#define SUPPLY_ST_ADDR 0x02 // Power supply status
#define FUNCT_ST_ADDR 0x03	// Functional status
#define OTP_ST_ADDR 0x04	// OTP status
#define ADC_ST_ADDR 0x05	// ADC status
#define CP_ST_ADDR 0x06		// Charge pump status
#define DEVICE_ID_ADDR 0x07 // Device ID

// Control Registers - there are 16 control registers, each 16 bits wide.  Most are R/W
// Note some values only work if set before EN_DRV.  Others only work if written via OTP (see datasheet)
#define FAULTS_CLR_ADDR 0x10	  // Fault clear
#define SUPPLY_CFG_ADDR 0x11	  // Power supply configuration
#define ADC_CFG_ADDR 0x12		  // ADC configuration
#define PWM_CFG_ADDR 0x13		  // PWM configuration
#define SENSOR_CFG_ADDR 0x14	  // Sensor configuration
#define WD_CFG_ADDR 0x15		  // Watchdog configuration
#define WD_CFG2_ADDR 0x16		  // Watchdog configuration 2
#define IDRIVE_CFG_ADDR 0x17	  // Gate driver current configuration
#define IDRIVE_PRE_CFG_ADDR 0x18  // Pre-charge gate driver current configuration
#define TDRIVE_SRC_CFG_ADDR 0x19  // Gate driver sourcing timing configuration
#define TDRIVE_SINK_CFG_ADDR 0x1A // Gate driver sinking timing configuration
#define DT_CFG_ADDR 0x1B		  // Dead time configuration
#define CP_CFG_ADDR 0x1C		  // Charge pump configuration
#define CSAMP_CFG_ADDR 0x1D		  // Current sense amplifier configuration
#define CSAMP_CFG2_ADDR 0x1E	  // Current sense amplifier configuration 2
#define OTP_PROG_ADDR 0x1F		  // OTP program

enum PWMMode
{
	PWM6_Mode = 0b000,
	PWM3_Mode = 0b001,
	PWM1_Mode = 0b010,
	PWM1_Hall_Mode = 0b011
};

enum GateDrivingVoltage
{
	_12V = 0b00,
	_15V = 0b01,
	_10V = 0b10,
	_7V = 0b11
};

enum CurrentSenseReference
{
	HALF_DVDD = 0b00,
	FIVE_TWELTHS_DVDD = 0b01,
	THIRD_DVDD = 0b10,
	QUARTER_DVDD = 0b11
};

enum CurrentThreshold
{
	CT_450mA = 0b00,
	CT_300mA = 0b01,
	CT_150mA = 0b10,
	CT_50mA = 0b11
};

enum DVDDVoltage
{
	ANALOG = 0b00, // the LDO voltage (DVDD) is set using resistors on VSENSE pin
	_3V3 = 0b10,
	_5V = 0b11
};

enum BuckFrequency
{
	BF_500kHz = 0b0,
	BF_1MHz = 0b1
};

enum DVDDTurnOnDelay
{
	DTOD_200us = 0b00,
	DTOD_400us = 0b01,
	DTOD_600us = 0b10,
	DTOD_800us = 0b11
};

enum ADCInputSelection
{
	AIS_IDIGITAL = 0b00,
	AIS_DVDD = 0b01,
	AIS_VDDB = 0b10
};

enum GenericFiltering
{
	GF_8_SAMPLES = 0b00,
	GF_16_SAMPLES = 0b01,
	GF_32_SAMPLES = 0b10,
	GF_64_SAMPLES = 0b11
};

enum PVDDFiltering
{
	PF_32_SAMPLES = 0b00,
	PF_16_SAMPLES = 0b01,
	PF_8_SAMPLES = 0b10,
	PF_1_SAMPLE = 0b11
};

enum OnePWMFreewheel
{
	Active = 0b0, // the low side MOSFETs will be switched synchronously to reduce conduction losses on the body diode conduction
	Diode = 0b1,  // the freewheeling current will flow through the low side MOSFET body diodes.
};

enum BrakeConfig
{
	LowSide = 0b00,
	HighSide = 0b01,
	HighZ = 0b10,
	BrakeToggle = 0b11 // Brake toggle-alternates between low and high side braking on
};

enum CurrentSenseAmplifierTiming
{
	ActiveOnGLxHigh = 0b00,
	ActiveOnGHxLow = 0b01,
	ActiveAlways = 0b10,
};

// Watchdog input selection b000: EN_DRV pin (measure input signal frequency), b001: Reserved, b010: DVDD (linear regulator), b011: VCCLS and VCCHS, (charge pumps), b100: Status register read
// uint16_t WD_FLTCFG : 1;
enum WatchDogInputSelection
{
	EnDrvPin = 0b000,
	DVDD = 0b010,
	ChargePumps = 0b011,
	StatusRegisterRead = 0b100,
};

enum WatchDogFaultConfig
{
	StatusRegisterOnly = 0b00,
	StatusRegisterAndNFaultPin = 0x01,
};

enum DVDDRestartDelay
{
	DRD_500us = 0b0000,
	DRD_1000us = 0b0001,
	DRD_1500us = 0b0010,
	DRD_2000us = 0b0011,
	DRD_2500us = 0b0100,
	DRD_3000us = 0b0101,
	DRD_3500us = 0b0110,
	DRD_4000us = 0b0111,
	DRD_4500us = 0b1000,
	DRD_5000us = 0b1001,
	DRD_5500us = 0b1010,
	DRD_6000us = 0b1011,
	DRD_6500us = 0b1100,
	DRD_7000us = 0b1101,
	DRD_7500us = 0b1110,
	DRD_8000us = 0b1111,
};

enum GateDriverCurrent // for slew control
{
	GDC_10mA = 0b0000,
	GDC_20mA = 0b0001,
	GDC_30mA = 0b0010,
	GDC_40mA = 0b0011,
	GDC_50mA = 0b0100,
	GDC_60mA = 0b0101,
	GDC_80mA = 0b0110,
	GDC_100mA = 0b0111,
	GDC_125mA = 0b1000,
	GDC_150mA = 0b1001,
	GDC_175mA = 0b1010,
	GDC_200mA = 0b1011,
	GDC_250mA = 0b1100,
	GDC_300mA = 0b1101,
	GDC_400mA = 0b1110,
	GDC_500mA = 0b1111,
};

/*
 */
enum ChargePumpClockFrequency
{
	_781_25_kHz = 0b00,
	_390_625_kHz = 0b01,
	_195_3125_kHz = 0b10,
	_1_5625_MHz = 0b11
};

enum CurrentSenseGain
{
	_4X = 0b000,
	_8X = 0b001,
	_12X = 0b010,
	_16X = 0b011,
	_20X = 0b100,
	_24X = 0b101,
	_32X = 0b110,
	_64X = 0b111
};

enum CurrentSenseGainMode
{
	ViaRegisterConfig = 0b0,
	ViaAnalogPinResistr = 0b1
};

enum BlankingTime
{
	BT_0_ns = 0b0000,
	BT_50_ns = 0b0001,
	BT_100_ns = 0b0010,
	BT_200_ns = 0b0011,
	BT_300_ns = 0b0100,
	BT_400_ns = 0b0101,
	BT_500_ns = 0b0110,
	BT_600_ns = 0b0111,
	BT_700_ns = 0b1000,
	BT_800_ns = 0b1001,
	BT_900_ns = 0b1010,
	BT_1_us = 0b1011,
	BT_2_us = 0b1100,
	BT_4_us = 0b1101, // page 93
	BT_6_us = 0b1110,
	BT_8_us = 0b1111,
};

enum DeglitchTime
{
	DT_0_us = 0b00,
	DT_2_us = 0b01,
	DT_4_us = 0b10,
	DT_8_us = 0b11,
};

enum EventCountTrigger
{
	_8_Events = 0b00,
	_16_Events = 0b01,
	All_Events = 0b10,
	Never = 0b11,
};

enum VoltageThreshold
{
	_300_mV = 0b0000,
	_250_mV = 0b0001,
	_225_mV = 0b0010,
	_200_mV = 0b0011,
	_175_mV = 0b0100,
	_150_mV = 0b0101,
	_125_mV = 0b0110,
	_100_mV = 0b0111,
	_90_mV = 0b1000,
	_80_mV = 0b1001,
	_70_mV = 0b1010,
	_60_mV = 0b1011,
	_50_mV = 0b1100,
	_40_mV = 0b1101,
	_30_mV = 0b1110,
	_20_mV = 0b1111,
};

enum CurrentSenseMode
{
	ExternalShunt = 0b0,
	InternalRdson = 0b1, // positive is connected to drain of the low side MOSFET to the positive input of the current sense amplifier
};

enum AutoZero
{
	EnabledWithInternalSynchronization = 0b00,
	Disabled = 0b01,
	EnabledWithExternalSynchronization = 0b10,
	EnabledWithExternalSynchronizationAndClockGating = 0b11,
};

/**
 * Status Registers (READ only)
 */
typedef union
{
	struct
	{
		uint16_t CS_OCP_FLT : 3;   // Current sense amplifier OCP fault status (on phase A, B, C)
		uint16_t CP_FLT : 1;	   // Charge pumps fault status
		uint16_t DVDD_OCP_FLT : 1; // DVDD OCP (Over-Current Protection) fault status
		uint16_t DVDD_UV_FLT : 1;  // DVDD UVLO (Under-Voltage Lockout) fault status
		uint16_t DVDD_OV_FLT : 1;  // DVDD OVLO (Over-Voltage Lock-Out)fault status
		uint16_t BK_OCP_FLT : 1;   // Buck OCP fault status
		uint16_t OTS_FLT : 1;	   // Over-Temperature Shutdown fault status
		uint16_t OTW_FLT : 1;	   // Over-Temperature Warning status
		uint16_t RLOCK_FLT : 1;	   // Locked rotor fault status
		uint16_t WD_FLT : 1;	   // Watchdog fault status
		uint16_t OTP_FLT : 1;	   // OTP (One Time Programmable) memory fault status
		uint16_t : 3;			   // unused, returns zeros
	};
	uint16_t reg;
	bool isFault() { return reg != 0; };
	bool isCurrentSenseOverCurrent() { return CS_OCP_FLT != 0; };
	bool isCurrentSenseOverCurrentPhaseA() { return CS_OCP_FLT & 0b001 != 0; };
	bool isCurrentSenseOverCurrentPhaseB() { return CS_OCP_FLT & 0b010 != 0; };
	bool isCurrentSenseOverCurrentPhaseC() { return CS_OCP_FLT & 0b100 != 0; };
	bool isChargePump() { return CP_FLT == 0b1; };
	bool isDVDDOverCurrent() { return DVDD_OCP_FLT == 0b1; };
	bool isDVDDUnderVoltage() { return DVDD_UV_FLT == 0b1; };
	bool isDVDDOverVoltage() { return DVDD_OV_FLT == 0b1; };
	bool isBuckOverCurrentProtection() { return BK_OCP_FLT == 0b1; };
	bool isOverTemperatureShutdown() { return OTS_FLT == 0b1; };
	bool isOverTemperatureWarning() { return OTW_FLT == 0b1; };
	bool isLockedRotor() { return RLOCK_FLT == 0b1; };
	bool isWatchdog() { return WD_FLT == 0b1; };
	bool isOTPMemory() { return OTP_FLT == 0b1; };

} FaultAndWarningStatus;

typedef union
{
	struct
	{
		uint16_t TEMP_VAL : 7; // Temperature value
		uint16_t : 9;		   // unused, returns zeros
	};
	uint16_t reg;
	int getTemperatureInCelsius() const
	{
		return -94 + (TEMP_VAL * 2);
	}
} TemperatureStatus;

typedef union
{
	struct
	{
		uint16_t VCCLS_UVST : 1; // Charge Pump low side UVLO status (below/above threshold)
		uint16_t VCCHS_UVST : 1; // Charge Pump high side UVLO status (below/above threshold)
		uint16_t DVDD_UVST : 1;	 // DVDD UVLO status  (below/above threshold)
		uint16_t DVDD_OVST : 1;	 // DVDD OVLO (Over-Voltage Lock-Out) status (below/above threshold)
		uint16_t VDDB_UVST : 1;	 // VDDB UVLO status  (below/above threshold)
		uint16_t VDDB_OVST : 1;	 // VDDB OVLO status  (below/above threshold)
		uint16_t PVDD_VAL : 7;	 // PVDD ADC result reading value
		uint16_t : 3;			 // unused, returns zeros
	};
	uint16_t reg;
	bool isChargePumpLowSideUnderVoltage() { return VCCLS_UVST == 0b1; };
	bool isChargePumpHighSideUnderVoltage() { return VCCHS_UVST == 0b1; };
	bool isDVDDUnderVoltage() { return DVDD_UVST == 0b1; };
	bool isDVDDOverVoltage() { return DVDD_OVST == 0b1; };
	bool isVDDBUnderVoltage() { return VDDB_UVST == 0b1; };
	bool isVDDBOverVoltage() { return VDDB_OVST == 0b1; };
	float getPVDDVoltage() { return 0.581f * PVDD_VAL + 5.52f; };

} PowerSupplyStatus;

typedef union
{
	struct
	{
		uint16_t HALLIN_ST : 3;	 // Hall sensor inputs status (for Phase A, B, C)
		uint16_t HALLPOL_ST : 1; // Hall sensor polarity equal indicator (all phases of the hall sensors have the same polarity)
		uint16_t DVDD_ST : 1;	 // DVDD set point status (0 -> 3.3v, 1 -> 5v)
		uint16_t CS_GAIN_ST : 3; // Status of the current sense amplifiers gain (4x to 64x gain, non linear increments)
		uint16_t : 8;			 // unused, returns zeros
	};
	uint16_t reg;
	bool getHallStatePhaseA() { return HALLIN_ST & 0b001 != 0; };
	bool getHallStatePhaseB() { return HALLIN_ST & 0b010 != 0; };
	bool getHallStatePhaseC() { return HALLIN_ST & 0b100 != 0; };
	bool isHallPolarityEqual() { return HALLPOL_ST == 0b1; };
	bool isDVDDSetPoint5V() { return DVDD_ST == 0b1; };
	CurrentSenseGain getCurrentSenseGain() { return static_cast<CurrentSenseGain>(CS_GAIN_ST); };

} FunctionalStatus;

typedef union
{
	struct
	{
		uint16_t OTP_USED : 1;		 // if OTP memory has been written by user or still holds factory defaults
		uint16_t OTP_PASS : 1;		 // if user OTP programming has passed without error
		uint16_t OTP_PROG_BLOCK : 1; // if OTP programming has been attempted when voltage or temperature outside range
		uint16_t OTP_PROG_FAIL : 1;	 // indicates that the programming of the OTP has failed
		uint16_t : 12;				 // unused, returns zeros
	};
	uint16_t reg;
	bool isOneTimeProgramUsed() { return OTP_USED == 0b1; };
	bool isOneTimeProgramPassed() { return OTP_PASS == 0b1; };
	bool isOneTimeProgramBlocked() { return OTP_PROG_BLOCK == 0b1; };
	bool isOneTimeProgramFailed() { return OTP_PROG_FAIL == 0b1; };

} OTPStatus;

typedef union
{
	struct
	{
		uint16_t ADC_OD_RDY : 1; // ADC on demand conversion result ready
		uint16_t ADC_OD_VAL : 7; // ADC result value for on demand conversions
		uint16_t : 8;			 // unused, returns zeros
	};
	uint16_t reg;
	bool isReady() { return ADC_OD_RDY == 0b1; };
	uint8_t getValue() { return ADC_OD_VAL; };
} ADCStatus;

typedef union
{
	struct
	{
		uint16_t VCCHS_VAL : 7; // VCCHS ADC result reading value
		uint16_t VCCLS_VAL : 7; // VCCLS ADC result reading value
		uint16_t : 2;			// unused, returns zeros
	};
	uint16_t reg;
	uint8_t getChargePumpHighSideVoltage() { return VCCHS_VAL; };
	uint8_t getChargePumpLowSideVoltage() { return VCCLS_VAL; };
} ChargePumpsStatus;

typedef union
{
	struct
	{
		uint16_t DEV_ID : 4; // Device identifier for user version control
		uint16_t : 12;		 // unused, returns zeros
	};
	uint16_t reg;
} DeviceID;

/**
 * Control Registers
 */
typedef union
{
	struct
	{
		uint16_t CLR_FLTS : 1;	// clear all non-latched faults
		uint16_t CLR_LATCH : 1; // Clear all latched faults
		uint16_t : 14;			// unused, returns zeros
	};
	uint16_t reg;
	void setClearNonLatchedFaults(bool clear) { CLR_FLTS = clear; };
	void setClearLatchedFaults(bool clear) { CLR_LATCH = clear; };
} FaultsClear;

typedef union
{
	struct
	{
		uint16_t PVCC_SETPT : 2;	  // Configures the target PVCC (gate driving voltage) b00: 12V, b01: 15V, b10: 10V, b11: 7V
		uint16_t CS_REF_CFG : 2;	  // Current sense reference configuration (internal VREF voltage) b00: ½ DVDD, b01: 5/12 DVDD, b10: 1/3 DVDD, b11: ¼ DVDD
		uint16_t DVDD_OCP_CFG : 2;	  // DVDD OCP threshold configuration b00: 450mA, b01: 300mA, b10: 150mA, b11: 50mA
		uint16_t DVDD_SFTSTRT : 4;	  // DVDD soft-start configuration from b0: 100us to 1.6ms
		uint16_t DVDD_SETPT : 2;	  // DVDD set point configuration b0x use VSENSE pin for analog programming, b10 DVDD = 3.3V, b11 DVDD = 5V
		uint16_t BK_FREQ : 1;		  // Buck converter switching frequency selection b0- Low frequency (500kHz), b1: High frequency (1MHz)
		uint16_t DVDD_TON_DELAY : 2;  // DVDD turn on delay configuration b00 - 200us, b01 - 400us, b10 - 600us, b11 - 800us
		uint16_t CP_PRECHARGE_EN : 1; // Charge pump pre-charge configuration b0 : pre-charge disabled, b1 : pre-charge enabled
	};
	uint16_t reg;
	void setGateDrivingVoltage(GateDrivingVoltage voltage) { PVCC_SETPT = voltage; };			// STANDBY
	void setCurrentSenseReference(CurrentSenseReference reference) { CS_REF_CFG = reference; }; // STANDBY
	void setDVDDOverCurrentThreshold(CurrentThreshold threshold) { DVDD_OCP_CFG = threshold; }; // ALWAYS
	void setDVDDSoftStart(uint16_t microseconds)												// OTP ONLY
	{
		if (microseconds >= 100 && microseconds <= 1600)
		{
			DVDD_SFTSTRT = (microseconds / 100) - 1;
		}
	};
	void setDVDDVoltage(DVDDVoltage voltage) { DVDD_SETPT = voltage; };			// OTP ONLY
	void setBuckFrequency(BuckFrequency frequency) { BK_FREQ = frequency; };	// STANDBY
	void setDVDDTurnOnDelay(DVDDTurnOnDelay delay) { DVDD_TON_DELAY = delay; }; // OTP ONLY
	void setChargePumpPreCharge(bool enabled) { CP_PRECHARGE_EN = enabled; };	// STANDBY
} PowerSupplyConfiguration;

typedef union
{
	struct
	{
		uint16_t ADC_OD_REQ : 1;		// ADC on demand conversion request (b0: No action. b1: Request the conversion of the signal selected in ADC_IN_SEL)
		uint16_t ADC_OD_INSEL : 2;		// ADC input selection for on demand conversions b00: IDIGITAL: device digital area current consumption, b01: DVDD, b10: VDDB, b11: Reserved
		uint16_t ADC_EN_FILT : 1;		// (Write only) Enables moving averaging filter for on demand ADC measurements
		uint16_t ADC_FILT_CFG : 2;		// ADC generic filtering configuration b00: 8 samples, b01: 16 samples, b10: 32 samples, b11: 64 Samples
		uint16_t ADC_FILT_CFG_PVDD : 2; // PVDD ADC measurement result filtering configuration b00: 32 samples, b01: 16 samples, b10: 8 samples, b11: 1 sample
		uint16_t : 8;					// unused, returns zeros
	};
	uint16_t reg;
	void setOnDemandRequest(bool requestEnabled) { ADC_OD_REQ = requestEnabled; };				// ALWAYS (NO OTP)
	void setOnDemandInputSelection(ADCInputSelection selection) { ADC_OD_INSEL = selection; };	// ALWAYS (NO OTP)
	void setOnDemandFilter(bool filterEnabled) { ADC_EN_FILT = filterEnabled; };				// ALWAYS (NO OTP)
	void setOnDemandGenericFiltering(GenericFiltering filtering) { ADC_FILT_CFG = filtering; }; // ALWAYS
	void setOnDemandPVDDFiltering(PVDDFiltering filtering) { ADC_FILT_CFG_PVDD = filtering; };	// ALWAYS

} ADCConfiguration;

typedef union PWMConfiguration
{
	struct
	{
		uint16_t PWM_MODE : 3;		// PWM commutation mode selection b000: 6 PWM mode, b001: 3 PWM mode, b010: 1 PWM mode, b011: 1 PWM mode with Hall sensor
		uint16_t PWM_FREEW_CFG : 1; // Either active or through the low side MOSFET body diodes
		uint16_t BRAKE_CFG : 2;		// Brake configuration b00: Brake disabled, b01: Brake enabled, b10: Brake enabled with PWM truncation, b11: Brake enabled with PWM truncation and PWM freewheeling
		uint16_t PWM_RECIRC : 1;
		uint16_t : 9;
	};

	uint16_t reg;

	void setPWMMode(PWMMode mode) { PWM_MODE = mode; };											   // STANDBY
	void setOnePWMFreewheel(OnePWMFreewheel freewheelConfig) { PWM_FREEW_CFG = freewheelConfig; }; // ALWAYS
	void setBrake(BrakeConfig brakeConfig) { BRAKE_CFG = brakeConfig; };						   // ALWAYS
	void setOnePwmHallRecirculating(bool enable) { PWM_RECIRC = enable; };						   // STANDBY

} PWMConfiguration;

typedef union
{
	struct
	{
		uint16_t HALL_DEGLITCH : 4; // Hall Sensor deglitch b0000: 0ns, b0001: 640 ns, … in steps of 640 ns, b1111- 9600 ns
		uint16_t OTS_DIS : 1;		// Over-temperature shutdown disable
		uint16_t CS_TMODE : 2;		// Current sense amplifier timing mode b00: CS amplifier outputs are active when GLx signal is high, b01: CS amplifier outputs are active when GHx signal is low, b1x: CS amplifier outputs are always active
		uint16_t : 9;				// unused, returns zeros
	};
	uint16_t reg;
	void setHallDeglitch(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds <= 9600)
		{
			HALL_DEGLITCH = nanoseconds / 640;
		}
	}
	void setDisableOvertemperatureShutdown(boolean disable) { OTS_DIS = disable; };						 // ALWAYS
	void setCurrentSensingTimingMode(CurrentSenseAmplifierTiming timingMode) { CS_TMODE = timingMode; }; // ALWAYS
} SensorConfiguration;

typedef union
{
	struct
	{
		uint16_t WD_EN : 1;		  // Watchdog enable
		uint16_t WD_INSEL : 3;	  // Watchdog input selection b000: EN_DRV pin (measure input signal frequency), b001: Reserved, b010: DVDD (linear regulator), b011: VCCLS and VCCHS, (charge pumps), b100: Status register read
		uint16_t WD_FLTCFG : 1;	  // Watchdog fault configuration b00: Status register only, b01: Status register and pull down of nFAULT pin
		uint16_t WD_TIMER_T : 10; // Watchdog timer period value b0000000000: 100 us, b0000000001: 200 us, ......
		uint16_t : 1;			  // unused, returns zeros
	};
	uint16_t reg;
	void setWatchDogEnable(boolean enable) { WD_EN = enable; };									// STANDBY
	void setWatchDogInputSelection(WatchDogInputSelection selection) { WD_INSEL = selection; }; // STANDBY
	void setWatchDogFaultConfig(WatchDogFaultConfig config) { WD_FLTCFG = config; };			// STANDBY
	void setWatchDogTimer(uint16_t microseconds)												// STANDBY
	{
		if (microseconds <= 102400)
		{
			WD_TIMER_T = (microseconds / 100) - 1;
		}
	}
} WatchDogConfiguration;

typedef union
{
	struct
	{
		uint16_t WD_BRAKE : 1;			// Brake on watchdog timer overflow
		uint16_t WD_EN_LATCH : 1;		// b0: Normal reaction to fault b1: Brake on watchdog fault (Automatically latched). The braking mode is configured in PWM_CFG register.
		uint16_t WD_DVDD_RSTRT_ATT : 2; // restart attempts for DVDD WD
		uint16_t WD_DVDD_RSTRT_DLY : 4; // DVDD restart delay b0000: 0.5 ms b0001: 1 ms……
		uint16_t WD_RLOCK_EN : 1;		// Enable rotor locked detection
		uint16_t WD_RLOCK_T : 3;		// Rotor locked watchdog timeout b000: 1 second, b001: 2 s ……….
		uint16_t WD_BK_DIS : 1;			// Buck watchdog disable
		uint16_t : 3;					// unused, returns zeros
	};
	uint16_t reg;
	void setBrakeOnWatchDogFault(bool enable) { WD_BRAKE = enable; };	 // STANDBY
	void setLatchOnWatchDogFault(bool enable) { WD_EN_LATCH = enable; }; // STANDBY
	void setNumberOfDVDDWatchDogAttemptsBeforeRestart(uint16_t attempts) // STANDBY
	{
		if (attempts <= 3)
		{
			WD_DVDD_RSTRT_ATT = attempts;
		}
	}
	void setDVDDRestartDelay(DVDDRestartDelay restartDelay) { WD_DVDD_RSTRT_DLY = restartDelay; }; // STANDBY
	void setRotorLockDetection(bool enable) { WD_RLOCK_EN = enable; };							   // ALWAYS
	void setRotorLockTimeout(uint16_t seconds)													   // ALWAYS
	{
		if (seconds <= 8)
		{
			WD_RLOCK_T = seconds - 1;
		}
	}
	void setBuckWatchDogDisable(bool disable) { WD_BK_DIS = disable; }; // OTP ONLY
} WatchDogConfiguration2;

typedef union
{
	struct
	{
		uint16_t IHS_SRC : 4;  // High-side source current (default 200mA range b0000: 50mA, b1111: 500mA)
		uint16_t IHS_SINK : 4; // High-side sink current (as above)
		uint16_t ILS_SRC : 4;  // Low-side source current (as above)
		uint16_t ILS_SINK : 4; // Low-side sink current (as above)
	};
	uint16_t reg;
	void setHighSideSourceCurrent(GateDriverCurrent current) { IHS_SRC = current; }; // ALWAYS
	void setHighSideSinkCurrent(GateDriverCurrent current) { IHS_SINK = current; };	 // ALWAYS
	void setLowSideSourceCurrent(GateDriverCurrent current) { ILS_SRC = current; };	 // ALWAYS
	void setLowSideSinkCurrent(GateDriverCurrent current) { ILS_SINK = current; };	 // ALWAYS
} GateDriverCurrentConfiguration;

typedef union
{
	struct
	{
		uint16_t I_PRE_SRC : 4;	 // Pre-charge source current setting (TDRIVE1) (as above)
		uint16_t I_PRE_SINK : 4; // Pre-charge sink current setting (TDRIVE3) (as above)
		uint16_t I_PRE_EN : 1;	 // Gate driver pre-charge mode enable b0: Pre-charge current enabled. Values I_PRE_SINK and I_PRE_SRC are applied during TDRIVE1 and TDRIVE3 respectively, b1: Pre-charge mode disabled. 1.5A applied during TDRIVE1 and TDRIVE3
		uint16_t : 7;			 // unused, returns zeros
	};
	uint16_t reg;
	void setPreChargeSourceCurrent(GateDriverCurrent current) { I_PRE_SRC = current; }; // ALWAYS
	void setPreChargeSinkCurrent(GateDriverCurrent current) { I_PRE_SINK = current; };	// ALWAYS
	void setPreChargeEnable(bool enable) { I_PRE_EN = enable; };						// ALWAYS

} PreChargeGateDriverCurrentConfiguration;

typedef union
{
	struct
	{
		uint16_t TDRIVE1 : 8; // TDRIVE1 timing b00000000 - 0ns, b00000001 - 50ns (values between 0ns and 50ns not allowed) 10ns steps
		uint16_t TDRIVE2 : 8; // TDRIVE2 timing TDRIVE2 value for high and low side. b00000000 - 0ns, b00000001 - 10ns 10ns steps
	};
	uint16_t reg;
	void setTDrive1Timing(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds < 50 || nanoseconds >= 2600)
		{
			TDRIVE1 = 0;
		}
		else
		{
			TDRIVE1 = (nanoseconds / 10) - 4;
		}
	};
	void setTDrive2Timing(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds >= 2560)
		{
			TDRIVE1 = 0;
		}
		else
		{
			TDRIVE1 = (nanoseconds / 10) - 1;
		}
	};
} GateDriverSourcingTimingConfiguration;

typedef union
{
	struct
	{
		uint16_t TDRIVE3 : 8; // TDRIVE3 timing (values as TDRIVE1)
		uint16_t TDRIVE4 : 8; // TDRIVE4timing (values as TDRIVE2)
	};
	uint16_t reg;
	void setTDrive3Timing(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds < 50 || nanoseconds >= 2600)
		{
			TDRIVE3 = 0;
		}
		else
		{
			TDRIVE3 = (nanoseconds / 10) - 4;
		}
	};
	void setTDrive4Timing(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds >= 2560)
		{
			TDRIVE4 = 0;
		}
		else
		{
			TDRIVE4 = (nanoseconds / 10) - 1;
		}
	};
} GateDriverSinkingTimingConfiguration;

typedef union
{
	struct
	{
		uint16_t DT_RISE : 8; // Dead time rise (of phase node voltage) b00000000: 120 ns, b00000001: 200 ns, In steps of 80ns
		uint16_t DT_FALL : 8; // Dead time fall (of phase node voltage) (as above)
	};
	uint16_t reg;
	void setDeadTimeRise(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds >= 120 || nanoseconds <= 12040)
		{
			DT_RISE = (nanoseconds - 120) / 80;
		}
	}
	void setDeadTimeFall(uint16_t nanoseconds) // ALWAYS
	{
		if (nanoseconds >= 120 || nanoseconds <= 12040)
		{
			DT_FALL = (nanoseconds - 120) / 80;
		}
	}
	uint16_t getDeadTimeRise()
	{
		return (DT_RISE * 80) + 120;
	}
	uint16_t getDeadTimeFall()
	{
		return (DT_FALL * 80) + 120;
	}

} DeadTimeConfiguration;

typedef union
{
	struct
	{
		uint16_t CP_CLK_CFG : 2;	// Charge pump clock frequency configuration b00: 781.25 kHz, b01: 390.625 kHz, b10: 195.3125 kHz, b11: 1.5625 MHz
		uint16_t CP_CLK_SS_DIS : 1; // Charge pump clock spread spectrum disable
		uint16_t : 13;				// unused, returns zeros
	};
	uint16_t reg;
	void setChargePumpClockFrequency(ChargePumpClockFrequency freq) { CP_CLK_CFG = freq; };	 // ALWAYS
	void setChargePumpClockSpreadSpectrumDisable(bool disable) { CP_CLK_SS_DIS = disable; }; // STANDBY
} ChargePumpConfiguration;

typedef union
{
	struct
	{
		uint16_t CS_GAIN : 3;		  // Gain of current sense amplifiers (4x to 64x gain, non linear increments)
		uint16_t CS_GAIN_ANA : 1;	  // CS Gain analogue programming enable (defaults to b1 analog)
		uint16_t CS_EN : 3;			  // Enable of each current shunt amplifier (phase A, B, C)
		uint16_t CS_BLANK : 4;		  // Current shunt amplifier blanking time (from 0ns to 8us)
		uint16_t CS_EN_DCCAL : 1;	  // Enable DC Calibration of CS amplifier
		uint16_t CS_OCP_DEGLITCH : 2; // Current sense amplifier OCP deglitch b00: 0 μs, b01: 2 μs, b10: 4 μs, b11: 8 μs
		uint16_t CS_OCPFLT_CFG : 2;	  // Current sense amplifier OCP fault trigger configuration b00: Count 8 OCP events, b01: Count 16 OCP events, b10: Trigger on all OCP events, b11: No fault trigger (PWM Truncation continues as defined in bitfield CS_TRUNC_DIS in register CSAMP_CFG2)
	};
	uint16_t reg;
	void setCurrentSenseGain(CurrentSenseGain gain) { CS_GAIN = gain; };			// ALWAYS (recommended to stop PWM first)
	void setCurrentSenseUsingAnalogResistor(bool enable) { CS_GAIN_ANA = enable; }; // STANDBY (change to digital mode)- change to analog mode only possible if written in OTP followed by power cycle
	void setEnableShunt(bool enableShuntA, bool enableShuntB, bool enableShuntC)	// ALWAYS
	{
		CS_EN = (enableShuntA ? 1 : 0) |
				(enableShuntB ? 2 : 0) |
				(enableShuntC ? 4 : 0);
	};
	void setCurrentSenseBlankingTime(BlankingTime blankingTime) { CS_BLANK = blankingTime; };	 // ALWAYS (recommended to stop PWM first)
	void setEnableDCCalibration(bool enable) { CS_EN_DCCAL = enable; };							 // STANDBY
	void setDeglitchTime(DeglitchTime deglitchTime) { CS_OCP_DEGLITCH = deglitchTime; };		 // STANDBY
	void setCurrentSenseOCPFaultTrigger(EventCountTrigger trigger) { CS_OCPFLT_CFG = trigger; }; // STANDBY

} CurrentSenseAmplifierConfiguration;

typedef union
{
	struct
	{
		uint16_t CS_OCP_PTHR : 4;	 // Current sense amplifier OCP positive thresholds (defaults to 200mV - range 20mV to 300mV)
		uint16_t CS_OCP_NTHR : 4;	 // Current sense amplifier OCP negative thresholds (as above)
		uint16_t CS_OCP_LATCH : 1;	 // OCP latch choice
		uint16_t CS_MODE : 1;		 // Current sense amplifier sensing mode b0: Shunt resistor, b1: RDSON sensing-CS_TMODE forced to be GL ON only
		uint16_t CS_OCP_BRAKE : 1;	 // Current sense amplifier brake on OCP configuration
		uint16_t CS_TRUNC_DIST : 1;	 // PWM truncation disable
		uint16_t VREF_INSEL : 1;	 // VREF source selection use external vref
		uint16_t CS_NEG_OCP_DIS : 1; // Current sense negative OCP disable
		uint16_t CS_AZ_CFG : 2;		 // Current sense Auto-Zero configuration
	};
	uint16_t reg;
	void setOCPPositiveThreshold(VoltageThreshold threshold) { CS_OCP_PTHR = threshold; }; // ALWAYS
	void setOCPNegativeThreshold(VoltageThreshold threshold) { CS_OCP_NTHR = threshold; }; // ALWAYS
	void setEnableLatch(bool enable) { CS_OCP_LATCH = enable; };						   // STANDBY
	void setCurrentSenseMode(CurrentSenseMode mode) { CS_MODE = mode; };				   // STANDBY
	void setEnableBrakeOnFault(bool enable) { CS_OCP_BRAKE = enable; };					   // STANDBY
	void setPWMTruncationDisable(bool disable) { CS_TRUNC_DIST = disable; };			   // ALWAYS
	void setVREFInputSelection(bool useExternalVREF) { VREF_INSEL = useExternalVREF; };	   // STANDBY
	void setDisableNegativeOCP(bool disable) { CS_NEG_OCP_DIS = disable; };				   // ALWAYS
	void setCurrentSenseAutoZero(AutoZero autoZero) { CS_AZ_CFG = autoZero; };			   // ALWAYS
} CurrentSenseAmplifierConfiguration2;

typedef union
{
	struct
	{
		uint16_t OTP_PROG : 1; // Start programming of OTP
		uint16_t USER_ID : 4;  // Space for user to enter an ID into OTP for version control
		uint16_t : 11;		   // unused, returns zeros
	};
	uint16_t reg;
	void setOTPProgram(bool enable) { OTP_PROG = enable; }; // STANDBY (programming of OTP only in STANDBY)
	void setUserID(uint16_t id) { USER_ID = id; };			// ALWAYS
} OTPProgram;

#endif
