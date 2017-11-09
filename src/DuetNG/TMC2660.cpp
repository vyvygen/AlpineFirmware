/*
 * TMC2660.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#include "TMC2660.h"
#include "RepRap.h"
#include "Movement/Move.h"
#include "sam/drivers/pdc/pdc.h"
#include "sam/drivers/usart/usart.h"

const float MaximumMotorCurrent = 2400.0;
const uint32_t DefaultMicrosteppingShift = 4;				// x16 microstepping
const bool DefaultInterpolation = true;						// interpolation enabled
const int DefaultStallDetectThreshold = 1;
const bool DefaultStallDetectFiltered = false;
const unsigned int DefaultMinimumStepsPerSecond = 200;		// for stall detection: 1 rev per second assuming 1.8deg/step, as per the TMC2660 datasheet

static size_t numTmc2660Drivers;

static bool driversPowered = false;

// Pin assignments, using USART1 in SPI mode
const Pin DriversClockPin = 15;								// PB15/TIOA1
const Pin DriversMosiPin = 22;								// PA13
const Pin DriversMisoPin = 21;								// PA22
const Pin DriversSclkPin = 23;								// PA23

const int ChopperControlRegisterMode = 999;					// mode passed to get/set microstepping to indicate we want the chopper control register

const uint32_t DriversSpiClockFrequency = 4000000;			// 4MHz SPI clock
const int StallGuardThreshold = 1;							// Range is -64..63. Zero seems to be too sensitive. Higher values reduce sensitivity of stall detection.

// TMC2660 register addresses
const uint32_t TMC_REG_DRVCTRL = 0;
const uint32_t TMC_REG_CHOPCONF = 0x80000;
const uint32_t TMC_REG_SMARTEN = 0xA0000;
const uint32_t TMC_REG_SGCSCONF = 0xC0000;
const uint32_t TMC_REG_DRVCONF = 0xE0000;
const uint32_t TMC_DATA_MASK = 0x0001FFFF;

// DRVCONF register bits
const uint32_t TMC_DRVCONF_RDSEL_0 = 0 << 4;
const uint32_t TMC_DRVCONF_RDSEL_1 = 1 << 4;
const uint32_t TMC_DRVCONF_RDSEL_2 = 2 << 4;
const uint32_t TMC_DRVCONF_RDSEL_3 = 3 << 4;
const uint32_t TMC_DRVCONF_VSENSE = 1 << 6;
const uint32_t TMC_DRVCONF_SDOFF = 1 << 7;
const uint32_t TMC_DRVCONF_TS2G_3P2 = 0 << 8;
const uint32_t TMC_DRVCONF_TS2G_1P6 = 1 << 8;
const uint32_t TMC_DRVCONF_TS2G_1P2 = 2 << 8;
const uint32_t TMC_DRVCONF_TS2G_0P8 = 3 << 8;
const uint32_t TMC_DRVCONF_DISS2G = 1 << 10;
const uint32_t TMC_DRVCONF_SLPL_MIN = 0 << 12;
const uint32_t TMC_DRVCONF_SLPL_MED = 2 << 12;
const uint32_t TMC_DRVCONF_SLPL_MAX = 3 << 12;
const uint32_t TMC_DRVCONF_SLPH_MIN = 0 << 14;
const uint32_t TMC_DRVCONF_SLPH_MIN_TCOMP = 1 << 14;
const uint32_t TMC_DRVCONF_SLPH_MED_TCOMP = 2 << 14;
const uint32_t TMC_DRVCONF_SLPH_MAX = 3 << 14;
const uint32_t TMC_DRVCONF_TST = 1 << 16;

// Chopper control register bits
const uint32_t TMC_CHOPCONF_TOFF_MASK = 15;
#define TMC_CHOPCONF_TOFF(n)	((((uint32_t)n) & 15) << 0)
#define TMC_CHOPCONF_HSTRT(n)	((((uint32_t)n) & 7) << 4)
#define TMC_CHOPCONF_HEND(n)	((((uint32_t)n) & 15) << 7)
#define TMC_CHOPCONF_HDEC(n)	((((uint32_t)n) & 3) << 11)
const uint32_t TMC_CHOPCONF_RNDTF = 1 << 13;
const uint32_t TMC_CHOPCONF_CHM = 1 << 14;
#define TMC_CHOPCONF_TBL(n)	(((uint32_t)n & 3) << 15)

// Driver control register bits, when SDOFF=0
const uint32_t TMC_DRVCTRL_MRES_MASK = 0x0F;
const uint32_t TMC_DRVCTRL_MRES_SHIFT = 0;
const uint32_t TMC_DRVCTRL_MRES_16 = 0x04;
const uint32_t TMC_DRVCTRL_MRES_32 = 0x03;
const uint32_t TMC_DRVCTRL_MRES_64 = 0x02;
const uint32_t TMC_DRVCTRL_MRES_128 = 0x01;
const uint32_t TMC_DRVCTRL_MRES_256 = 0x00;
const uint32_t TMC_DRVCTRL_DEDGE = 1 << 8;
const uint32_t TMC_DRVCTRL_INTPOL = 1 << 9;

// stallGuard2 control register
const uint32_t TMC_SGCSCONF_CS_MASK = 31;
#define TMC_SGCSCONF_CS(n) ((((uint32_t)n) & 31) << 0)
const uint32_t TMC_SGCSCONF_SGT_MASK = 127 << 8;
const uint32_t TMC_SGCSCONF_SGT_SHIFT = 8;
#define TMC_SGCSCONF_SGT(n) ((((uint32_t)n) & 127) << 8)
const uint32_t TMC_SGCSCONF_SGT_SFILT = 1 << 16;

// coolStep control register
const uint32_t TMC_SMARTEN_SEMIN_MASK = 15;
const uint32_t TMC_SMARTEN_SEMIN_SHIFT = 0;
const uint32_t TMC_SMARTEN_SEUP_1 = 0 << 5;
const uint32_t TMC_SMARTEN_SEUP_2 = 1 << 5;
const uint32_t TMC_SMARTEN_SEUP_4 = 2 << 5;
const uint32_t TMC_SMARTEN_SEUP_8 = 3 << 5;
const uint32_t TMC_SMARTEN_SEMAX_MASK = 15;
const uint32_t TMC_SMARTEN_SEMAX_SHIFT = 8;
const uint32_t TMC_SMARTEN_SEDN_32 = 0 << 13;
const uint32_t TMC_SMARTEN_SEDN_8 = 1 << 13;
const uint32_t TMC_SMARTEN_SEDN_2 = 2 << 13;
const uint32_t TMC_SMARTEN_SEDN_1 = 3 << 13;
const uint32_t TMC_SMARTEN_SEIMIN_HALF = 0 << 15;
const uint32_t TMC_SMARTEN_SEIMIN_QTR = 1 << 15;

const unsigned int NumWriteRegisters = 5;

// Chopper control register defaults
// 0x901B4 as per datasheet example
// CHM bit not set, so uses spread cycle mode
const uint32_t defaultChopConfReg =
	  TMC_REG_CHOPCONF
	| TMC_CHOPCONF_TBL(2)				// blanking time 36 clocks which is about 2.4us typical (should maybe use 16 or 24 instead?)
	| TMC_CHOPCONF_HDEC(0)				// no hysteresis decrement
	| TMC_CHOPCONF_HEND(3)				// HEND = 0
	| TMC_CHOPCONF_HSTRT(3)				// HSTRT = 4
	| TMC_CHOPCONF_TOFF(4);				// TOFF = 9.2us

// StallGuard configuration register
const uint32_t defaultSgscConfReg =
	  TMC_REG_SGCSCONF
	| TMC_SGCSCONF_SGT(StallGuardThreshold);

// Driver configuration register
const uint32_t defaultDrvConfReg =
	TMC_REG_DRVCONF
	| TMC_DRVCONF_VSENSE				// use high sensitivity range
	| TMC_DRVCONF_TS2G_0P8				// fast short-to-ground detection
	| 0;

// Driver control register
const uint32_t defaultDrvCtrlReg =
	  TMC_REG_DRVCTRL
	| TMC_DRVCTRL_MRES_16
	| TMC_DRVCTRL_INTPOL;				// x16 microstepping with interpolation

// coolStep control register
const uint32_t defaultSmartEnReg =
	  TMC_REG_SMARTEN
	| 0;								// disable coolStep, it needs to be tuned ot the motor to work properly

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class TmcDriverState
{
public:
	void Init(uint32_t p_axisNumber, uint32_t p_pin);
	void SetAxisNumber(size_t p_axisNumber);
	void WriteAll();
	void SetChopConf(uint32_t newVal);
	void SetMicrostepping(uint32_t shift, bool interpolate);
	void SetCurrent(float current);
	void Enable(bool en);
	void SetStallDetectThreshold(int sgThreshold);
	void SetStallDetectFilter(bool sgFilter);
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond);
	void SetCoolStep(uint16_t coolStepConfig);
	void AppendStallConfig(StringRef& reply) const;

	void TransferDone() __attribute__ ((hot));				// called by the ISR when the SPI transfer has completed
	void StartTransfer() __attribute__ ((hot));				// called to start a transfer

	unsigned int GetMicrostepping(int mode, bool& interpolation) const;		// Get microstepping or chopper control register
	uint32_t ReadLiveStatus() const;
	uint32_t ReadAccumulatedStatus(uint32_t bitsToKeep);

private:
	static void SetupDMA(uint32_t outVal) __attribute__ ((hot));	// Set up the PDC to send a register and receive the status

	uint32_t registers[5];									// the values we want the TMC2660 writable registers to have

	// Register numbers are in priority order, most urgent first
	static constexpr unsigned int DriveControl = 0;			// microstepping
	static constexpr unsigned int StallGuardConfig = 1;		// motor current and stall threshold
	static constexpr unsigned int ChopperControl = 2;		// enable/disable
	static constexpr unsigned int DriveConfig = 3;			// read register select, sense voltage high/low sensitivity
	static constexpr unsigned int SmartEnable = 4;			// coolstep configuration

	uint32_t pin;											// the pin number that drives the chip select pin of this driver
	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state
	uint32_t registersToUpdate;								// bitmap of register values that need to be sent to the driver chip
	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	uint32_t maxStallStepInterval;							// maximum interval between full steps to take any notice of stall detection
	static constexpr uint32_t UpdateAllRegisters = (1u << ARRAY_SIZE(registers)) - 1;	// bitmap for all registers

	volatile uint32_t lastReadStatus;						// the status word that we read most recently, updated by the ISR
	volatile uint32_t accumulatedStatus;
};

// State structures for all drivers
static TmcDriverState driverStates[MaxSmartDrivers];

// PDC address for the USART
static Pdc * const usartPdc = usart_get_pdc_base(USART_TMC_DRV);

// Words to send and receive driver SPI data from/to
volatile static uint32_t spiDataOut = 0;					// volatile because we care about when it is written
volatile static uint32_t spiDataIn = 0;						// volatile because the PDC writes it

// Variables used by the ISR
static TmcDriverState * volatile currentDriver = nullptr;	// volatile because the ISR changes it

// Set up the PDC to send a register and receive the status
/*static*/ inline void TmcDriverState::SetupDMA(uint32_t outVal)
{
	// Faster code, not using the ASF
	usartPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);			// disable the PDC

	// SPI sends data MSB first, but the firmware uses little-endian mode, so we need to reverse the byte order
	spiDataOut = cpu_to_be32(outVal << 8);

	usartPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(&spiDataOut);
	usartPdc->PERIPH_TCR = 3;

	usartPdc->PERIPH_RPR = reinterpret_cast<uint32_t>(&spiDataIn);
	usartPdc->PERIPH_RCR = 3;

	usartPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);			// enable the PDC
}

// Initialise the state of the driver and its CS pin
void TmcDriverState::Init(uint32_t p_axisNumber, uint32_t p_pin)
pre(!driversPowered)
{
	axisNumber = p_axisNumber;
	pin = p_pin;
	pinMode(pin, OUTPUT_HIGH);
	registers[DriveControl] = defaultDrvCtrlReg;
	configuredChopConfReg = defaultChopConfReg;
	registers[ChopperControl] = configuredChopConfReg & ~TMC_CHOPCONF_TOFF_MASK;		// disable driver at startup
	registers[SmartEnable] = defaultSmartEnReg;
	registers[StallGuardConfig] = defaultSgscConfReg;
	registers[DriveConfig] = defaultDrvConfReg;
	registersToUpdate = UpdateAllRegisters;
	accumulatedStatus = lastReadStatus = 0;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);
	SetStallDetectThreshold(DefaultStallDetectThreshold);
	SetStallDetectFilter(DefaultStallDetectFiltered);
	SetStallMinimumStepsPerSecond(DefaultMinimumStepsPerSecond);
}

inline void TmcDriverState::SetAxisNumber(size_t p_axisNumber)
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void TmcDriverState::WriteAll()
{
	registersToUpdate = UpdateAllRegisters;
}

// Set the chopper control register
void TmcDriverState::SetChopConf(uint32_t newVal)
{
	configuredChopConfReg = (newVal & 0x0001FFFF) | TMC_REG_CHOPCONF;		// save the new value
	Enable((registers[ChopperControl] & TMC_CHOPCONF_TOFF_MASK) != 0);		// send the new value, keeping the current Enable status
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift).
void TmcDriverState::SetMicrostepping(uint32_t shift, bool interpolate)
{
	microstepShiftFactor = shift;
	registers[DriveControl] &= ~TMC_DRVCTRL_MRES_MASK;
	registers[DriveControl] |= ((8u - shift) << TMC_DRVCTRL_MRES_SHIFT) & TMC_DRVCTRL_MRES_MASK;
	if (interpolate)
	{
		registers[DriveControl] |= TMC_DRVCTRL_INTPOL;
	}
	else
	{
		registers[DriveControl] &= ~TMC_DRVCTRL_INTPOL;
	}
	registersToUpdate |= 1u << DriveControl;
}

// Set the motor current
void TmcDriverState::SetCurrent(float current)
{
	// The current sense resistor on the production Duet WiFi is 0.051 ohms.
	// This gives us a range of 101mA to 3.236A in 101mA steps in the high sensitivity range (VSENSE = 1)
	const uint32_t iCurrent = static_cast<uint32_t>(constrain<float>(current, 100.0, MaximumMotorCurrent));
	const uint32_t csBits = (32 * iCurrent - 1600)/3236;		// formula checked by simulation on a spreadsheet
	registers[StallGuardConfig] &= ~TMC_SGCSCONF_CS_MASK;
	registers[StallGuardConfig] |= TMC_SGCSCONF_CS(csBits);
	registersToUpdate |= 1u << StallGuardConfig;
}

// Enable or disable the driver. Also called from SetChopConf after the chopper control configuration has been changed.
void TmcDriverState::Enable(bool en)
{
	registers[ChopperControl] = (en) ? configuredChopConfReg : (configuredChopConfReg & ~TMC_CHOPCONF_TOFF_MASK);
	registersToUpdate |= 1u << ChopperControl;
}

// Read the status
uint32_t TmcDriverState::ReadLiveStatus() const
{
	return lastReadStatus & (TMC_RR_SG | TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST);
}

// Read the status
uint32_t TmcDriverState::ReadAccumulatedStatus(uint32_t bitsToKeep)
{
	const irqflags_t flags = cpu_irq_save();
	const uint32_t status = accumulatedStatus;
	accumulatedStatus &= bitsToKeep;
	cpu_irq_restore(flags);
	return status & (TMC_RR_SG | TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST);
}

void TmcDriverState::SetStallDetectThreshold(int sgThreshold)
{
	const uint32_t sgVal = ((uint32_t)constrain<int>(sgThreshold, -64, 63)) & 127;
	registers[StallGuardConfig] = (registers[StallGuardConfig] & ~TMC_SGCSCONF_SGT_MASK) | (sgVal << TMC_SGCSCONF_SGT_SHIFT);
	registersToUpdate |= 1u << StallGuardConfig;
}

void TmcDriverState::SetStallDetectFilter(bool sgFilter)
{
	if (sgFilter)
	{
		registers[StallGuardConfig] |= TMC_SGCSCONF_SGT_SFILT;
	}
	else
	{
		registers[StallGuardConfig] &= ~TMC_SGCSCONF_SGT_SFILT;
	}
	registersToUpdate |= 1u << StallGuardConfig;
}

void TmcDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond)
{
	maxStallStepInterval = DDA::stepClockRate/max<unsigned int>(stepsPerSecond, 1);
}

void TmcDriverState::SetCoolStep(uint16_t coolStepConfig)
{
	registers[SmartEnable] = TMC_REG_SMARTEN | coolStepConfig;
	registersToUpdate |= 1u << SmartEnable;
}

void TmcDriverState::AppendStallConfig(StringRef& reply) const
{
	const bool filtered = ((registers[StallGuardConfig] & TMC_SGCSCONF_SGT_SFILT) != 0);
	int threshold = (int)((registers[StallGuardConfig] & TMC_SGCSCONF_SGT_MASK) >> TMC_SGCSCONF_SGT_SHIFT);
	if (threshold >= 64)
	{
		threshold -= 128;
	}
	reply.catf("stall threshold %d, filter %s, steps/sec %" PRIu32 ", coolstep %" PRIx32,
				threshold, ((filtered) ? "on" : "off"), DDA::stepClockRate/maxStallStepInterval, registers[SmartEnable] & 0xFFFF);
}

// Get microstepping or chopper control register
unsigned int TmcDriverState::GetMicrostepping(int mode, bool& interpolation) const
{
	interpolation = (registers[DriveControl] & TMC_DRVCTRL_INTPOL) != 0;
	if (mode == ChopperControlRegisterMode)
	{
		return configuredChopConfReg & TMC_DATA_MASK;
	}
	else
	{
		return 1u << microstepShiftFactor;
	}
}

// This is called by the ISR when the SPI transfer has completed
inline void TmcDriverState::TransferDone()
{
	fastDigitalWriteHigh(pin);								// set the CS pin high for the driver we just polled
	uint32_t status = be32_to_cpu(spiDataIn) >> 12;			// get the status
	if ((status & TMC_RR_SG) != 0)
	{
		// Stall status is unreliable at low motor speeds, so remove the status bit if the motor is moving slowly
		const uint32_t interval = reprap.GetMove().GetStepInterval(axisNumber, microstepShiftFactor);		// get the full step interval
		if (interval == 0 || interval > maxStallStepInterval)
		{
			status &= ~TMC_RR_SG;
		}
	}
	lastReadStatus = status;
	accumulatedStatus |= status;
}

// This is called from the ISR or elsewhere to start a new SPI transfer. Inlined for ISR speed.
inline void TmcDriverState::StartTransfer()
{
	currentDriver = this;

	// Find which register to send. The common case is when no registers need to be updated.
	uint32_t regVal;
	if (registersToUpdate == 0)
	{
		regVal = registers[SmartEnable];
	}
	else
	{
		size_t regNum = 0;
		uint32_t mask = 1;
		do
		{
			if ((registersToUpdate & mask) != 0)
			{
				break;
			}
			++regNum;
			mask <<= 1;
		} while (regNum < 4);
		registersToUpdate &= ~mask;
		regVal = registers[regNum];
	}

	// Kick off a transfer for that register
	USART_TMC_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX;	// reset transmitter and receiver
	fastDigitalWriteLow(pin);							// set CS low
	SetupDMA(regVal);									// set up the PDC
	USART_TMC_DRV->US_IER = US_IER_ENDRX;				// enable end-of-transfer interrupt
	USART_TMC_DRV->US_CR = US_CR_RXEN | US_CR_TXEN;		// enable transmitter and receiver
}

// ISR for the USART
void USART_TMC_DRV_Handler(void) __attribute__ ((hot));

void USART_TMC_DRV_Handler(void)
{
	TmcDriverState *driver = currentDriver;				// capture volatile variable
	driver->TransferDone();								// tidy up after the transfer we just completed

	if (driversPowered)
	{
		// Power is still good, so send/receive to/from the next driver
		++driver;										// advance to the next driver
		if (driver == driverStates + numTmc2660Drivers)
		{
			driver = driverStates;
		}
		driver->StartTransfer();
	}
	else
	{
		// Driver power is down, so stop polling
		USART_TMC_DRV->US_IDR = US_IDR_ENDRX;
		currentDriver = nullptr;						// signal that we are not waiting for an interrupt
	}
}

//--------------------------- Public interface ---------------------------------

namespace SmartDrivers
{
	// Initialise the driver interface and the drivers, leaving each drive disabled.
	// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
	void Init(const Pin driverSelectPins[DRIVES], size_t numTmcDrivers)
	{
		numTmc2660Drivers = min<size_t>(numTmcDrivers, MaxSmartDrivers);

		// Make sure the ENN pins are high
		pinMode(GlobalTmcEnablePin, OUTPUT_HIGH);

		// The pins are already set up for SPI in the pins table
		ConfigurePin(GetPinDescription(DriversMosiPin));
		ConfigurePin(GetPinDescription(DriversMisoPin));
		ConfigurePin(GetPinDescription(DriversSclkPin));

		// Enable the clock to the USART
		pmc_enable_periph_clk(ID_USART_TMC_DRV);

		// Set USART_EXT_DRV in SPI mode, with data changing on the falling edge of the clock and captured on the rising edge
		USART_TMC_DRV->US_IDR = ~0u;
		USART_TMC_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		USART_TMC_DRV->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		USART_TMC_DRV->US_BRGR = VARIANT_MCK/DriversSpiClockFrequency;		// set SPI clock frequency
		USART_TMC_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

		// We need a few microseconds of delay here for the USART to sort itself out before we send any data,
		// otherwise the processor generates two short reset pulses on its own NRST pin, and resets itself.
		// 2016-07-07: removed this delay, because we no longer send commands to the TMC2660 drivers immediately.
		//delay(10);

		driversPowered = false;
		for (size_t drive = 0; drive < numTmc2660Drivers; ++drive)
		{
			driverStates[drive].Init(drive, driverSelectPins[drive]);		// axes are mapped straight through to drivers initially
		}
	}

	void SetAxisNumber(size_t drive, uint32_t axisNumber)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetAxisNumber(axisNumber);
		}
	}

	void SetCurrent(size_t drive, float current)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetCurrent(current);
		}
	}

	void EnableDrive(size_t drive, bool en)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].Enable(en);
		}
	}

	uint32_t GetLiveStatus(size_t drive)
	{
		return (drive < numTmc2660Drivers) ? driverStates[drive].ReadLiveStatus() : 0;
	}

	uint32_t GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep)
	{
		return (drive < numTmc2660Drivers) ? driverStates[drive].ReadAccumulatedStatus(bitsToKeep) : 0;
	}

	// Set microstepping or chopper control register
	bool SetMicrostepping(size_t drive, int microsteps, int mode)
	{
		if (drive < numTmc2660Drivers)
		{
			if (mode == ChopperControlRegisterMode && microsteps >= 0)
			{
				driverStates[drive].SetChopConf((uint32_t)microsteps);	// set the chopper control register
				return true;
			}
			else if (microsteps > 0 && (mode == 0 || mode == 1))
			{
				// Set the microstepping. We need to determine how many bits right to shift the desired microstepping to reach 1.
				unsigned int shift = 0;
				unsigned int uSteps = (unsigned int)microsteps;
				while (uSteps > 1)
				{
					uSteps >>= 1;
					++shift;
				}
				if (shift <= 8)
				{
					driverStates[drive].SetMicrostepping(shift, mode != 0);
					return true;
				}
			}
		}
		return false;
	}

	// Get microstepping or chopper control register
	unsigned int GetMicrostepping(size_t drive, int mode, bool& interpolation)
	{
		return (drive < numTmc2660Drivers) ? driverStates[drive].GetMicrostepping(mode, interpolation) : 1;
	}

	// Flag the the drivers have been powered up.
	// Important notes:
	// 1. Before the first call to this function with powered true, you must call Init().
	// 2. This may be called by the tick ISR with powered false, possibly while another call (with powered either true or false) is being executed
	void SetDriversPowered(bool powered)
	{
		const bool wasPowered = driversPowered;
		driversPowered = powered;
		if (powered)
		{
			if (!wasPowered)
			{
				// Power to the drivers has been provided or restored, so we need to enable and re-initialise them
				digitalWrite(GlobalTmcEnablePin, LOW);
				delayMicroseconds(10);

				for (size_t drive = 0; drive < numTmc2660Drivers; ++drive)
				{
					driverStates[drive].WriteAll();
				}
			}
			if (currentDriver == nullptr && numTmc2660Drivers != 0)
			{
				// Kick off the first transfer
				NVIC_EnableIRQ(USART_TMC_DRV_IRQn);
				driverStates[0].StartTransfer();
			}
		}
		else if (wasPowered)
		{
			digitalWrite(GlobalTmcEnablePin, HIGH);			// disable the drivers
		}
	}

	void SetStallThreshold(size_t drive, int sgThreshold)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetStallDetectThreshold(sgThreshold);
		}
	}

	void SetStallFilter(size_t drive, bool sgFilter)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetStallDetectFilter(sgFilter);
		}
	}

	void SetStallMinimumStepsPerSecond(size_t drive, unsigned int stepsPerSecond)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetStallMinimumStepsPerSecond(stepsPerSecond);
		}
	}

	void SetCoolStep(size_t drive, uint16_t coolStepConfig)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetCoolStep(coolStepConfig);
		}
	}

	void AppendStallConfig(size_t drive, StringRef& reply)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].AppendStallConfig(reply);
		}
	}

};	// end namespace

// End




