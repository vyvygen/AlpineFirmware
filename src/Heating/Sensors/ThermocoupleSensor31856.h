/*
 * ThermocoupleSensor31856.h
 *
 *  Created on: 27 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_THERMOCOUPLESENSOR31856_H_
#define SRC_HEATING_SENSORS_THERMOCOUPLESENSOR31856_H_

#include "SpiTemperatureSensor.h"

class ThermocoupleSensor31856 : public SpiTemperatureSensor
{
public:
	ThermocoupleSensor31856(unsigned int channel);
	bool Configure(unsigned int mCode, unsigned int heater, GCodeBuffer& gb, StringRef& reply, bool& error) override;
	void Init() override;
	TemperatureError GetTemperature(float& t) override;

private:
	TemperatureError TryInitThermocouple() const;

	uint8_t cr0;
	uint8_t thermocoupleType;
};

#endif /* SRC_HEATING_SENSORS_THERMOCOUPLESENSOR31856_H_ */
