#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_BMS.h"
#include <../ArduCopter/Copter.h>
//extern const AP_HAL::HAL& hal;

// Constructor
AP_BattMonitor_BMS::AP_BattMonitor_BMS(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{


    // always healthy
    _state.healthy = true;
}

// read - read the voltage and current
void
AP_BattMonitor_BMS::read()
{
    // this copes with changing the pin at runtime
    // get voltage
    _state.voltage = copter.serial_concentrator.bms_state.voltages/1000.0;
    _state.current_amps = copter.serial_concentrator.bms_state.current_battery/1000.0;
    _state.consumed_mah = copter.serial_concentrator.bms_state.capacity_remaining;
    _state.last_time_micros = copter.serial_concentrator.bms_state.time_s;
    _state.temperature = copter.serial_concentrator.bms_state.bat_temp1;
    _state.temperature_time = AP_HAL::millis();
    for(int i=0;i<12;i++)
    {
        _state.cell_voltages.cells[i] = copter.serial_concentrator.bms_state.cell_vot[i];

    }

}


