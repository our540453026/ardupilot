#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"
#include <AP_SerialConcentrator/AP_SerialConcentrator.h>


// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

class AP_BattMonitor_BMS : public AP_BattMonitor_Backend
{
public:

    AP_BattMonitor_BMS(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);
    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;
    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }
    bool has_cell_voltages() const override{ return true; }

        // returns true if battery monitor provides temperature
    //bool has_temperature() const override{ return true; }
    void init(void) override {}
    AP_SerialConcentrator *serialConc;
protected:


};
