# roboteq_bms

## Params
* port (string)
  * default: /dev/ttyUSB_ROBOTEQ_BMS

## Topics
### Publishers

* ~battery_status (robotnik_msgs/BatteryStatus)
  * Publish all data received by BMS

* ~bms_temperature (std_msgs/Int32)
  * Publish the temperature of the BMS

* ~cell_currents (std_msgs/String)
  * Description

* ~cell_voltages (std_msgs/String)
  * Description

* ~state (robotnik_msgs/State)
  * Publish the state of the node

* ~status_flags (std_msgs/String)
  * Description
