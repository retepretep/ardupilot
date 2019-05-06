/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RangeFinder.h"
#include "AP_RangeFinder_analog.h"
#include "AP_RangeFinder_PulsedLightLRF.h"
#include "AP_RangeFinder_MaxsonarI2CXL.h"
#include "AP_RangeFinder_MaxsonarSerialLV.h"
#include "AP_RangeFinder_PX4_PWM.h"
#include "AP_RangeFinder_BBB_PRU.h"
#include "AP_RangeFinder_LightWareI2C.h"
#include "AP_RangeFinder_LightWareSerial.h"
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) &&      \
    defined(HAVE_LIBIIO)
#include "AP_RangeFinder_Bebop.h"
#endif
#include "AP_RangeFinder_MAVLink.h"
#include "AP_RangeFinder_LeddarOne.h"
#include "AP_RangeFinder_uLanding.h"
#include "AP_RangeFinder_TeraRangerI2C.h"
#include "AP_RangeFinder_VL53L0X.h"
#include "AP_RangeFinder_NMEA.h"
#include "AP_RangeFinder_Wasp.h"
#include "AP_RangeFinder_Benewake.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, RangeFinder, state[0].type, 0),

    // @Param: _PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Standard
    AP_GROUPINFO("_PIN",     1, RangeFinder, state[0].pin, -1),

    // @Param: _SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_SCALING", 2, RangeFinder, state[0].scaling, 3.0f),

    // @Param: _OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance for analog rangefinders. Offset added to distance in centimeters for PWM and I2C Lidars
    // @Units: V
    // @Increment: 0.001
    // @User: Standard
    AP_GROUPINFO("_OFFSET",  3, RangeFinder, state[0].offset, 0.0f),

    // @Param: _FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Standard
    AP_GROUPINFO("_FUNCTION", 4, RangeFinder, state[0].function, 0),

    // @Param: _MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN_CM",  5, RangeFinder, state[0].min_distance_cm, 20),

    // @Param: _MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_CM",  6, RangeFinder, state[0].max_distance_cm, 700),

    // @Param: _STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Standard
    AP_GROUPINFO("_STOP_PIN", 7, RangeFinder, state[0].stop_pin, -1),

    // @Param: _SETTLE
    // @DisplayName: Rangefinder settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_SETTLE", 8, RangeFinder, state[0].settle_time_ms, 0),

    // @Param: _RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Standard
    AP_GROUPINFO("_RMETRIC", 9, RangeFinder, state[0].ratiometric, 1),

    // @Param: _PWRRNG
    // @DisplayName: Powersave range
    // @Description: This parameter sets the estimated terrain distance in meters above which the sensor will be put into a power saving mode (if available). A value of zero means power saving is not enabled
    // @Units: m
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("_PWRRNG", 10, RangeFinder, _powersave_range, 0),

    // @Param: _GNDCLEAR
    // @DisplayName: Distance (in cm) from the range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 5 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_GNDCLEAR", 11, RangeFinder, state[0].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: _ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ADDR", 23, RangeFinder, state[0].address, 0),

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the first rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the first rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the first rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_POS", 49, RangeFinder, state[0].pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Rangefinder orientation
    // @Description: Orientation of rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 53, RangeFinder, state[0].orientation, ROTATION_PITCH_270),

    // @Group: _
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "_",  57, RangeFinder, backend_var_info[0]),

#if RANGEFINDER_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Advanced
    AP_GROUPINFO("2_TYPE",    12, RangeFinder, state[1].type, 0),

    // @Param: 2_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("2_PIN",     13, RangeFinder, state[1].pin, -1),

    // @Param: 2_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_SCALING", 14, RangeFinder, state[1].scaling, 3.0f),

    // @Param: 2_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("2_OFFSET",  15, RangeFinder, state[1].offset, 0.0f),

    // @Param: 2_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("2_FUNCTION",  16, RangeFinder, state[1].function, 0),

    // @Param: 2_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MIN_CM",  17, RangeFinder, state[1].min_distance_cm, 20),

    // @Param: 2_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_MAX_CM",  18, RangeFinder, state[1].max_distance_cm, 700),

    // @Param: 2_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("2_STOP_PIN", 19, RangeFinder, state[1].stop_pin, -1),

    // @Param: 2_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_SETTLE", 20, RangeFinder, state[1].settle_time_ms, 0),

    // @Param: 2_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("2_RMETRIC", 21, RangeFinder, state[1].ratiometric, 1),

    // @Param: 2_GNDCLEAR
    // @DisplayName: Distance (in cm) from the second range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the second range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_GNDCLEAR", 22, RangeFinder, state[1].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 2_ADDR
    // @DisplayName: Bus address of second rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("2_ADDR", 24, RangeFinder, state[1].address, 0),

    // @Param: 2_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the second rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 2_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the second rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 2_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the second rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("2_POS", 50, RangeFinder, state[1].pos_offset, 0.0f),

    // @Param: 2_ORIENT
    // @DisplayName: Rangefinder 2 orientation
    // @Description: Orientation of 2nd rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("2_ORIENT", 54, RangeFinder, state[1].orientation, ROTATION_PITCH_270),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_", 58, RangeFinder, backend_var_info[1]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 2

    // @Param: 3_TYPE
    // @DisplayName: Third Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Advanced
    AP_GROUPINFO("3_TYPE",    25, RangeFinder, state[2].type, 0),

    // @Param: 3_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("3_PIN",     26, RangeFinder, state[2].pin, -1),

    // @Param: 3_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("3_SCALING", 27, RangeFinder, state[2].scaling, 3.0f),

    // @Param: 3_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("3_OFFSET",  28, RangeFinder, state[2].offset, 0.0f),

    // @Param: 3_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("3_FUNCTION",  29, RangeFinder, state[2].function, 0),

    // @Param: 3_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MIN_CM",  30, RangeFinder, state[2].min_distance_cm, 20),

    // @Param: 3_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_MAX_CM",  31, RangeFinder, state[2].max_distance_cm, 700),

    // @Param: 3_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("3_STOP_PIN", 32, RangeFinder, state[2].stop_pin, -1),

    // @Param: 3_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_SETTLE", 33, RangeFinder, state[2].settle_time_ms, 0),

    // @Param: 3_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("3_RMETRIC", 34, RangeFinder, state[2].ratiometric, 1),

    // @Param: 3_GNDCLEAR
    // @DisplayName: Distance (in cm) from the third range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the third range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_GNDCLEAR", 35, RangeFinder, state[2].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 3_ADDR
    // @DisplayName: Bus address of third rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("3_ADDR", 36, RangeFinder, state[2].address, 0),

    // @Param: 3_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the third rangefinder in body frame. Positive X is forward of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 3_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the third rangefinder in body frame. Positive Y is to the right of the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 3_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the third rangefinder in body frame. Positive Z is down from the origin. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("3_POS", 51, RangeFinder, state[2].pos_offset, 0.0f),

    // @Param: 3_ORIENT
    // @DisplayName: Rangefinder 3 orientation
    // @Description: Orientation of 3rd rangefinder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("3_ORIENT", 55, RangeFinder, state[2].orientation, ROTATION_PITCH_270),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_", 59, RangeFinder, backend_var_info[2]),
#endif

#if RANGEFINDER_MAX_INSTANCES > 3

    // @Param: 4_TYPE
    // @DisplayName: Fourth Rangefinder type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,1:Analog,2:MaxbotixI2C,3:LidarLiteV2-I2C,5:PX4-PWM,6:BBB-PRU,7:LightWareI2C,8:LightWareSerial,9:Bebop,10:MAVLink,11:uLanding,12:LeddarOne,13:MaxbotixSerial,14:TeraRangerI2C,15:LidarLiteV3-I2C,16:VL53L0X,17:NMEA,18:WASP-LRF,19:BenewakeTF02,20:BenewakeTFmini,21:LidarLiteV3HP-I2C
    // @User: Advanced
    AP_GROUPINFO("4_TYPE",    37, RangeFinder, state[3].type, 0),

    // @Param: 4_PIN
    // @DisplayName: Rangefinder pin
    // @Description: Analog pin that rangefinder is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated 'airspeed' port on the end of the board. Set to 11 on PX4 for the analog 'airspeed' port. Set to 15 on the Pixhawk for the analog 'airspeed' port.
    // @Values: -1:Not Used, 0:APM2-A0, 1:APM2-A1, 2:APM2-A2, 3:APM2-A3, 4:APM2-A4, 5:APM2-A5, 6:APM2-A6, 7:APM2-A7, 8:APM2-A8, 9:APM2-A9, 11:PX4-airspeed port, 15:Pixhawk-airspeed port, 64:APM1-airspeed port
    // @User: Advanced
    AP_GROUPINFO("4_PIN",     38, RangeFinder, state[3].pin, -1),

    // @Param: 4_SCALING
    // @DisplayName: Rangefinder scaling
    // @Description: Scaling factor between rangefinder reading and distance. For the linear and inverted functions this is in meters per volt. For the hyperbolic function the units are meterVolts.
    // @Units: m/V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("4_SCALING", 39, RangeFinder, state[3].scaling, 3.0f),

    // @Param: 4_OFFSET
    // @DisplayName: rangefinder offset
    // @Description: Offset in volts for zero distance
    // @Units: V
    // @Increment: 0.001
    // @User: Advanced
    AP_GROUPINFO("4_OFFSET",  40, RangeFinder, state[3].offset, 0.0f),

    // @Param: 4_FUNCTION
    // @DisplayName: Rangefinder function
    // @Description: Control over what function is used to calculate distance. For a linear function, the distance is (voltage-offset)*scaling. For a inverted function the distance is (offset-voltage)*scaling. For a hyperbolic function the distance is scaling/(voltage-offset). The functions return the distance in meters.
    // @Values: 0:Linear,1:Inverted,2:Hyperbolic
    // @User: Advanced
    AP_GROUPINFO("4_FUNCTION",  41, RangeFinder, state[3].function, 0),

    // @Param: 4_MIN_CM
    // @DisplayName: Rangefinder minimum distance
    // @Description: Minimum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MIN_CM",  42, RangeFinder, state[3].min_distance_cm, 20),

    // @Param: 4_MAX_CM
    // @DisplayName: Rangefinder maximum distance
    // @Description: Maximum distance in centimeters that rangefinder can reliably read
	// @Units: cm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_MAX_CM",  43, RangeFinder, state[3].max_distance_cm, 700),

    // @Param: 4_STOP_PIN
    // @DisplayName: Rangefinder stop pin
    // @Description: Digital pin that enables/disables rangefinder measurement for an analog rangefinder. A value of -1 means no pin. If this is set, then the pin is set to 1 to enable the rangefinder and set to 0 to disable it. This can be used to ensure that multiple sonar rangefinders don't interfere with each other.
    // @Values: -1:Not Used,50:Pixhawk AUXOUT1,51:Pixhawk AUXOUT2,52:Pixhawk AUXOUT3,53:Pixhawk AUXOUT4,54:Pixhawk AUXOUT5,55:Pixhawk AUXOUT6,111:PX4 FMU Relay1,112:PX4 FMU Relay2,113:PX4IO Relay1,114:PX4IO Relay2,115:PX4IO ACC1,116:PX4IO ACC2
    // @User: Advanced
    AP_GROUPINFO("4_STOP_PIN", 44, RangeFinder, state[3].stop_pin, -1),

    // @Param: 4_SETTLE
    // @DisplayName: Sonar settle time
    // @Description: The time in milliseconds that the rangefinder reading takes to settle. This is only used when a STOP_PIN is specified. It determines how long we have to wait for the rangefinder to give a reading after we set the STOP_PIN high. For a sonar rangefinder with a range of around 7m this would need to be around 50 milliseconds to allow for the sonar pulse to travel to the target and back again.
    // @Units: ms
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_SETTLE", 45, RangeFinder, state[3].settle_time_ms, 0),

    // @Param: 4_RMETRIC
    // @DisplayName: Ratiometric
    // @Description: This parameter sets whether an analog rangefinder is ratiometric. Most analog rangefinders are ratiometric, meaning that their output voltage is influenced by the supply voltage. Some analog rangefinders (such as the SF/02) have their own internal voltage regulators so they are not ratiometric.
    // @Values: 0:No,1:Yes
    // @User: Advanced
    AP_GROUPINFO("4_RMETRIC", 46, RangeFinder, state[3].ratiometric, 1),

    // @Param: 4_GNDCLEAR
    // @DisplayName: Distance (in cm) from the fourth range finder to the ground
    // @Description: This parameter sets the expected range measurement(in cm) that the fourth range finder should return when the vehicle is on the ground.
    // @Units: cm
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_GNDCLEAR", 47, RangeFinder, state[3].ground_clearance_cm, RANGEFINDER_GROUND_CLEARANCE_CM_DEFAULT),

    // @Param: 4_ADDR
    // @DisplayName: Bus address of fourth rangefinder
    // @Description: This sets the bus address of the sensor, where applicable. Used for the LightWare I2C sensor to allow for multiple sensors on different addresses. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("4_ADDR", 48, RangeFinder, state[3].address, 0),

    // @Param: 4_POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 4_POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced

    // @Param: 4_POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the fourth rangefinder in body frame. Use the zero range datum point if supplied.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("4_POS", 52, RangeFinder, state[3].pos_offset, 0.0f),

    // @Param: 4_ORIENT
    // @DisplayName: Rangefinder 4 orientation
    // @Description: Orientation of 4th range finder
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("4_ORIENT", 56, RangeFinder, state[3].orientation, ROTATION_PITCH_270),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_", 60, RangeFinder, backend_var_info[3]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *RangeFinder::backend_var_info[RANGEFINDER_MAX_INSTANCES];

RangeFinder::RangeFinder(AP_SerialManager &_serial_manager, enum Rotation orientation_default) :
    num_instances(0),
    estimated_terrain_height(0),
    serial_manager(_serial_manager)
{
    // added by peter
    if (ISDOVERBOSEINITPRINTOUTS) {
        printf("called RangeFinder::RangeFinder(...)\n");
    }

    AP_Param::setup_object_defaults(this, var_info);

    // set orientation defaults
    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        state[i].orientation.set_default(orientation_default);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Rangefinder must be singleton");
    }
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _singleton = this;
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void RangeFinder::init(void)
{
    // added by peter
    if (ISDOVERBOSEINITPRINTOUTS) {
        printf("called RangeFinder::init()\n");
    }

    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_distance_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_distance_max = 0;

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void RangeFinder::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == RangeFinder_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = RangeFinder_NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool RangeFinder::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == RANGEFINDER_MAX_INSTANCES) {
        AP_HAL::panic("Too many RANGERS backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a rangefinder is connected. 
 */
void RangeFinder::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
    enum RangeFinder_Type _type = (enum RangeFinder_Type)state[instance].type.get();
    switch (_type) {
    case RangeFinder_TYPE_PLI2C:
    case RangeFinder_TYPE_PLI2CV3:
    case RangeFinder_TYPE_PLI2CV3HP:
        if (!_add_backend(AP_RangeFinder_PulsedLightLRF::detect(1, state[instance], _type))) {
            _add_backend(AP_RangeFinder_PulsedLightLRF::detect(0, state[instance], _type));
        }
        break;
    case RangeFinder_TYPE_MBI2C:
        if (!_add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance],
                                                hal.i2c_mgr->get_device(1, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)))) {
            _add_backend(AP_RangeFinder_MaxsonarI2CXL::detect(state[instance],
                                               hal.i2c_mgr->get_device(0, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR)));
        }
        break;
    case RangeFinder_TYPE_LWI2C:
        if (state[instance].address) {
#ifdef HAL_RANGEFINDER_LIGHTWARE_I2C_BUS
            _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance],
                hal.i2c_mgr->get_device(HAL_RANGEFINDER_LIGHTWARE_I2C_BUS, state[instance].address)));
#else
            if (!_add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance],
                                                                  hal.i2c_mgr->get_device(1, state[instance].address)))) {
                _add_backend(AP_RangeFinder_LightWareI2C::detect(state[instance],
                                                                 hal.i2c_mgr->get_device(0, state[instance].address)));
            }
#endif
        }
        break;
    case RangeFinder_TYPE_TRI2C:
        if (state[instance].address) {
            if (!_add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance],
                                                                   hal.i2c_mgr->get_device(1, state[instance].address)))) {
                _add_backend(AP_RangeFinder_TeraRangerI2C::detect(state[instance],
                                                                  hal.i2c_mgr->get_device(0, state[instance].address)));
            }
        }
        break;
    case RangeFinder_TYPE_VL53L0X:
        if (!_add_backend(AP_RangeFinder_VL53L0X::detect(state[instance],
                                                         hal.i2c_mgr->get_device(1, 0x29)))) {
            _add_backend(AP_RangeFinder_VL53L0X::detect(state[instance],
                                                        hal.i2c_mgr->get_device(0, 0x29)));
        }
        break;
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    case RangeFinder_TYPE_PX4_PWM:
        if (AP_RangeFinder_PX4_PWM::detect()) {
            drivers[instance] = new AP_RangeFinder_PX4_PWM(state[instance], _powersave_range, estimated_terrain_height);
        }
        break;
#endif
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    case RangeFinder_TYPE_BBB_PRU:
        if (AP_RangeFinder_BBB_PRU::detect()) {
            drivers[instance] = new AP_RangeFinder_BBB_PRU(state[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_LWSER:
        if (AP_RangeFinder_LightWareSerial::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LightWareSerial(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_LEDDARONE:
        if (AP_RangeFinder_LeddarOne::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_LeddarOne(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ULANDING:
        if (AP_RangeFinder_uLanding::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_uLanding(state[instance], serial_manager, serial_instance++);
        }
        break;
#if (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || \
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO) && defined(HAVE_LIBIIO)
    case RangeFinder_TYPE_BEBOP:
        if (AP_RangeFinder_Bebop::detect()) {
            drivers[instance] = new AP_RangeFinder_Bebop(state[instance]);
        }
        break;
#endif
    case RangeFinder_TYPE_MAVLink:
        if (AP_RangeFinder_MAVLink::detect()) {
            drivers[instance] = new AP_RangeFinder_MAVLink(state[instance]);
        }
        break;
    case RangeFinder_TYPE_MBSER:
        if (AP_RangeFinder_MaxsonarSerialLV::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_MaxsonarSerialLV(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_ANALOG:
        // note that analog will always come back as present if the pin is valid
        if (AP_RangeFinder_analog::detect(state[instance])) {
            drivers[instance] = new AP_RangeFinder_analog(state[instance]);
        }
        break;
    case RangeFinder_TYPE_NMEA:
        if (AP_RangeFinder_NMEA::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_NMEA(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_WASP:
        if (AP_RangeFinder_Wasp::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Wasp(state[instance], serial_manager, serial_instance++);
        }
        break;
    case RangeFinder_TYPE_BenewakeTF02:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TF02);
        }
        break;
    case RangeFinder_TYPE_BenewakeTFmini:
        if (AP_RangeFinder_Benewake::detect(serial_manager, serial_instance)) {
            drivers[instance] = new AP_RangeFinder_Benewake(state[instance], serial_manager, serial_instance++, AP_RangeFinder_Benewake::BENEWAKE_TFmini);
        }
        break;
    default:
        break;
    }

    // if the backend has some local parameters then make those available in the tree
    if (drivers[instance] && state[instance].var_info) {
        backend_var_info[instance] = state[instance].var_info;
        AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
    }
}

AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == RangeFinder_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

RangeFinder::RangeFinder_Status RangeFinder::status_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return RangeFinder_NotConnected;
    }
    return backend->status();
}

void RangeFinder::handle_msg(mavlink_message_t *msg)
{
    uint8_t i;
    for (i=0; i<num_instances; i++) {
        if ((drivers[i] != nullptr) && (state[i].type != RangeFinder_TYPE_NONE)) {
          drivers[i]->handle_msg(msg);
        }
    }
}

// return true if we have a range finder with the specified orientation
bool RangeFinder::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

// find first range finder instance with the specified orientation
AP_RangeFinder_Backend *RangeFinder::find_instance(enum Rotation orientation) const
{
    for (uint8_t i=0; i<num_instances; i++) {
        AP_RangeFinder_Backend *backend = get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        if (backend->orientation() != orientation) {
            continue;
        }
        return backend;
    }
    return nullptr;
}

uint16_t RangeFinder::distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->distance_cm();
}

uint16_t RangeFinder::voltage_mv_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->voltage_mv();
}

int16_t RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->max_distance_cm();
}

int16_t RangeFinder::min_distance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->min_distance_cm();
}

int16_t RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->ground_clearance_cm();
}

bool RangeFinder::has_data_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t RangeFinder::range_valid_count_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

/*
  returns true if pre-arm checks have passed for all range finders
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool RangeFinder::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != RangeFinder_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}

const Vector3f &RangeFinder::get_pos_offset_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return pos_offset_zero;
    }
    return backend->get_pos_offset();
}

MAV_DISTANCE_SENSOR RangeFinder::get_mav_distance_sensor_type_orient(enum Rotation orientation) const
{
    AP_RangeFinder_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    return backend->get_mav_distance_sensor_type();
}

RangeFinder *RangeFinder::_singleton;

#if IS_USE_CSMAG_RANGEFINDER_WORKAROUND

// added by peter
// TODO: put into Csmag.cpp and make sure csmag files are added to compile path

Csmag::Csmag() {

    csmag_state = new CsmagState();
    csmag_state->time_usec = 0;
    int i;
    for (i = 0; i < CSMAG_INDUCTION_ARRAY_SIZE; i++) {
        csmag_state->induction[i] = CSMAG_INVALID_INDUCTION_VALUE;
    }

    // initing buffers here or in Copter::init_csmag() ?
    induction_value_buffer = new RingBuffer<int32_t>(CSMAG_INDUCTION_VALUE_BUFFER_SIZE);
    induction_value_timestamp_buffer = new RingBuffer<uint64_t>(CSMAG_INDUCTION_VALUE_BUFFER_SIZE);

    uart = UART_FOR_CSMAG_DATA;

    _singleton = this;

    // if (ISDOVERBOSEINITPRINTOUTS) {
    //     printf("called Csmag::Csmag()\n");          // gets called in the very beginning (probably when this class is included)
    // }
}

Csmag *Csmag::_singleton;


//bool Csmag::update(void) {
int Csmag::update(void) {    
    int counter_read_data = 0;
    uint64_t induction_value_timestamp_i = 0;
    int32_t induction_value_i = CSMAG_INVALID_INDUCTION_VALUE;
    int16_t nbytes_i = -1;
    // int n_uart_bytes_left = -1;
    bool is_successfully_read_data = false;

    // is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    // // TODO: while? and parse MAG data in UART buffer line by line? (FIFO?)
    // if (is_successfully_read_data) {
    //     // checking data probably not necessary here
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    // }

    // while (is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i)) {
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    // }

    if (ISDOCSMAGUPDATEDEBUG) {
        hal.console->printf("(Up 10) call Csmag::update()\n");
    }
    
    // variante using the weird for loop, might have another behaviour than expected, but more compact

    // read mag values from uart, as long as there is data in Pixhawk's UART buffer
    // for (n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    //         n_uart_bytes_left > 0;
    //         n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i)
    // ) {
    //     if (ISDOCSMAGUPDATEDEBUG) {
    //         hal.console->printf("read, ");
    //     }
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    //     is_successfully_read_data = true;
    // }

    // read mag values from uart, as long as there is data in Pixhawk's UART buffer
    // TODO: CONTINUE HERE

    for (is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i, nbytes_i);
            //n_uart_bytes_left > 0; // old
            //n_uart_bytes_left >= MAG_INTERFACE_V30_MESSAGE_SIZE;
            is_successfully_read_data || (nbytes_i > 0);
            is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i, nbytes_i)
    ) {
        if (ISDOCSMAGUPDATEDEBUG || ISDOTEMPVERBOSEDEBUG) {
            hal.console->printf("read, ");
        }

        if (is_successfully_read_data) {
            #if ISDOTEMPVERBOSEDEBUG 
                hal.console->printf("induction_value_timestamp_buffer->enqueue(%" PRIu64 ");\n", induction_value_timestamp_i);
                hal.console->printf("induction_value_buffer->enqueue(%" PRId32 ");\n", induction_value_i);
            #endif

            induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
            induction_value_buffer->enqueue(induction_value_i);
            counter_read_data++;
        }        
    }

    // variante: clearer and less compact

    // n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    // while (n_uart_bytes_left > 0) {
    //     if (ISDOCSMAGUPDATEDEBUG) {
    //         hal.console->printf("read, ");
    //     }
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    //     is_successfully_read_data = true;
    //     //
    //     n_uart_bytes_left = Csmag::get_reading(induction_value_timestamp_i, induction_value_i);
    // }

    // 

    if (ISDOCSMAGUPDATEDEBUG) {
        hal.console->printf("(Up 20) \n----\n");
    }

    // old return value of get_reading
    // while (is_successfully_read_data = Csmag::get_reading(induction_value_timestamp_i, induction_value_i)) {
    //     induction_value_timestamp_buffer->enqueue(induction_value_timestamp_i);
    //     induction_value_buffer->enqueue(induction_value_i);
    // }
    // perhaps some status update here, unnecessary for now
    return counter_read_data;
}

bool Csmag::get_reading(uint64_t &induction_timestamp_i, int32_t &induction_value_i, int16_t &nbytes) {
// int Csmag::get_reading(uint64_t &induction_timestamp_i, int32_t &induction_value_i) {

    bool is_successfully_read_data = false;

    if (uart == nullptr) {
        #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
            hal.console->printf("Error! Can't find UART.\n");
        #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
            hal.console->printf("X");
        #endif

        return false;
        // return 0;   // no uart ==> no data available
    }
    
    // old variables from benewake
    // float sum_cm = 0;
    // uint16_t count = 0;
    // uint16_t count_out_of_range = 0;

    if (ISDOMAGDATAREADUARTCHECK) {
        hal.console->printf("(U005) read from UART %p (as hexdump):\n", uart);
    }

    // read any available lines from the lidar
    //int16_t nbytes = uart->available();
    nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t r = uart->read();       // try to read 1 byte from UART

        if (ISDOMAGDATAREADUARTCHECK) {
            if (r == -1) {
                hal.console->printf("(-1) ==> no available data at UART!\n");
            }
        }

        if (r < 0) {                    // read() returned (int16_t) -1 ==> nothing available
            continue;
        }

        uint8_t c = (uint8_t)r;         // otherwise read() returned uint8_t data byte

        if (ISDOMAGDATAREADUARTCHECK) {
            hal.console->printf("0x%02x ", c);
        }

        if (ISDOTEMPVERBOSEDEBUG) {
            hal.console->printf(" (U007) linebuf_len: %d\n", linebuf_len);
        }

        // if buffer is empty and this byte is <magic number>, add to buffer
        if (linebuf_len == 0) {
            if (ISDOTEMPVERBOSEDEBUG) {
                hal.console->printf("(U010) interpreting byte from UART: 0x%02x\n", c);
            }

            if (c == MAG_INTERFACE_V30_MAGIC_NUMBER1) {
                if (ISDOTEMPVERBOSEDEBUG) {
                    hal.console->printf("(U015) matches magic number1\n");
                }
                linebuf[linebuf_len++] = c;
            } else {
                // TODO: invalid MAGInterface message! clear buffer and return false here?
                // incorrect data
                #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                    hal.console->printf("Error!!! Can't interpret MAGInterface data 0x%02x == '%c'. linebuf_len: %d\n", 
                        c, c, (int) linebuf_len);
                #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                    hal.console->printf("?");
                #endif

                if (ISDOTEMPVERBOSEDEBUG) {
                    //hal.console->printf("(U030) interpreting byte from UART: 0x%02x\n", c);

                }
            }
        // } else if (linebuf_len == 1) {
        //     // if buffer has 1 element and this byte is 0x59, add it to buffer
        //     // if not clear the buffer
        //     // TODO:find out why this happens???
        //     if (c == MAG_INTERFACE_V30_MAGIC_NUMBER1) {
        //         linebuf[linebuf_len++] = c;
        //     } else {
        //         linebuf_len = 0;
        //     }
        } else if (linebuf_len == 1) {
            if (ISDOTEMPVERBOSEDEBUG) {
                hal.console->printf("(U020) interpreting byte from UART: 0x%02x\n", c);
            }

            if (c == MAG_INTERFACE_V30_MAGIC_NUMBER0) {
                linebuf[linebuf_len++] = c;
            } else {
                // incorrect data
                #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                    hal.console->printf("Error!!! Can't interpret MAGInterface data 0x%02x == '%c'. linebuf_len: %d\n", 
                        c, c, (int) linebuf_len);
                #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                    hal.console->printf("?");
                #endif

                // TODO: invalid MAGInterface message! clear buffer and return false here?
                if (ISDOTEMPVERBOSEDEBUG) {
                    hal.console->printf("(U030) Reset linebuf_len, return\n");
                }
                linebuf_len = 0;
                // return nbytes;
                return is_successfully_read_data;
            }
        } else {
            // add character to buffer
            linebuf[linebuf_len++] = c;

            if (ISDOTEMPVERBOSEDEBUG) {
                hal.console->printf("(U040) add char to linebuf from UART: 0x%02x\n", c);
            }

            // if buffer now has a full MAGInterface message: try to decode it
            if (linebuf_len == MAG_INTERFACE_V30_MESSAGE_SIZE) {

                if (ISDOTEMPVERBOSEDEBUG) {
                    //hal.console->printf("(U050) linebuf full, interpreting last byte from UART: 0x%02x\n", c);
                    hal.console->printf("(U050) linebuf full\n");
                    hal.console->printf("(U060) checking checksum\n");
                }

                // calculate checksum
                uint8_t checksum = 0;
                // exclude transmitted checksum from checksum calculation
                for (uint8_t i=0; i < (MAG_INTERFACE_V30_MESSAGE_SIZE - 1); i++) {    
                    checksum += linebuf[i];
                }
                // if checksum matches extract contents
                if (checksum == linebuf[MAG_INTERFACE_V30_MESSAGE_SIZE - 1]) {

                    if (ISDOTEMPVERBOSEDEBUG) {
                        hal.console->printf("(U070) checksums match. checksum: %d\n", checksum);
                    }

                    int i;
                    // read timestamp
                    induction_timestamp_i = 0;
                    for (i = 0; i < MAG_INTERFACE_SIZE_INDUCTION_VALUE_TIMESTAMP_I; i++) {
                        induction_timestamp_i |= (uint64_t) linebuf[\
                        MAG_INTERFACE_POS_START_INDUCTION_VALUE_TIMESTAMP_I\
                         + (MAG_INTERFACE_SIZE_INDUCTION_VALUE_TIMESTAMP_I - 1 - i)] << (8 * i);
                    }   // hopefully compiler uses loopunrolling here
                    // TODO: perhaps count out of range? for now we won't handle it here, but on GCS
                    // mag values outside of range are possible due to low pass filter
                    // uint16_t dist = ((uint16_t)linebuf[3] << 8) | linebuf[2];

                    // read induction values
                    induction_value_i = 0;
                    for (i = 0; i < MAG_INTERFACE_SIZE_INDUCTION_VALUE_I; i++) {
                        induction_value_i |= (int32_t) linebuf[\
                        MAG_INTERFACE_POS_START_INDUCTION_VALUE_I\
                         + (MAG_INTERFACE_SIZE_INDUCTION_VALUE_I - 1 - i)] << (8 * i);
                    }
                    is_successfully_read_data = true;
                    // only read 1 line/MAGinterface message for each function call of get_reading()
                    // return is_successfully_read_data;
                    if (ISDOTEMPVERBOSEDEBUG) {
                        hal.console->printf("(U080) return, nbytes: %d, is_successfully_read_data: %d\n",
                            nbytes, is_successfully_read_data);
                    }       
                    // added clear buffer to prevent from buffer overflow 
                    // clear buffer
                    linebuf_len = 0;

                    // return nbytes;
                    return is_successfully_read_data;
                } else {
                    // incorrect checksum
                    #if ISPRINTOUTNOUARTCONNECTIONVERBOSE
                        hal.console->printf("Error!!! Incorrect checksum for MAGinterface data.\n");
                    #elif ISPRINTOUTNOUARTCONNECTIONSIMPLE
                        hal.console->printf("C");
                    #endif
                }
                // clear buffer
                linebuf_len = 0;
            }
        }
    }

    if (ISDOMAGDATAREADUARTCHECK) {
        hal.console->printf("\n(U200) nbytes: %d\n--\n", (int) nbytes);
    }

    // if (count > 0) {
    //     // return average distance of readings
    //     reading_cm = sum_cm / count;
    //     return true;
    // }

    // if (count_out_of_range > 0) {
    //     // if only out of range readings return larger of
    //     // driver defined maximum range for the model and user defined max range + 1m
    //     float model_dist_max_cm = (model_type == BENEWAKE_TFmini) ? BENEWAKE_TFMINI_OUT_OF_RANGE_CM : BENEWAKE_TF02_OUT_OF_RANGE_CM;
    //     reading_cm = MAX(model_dist_max_cm, max_distance_cm() + BENEWAKE_OUT_OF_RANGE_ADD_CM);
    //     return true;
    // }

    // no readings so return false
    return is_successfully_read_data;
    // return nbytes;
}


//bool Csmag::synch_timestamp(uint64_t timestamp) {
    
void Csmag::synch_timestamp(uint64_t timestamp) {
    // bool is_synch_successfully = true;
    // // TODO: check if synch has been successfull

    // build message in linebuf
    char linebuf[MAG_INTERFACE_V30_MESSAGE_SIZE];
    //timestamp_micros = micros();                            // get us timestamp
    int i;
    int checksum;
    linebuf[ 0] = MAG_INTERFACE_V30_MAGIC_NUMBER1;          // byte 1 of magic number (not of linebuf)
    linebuf[ 1] = MAG_INTERFACE_V30_MAGIC_NUMBER0;
    
    linebuf[ 2] = MAG_INTERFACE_V30_MODE_TIMESTAMP_SYNCH;
    
    linebuf[ 3] = GET_BYTE_FROM_NUMBER(timestamp, 7);       // higher uint32_t 0 if timestamp is uint32_t
    linebuf[ 4] = GET_BYTE_FROM_NUMBER(timestamp, 6);
    linebuf[ 5] = GET_BYTE_FROM_NUMBER(timestamp, 5);
    linebuf[ 6] = GET_BYTE_FROM_NUMBER(timestamp, 4);
    
    linebuf[ 7] = GET_BYTE_FROM_NUMBER(timestamp, 3);
    linebuf[ 8] = GET_BYTE_FROM_NUMBER(timestamp, 2);
    linebuf[ 9] = GET_BYTE_FROM_NUMBER(timestamp, 1);
    linebuf[10] = GET_BYTE_FROM_NUMBER(timestamp, 0);

    linebuf[11] = 0;
    linebuf[12] = 0;
    linebuf[13] = 0;
    linebuf[14] = 0;

    for (i = 0, checksum = 0; i < MAG_INTERFACE_V30_MESSAGE_SIZE-1; i++) { // exclude checksum's slot itself
        checksum += linebuf[i];
    }
    linebuf[15] = (uint8_t) (checksum & 0xFF);

    // send message via serial port
    //Serial.print(linebuf);      // problem: 0-terminated strings
    for (i = 0; i < MAG_INTERFACE_V30_MESSAGE_SIZE; i++) {
        uart->printf("%c", linebuf[i]);
        // uart->printf(&(linebuf[i]));
        //Serial.write(linebuf[i]);
    }
}



#if IS_USE_CSMAGSTATEBUFFER

CsmagStateBuffer::CsmagStateBuffer() {

    buf = new Csmag::CsmagState*[CSMAG_BUFFER_SIZE];
    int i;
    for (i = 0; i < CSMAG_BUFFER_SIZE; i++) {
        buf[i] = nullptr;
    }
    object_counter = 0;
    first_index = INVALID_INDEX;
    next_index = 0;
    // set all available buffer slots as free
    is_free_mask = (1 << CSMAG_BUFFER_SIZE) - 1;    // get CSMAG_BUFFER_SIZE bits, rest is 0 (not free)

    // ..

    _singleton = this;
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
bool CsmagStateBuffer::push(Csmag::CsmagState *new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("ERROR! CsmagStateBuffer buffer overflow\n");
        //throw "CsmagStateBuffer buffer overflow";   // throw disabled
        return false;
    }
    // TODO: perhaps check if it is actually free (in mask)
    buf[next_index] = new_obj;
    if (object_counter == 0) {
        first_index = next_index;   // the new first index, if no object has been stored before
    }
    mark_occupied(next_index);
    //next_index++;
    next_index = (next_index + 1) % CSMAG_BUFFER_SIZE;
    object_counter++;
    //
    return true;
}

// pop object at relativeIndex (counting from first_index with 0), defaultly pop first object
Csmag::CsmagState* CsmagStateBuffer::pop(int relative_index) {
    //throw "This is not implemented yet";
    printf("ERROR! This is not implemented yet\n");
    return nullptr;
    // TODO: implement defragmentation if objects from the middle are popped

    int absolute_index = (first_index + relative_index) % CSMAG_BUFFER_SIZE;
    // check is there is actually an object
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! CsmagStateBuffer does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
    //first_index++;
    first_index = (first_index + 1) % CSMAG_BUFFER_SIZE;
    object_counter--;
    return buf[absolute_index];
}

// pop first object at (relative_index 0, counting from first_index with 0)
Csmag::CsmagState* CsmagStateBuffer::pop() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % CSMAG_BUFFER_SIZE;
    // check is there is actually an object
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! CsmagStateBuffer does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
    //first_index++;
    first_index = (first_index + 1) % CSMAG_BUFFER_SIZE;
    object_counter--;
    return buf[absolute_index];
}

// print n as binary digits on console
void CsmagStateBuffer::print_bits(uint32_t n) {
    uint32_t p;
    for (p = 1 << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

void CsmagStateBuffer::print_info() {
    printf("\n");
    printf("info of CsmagStateBuffer object at %p:\n", this);
    printf("contained objects: %d of max %d\n", object_counter, CSMAG_BUFFER_SIZE);
    printf("first_index: %d, next_index: %d\n", first_index, next_index);
    printf("is_free_mask: 0x%x\n", is_free_mask);
    printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
    printf("\n");
}

CsmagStateBuffer *CsmagStateBuffer::_singleton;

#endif


// TODO: test RingBuffer<T> class and put it into another file
// removed RingBuffer<T> definition to RangeFinder.h, because of 
//  "undefined reference to RingBuffer<int>"-compile error
//  see https://bytefreaks.net/programming-2/c/c-undefined-reference-to-templated-class-function


// OLD (deprecated) BEGIN
// template<typename T>
// RingBuffer<T>::RingBuffer(int _buffer_size) {
//     buffer_size = _buffer_size;
// // template<typename T>
// // RingBuffer<T>::RingBuffer(const int &_buffer_size) :
// //     buffer_size(_buffer_size) {

//     buf = new T[buffer_size];                          // should work in C++, even with dynamic variable
    
//     int i;
//     for (i = 0; i < buffer_size; i++) {
//         buf[i] = 0;                                     // TODO: mark as invalid???
//     }
//     object_counter = 0;
//     first_index = INVALID_INDEX;
//     next_index = 0;
//     // set all available buffer slots as free
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     is_free_mask = (1 << buffer_size) - 1;    // get buffer_size bits, rest is 0 (not free)
// #endif

//     // ..

//     _singleton = this;
// }

// // push object to the end of ring queue buffer
// // return true if pushing went successfully
// template<typename T>
// bool RingBuffer<T>::enqueue(T new_obj) {
//     //bool ret = false;
//     if (is_full()) {
//         // TODO: overwrite first object
//         printf("WARNING! RingBuffer<T> buffer overflow, old objects get overwritten\n");
//         //throw "CsmagStateBuffer buffer overflow";   // throw disabled
//         //return false;
//     }
//     // TODO: perhaps check if it is actually free (in mask)
//     buf[next_index] = new_obj;
//     if (object_counter == 0) {
//         first_index = next_index;   // the new first index, if no object has been stored before
//     }
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     mark_occupied(next_index);
// #endif
//     //next_index++;
//     next_index = (next_index + 1) % buffer_size;
//     object_counter++;
//     //
//     return true;
// }

// // pop first object at (relative_index 0, counting from first_index with 0)
// template<typename T>
// T RingBuffer<T>::dequeue() {
//     int relative_index = 0;                     
//     int absolute_index = (first_index + relative_index) % buffer_size;
//     // check is there is actually an object
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     if (is_free(absolute_index)) {
//         //throw "CsmagStateBuffer does not contain an object at the given relative_index";
//         printf("ERROR! RingBuffer does not contain an object at the given relative_index\n");
//     }
//     //
//     mark_free(absolute_index);
// #endif
//     //first_index++;
//     first_index = (first_index + 1) % buffer_size;
//     object_counter--;
//     return buf[absolute_index];
// }

// // print n as binary digits on console
// template<typename T>
// void RingBuffer<T>::print_bits(uint32_t n) {
//     uint32_t p;
//     for (p = 1 << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
//         printf("%d", (bool) (n & p));
//     }
// }

// template<typename T>
// void RingBuffer<T>::print_info() {
//     printf("\n");
//     printf("info of RingBuffer object at %p:\n", this);
//     printf("contained objects: %d of max %d\n", object_counter, buffer_size);
//     printf("first_index: %d, next_index: %d\n", first_index, next_index);
// #if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
//     printf("is_free_mask: 0x%x\n", is_free_mask);
//     printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
// #endif
//     printf("\n");
// }

// template<typename T>                                                    // TODO: verify: does this work?
// RingBuffer<T> *RingBuffer<T>::_singleton;

// OLD (deprecated) END

#if IS_USE_RINGBUFFER_SINGLETON_CLASSES

// since there seems to be some problem with RingBuffer using templates, let's try fixed data type int32_t for induction values:

RingBufferInt32::RingBufferInt32(int _buffer_size) {
    buffer_size = _buffer_size;
// template<typename int32_t>
// RingBufferInt32::RingBuffer(const int &_buffer_size) :
//     buffer_size(_buffer_size) {

    buf = new int32_t[buffer_size];                          // should work in C++, even with dynamic variable
    
    int i;
    for (i = 0; i < buffer_size; i++) {
        buf[i] = 0;                                     // TODO: mark as invalid???
    }
    object_counter = 0;
    first_index = INVALID_INDEX;
    next_index = 0;
    // set all available buffer slots as free
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    is_free_mask = (1 << buffer_size) - 1;    // get buffer_size bits, rest is 0 (not free)
#endif

    // ..

    _singleton = this;
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
bool RingBufferInt32::enqueue(int32_t new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("WARNING! RingBufferInt32 buffer overflow, old objects get overwritten\n");
        //throw "CsmagStateBuffer buffer overflow";   // throw disabled
        //return false;
    }
    // TODO: perhaps check if it is actually free (in mask)
    buf[next_index] = new_obj;
    if (object_counter == 0) {
        first_index = next_index;   // the new first index, if no object has been stored before
    }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    mark_occupied(next_index);
#endif
    //next_index++;
    next_index = (next_index + 1) % buffer_size;
    object_counter++;
    //
    return true;
}

// pop first object at (relative_index 0, counting from first_index with 0)
int32_t RingBufferInt32::dequeue() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % buffer_size;
    // check is there is actually an object
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! RingBufferInt32 does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
#endif
    //first_index++;
    first_index = (first_index + 1) % buffer_size;
    object_counter--;
    return buf[absolute_index];
}

// print n as binary digits on console
template<typename INTTYPE>
void RingBufferInt32::print_bits(INTTYPE n) {
    INTTYPE p;
    for (p = ( (INTTYPE) 1) << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

void RingBufferInt32::print_info() {
    printf("\n");
    printf("info of RingBuffer object at %p:\n", this);
    printf("contained objects: %d of max %d\n", object_counter, buffer_size);
    printf("first_index: %d, next_index: %d\n", first_index, next_index);
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    printf("is_free_mask: 0x%x\n", is_free_mask);
    printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
#endif
    printf("\n");
}

RingBufferInt32 *RingBufferInt32::_singleton;



// CONTINUE HERE
//InductionValueBuffer *InductionValueBuffer




// since there seems to be some problem with RingBuffer using templates, let's try fixed data type int64_t for induction values:

RingBufferUInt64::RingBufferUInt64(int _buffer_size) {
    buffer_size = _buffer_size;

    buf = new uint64_t[buffer_size];                          // should work in C++, even with dynamic variable
    
    int i;
    for (i = 0; i < buffer_size; i++) {
        buf[i] = 0;                                     // TODO: mark as invalid???
    }
    object_counter = 0;
    first_index = INVALID_INDEX;
    next_index = 0;
    // set all available buffer slots as free
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    is_free_mask = (1 << buffer_size) - 1;    // get buffer_size bits, rest is 0 (not free)
#endif

    // ..

    _singleton = this;
}

// push object to the end of ring queue buffer
// return true if pushing went successfully
bool RingBufferUInt64::enqueue(uint64_t new_obj) {
    //bool ret = false;
    if (is_full()) {
        // TODO: overwrite first object
        printf("ERROR! RingBufferUInt64 buffer overflow\n");
        //throw "CsmagStateBuffer buffer overflow";   // throw disabled
        return false;
    }
    // TODO: perhaps check if it is actually free (in mask)
    buf[next_index] = new_obj;
    if (object_counter == 0) {
        first_index = next_index;   // the new first index, if no object has been stored before
    }
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    mark_occupied(next_index);
#endif
    //next_index++;
    next_index = (next_index + 1) % buffer_size;
    object_counter++;
    //
    return true;
}

// pop first object at (relative_index 0, counting from first_index with 0)
uint64_t RingBufferUInt64::dequeue() {
    int relative_index = 0;                     
    int absolute_index = (first_index + relative_index) % buffer_size;
    // check is there is actually an object
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    if (is_free(absolute_index)) {
        //throw "CsmagStateBuffer does not contain an object at the given relative_index";
        printf("ERROR! RingBufferUInt64 does not contain an object at the given relative_index\n");
    }
    //
    mark_free(absolute_index);
#endif
    //first_index++;
    first_index = (first_index + 1) % buffer_size;
    object_counter--;
    return buf[absolute_index];
}

// print n as binary digits on console
template<typename INTTYPE>
void RingBufferUInt64::print_bits(INTTYPE n) {
    INTTYPE p;
    for (p = ( (INTTYPE) 1) << (8 * sizeof(n) - 1); p > 0; p >>= 1) {
        printf("%d", (bool) (n & p));
    }
}

void RingBufferUInt64::print_info() {
    printf("\n");
    printf("info of RingBuffer object at %p:\n", this);
    printf("contained objects: %d of max %d\n", object_counter, buffer_size);
    printf("first_index: %d, next_index: %d\n", first_index, next_index);
#if IS_USE_IS_FREE_MASK_FOR_RINGBUFFER
    printf("is_free_mask: 0x%x\n", is_free_mask);
    printf("is_free_mask: 0b"); print_bits(is_free_mask); printf("\n");
#endif
    printf("\n");
}

RingBufferUInt64 *RingBufferUInt64::_singleton;

#endif 

#endif