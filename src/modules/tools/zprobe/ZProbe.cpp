/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ZProbe.h"

#include "Kernel.h"
#include "BaseSolution.h"
#include "Config.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "Conveyor.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "SlowTicker.h"
#include "Planner.h"
#include "SerialMessage.h"
#include "PublicDataRequest.h"
#include "EndstopsPublicAccess.h"
#include "PublicData.h"
#include "LevelingStrategy.h"
#include "StepTicker.h"
#include "utils.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "ToolManager.h"
#include "ToolManagerPublicAccess.h"
//#include "wait_api.h"

// strategies we know about
#include "DeltaCalibrationStrategy.h"
#include "ThreePointStrategy.h"
#include "DeltaGridStrategy.h"
#include "CartGridStrategy.h"

#define enable_checksum          CHECKSUM("enable")
#define probe_pin_checksum       CHECKSUM("probe_pin")
#define probe2_pin_checksum       CHECKSUM("probe2_pin")
#define debounce_ms_checksum     CHECKSUM("debounce_ms")
#define slow_feedrate_checksum   CHECKSUM("slow_feedrate")
#define fast_feedrate_checksum   CHECKSUM("fast_feedrate")
#define return_feedrate_checksum CHECKSUM("return_feedrate")
#define probe_height_checksum    CHECKSUM("probe_height")
#define disable_check_probe_checksum    CHECKSUM("disable_check")
#define gamma_max_checksum       CHECKSUM("gamma_max")
#define max_z_checksum           CHECKSUM("max_z")
#define reverse_z_direction_checksum CHECKSUM("reverse_z")
#define dwell_before_probing_checksum CHECKSUM("dwell_before_probing")
#define home_offset_checksum     CHECKSUM("home_offset")
#define home_offset2_checksum     CHECKSUM("home_offset2")
#define calibrate_pin_checksum      CHECKSUM("calibrate_pin")
#define sensor_on_pin_checksum      CHECKSUM("sensor_on_pin")
#define x_pos_0_checksum           CHECKSUM("x_pos")
#define y_pos_0_checksum           CHECKSUM("y_pos")
#define x_pos_1_checksum           CHECKSUM("x_pos_1")
#define y_pos_1_checksum           CHECKSUM("y_pos_1")

#define mount_turns_mm_checksum     CHECKSUM("mount.turns_mm")
#define mount_pos1_x_checksum           CHECKSUM("mount_pos1_x")
#define mount_pos1_y_checksum           CHECKSUM("mount_pos1_y")
#define mount_pos2_x_checksum           CHECKSUM("mount_pos2_x")
#define mount_pos2_y_checksum           CHECKSUM("mount_pos2_y")
#define mount_pos3_x_checksum           CHECKSUM("mount_pos3_x")
#define mount_pos3_y_checksum           CHECKSUM("mount_pos3_y")

// from endstop section
#define delta_homing_checksum    CHECKSUM("delta_homing")
#define rdelta_homing_checksum    CHECKSUM("rdelta_homing")

#define probe_up_checksum    CHECKSUM("probe_up")
#define probe_down_checksum    CHECKSUM("probe_down")
#define probe2_up_checksum    CHECKSUM("probe2_up")
#define probe2_down_checksum    CHECKSUM("probe2_down")

#define cam_speed_checksum    CHECKSUM("cam_speed")
#define cam_steps_degree_checksum    CHECKSUM("cam_steps_degree")
#define cam_angle_offset_checksum    CHECKSUM("cam_angle_offset")
#define cam_pin_checksum    CHECKSUM("cam_pin")
#define cam_tool0_checksum    CHECKSUM("cam_tool0")
#define cam_tool1_checksum    CHECKSUM("cam_tool1")
#define cam_neutral_checksum    CHECKSUM("cam_neutral")

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define S_RAISED 1
#define S_NEUTRAL 0
#define S_LOWERED -1
#define S_CENTER 2

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

#define abs(a) ((a<0) ? -a : a)

void ZProbe::wait(int sec)
{
	char buf[32];
	int n = 0;
	n = snprintf(buf, sizeof(buf), "G4 S%d", sec);
	string g2(buf, n);
	Gcode gc2(g2, &(StreamOutput::NullStream));
	THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);

	//let everything settle down
	THEKERNEL->conveyor->wait_for_idle();
}


void ZProbe::on_module_loaded()
{
    // if the module is disabled -> do nothing
    if(!THEKERNEL->config->value( zprobe_checksum, enable_checksum )->by_default(false)->as_bool()) {
        // as this module is not needed free up the resource
        delete this;
        return;
    }

    // load settings
    this->config_load();
    // register event-handlers
    register_for_event(ON_GCODE_RECEIVED);


    // we read the probe in this timer
    probing= false;
    //reset the sensor state
    reset_sensor_state();

	this->cam_position = -1;

    THEKERNEL->slow_ticker->attach(1000, this, &ZProbe::read_probe);
}

void ZProbe::config_load()
{
    this->pin.from_string( THEKERNEL->config->value(zprobe_checksum, probe_pin_checksum)->by_default("nc" )->as_string())->as_input();
    this->pin2.from_string( THEKERNEL->config->value(zprobe_checksum, probe2_pin_checksum)->by_default("nc" )->as_string())->as_input();
    this->debounce_ms    = THEKERNEL->config->value(zprobe_checksum, debounce_ms_checksum)->by_default(0  )->as_number();

    this->probe_up_val    = THEKERNEL->config->value(zprobe_checksum, probe_up_checksum)->by_default(0  )->as_number();
    this->probe_down_val    = THEKERNEL->config->value(zprobe_checksum, probe_down_checksum)->by_default(0  )->as_number();
    this->probe2_up_val    = THEKERNEL->config->value(zprobe_checksum, probe2_up_checksum)->by_default(0  )->as_number();
    this->probe2_down_val    = THEKERNEL->config->value(zprobe_checksum, probe2_down_checksum)->by_default(0  )->as_number();

	this->disable_check_probe    = THEKERNEL->config->value(zprobe_checksum, disable_check_probe_checksum)->by_default(true )->as_bool();

    // set the active pin to the first one
    this->active_pin = this->pin;
    this->active_tool = 0;

    //reset the delta z value
    this->tool_delta = 0;

    // get strategies to load
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, leveling_strategy_checksum);
    for( auto cs : modules ){
        if( THEKERNEL->config->value(leveling_strategy_checksum, cs, enable_checksum )->as_bool() ){
            bool found= false;
            LevelingStrategy *ls= nullptr;

            // check with each known strategy and load it if it matches
            switch(cs) {
                case delta_calibration_strategy_checksum:
                    ls= new DeltaCalibrationStrategy(this);
                    found= true;
                    break;

                case three_point_leveling_strategy_checksum:
                    // NOTE this strategy is mutually exclusive with the delta calibration strategy
                    ls= new ThreePointStrategy(this);
                    found= true;
                    break;

                case delta_grid_leveling_strategy_checksum:
                    ls= new DeltaGridStrategy(this);
                    found= true;
                    break;

                case cart_grid_leveling_strategy_checksum:
                    ls= new CartGridStrategy(this);
                    found= true;
                    break;
            }
            if(found) {
                if(ls->handleConfig()) {
                    this->strategies.push_back(ls);
                }else{
                    delete ls;
                }
            }
        }
    }

    // need to know if we need to use delta kinematics for homing
    this->is_delta = THEKERNEL->config->value(delta_homing_checksum)->by_default(false)->as_bool();
    this->is_rdelta = THEKERNEL->config->value(rdelta_homing_checksum)->by_default(false)->as_bool();

    // default for backwards compatibility add DeltaCalibrationStrategy if a delta
    // may be deprecated
    if(this->strategies.empty()) {
        if(this->is_delta) {
            this->strategies.push_back(new DeltaCalibrationStrategy(this));
            this->strategies.back()->handleConfig();
        }
    }

    this->probe_height  = THEKERNEL->config->value(zprobe_checksum, probe_height_checksum)->by_default(5.0F)->as_number();
    this->slow_feedrate = THEKERNEL->config->value(zprobe_checksum, slow_feedrate_checksum)->by_default(5)->as_number(); // feedrate in mm/sec
    this->fast_feedrate = THEKERNEL->config->value(zprobe_checksum, fast_feedrate_checksum)->by_default(100)->as_number(); // feedrate in mm/sec
    this->return_feedrate = THEKERNEL->config->value(zprobe_checksum, return_feedrate_checksum)->by_default(0)->as_number(); // feedrate in mm/sec
    this->reverse_z     = THEKERNEL->config->value(zprobe_checksum, reverse_z_direction_checksum)->by_default(false)->as_bool(); // Z probe moves in reverse direction
    this->max_z         = THEKERNEL->config->value(zprobe_checksum, max_z_checksum)->by_default(NAN)->as_number(); // maximum zprobe distance
    this->home_offset         = THEKERNEL->config->value(zprobe_checksum, home_offset_checksum)->by_default(0.0F)->as_number(); // z home offset
    this->home_offset2         = THEKERNEL->config->value(zprobe_checksum, home_offset2_checksum)->by_default(0.0F)->as_number(); // z home offset2
    this->x_pos_0         = THEKERNEL->config->value(zprobe_checksum, x_pos_0_checksum)->by_default(0.0F)->as_number(); // x position for probing
    this->y_pos_0         = THEKERNEL->config->value(zprobe_checksum, y_pos_0_checksum)->by_default(0.0F)->as_number(); // y position for probing
	this->x_pos_1         = THEKERNEL->config->value(zprobe_checksum, x_pos_1_checksum)->by_default(this->x_pos_0)->as_number(); // x position for probing
    this->y_pos_1         = THEKERNEL->config->value(zprobe_checksum, y_pos_1_checksum)->by_default(this->y_pos_0)->as_number(); // y position for probing

	this->mount_turns_mm  = THEKERNEL->config->value(zprobe_checksum, mount_turns_mm_checksum)->by_default(0.91F)->as_number(); // mount points turns per mm
	this->mount_pos1[0]  = THEKERNEL->config->value(zprobe_checksum, mount_pos1_x_checksum)->by_default(125.0F)->as_number(); // mount point1 x
	this->mount_pos1[1]  = THEKERNEL->config->value(zprobe_checksum, mount_pos1_y_checksum)->by_default(410.0F)->as_number(); // mount point1 y
	this->mount_pos2[0]  = THEKERNEL->config->value(zprobe_checksum, mount_pos2_x_checksum)->by_default(485.0F)->as_number(); // mount point2 x
	this->mount_pos2[1]  = THEKERNEL->config->value(zprobe_checksum, mount_pos2_y_checksum)->by_default(410.0F)->as_number(); // mount point2 y
	this->mount_pos3[0]  = THEKERNEL->config->value(zprobe_checksum, mount_pos3_x_checksum)->by_default(300.0F)->as_number(); // mount point3 x
	this->mount_pos3[1]  = THEKERNEL->config->value(zprobe_checksum, mount_pos3_y_checksum)->by_default(110.0F)->as_number(); // mount point3 y

    this->calibrate_pin.from_string( THEKERNEL->config->value(zprobe_checksum, calibrate_pin_checksum)->by_default("nc" )->as_string())->as_output();
    this->sensor_on_pin.from_string( THEKERNEL->config->value(zprobe_checksum, sensor_on_pin_checksum)->by_default("nc" )->as_string())->as_output();

    if(isnan(this->max_z)){
        this->max_z = THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number(); // maximum zprobe distance
    }
    this->dwell_before_probing = THEKERNEL->config->value(zprobe_checksum, dwell_before_probing_checksum)->by_default(0)->as_number(); // dwell time in seconds before probing

	this->cam_steps_degree = THEKERNEL->config->value(zprobe_checksum, cam_steps_degree_checksum)->by_default(302.00F)->as_number(); // steps per degree for the cam
	this->cam_speed = THEKERNEL->config->value(zprobe_checksum, cam_speed_checksum)->by_default(360.00F)->as_number(); // degrees per second
	this->cam_tool0 = THEKERNEL->config->value(zprobe_checksum, cam_tool0_checksum)->by_default(180.00F)->as_number(); // steps per degree for the cam
	this->cam_tool1 = THEKERNEL->config->value(zprobe_checksum, cam_tool1_checksum)->by_default(0.00F)->as_number(); // steps per degree for the cam
	this->cam_neutral = THEKERNEL->config->value(zprobe_checksum, cam_neutral_checksum)->by_default(90.00F)->as_number(); // steps per degree for the cam
	this->cam_angle_offset = THEKERNEL->config->value(zprobe_checksum, cam_angle_offset_checksum)->by_default(-35.00F)->as_number(); // steps per degree for the cam


	this->cam_pin= new Pin();
	this->cam_pin->from_string(THEKERNEL->config->value(zprobe_checksum, cam_pin_checksum)->by_default("nc")->as_string())->as_output();
	if(!this->cam_pin->connected()) {
		delete this->cam_pin;
		this->cam_pin= nullptr;
	}

}

uint32_t ZProbe::read_probe(uint32_t dummy)
{
    if(!probing || probe_detected) return 0;

    // we check all axis as it maybe a G38.2 X10 for instance, not just a probe in Z
    if(STEPPER[X_AXIS]->is_moving() || STEPPER[Y_AXIS]->is_moving() || STEPPER[Z_AXIS]->is_moving()) {
        // if it is moving then we check the probe, and debounce it
        if(this->active_pin.get() != invert_probe) {
            if(debounce < debounce_ms) {
                debounce++;
            } else {
                // we signal the motors to stop, which will preempt any moves on that axis
                // we do all motors as it may be a delta
                for(auto &a : THEROBOT->actuators) a->stop_moving();
                probe_detected= true;
                debounce= 0;
            }

        } else {
            // The endstop was not hit yet
            debounce= 0;
        }
    }

    return 0;
}

// single probe in Z with custom feedrate
// returns boolean value indicating if probe was triggered
bool ZProbe::run_probe(float& mm, float feedrate, float max_dist, bool reverse)
{
    if(dwell_before_probing > .0001F) safe_delay_ms(dwell_before_probing*1000);

    if(this->active_pin.get()) {
        // probe already triggered so abort
        return false;
    }


    float maxz= max_dist < 0 ? this->max_z*2 : max_dist;

    probing= true;
    probe_detected= false;
    debounce= 0;

    // save current actuator position so we can report how far we moved
    float z_start_pos= THEROBOT->actuators[Z_AXIS]->get_current_position();

    // move Z down
    bool dir= (!reverse_z != reverse); // xor
    float delta[3]= {0,0,0};
    delta[Z_AXIS]= dir ? -maxz : maxz;
    THEROBOT->delta_move(delta, feedrate, 3);

    // wait until finished
    THECONVEYOR->wait_for_idle();

    // now see how far we moved, get delta in z we moved
    // NOTE this works for deltas as well as all three actuators move the same amount in Z
    mm= z_start_pos - THEROBOT->actuators[2]->get_current_position();

    // set the last probe position to the actuator units moved during this home
    THEROBOT->set_last_probe_position(std::make_tuple(0, 0, mm, probe_detected?1:0));

    probing= false;

    if(probe_detected) {
        // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
        THEROBOT->reset_position_from_current_actuator_position();
    }

    return probe_detected;
}

// do probe then return to start position
bool ZProbe::run_probe_return(float& mm, float feedrate, float max_dist, bool reverse)
{
    float save_z_pos= THEROBOT->get_axis_position(Z_AXIS);

    bool ok= run_probe(mm, feedrate, max_dist, reverse);

    // move probe back to where it was
    float fr;
    if(this->return_feedrate != 0) { // use return_feedrate if set
        fr = this->return_feedrate;
    } else {
        fr = this->slow_feedrate*2; // nominally twice slow feedrate
        if(fr > this->fast_feedrate) fr = this->fast_feedrate; // unless that is greater than fast feedrate
    }

    // absolute move back to saved starting position
    coordinated_move(NAN, NAN, save_z_pos, fr, false);

    return ok;
}

bool ZProbe::doProbeAt(float &mm, float x, float y)
{
    // move to xy
    coordinated_move(x, y, NAN, getFastFeedrate());
    return run_probe_return(mm, slow_feedrate, -1, false);
}

void ZProbe::set_sensor_state(Gcode *gcode, int mode)
{

  if (sensor_mode != mode) {
	gcode->stream->printf("Setting sensor state to %d\n", mode);
    switch (mode) {
      case 0:
        this->sensor_on_pin.set(false);
        this->calibrate_pin.set(false);
        break;
      case 1:
        this->sensor_on_pin.set(true);
        this->calibrate_pin.set(false);
        break;
      case 2:
        this->sensor_on_pin.set(false);
        this->calibrate_pin.set(true);
        break;
      case 3:
        this->sensor_on_pin.set(true);
        this->calibrate_pin.set(true);
        break;
    }
  }

  sensor_mode = mode;
}

void ZProbe::set_active_probe(int pval)
{
  if (pval != 0) {
    this->active_pin = this->pin2;
  } else {
    this->active_pin = this->pin;
  }
}

void ZProbe::store_delta()
{
  float newdata[3];
  newdata[0] = 1; //tool number
  newdata[1] = Z_AXIS; //axis
  newdata[2] = this->tool_delta;  //new value

  PublicData::set_value( tool_manager_checksum, set_offset_checksum, newdata);
//  if (!ok) {
//      THEKERNEL->streams->printf("// Failed changing tool delta.\r\n");
//  }
}


// This just pushes a move to the current Z value
void ZProbe::set_active_tool(Gcode *gcode, int pval)
{
  float mpos[3];
  THEROBOT->get_current_machine_position(mpos);
  if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true); // get inverse compensation transform

  Robot::wcs_t wpos = THEROBOT->mcs2wcs(mpos);
  float curz = THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));

  char buf[32];
  int n = snprintf(buf, sizeof(buf), "G1 Z%f", curz);
  string g(buf, n);
  Gcode gc(g, &(StreamOutput::NullStream));
  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

  gcode->stream->printf("Sent: %s\n", buf);

}



void ZProbe::reset_sensor_state()
{
  this->sensor_on_pin.set(false);
  this->calibrate_pin.set(false);
  sensor_mode = SENSOR_STATE_OFF;
}

float ZProbe::get_tool_temperature(int toolnum)
{
    struct pad_temperature temp;
    uint16_t heater_cs;
    if (toolnum == 0) {
        heater_cs = CHECKSUM("hotend");
    } else {
      heater_cs = CHECKSUM("hotend2");
    }
    bool ok = PublicData::get_value( temperature_control_checksum, current_temperature_checksum, heater_cs, &temp );

    if (ok) {
        return temp.current_temperature;
    }

    return 0.0F;
}

bool ZProbe::check_probe_state(bool check1a, bool check2a)
{
	bool checkval = true;

	if (check1a) {
	  if(this->pin.get() != invert_probe) {
		  checkval = true;
	  } else {
		checkval = false;
	  }

	  if (checkval) {
		  return(checkval);
	  }
  	}

  	if (check2a) {
	  if(this->pin2.get() != invert_probe) {
		checkval = true;
	  } else {
		checkval = false;
	  }
	}

	return(checkval);
}

void ZProbe::clear_cam(Gcode *gcode)
{
	//make sure sensor is initialized properly
	//for this task we just want a simple on/off state without all the smart stuff
 	set_sensor_state(gcode, SENSOR_STATE_OFF);  // was SENSOR_STATE_ON

	//first check if either probe is already triggered.  If it is, move a bit until it goes off.
	if (this->check_probe_state(true,true)) {
		gcode->stream->printf(" Moving off sensor\n");

		this->cam_position = 0;

		for (int i=1;i<360;i++) {
			this->move_cam(gcode, i);
			if (!this->check_probe_state(true,true)) {
				gcode->stream->printf(" Found neutral position\n");
				this->move_cam(gcode, i);
				break;
			}
		}

		//if either one of the sensors is still triggered, then something is wrong
		if (this->check_probe_state(true,true)) {
			//throw an error
			//probe never un-triggered
			gcode->stream->printf("//action:error Sensor setup failure\n");
//			THEKERNEL->call_event(ON_HALT, nullptr);
		} else {
			//reset the sensor, since it didn't initialize correctly if the sensor is on
			set_sensor_state(gcode, SENSOR_STATE_OFF);
		}
	}
}

void ZProbe::init_cam(Gcode *gcode)
{
	bool sensor1_triggered = false;
	bool sensor2_triggered = false;
	float start = -1;
	float end = -1;
	float home;

	gcode->stream->printf(" Initializing cam\n");

	//reset the cam to a neutral position first
	clear_cam(gcode);

	//make sure sensor is initialized properly
	//for this task we just want a simple on/off state without all the smart stuff
 	set_sensor_state(gcode, SENSOR_STATE_OFF);  // was SENSOR_STATE_ON

	this->cam_position = 0;

	for (int i=1;i<370;i++) {
		this->move_cam(gcode, i);
		if (this->check_probe_state(true,false)) {
			sensor1_triggered = true;
			if (start < 0) {
				gcode->stream->printf(" Found start at %d\n", i);
				start = i;
			}
		} else {
			if (end < 0) {
				if (start >= 0) {
					gcode->stream->printf(" Found end at %d\n", i);
					end = i;

					//check for noise.  ingore short duration
					if (end - start < 3) {
						gcode->stream->printf(" Discarding noise\n");
						start = -1;
						end = -1;
					}
				}
			}
		}

		if (this->check_probe_state(false,true)) {
			sensor2_triggered = true;
		}
	}

	if ((!sensor1_triggered) || (!sensor2_triggered)) {
		//throw an error
		//probe did not trigger
		gcode->stream->printf("//action:error Sensor initialization failure\n");
		THEKERNEL->call_event(ON_HALT, nullptr);
	}

	this->cam_position = 0;
	home = (start+end)/2;
	home = home + this->cam_angle_offset; //center_offset
	gcode->stream->printf(" Center position set to %1.4f\n", home);
	this->move_cam(gcode, home);
	this->cam_position = 0;

}

void ZProbe::move_cam(Gcode *gcode, float angle)
{
	float deltaangle;
	float steps;
	float step_time = 1000000/(this->cam_speed * this->cam_steps_degree)*2;

	if (angle == cam_position) {
		gcode->stream->printf("CAM already at angle\n");
		return;
	}

	if (angle > cam_position) {
		deltaangle = (angle - cam_position);
	} else {
		deltaangle = (360.0 - cam_position + angle);
	}

	steps = round(deltaangle * this->cam_steps_degree);
//	gcode->stream->printf("Moving cam %1.4f degrees\n", deltaangle);

	this->cam_pin->set(false);
	for(int x= 0; x<steps; x++)  //Loop the forward stepping enough times for motion to be visible
	{
		this->cam_pin->set(true);
		safe_delay_us(step_time);
		this->cam_pin->set(false);
		safe_delay_us(step_time);
	}

	this->cam_position = angle;
}


void ZProbe::set_sensor_position_new(Gcode *gcode, int toolnum, int pos, bool checkprobe)
{
	float newpos = -1;

	//ignore this command if the cam hasn't been initialized yet
	if (this->cam_position == -1) {
		return;
	}


    gcode->stream->printf("Set Sensor Position  Tool: %d Position: %d\n", toolnum, pos);

	switch (pos) {
	  case S_LOWERED:
	  	if (toolnum == 0) {
			newpos = this->cam_tool0;
		} else {
			newpos = this->cam_tool1;
		}
		break;

	  case S_NEUTRAL:
	  	newpos = this->cam_neutral;
		break;
	}

	if (newpos > -1) {
		this->move_cam(gcode, newpos);
	}
}

void ZProbe::set_sensor_position_old(Gcode *gcode, int toolnum, int pos, bool checkprobe)
{
  char buf[32];
  int n = 0;
  bool check1 = false;
  bool check2 = false;

  gcode->stream->printf("Set Sensor Position  Tool: %d Position: %d\n", toolnum, pos);

   if (toolnum == 0) {
      switch (pos) {
        case S_RAISED:
          n = snprintf(buf, sizeof(buf), "M280 S%1.4f", this->probe_up_val);
          check1 = true;
          break;

        case S_LOWERED:
          //HACKHACK force center position first to slow it down
          set_sensor_position(gcode, toolnum, S_CENTER);
          n = snprintf(buf, sizeof(buf), "M280 S%1.4f", this->probe_down_val);
          break;

        case S_CENTER:
          {
            float center = (this->probe_up_val + this->probe_down_val)/2;
            n = snprintf(buf, sizeof(buf), "M280 S%1.4f", center);
            gcode->stream->printf("Center position: %f\n", center);
          }
          break;

        case S_NEUTRAL:
          //HACKHACK force center position first to slow it down
          set_sensor_position(gcode, toolnum, S_CENTER);
          n = snprintf(buf, sizeof(buf), "M281");
          break;
      }
  }
  else {
    switch (pos) {
      case S_RAISED:
        n = snprintf(buf, sizeof(buf), "M280.1 S%1.4f", this->probe2_up_val);
        check2 = true;
        break;

      case S_LOWERED:
        //HACKHACK force center position first to slow it down
        set_sensor_position(gcode, toolnum, S_CENTER);
        n = snprintf(buf, sizeof(buf), "M280.1 S%1.4f", this->probe2_down_val);
        break;

      case S_CENTER:
        {
          float center = (this->probe2_up_val + this->probe2_down_val)/2;
          gcode->stream->printf("Center position: %f\n", center);
          n = snprintf(buf, sizeof(buf), "M280.1 S%1.4f", center);
        }
        break;

      case S_NEUTRAL:
        //HACKHACK force center position first to slow it down
        set_sensor_position(gcode, toolnum, S_CENTER);
        n = snprintf(buf, sizeof(buf), "M281.1");
        break;
    }
  }

  //make sure sensor is turned on first
  if ((this->disable_check_probe == false) && (checkprobe == true)) {
	  if (check1 || check2) {
		  set_sensor_state(gcode, SENSOR_STATE_ON);
	  }
	}

  gcode->stream->printf("Sent: %s\n", buf);

  string g(buf, n);

  Gcode gc(g, &(StreamOutput::NullStream));
  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

  //wait a bit to make sure it settles
  wait(2);

  //if we are raising the servo double check that the sensor triggered correctly
  bool checkval = true;
  if ((this->disable_check_probe == false) && (checkprobe == true)) {
	  checkval = this->check_probe_state(check1,check2);
	  if (checkval == false) {
    	gcode->stream->printf("//action:error Sensor initialization failure\n");
    	THEKERNEL->call_event(ON_HALT, nullptr);
	  }
	}
}

void ZProbe::set_sensor_position(Gcode *gcode, int toolnum, int pos)
{
	set_sensor_position(gcode, toolnum, pos, false);
}

void ZProbe::set_sensor_position(Gcode *gcode, int toolnum, int pos, bool checkprobe)
{
	if (this->cam_pin == nullptr) {
		set_sensor_position_old(gcode, toolnum, pos, checkprobe);
	} else {
		set_sensor_position_new(gcode, toolnum, pos, checkprobe);
	}

}


void ZProbe::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    int toolnum = 0;

    if( gcode->has_g && gcode->g >= 29 && gcode->g <= 33) {

        invert_probe = false;
        // make sure the probe is defined and not already triggered before moving motors
        if(!this->active_pin.connected()) {
            gcode->stream->printf("ZProbe pin not configured.\n");
            return;
        }

		//make sure both arms are in a neutral state BEFORE turning on the sensor otherwise it may not initialize correctly
		if((gcode->subcode == 1) || (gcode->subcode == 2) || ( gcode->g == 32 ) || ( gcode->g == 33 )) {
          gcode->stream->printf("Setting probe positions.\n");
          //release tools
          set_sensor_position(gcode, 0, S_NEUTRAL, false);
		  set_sensor_position(gcode, 1, S_NEUTRAL, false);
        }


        set_sensor_state(gcode, SENSOR_STATE_ON);


        //tool number used to be indicated with a T.  However this causes the tool ToolManager
        //to switch coordinate systems.  That messes up the Z.  Keep the T for compatibility
        //but use V going forward for dual material stuff.
        if (gcode->has_letter('T') || gcode->has_letter('V')) {
            if (gcode->has_letter('T')) {
              toolnum = gcode->get_value('T');
            }
            if (gcode->has_letter('V')) {
              toolnum = gcode->get_value('V');
            }

            set_active_probe(toolnum);
            // make sure hotend is actually connected before doing probe
            float curtemp = get_tool_temperature(toolnum);
            gcode->stream->printf("ZProbe temperature: %1.4f\n", curtemp);
            if ((curtemp == 0) || (curtemp == infinityf())) {
              gcode->stream->printf("ZProbe tool not connected, aborting command.\n");
//			  gcode->stream->printf("//action:error ZProbe tool not connected. Make sure hotend is attached.\n");
              return;
            }
        }


//        if(this->active_pin.get()) {
//            gcode->stream->printf("ZProbe triggered before move, aborting command.\n");
//            return;
//        }

        if( gcode->g == 30 ) { // simple Z probe
            // first wait for all moves to finish
            THEKERNEL->conveyor->wait_for_idle();
			bool set_z= (gcode->has_letter('Z') && !is_rdelta);
            bool set_q= (gcode->has_letter('Q') && !is_rdelta);
            bool set_d= (gcode->has_letter('D') && !is_rdelta);

			if ((gcode ->subcode == 1) || (gcode ->subcode == 3)) {
				//HACKHACK we need to make sure the X/Y position accounts for the X/Y tool offset
				//easiest way to do that is to put it into the tool mode for that tool
				if (toolnum == 1) {

					gcode->stream->printf("Sent: T1\n");
					Gcode gc2("T1", &(StreamOutput::NullStream));
					THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);

					coordinated_move(this->x_pos_1, this->y_pos_1, NAN, getFastFeedrate(), false, true);

					if(set_d) {
						gcode->stream->printf("Sent: T0\n");
						Gcode gc3("T0", &(StreamOutput::NullStream));
						THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc3);
					}
				} else {
					gcode->stream->printf("Sent: T0\n");
					Gcode gc3("T0", &(StreamOutput::NullStream));
					THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc3);
					coordinated_move(this->x_pos_0, this->y_pos_0, NAN, getFastFeedrate());
				}

				float mpos[3];
				THEROBOT->get_current_machine_position(mpos);
				if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true); // get inverse compensation transform

				Robot::wcs_t wpos = THEROBOT->mcs2wcs(mpos);
				float curz = THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));

				gcode->stream->printf("Original Z:%1.4f\n", curz);
			}

            if(this->active_pin.get()) {
				gcode->stream->printf("//action:error ZProbe triggered early. Check the probe.\n");
                return;
            }

            bool probe_result;
            bool reverse= (gcode->has_letter('R') && gcode->get_value('R') != 0); // specify to probe in reverse direction
            float rate= gcode->has_letter('F') ? gcode->get_value('F') / 60 : this->slow_feedrate;
            float mm;

            // if not setting Z then return probe to where it started, otherwise leave it where it is
            probe_result = ((set_z || set_q || set_d) ? run_probe(mm, rate, -1, reverse) : run_probe_return(mm, rate, -1, reverse));

            if(probe_result) {
                // the result is in actuator coordinates moved
				gcode->stream->printf("Probe succeeded\n");
//                gcode->stream->printf("Z:%1.4f\n", THEKERNEL->robot->from_millimeters(mm));

                if(set_z) {
                    // set current Z to the specified value, shortcut for G92 Znnn
                    char buf[32];
                    int n = snprintf(buf, sizeof(buf), "G92 Z%f", gcode->get_value('Z'));
                    string g(buf, n);
					gcode->stream->printf("Sent: %s\n", buf);
                    Gcode gc(g, &(StreamOutput::NullStream));
                    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
                } else if(set_q) {
                    // set Z to the stored value, and leave probe where it is
                    char buf[32];
                    int n;
                    if (toolnum == 0) {
                      n = snprintf(buf, sizeof(buf), "G92 Z%f", this->home_offset);
                    } else {
                      n = snprintf(buf, sizeof(buf), "G92 Z%f", this->home_offset2);
                    }
                    string g(buf, n);
                    Gcode gc(g, &(StreamOutput::NullStream));
				    gcode->stream->printf("Sent: %s\n", buf);
                    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
                } else if(set_d) {
	                //set relative offset to first sensor
	                float mpos[3];
	                THEROBOT->get_current_machine_position(mpos);
	                if(THEROBOT->compensationTransform) THEROBOT->compensationTransform(mpos, true); // get inverse compensation transform

	                Robot::wcs_t wpos = THEROBOT->mcs2wcs(mpos);
	                float curz = THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));

	                this->tool_delta = curz - this->home_offset2;
	                this->store_delta();
	                gcode->stream->printf("Current Z:%1.4f Home Offset: %1.4f  Home Offset2: %1.4f  New Delta: %1.4f\n", curz, this->home_offset, this->home_offset2, this->tool_delta);

                }

            } else {
				gcode->stream->printf("//action:error ZProbe not triggered. Check the probe.\n");
            }

			gcode->stream->printf("//action:probecomplete\n");
            set_sensor_state(gcode, SENSOR_STATE_OFF);

		} else if (gcode->g == 33 ) { // probe three points for physical bed leveling
			float mm;
			float z_off[3];
			float z_corr[3];

			if(!this->doProbeAt(mm, this->mount_pos1[0], this->mount_pos1[1])) {
				gcode->stream->printf("//action:error ZProbe triggered early. Check the probe.\n");
				return;
			}
			z_off[0] = this->getProbeHeight() - mm;
			gcode->stream->printf("first probe at X%1.3f, Y%1.3f is %1.3f mm\n", this->mount_pos1[0], this->mount_pos1[1], z_off[0]);

			if(!this->doProbeAt(mm, this->mount_pos2[0], this->mount_pos2[1])) {
				gcode->stream->printf("//action:error ZProbe triggered early. Check the probe.\n");
				return;
			}
			z_off[1] = this->getProbeHeight() - mm;
			gcode->stream->printf("second probe at X%1.3f, Y%1.3f is %1.3f mm\n", this->mount_pos2[0], this->mount_pos2[1], z_off[1]);

			if(!this->doProbeAt(mm, this->mount_pos3[0], this->mount_pos3[1])) {
				gcode->stream->printf("//action:error ZProbe triggered early. Check the probe.\n");
				return;
			}
			z_off[2] = this->getProbeHeight() - mm;
			gcode->stream->printf("third probe at X%1.3f, Y%1.3f is %1.3f mm\n", this->mount_pos3[0], this->mount_pos3[1], z_off[2]);

			float mindist = z_off[0];

			z_corr[0] = z_off[0];
			z_corr[1] = (z_off[1] - mindist)*this->mount_turns_mm;
			z_corr[2] = (z_off[2] - mindist)*this->mount_turns_mm;
			gcode->stream->printf("Leveling %1.3f %1.3f %1.3f\n", z_corr[0], z_corr[1], z_corr[2]);
			gcode->stream->printf("//action:levelcomplete\n");
			return;
        } else {
            if(this->active_pin.get()) {
				gcode->stream->printf("Probe pin not active?\n");
//				gcode->stream->printf("//action:error ZProbe triggered early. Check the probe.\n");
                return;
              }

            if(!gcode->has_letter('P')) {
                // find the first strategy to handle the gcode
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
                gcode->stream->printf("No strategy found to handle G%d\n", gcode->g);

            }else{
                // P paramater selects which strategy to send the code to
                // they are loaded in the order they are defined in config, 0 being the first, 1 being the second and so on.
                uint16_t i= gcode->get_value('P');
                if(i < strategies.size()) {
                    if(!strategies[i]->handleGcode(gcode)){
                        gcode->stream->printf("strategy #%d did not handle G%d\n", i, gcode->g);
                    }
                    return;

                }else{
                    gcode->stream->printf("strategy #%d is not loaded\n", i);
                }
            }
        }
    } else if(gcode->has_g && gcode->g == 38 ) { // G38.2 Straight Probe with error, G38.3 straight probe without error
        // linuxcnc/grbl style probe http://www.linuxcnc.org/docs/2.5/html/gcode/gcode.html#sec:G38-probe
        if(gcode->subcode < 2 || gcode->subcode > 5) {
            gcode->stream->printf("error:Only G38.2 to G38.5 are supported\n");
            return;
        }

        // make sure the probe is defined and not already triggered before moving motors
        if(!this->active_pin.connected()) {
            gcode->stream->printf("error:ZProbe not connected.\n");
            return;
        }

        set_sensor_state(gcode, SENSOR_STATE_ON);

        if(this->active_pin.get() ^ (gcode->subcode >= 4)) {
			gcode->stream->printf("//action:error ZProbe triggered early. Check the probe.\n");
            return;
        }

        // first wait for all moves to finish
        THEKERNEL->conveyor->wait_for_idle();

        float x= NAN, y=NAN, z=NAN;
        if(gcode->has_letter('X')) {
            x= gcode->get_value('X');
        }

        if(gcode->has_letter('Y')) {
            y= gcode->get_value('Y');
        }

        if(gcode->has_letter('Z')) {
            z= gcode->get_value('Z');
        }

        if(isnan(x) && isnan(y) && isnan(z)) {
            gcode->stream->printf("error:at least one of X Y or Z must be specified\n");
            return;
        }

        if(gcode->subcode == 4 || gcode->subcode == 5) {
            invert_probe = true;
        } else {
            invert_probe = false;
        }

        probe_XYZ(gcode, x, y, z);

        invert_probe = false;

        return;
	} else if(gcode->has_g && gcode->g == 28 ) { // watch for the homing command & initialize the CAM
		if ((gcode->subcode != 1) && (gcode->subcode != 2) && (this->cam_pin != nullptr)) {
			init_cam(gcode);
		}
		return;

    } else if(gcode->has_m) {
        // M code processing here
        int c;
        switch (gcode->m) {
			case 118: //echo input to output
				{
					string retval2 = gcode->get_command();
					//the command starts with a space.  remove it
					retval2.erase(0,1);
					gcode->stream->printf("%s\n", retval2.c_str());
				}
                break;

            case 119:
                c = this->active_pin.get();
                gcode->stream->printf(" Probe: %d", c);
                gcode->add_nl = true;
                break;

            case 670:
                if (gcode->has_letter('S')) this->slow_feedrate = gcode->get_value('S');
                if (gcode->has_letter('K')) this->fast_feedrate = gcode->get_value('K');
                if (gcode->has_letter('R')) this->return_feedrate = gcode->get_value('R');
                if (gcode->has_letter('Z')) this->max_z = gcode->get_value('Z');
                if (gcode->has_letter('H')) this->probe_height = gcode->get_value('H');
                if (gcode->has_letter('O')) this->home_offset = gcode->get_value('O');
                if (gcode->has_letter('Q')) this->home_offset2 = gcode->get_value('Q');

                if (gcode->has_letter('P')) this->home_offset = this->home_offset + gcode->get_value('P');
                if (gcode->has_letter('U')) this->home_offset2 = this->home_offset2 + gcode->get_value('U');

                if (gcode->has_letter('I')) { // NOTE this is temporary and toggles the invertion status of the pin
                    invert_override= (gcode->get_value('I') != 0);
                    pin.set_inverting(pin.is_inverting() != invert_override); // XOR so inverted pin is not inverted and vice versa
                }
                if (gcode->has_letter('D')) this->dwell_before_probing = gcode->get_value('D');
                break;

            case 671:
                {
                  float mpos[3];
                  THEROBOT->get_current_machine_position(mpos);
                  Robot::wcs_t wpos = THEROBOT->mcs2wcs(mpos);
                  this->home_offset = .15-THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));
                }
                break;

            case 672:
                {
                  if (gcode->has_letter('T')) {
                    toolnum = gcode->get_value('T');
                  }
                  if (gcode->has_letter('V')) {
                    toolnum = gcode->get_value('V');
                  }

                  int probe_pos = S_NEUTRAL;
                  if (gcode->has_letter('P')) {
                    probe_pos = gcode->get_value('P');
                  }

                  set_sensor_position(gcode, toolnum, probe_pos);
                }
                break;

			case 673:
				if (gcode->has_letter('A')) this->probe_up_val = gcode->get_value('A');
				if (gcode->has_letter('B')) this->probe_down_val = gcode->get_value('B');
				if (gcode->has_letter('C')) this->probe2_up_val = gcode->get_value('C');
				if (gcode->has_letter('D')) this->probe2_down_val = gcode->get_value('D');
				break;

			case 676:
				if (this->cam_pin != nullptr) {
				  	init_cam(gcode);
				}
				break;


            case 505:
                gcode->stream->printf("Z Offset: %1.4f Z Offset2: %1.4f Delta: %1.4f", this->home_offset, this->home_offset2, this->tool_delta);
                gcode->add_nl = true;
                break;

            case 506: //set the active sensor to either T0 or T1
                if (gcode->has_letter('T')) {
                  toolnum = gcode->get_value('T');
                }
                if (gcode->has_letter('V')) {
                  toolnum = gcode->get_value('V');
                }

                //we are now setting the delta in the toolmanager - don't need to do anything here anymore
                set_active_tool(gcode, toolnum);
                break;

            case 507:
                if (gcode->has_letter('T')) {
                  toolnum = gcode->get_value('T');
                }
                if (gcode->has_letter('V')) {
                  toolnum = gcode->get_value('V');
                }

                set_active_probe(toolnum);
                break;

            case 508:
              {
                if (gcode->has_letter('T')) {
                  toolnum = gcode->get_value('T');
                }
                if (gcode->has_letter('V')) {
                  toolnum = gcode->get_value('V');
                }

                int probe_pos = S_NEUTRAL;
                if (gcode->has_letter('P')) {
                  probe_pos = gcode->get_value('P');
                }

                set_sensor_position(gcode, toolnum, probe_pos);
              }
              break;

            case 510:
                // M510: calibrate on
				if (this->cam_pin != nullptr) {
					init_cam(gcode);
					set_sensor_position(gcode, 0, S_NEUTRAL);
				}
                set_sensor_state(gcode, SENSOR_STATE_CALIBRATE);
                break;

            case 511:
                // M511: calibrate off
                set_sensor_state(gcode, SENSOR_STATE_OFF);
                break;

            case 515:
                set_sensor_state(gcode, SENSOR_STATE_ON);
                break;

            case 516:
                set_sensor_state(gcode, SENSOR_STATE_OFF);
                break;

			case 517:
				//reset to initial probe state
				//right now that is only reset the tool deltas
				this->tool_delta = 0;
                this->store_delta();
                gcode->stream->printf("Reset tool state\n");
				break;


            case 500: // save settings
            case 503: // print settings
                gcode->stream->printf(";Probe feedrates Slow/fast(K)/Return (mm/sec) max_z (mm) height (mm) dwell (s):\nM670 S%1.2f K%1.2f R%1.2f Z%1.2f H%1.2f D%1.2f O%1.4f Q%1.4f\n",
                    this->slow_feedrate, this->fast_feedrate, this->return_feedrate, this->max_z, this->probe_height, this->dwell_before_probing, this->home_offset, this->home_offset2);

				gcode->stream->printf(";Probe servo positions:\nM673 A%1.4f B%1.4f C%1.4f D%1.4f\n",
					this->probe_up_val, this->probe_down_val, this->probe2_up_val, this->probe2_down_val);

                // fall through is intended so leveling strategies can handle m-codes too

            default:
                for(auto s : strategies){
                    if(s->handleGcode(gcode)) {
                        return;
                    }
                }
        }
    }
}

// special way to probe in the X or Y or Z direction using planned moves, should work with any kinematics
void ZProbe::probe_XYZ(Gcode *gcode, float x, float y, float z)
{
    // enable the probe checking in the timer
    probing= true;
    probe_detected= false;
    THEROBOT->disable_segmentation= true; // we must disable segmentation as this won't work with it enabled (beware on deltas probing in X or Y)

    // get probe feedrate in mm/min and convert to mm/sec if specified
    float rate = (gcode->has_letter('F')) ? gcode->get_value('F')/60 : this->slow_feedrate;

    // do a regular move which will stop as soon as the probe is triggered, or the distance is reached
    coordinated_move(x, y, z, rate, true);

    // coordinated_move returns when the move is finished

    // disable probe checking
    probing= false;
    THEROBOT->disable_segmentation= false;

    // if the probe stopped the move we need to correct the last_milestone as it did not reach where it thought
    // this also sets last_milestone to the machine coordinates it stopped at
    THEROBOT->reset_position_from_current_actuator_position();
    float pos[3];
    THEROBOT->get_axis_position(pos, 3);

    uint8_t probeok= this->probe_detected ? 1 : 0;

    // print results using the GRBL format
    gcode->stream->printf("[PRB:%1.3f,%1.3f,%1.3f:%d]\n", THEKERNEL->robot->from_millimeters(pos[X_AXIS]), THEKERNEL->robot->from_millimeters(pos[Y_AXIS]), THEKERNEL->robot->from_millimeters(pos[Z_AXIS]), probeok);
    THEROBOT->set_last_probe_position(std::make_tuple(pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS], probeok));
    set_sensor_state(gcode, SENSOR_STATE_OFF);

    if(gcode->has_letter('Q') && (probeok == 1)) {
          // set Z to the stored value, and leave probe where it is
          char buf[32];
          int n = snprintf(buf, sizeof(buf), "G92 Z%f", this->home_offset);
          string g(buf, n);
          Gcode gc(g, &(StreamOutput::NullStream));
		  gcode->stream->printf("Sent: %s\n", buf);
          THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
    }

    if(probeok == 0 && (gcode->subcode == 2 || gcode->subcode == 4)) {
        // issue error if probe was not triggered and subcode is 2 or 4
//        gcode->stream->printf("ALARM: Probe fail\n");
		gcode->stream->printf("//action:error Probe failure. Check the probe.\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
    }
}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ZProbe::coordinated_move(float x, float y, float z, float feedrate, bool relative, bool toolcoord)
{
    #define CMDLEN 128
    char *cmd= new char[CMDLEN]; // use heap here to reduce stack usage

    if(relative) strcpy(cmd, "G91 G0 ");
    else {
		if (toolcoord) {
			strcpy(cmd, "G0 ");
		} else {
			strcpy(cmd, "G53 G0 "); // G53 forces movement in machine coordinate system
		}
	}

    if(!isnan(x)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " X%1.3f", x);
    }
    if(!isnan(y)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Y%1.3f", y);
    }
    if(!isnan(z)) {
        size_t n= strlen(cmd);
        snprintf(&cmd[n], CMDLEN-n, " Z%1.3f", z);
    }

    {
        size_t n= strlen(cmd);
        // use specified feedrate (mm/sec)
        snprintf(&cmd[n], CMDLEN-n, " F%1.1f", feedrate * 60); // feed rate is converted to mm/min
    }

    if(relative) strcat(cmd, " G90");


//    THEKERNEL->streams->printf("DEBUG: move: %s: %u\n", cmd, strlen(cmd));

    // send as a command line as may have multiple G codes in it
    THEROBOT->push_state();
    struct SerialMessage message;
    message.message = cmd;
    delete [] cmd;

    message.stream = &(StreamOutput::NullStream);
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    THEKERNEL->conveyor->wait_for_idle();
    THEROBOT->pop_state();

}

// issue home command
void ZProbe::home()
{
    Gcode gc(THEKERNEL->is_grbl_mode() ? "G28.2" : "G28", &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
}
