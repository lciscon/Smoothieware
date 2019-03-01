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
#define gamma_max_checksum       CHECKSUM("gamma_max")
#define max_z_checksum           CHECKSUM("max_z")
#define reverse_z_direction_checksum CHECKSUM("reverse_z")
#define dwell_before_probing_checksum CHECKSUM("dwell_before_probing")
#define home_offset_checksum     CHECKSUM("home_offset")
#define calibrate_pin_checksum      CHECKSUM("calibrate_pin")
#define sensor_on_pin_checksum      CHECKSUM("sensor_on_pin")

// from endstop section
#define delta_homing_checksum    CHECKSUM("delta_homing")
#define rdelta_homing_checksum    CHECKSUM("rdelta_homing")

#define probe_up_checksum    CHECKSUM("probe_up")
#define probe_down_checksum    CHECKSUM("probe_down")
#define probe2_up_checksum    CHECKSUM("probe2_up")
#define probe2_down_checksum    CHECKSUM("probe2_down")

#define SENSOR_STATE_OFF        0
#define SENSOR_STATE_ON         1
#define SENSOR_STATE_CALIBRATE  2
#define SENSOR_STATE_DEBUG      3

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define S_RAISED 1
#define S_NEUTRAL 0
#define S_LOWERED -1

#define STEPPER THEROBOT->actuators
#define STEPS_PER_MM(a) (STEPPER[a]->get_steps_per_mm())
#define Z_STEPS_PER_MM STEPS_PER_MM(Z_AXIS)

#define abs(a) ((a<0) ? -a : a)

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

    this->calibrate_pin.from_string( THEKERNEL->config->value(zprobe_checksum, calibrate_pin_checksum)->by_default("nc" )->as_string())->as_output();
    this->sensor_on_pin.from_string( THEKERNEL->config->value(zprobe_checksum, sensor_on_pin_checksum)->by_default("nc" )->as_string())->as_output();

    if(isnan(this->max_z)){
        this->max_z = THEKERNEL->config->value(gamma_max_checksum)->by_default(200)->as_number(); // maximum zprobe distance
    }
    this->dwell_before_probing = THEKERNEL->config->value(zprobe_checksum, dwell_before_probing_checksum)->by_default(0)->as_number(); // dwell time in seconds before probing

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
    return run_probe_return(mm, slow_feedrate);
}

void ZProbe::set_sensor_state(int mode)
{
  if (sensor_mode != mode) {
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

void ZProbe::set_active_tool(Gcode *gcode, int pval)
{
  float mpos[3];
  THEROBOT->get_current_machine_position(mpos);
  Robot::wcs_t wpos = THEROBOT->mcs2wcs(mpos);
  float curz = THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));
  float delta;

  if (pval != 0) {
    delta = this->tool_delta;
  } else {
    delta = -this->tool_delta;
  }

  gcode->stream->printf("Old tool: %d  New tool: %d\n", this->active_tool, pval);
  gcode->stream->printf("Tool Change old Z:%1.4f\n", curz);

  if (pval != this->active_tool) {
      curz = curz - delta;

      // set Z to the stored value, and leave probe where it is
      char buf[32];
      int n = snprintf(buf, sizeof(buf), "G92 Z%f", curz);
      string g(buf, n);
      Gcode gc(g, &(StreamOutput::NullStream));
      THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
      gcode->stream->printf("Tool Change new Z:%1.4f\n", curz);
  }

  this->active_tool = pval;
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

void ZProbe::set_sensor_position(Gcode *gcode, int toolnum, int pos)
{
  char buf[32];
  int n = 0;

  gcode->stream->printf("Set Sensor Position  Tool: %d Position: %d\n", toolnum, pos);

   if (toolnum == 0) {
      switch (pos) {
        case S_RAISED:
          n = snprintf(buf, sizeof(buf), "M280 S%1.4f", this->probe_up_val);
          break;

        case S_LOWERED:
          n = snprintf(buf, sizeof(buf), "M280 S%1.4f", this->probe_down_val);
          break;

        case S_NEUTRAL:
          n = snprintf(buf, sizeof(buf), "M281");
          break;
      }
  }
  else {
    switch (pos) {
      case S_RAISED:
        n = snprintf(buf, sizeof(buf), "M280.1 S%1.4f", this->probe2_up_val);
        break;

      case S_LOWERED:
        n = snprintf(buf, sizeof(buf), "M280.1 S%1.4f", this->probe2_down_val);
        break;

      case S_NEUTRAL:
        n = snprintf(buf, sizeof(buf), "M281.1");
        break;
    }
  }

  gcode->stream->printf("Output %s\n", buf);

  string g(buf, n);

  Gcode gc(g, &(StreamOutput::NullStream));
  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);

  //wait a bit to make sure it settles
  n = snprintf(buf, sizeof(buf), "G4 S2");
  string g2(buf, n);
  Gcode gc2(g2, &(StreamOutput::NullStream));
  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc2);

}

void ZProbe::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    int toolnum = 0;

    if( gcode->has_g && gcode->g >= 29 && gcode->g <= 32) {

        invert_probe = false;
        // make sure the probe is defined and not already triggered before moving motors
        if(!this->active_pin.connected()) {
            gcode->stream->printf("ZProbe pin not configured.\n");
            return;
        }

        set_sensor_state(SENSOR_STATE_ON);

        if (gcode->has_letter('T')) {
          toolnum = gcode->get_value('T');
          set_active_probe(toolnum);
          // make sure hotend is actually connected before doing probe
          float curtemp = get_tool_temperature(toolnum);
          gcode->stream->printf("ZProbe temperature: %1.4f\n", curtemp);
          if ((curtemp == 0) || (curtemp == infinityf())) {
            gcode->stream->printf("ZProbe tool not connected, aborting command.\n");
            return;
          }
        }


        if(this->active_pin.get()) {
            gcode->stream->printf("ZProbe triggered before move, aborting command.\n");
            return;
        }

        if( gcode->g == 30 ) { // simple Z probe
            // first wait for all moves to finish
            THEKERNEL->conveyor->wait_for_idle();

            if(gcode->subcode == 1) {
              gcode->stream->printf("Setting probe positions.\n");
              //set probe physical positions
              //lift up other tool
              set_sensor_position(gcode, abs(toolnum-1), S_RAISED);
              //push down current tool
              set_sensor_position(gcode, toolnum, S_LOWERED);
              //release current tool
              set_sensor_position(gcode, toolnum, S_NEUTRAL);
            }


            bool set_z= (gcode->has_letter('Z') && !is_rdelta);
            bool set_q= (gcode->has_letter('Q') && !is_rdelta);
            bool set_d= (gcode->has_letter('D') && !is_rdelta);
            bool probe_result;
            bool reverse= (gcode->has_letter('R') && gcode->get_value('R') != 0); // specify to probe in reverse direction
            float rate= gcode->has_letter('F') ? gcode->get_value('F') / 60 : this->slow_feedrate;
            float mm;

            // if not setting Z then return probe to where it started, otherwise leave it where it is
            probe_result = ((set_z || set_q || set_d) ? run_probe(mm, rate, -1, reverse) : run_probe_return(mm, rate, -1, reverse));

            if(probe_result) {
                // the result is in actuator coordinates moved
                gcode->stream->printf("Z:%1.4f\n", THEKERNEL->robot->from_millimeters(mm));

                if(set_z) {
                    // set current Z to the specified value, shortcut for G92 Znnn
                    char buf[32];
                    int n = snprintf(buf, sizeof(buf), "G92 Z%f", gcode->get_value('Z'));
                    string g(buf, n);
                    Gcode gc(g, &(StreamOutput::NullStream));
                    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
              } else if(set_q) {
                  // set Z to the stored value, and leave probe where it is
                  char buf[32];
                  int n = snprintf(buf, sizeof(buf), "G92 Z%f", this->home_offset);
                  string g(buf, n);
                  Gcode gc(g, &(StreamOutput::NullStream));
                  THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
              } else if(set_d) {
                //set relative offset to first sensor
                float mpos[3];
                THEROBOT->get_current_machine_position(mpos);
                Robot::wcs_t wpos = THEROBOT->mcs2wcs(mpos);
                float curz = THEROBOT->from_millimeters(std::get<Z_AXIS>(wpos));

                this->tool_delta = curz - this->home_offset;
                gcode->stream->printf("Current Z:%1.4f Home Offset: %1.4f  New Delta: %1.4f\n", curz, this->home_offset, this->tool_delta);
              }

            } else {
                gcode->stream->printf("ZProbe not triggered\n");
            }

            set_sensor_state(SENSOR_STATE_OFF);

        } else {
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

        set_sensor_state(SENSOR_STATE_ON);

        if(this->active_pin.get() ^ (gcode->subcode >= 4)) {
            gcode->stream->printf("error:ZProbe triggered before move, aborting command.\n");
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

    } else if(gcode->has_m) {
        // M code processing here
        int c;
        switch (gcode->m) {
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

                      int probe_pos = S_NEUTRAL;
                      if (gcode->has_letter('P')) {
                        probe_pos = gcode->get_value('P');
                      }

                      set_sensor_position(gcode, toolnum, probe_pos);
                    }
                    break;

                case 505:
                    gcode->stream->printf("Z Offset: %1.4f Delta: %1.4f", this->home_offset, this->tool_delta);
                    gcode->add_nl = true;
                    break;

                case 506: //set the active sensor to either T0 or T1
                    if (gcode->has_letter('T')) {
                      toolnum = gcode->get_value('T');
                    }

                    set_active_tool(gcode, toolnum);
                    break;

                case 507:
                    if (gcode->has_letter('T')) {
                      toolnum = gcode->get_value('T');
                    }

                    set_active_probe(toolnum);
                    break;

                case 508:
                  {
                    if (gcode->has_letter('T')) {
                      toolnum = gcode->get_value('T');
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
                    set_sensor_state(SENSOR_STATE_CALIBRATE);
                    break;

                case 511:
                    // M511: calibrate off
                    set_sensor_state(SENSOR_STATE_OFF);
                    break;

                case 515:
                    set_sensor_state(SENSOR_STATE_ON);
                    break;

                case 516:
                    set_sensor_state(SENSOR_STATE_OFF);
                    break;


            case 500: // save settings
            case 503: // print settings
                gcode->stream->printf(";Probe feedrates Slow/fast(K)/Return (mm/sec) max_z (mm) height (mm) dwell (s):\nM670 S%1.2f K%1.2f R%1.2f Z%1.2f H%1.2f D%1.2f O%1.4f\n",
                    this->slow_feedrate, this->fast_feedrate, this->return_feedrate, this->max_z, this->probe_height, this->dwell_before_probing, this->home_offset);

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
    set_sensor_state(SENSOR_STATE_OFF);

    if(gcode->has_letter('Q') && (probeok == 1)) {
          // set Z to the stored value, and leave probe where it is
          char buf[32];
          int n = snprintf(buf, sizeof(buf), "G92 Z%f", this->home_offset);
          string g(buf, n);
          Gcode gc(g, &(StreamOutput::NullStream));
          THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
    }

    if(probeok == 0 && (gcode->subcode == 2 || gcode->subcode == 4)) {
        // issue error if probe was not triggered and subcode is 2 or 4
        gcode->stream->printf("ALARM: Probe fail\n");
        THEKERNEL->call_event(ON_HALT, nullptr);
    }
}

// issue a coordinated move directly to robot, and return when done
// Only move the coordinates that are passed in as not nan
// NOTE must use G53 to force move in machine coordinates and ignore any WCS offsets
void ZProbe::coordinated_move(float x, float y, float z, float feedrate, bool relative)
{
    #define CMDLEN 128
    char *cmd= new char[CMDLEN]; // use heap here to reduce stack usage

    if(relative) strcpy(cmd, "G91 G0 ");
    else strcpy(cmd, "G53 G0 "); // G53 forces movement in machine coordinate system

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

    //THEKERNEL->streams->printf("DEBUG: move: %s: %u\n", cmd, strlen(cmd));

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
