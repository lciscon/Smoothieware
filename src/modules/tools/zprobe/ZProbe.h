/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ZPROBE_H_
#define ZPROBE_H_

#include "Module.h"
#include "Pin.h"

#include <vector>

// defined here as they are used in multiple files
#define zprobe_checksum            CHECKSUM("zprobe")
#define leveling_strategy_checksum CHECKSUM("leveling-strategy")

#define SENSOR_STATE_OFF        0
#define SENSOR_STATE_ON         1
#define SENSOR_STATE_CALIBRATE  2
#define SENSOR_STATE_DEBUG      3

class StepperMotor;
class Gcode;
class StreamOutput;
class LevelingStrategy;

class ZProbe: public Module
{

public:
    ZProbe() : invert_override(false),invert_probe(false) {};
    virtual ~ZProbe() {};

    void on_module_loaded();
    void on_gcode_received(void *argument);

    bool run_probe(float& mm, float feedrate, float max_dist= -1, bool reverse= false);
    bool run_probe_return(float& mm, float feedrate, float max_dist= -1, bool reverse= false);
    bool doProbeAt(float &mm, float x, float y);

    void coordinated_move(float x, float y, float z, float feedrate, bool relative=false, bool toolcoord=false);
    void home();

    void set_sensor_state(Gcode *gcode, int mode);
    void reset_sensor_state();
    void set_active_tool(Gcode *gcode, int tnum);
    void set_active_probe(int pnum);
	void wait(int sec);
    float get_tool_temperature(int toolnum);
    void set_sensor_position(Gcode *gcode, int toolnum, int pos);
	void set_sensor_position(Gcode *gcode, int toolnum, int pos, bool checkprobe);
	bool check_probe_state(bool check1, bool check2);

    bool getProbeStatus() { return this->pin.get(); }
    float getSlowFeedrate() const { return slow_feedrate; }
    float getFastFeedrate() const { return fast_feedrate; }
    float getProbeHeight() const { return probe_height; }
    float getMaxZ() const { return max_z; }

private:
    void config_load();
    void probe_XYZ(Gcode *gc, float x, float y, float z);
    uint32_t read_probe(uint32_t dummy);
    void store_delta();

    float slow_feedrate;
    float fast_feedrate;
    float return_feedrate;
    float probe_height;
    float max_z;
    float home_offset;
    float home_offset2;
    float x_pos_0;
    float y_pos_0;
	float x_pos_1;
    float y_pos_1;

	float mount_turns_mm;
	float mount_pos1[2];
	float mount_pos2[2];
	float mount_pos3[2];

    float dwell_before_probing;
    float probe_up_val;
    float probe_down_val;
    float probe2_up_val;
    float probe2_down_val;
	bool disable_check_probe;

    Pin pin;
    Pin pin2;
    Pin active_pin;
    int active_tool;
    float tool_delta;
    Pin calibrate_pin;
    Pin sensor_on_pin;
    std::vector<LevelingStrategy*> strategies;
    uint16_t debounce_ms, debounce;
    int sensor_mode;

    volatile struct {
        bool is_delta:1;
        bool is_rdelta:1;
        bool probing:1;
        bool reverse_z:1;
        bool invert_override:1;
        bool invert_probe:1;
        volatile bool probe_detected:1;
    };
};

#endif /* ZPROBE_H_ */
