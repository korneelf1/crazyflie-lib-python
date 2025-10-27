# Crazyflie SNN Controller - Mocap Examples

## Quick Start

**Main file to use:** `snn_l2f_test.py`

This script flies the Crazyflie with the SNN (Spiking Neural Network) controller using motion capture feedback.

### What it does:
- Connects to motion capture system (Qualisys/OptiTrack/etc.)
- Activates the SNN motor controller running on the Teensy microcontroller
- Flies various trajectories including figure-eight patterns
- Logs flight data to SD card

### Key Configuration (lines 11-16):
```python
SNN_CONTROL = True          # Enable/disable SNN controller
DO_SQUARE = False           # Square trajectory
DO_FORWARD = False          # Forward step
AXIS_EXPLORATION = False    # Axis exploration
GATHER_DATA = False         # Data gathering mode
DO_EIGHT_FIGURE = True      # Figure-eight trajectory
```

### Usage:
```bash
cd /Users/korneel/code/personal/NeurIPS-depoloy/crazyflie-lib-python/examples/mocap
python snn_l2f_test.py
```



## Other Files (Historical/Alternative):

- **`l2f_trigger.py`** - Learning2Fly controller with command-line arguments for different modes
- **`snn_test_figure8.py`** - Uses pre-uploaded polynomial trajectory (used in Nov 2024 experiments)
- **`snn_test_step.py`** - Simple step response test
- **`snn_position_command.py`** - Similar to l2f_trigger but without mocap wrapper

## Controller Parameters

The SNN controller is activated via:
- `snn_mc.use_snn` - Enable SNN motor control
- `snn_ct.snnType` - SNN controller type

(See `cf_utils.py` for helper functions like `activate_snn_controller()`)

