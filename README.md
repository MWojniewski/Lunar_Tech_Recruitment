# Lunar Lander Simulation — AGH Lunar Technologies Recruitment

A modular C simulation of a manual and assisted lunar landing following an onboard autopilot failure. Designed with extensibility in mind: supports baseline 1D vertical descent, stochastic engine ignition faults, 2D lateral dynamics, and pluggable pilot controllers (analytical, hysteresis, manual, and neural network).

## 1. Physical Model & Mathematical Analysis

In the baseline 1D model, a spacecraft of mass $m$ starts at altitude $H_0$ with a downward vertical velocity $v_0$ under lunar gravity $g = 1.6\text{ m/s}^2$. The engine operates in ON/OFF mode (`ON` = 100% thrust $F_c$, `OFF` = 0% thrust) with constant mass (mass may be various in future extensions). Touchdown is successful if the final velocity satisfies $0 \le V < 1.0\text{ m/s}$ at $H = 0$.

### Mock Parameters
* Mass: $m = 2000\text{ kg}$
* Initial altitude: $H_0 = 500\text{ m}$
* Initial downward velocity: $v_0 = 10\text{ m/s}$
* Engine thrust: $F_c = 4500\text{ N}$
* Lunar gravity: $g = 1.6\text{ m/s}^2$

### Critical Altitude Calculation
With the engine active (`ON`), the upward thrust acceleration is $a_c = \frac{F_c}{m} = 2.25\text{ m/s}^2$, yielding a deceleration of:
$$a_{\text{net}} = a_c - g = 2.25 - 1.60 = 0.65\text{ m/s}^2$$

The critical altitude $H_{\text{burn}}$ at which the pilot must engage continuous full thrust to reach $V = 0\text{ m/s}$ exactly at the surface ($H = 0\text{ m}$) is:
$$H_{\text{burn}} = \frac{v_0^2 + 2 g H_0}{2 a_c}$$

## 2. Project Structure

```text
.
├── include/
│   └── lander.h        # Data structures, configuration, and module interfaces
├── src/
│   ├── main.c          # CLI argument parser and main simulation loop
│   ├── physics.c       # Physics integration and hardware fault injection
│   ├── pilot.c         # Pilot controllers (Analytical, Hysteresis, Neural, Manual)
│   └── telemetry.c     # Console formatting and CSV data logging
├── scripts/
│   └── plot_results.py # Trajectory and phase-portrait visualization
├── results/            # Generated CSV logs and plots for baseline inputs
└── Makefile            # Build configuration

## 3. Build & Usage

Compile using `make`:
```bash
make
```

Run with default parameters and save telemetry to a CSV file:
```bash
./lunar_lander results/baseline.csv
```

Run with extended CLI flags:
```bash
./lunar_lander --output results/fault_test.csv --pilot analytic --fault-prob 0.05 --dt 0.01
```

### Plot telemetry

For WSL, create the project virtual environment once:
```bash
sudo apt update
sudo apt install python3-venv
./scripts/setup_venv.sh
source .venv/bin/activate
```

The setup script is safe to run again; it reuses the existing environment and
ensures the dependencies from `requirements.txt` are installed. After
activation, use `python` and `make plot` normally:
```bash
python --version
make plot
```

Generate a PNG beside a telemetry CSV:
```bash
python scripts/plot_results.py results/baseline.csv
```

The plot includes altitude, downward velocity, engine state, touchdown status,
and (when the CSV contains the 2D columns) horizontal position and velocity.
Use `--show` to open an interactive window when running with WSLg or another
GUI display, or `--output path/to/plot.png` to choose the output path. WSL1 is
headless, so it can save plots but cannot display them interactively. The
equivalent Make target is `make plot`.

## 4. Roadmap
- Baseline 1D physics model and CSV telemetry logger
- Python visualization script for $H(t)$ and $V(t)$ plots
- Stochastic engine ignition fault model (`--fault-prob`)
- 2D lateral physics extension with RCS thrusters (`--mode-2d`)
- Pure-C Multi-Layer Perceptron (MLP) Neural Pilot (`--pilot neural`)
- Mass change due to fuel burning