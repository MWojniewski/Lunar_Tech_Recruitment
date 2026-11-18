# Lunar Lander Simulation — AGH Lunar Technologies Recruitment

A modular C simulation of a manual and assisted lunar landing following an onboard autopilot failure. Built in C with zero external C dependencies, the simulator satisfies all baseline 1D vertical descent requirements by default while providing extensible CLI flags for variable fuel mass, Gaussian sensor noise, stochastic engine ignition faults, randomized initial conditions, and real-time interactive keyboard control.

## 1. Physical Model & Mathematical Analysis

In the 1D vertical model, a spacecraft of initial mass $m$ starts at altitude $H_0$ with a downward vertical velocity $v_0$ under lunar gravity $g = 1.6\text{ m/s}^2$. Following the telemetry convention, altitude $H$ is measured upward from the lunar surface ($H = 0$ at touchdown), while downward velocity is positive ($V > 0$ during descent).

The main engine operates in binary `ON`/`OFF` mode (`ON` = $100\%$ thrust $F_c$, `OFF` = $0\%$ thrust). Touchdown is successful if the vertical velocity at $H = 0$ satisfies:
$$0 \le V < 1.0\text{ m/s}$$

### Default Baseline Parameters
* Spacecraft mass: $m = 2000\text{ kg}$
* Initial altitude: $H_0 = 500\text{ m}$
* Initial downward velocity: $v_0 = 10\text{ m/s}$
* Main engine thrust: $F_c = 4500\text{ N}$
* Lunar gravity: $g = 1.6\text{ m/s}^2$
* Integration time step: $\Delta t = 0.01\text{ s}$ (midpoint kinematic integration)

### Critical Suicide-Burn Altitude Derivation
With constant mass and the engine active (`ON`), the upward thrust acceleration is $a_c = \frac{F_c}{m} = 2.25\text{ m/s}^2$, yielding a net upward deceleration of:
$$a_{\text{net}} = a_c - g = 2.25 - 1.60 = 0.65\text{ m/s}^2$$

Combining the free-fall phase ($v_{\text{burn}}^2 - v_0^2 = 2 g (H_0 - H_{\text{burn}})$) with the full-thrust braking phase ($0 - v_{\text{burn}}^2 = 2 (g - a_c) H_{\text{burn}}$) eliminates $v_{\text{burn}}$ and gives the critical ignition altitude $H_{\text{burn}}$ required to reach $V = 0\text{ m/s}$ at $H = 0\text{ m}$:
$$H_{\text{burn}} = \frac{v_0^2 + 2 g H_0}{2 a_c} = \frac{10^2 + 2 \cdot 1.6 \cdot 500}{2 \cdot 2.25} \approx 377.78\text{ m}$$

### Extended Physics Modules (Optional CLI Flags)
1. **Variable Mass & Propellant Depletion (`--fuel [kg]`, `--fuel-rate <kg/s>`):**
   When enabled, total spacecraft mass decreases as fuel burns at a configurable mass flow rate $\dot{m}$ (default: $6.0\text{ kg/s}$, default initial propellant: $300\text{ kg}$). Over each step $\Delta t$, $\Delta m = \dot{m} \Delta t$ is consumed and thrust acceleration is evaluated at the midpoint mass $m_{\text{mid}} = m(t) - \frac{1}{2}\Delta m$. Ignition is physically locked out once $m_{\text{fuel}} = 0$.
2. **Gaussian Sensor Noise (`--noise <scale>`):**
   Separates true physical state $(H, V)$ from noisy onboard sensor measurements $(\tilde{H}, \tilde{V})$ using a Box-Muller normal generator $\mathcal{N}(0, 1)$ with $\sigma_H = 1.2 \cdot \text{scale}\text{ [m]}$ and $\sigma_V = 0.20 \cdot \text{scale}\text{ [m/s]}$.
3. **Stochastic Ignition Faults (`--fault-prob <0..1>`):**
   Injects per-step ignition misfires with probability $p \in [0, 1]$ whenever the pilot commands `ON`.

## 2. Pilot Controllers

* **`analytic` (Default):** Computes the real-time stopping distance $d_{\text{stop}} = \frac{\tilde{V}^2 - V_{\text{target}}^2}{2 a_{\text{eff}}}$ using the current spacecraft mass $m(t)$ and fault safety margin, targeting a gentle touchdown speed of $V_{\text{target}} = 0.4\text{ m/s}$.
* **`hysteresis`:** `ON`/`OFF` velocity controller tracking an altitude-proportional descent profile $V_{\text{desired}}(\tilde{H}) = 0.5 V_{\text{safe}} + 0.025 \tilde{H}$ within a $\pm 0.4\text{ m/s}$ deadband.
* **`manual`:** Real-time interactive terminal pilot synchronized to wall-clock time via POSIX `nanosleep` and non-blocking `termios`/`select` input (`SPACE` toggles `ON`/`OFF`, `1`/`W` forces `ON`, `0`/`S` forces `OFF`).

## 3. Project Structure

```text
.
├── config/
│   └── bounds.cfg       # Parameter ranges for --randomize mode
├── include/
│   └── lander.h         # Data structures, configuration, and module interfaces
├── src/
│   ├── main.c           # CLI argument parser, real-time loop, and simulation driver
│   ├── physics.c        # Midpoint kinematics, variable mass, noise, and fault models
│   ├── pilot.c          # Pilot controllers (Analytic, Hysteresis, Manual)
│   └── telemetry.c      # Console output, touchdown summary, and CSV logger
├── scripts/
│   ├── setup_venv.sh    # Idempotent Python virtual environment setup script
│   └── plot_results.py  # 3-panel matplotlib telemetry visualization
├── results/             # Generated CSV telemetry logs and PNG trajectory plots
├── Makefile             # Build automation (-std=c11 -Wall -Wextra -Werror)
└── requirements.txt     # Python dependencies (matplotlib)
```

## 4. Build & Usage

Compile the simulator with `make`:
```bash
make
```

Run with default deterministic parameters and save telemetry to CSV:
```bash
./lunar_lander results/baseline.csv
```

### CLI Options
```text
Usage: ./lunar_lander [csv_filename] [options]
Options:
  --output <file>       Output CSV path (default: results/baseline.csv)
  --pilot <type>        analytic | hysteresis | manual
  --fault-prob <0..1>   Engine ignition failure probability
  --fuel [kg]           Enable variable mass & fuel burn (default: 300 kg)
  --fuel-rate <kg/s>    Set fuel burn rate when engine is ON (default: 6.0 kg/s)
  --noise <scale>       Gaussian sensor noise scale (e.g. 1.0)
  --randomize           Randomize initial conditions from config/bounds.cfg (supports optional fuel bounds)
  --help                Show this help message
```

### Example Scenarios
```bash
# Variable mass + Gaussian sensor noise + ignition faults (Analytic vs Hysteresis)
./lunar_lander --pilot analytic --fuel --noise 1.0 --fault-prob 0.02 --output results/analytic_noisy.csv
./lunar_lander --pilot hysteresis --fuel --noise 1.0 --fault-prob 0.02 --output results/hysteresis_noisy.csv

# Custom fuel tank capacity (250 kg) and higher propellant burn rate (10.5 kg/s)
./lunar_lander --fuel 250 --fuel-rate 10.5 --output results/high_burn.csv

# Randomized initial conditions from config/bounds.cfg
./lunar_lander --randomize --fuel --output results/random_run.csv

# Interactive real-time manual landing in terminal (press SPACE to toggle engine)
./lunar_lander --pilot manual --fuel
```

## 5. Visualization

Set up the local Python virtual environment once (Ubuntu/WSL):
```bash
./scripts/setup_venv.sh
source .venv/bin/activate
```

Generate a 3-panel plot (Altitude, Downward Velocity, and Engine State) from any CSV log:
```bash
python scripts/plot_results.py results/baseline.csv
```
*(Or use `make plot` to render the default baseline plot).*
