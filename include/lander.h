#ifndef LANDER_H
#define LANDER_H

#include <stdbool.h>
#include <stdio.h>

/* Pilot controller strategies */
typedef enum {
    PILOT_ANALYTIC,   /* Suicide burn + terminal velocity hold */
    PILOT_HYSTERESIS, /* Reactive velocity controller */
    PILOT_MANUAL      /* Interactive keyboard control */
} PilotType;

/* Simulation configuration and initial parameters */
typedef struct {
    double mass;              /* Initial total spacecraft mass [kg] (default: 2000.0) */
    double initial_height;    /* Initial altitude H_0 [m] (default: 500.0) */
    double initial_velocity;  /* Initial downward speed v_0 [m/s] (default: 10.0) */
    double thrust_force;      /* Main engine thrust F_c [N] (default: 4500.0) */
    double gravity;           /* Lunar gravity g [m/s^2] (default: 1.6) */
    double max_safe_velocity; /* Max touchdown velocity [m/s] (default: 1.0) */

    double dt;                /* Physics integration time step [s] */
    double print_interval;    /* Console output interval [s] */

    /* Extensions (disabled by default for baseline compliance) */
    bool randomize;           /* Randomize initial conditions from bounds file */
    double fault_prob;        /* Probability [0.0 - 1.0] of engine ignition failure */
    bool enable_fuel;         /* Enable variable mass and fuel depletion */
    double initial_fuel;      /* Initial fuel mass [kg] (default: 300.0) */
    double fuel_burn_rate;    /* Fuel mass flow rate [kg/s] when ON (default: 6.0) */
    double sensor_noise;      /* Gaussian noise scale factor (default: 0.0) */

    PilotType pilot_type;     /* Selected pilot controller */
    const char *csv_filename; /* Output CSV file path */
} SimConfig;

/* Instantaneous state of the lunar lander */
typedef struct {
    double time;              /* Elapsed time [s] */
    double height;            /* True altitude H [m] */
    double velocity;          /* True downward velocity V [m/s] (V > 0 is down) */
    double mass;              /* Current total mass m(t) [kg] */
    double fuel;              /* Remaining fuel mass [kg] */

    int engine_cmd;           /* Pilot command: 0 = OFF, 1 = ON */
    int engine_actual;        /* Actual engine state after fault & fuel check: 0 or 1 */
    int engine_switches;      /* Total count of engine command transitions */

    /* Sensor readings (equal to true state when sensor_noise == 0.0) */
    double measured_height;   /* Altimeter reading with Gaussian noise [m] */
    double measured_velocity; /* Velocimeter reading with Gaussian noise [m/s] */
} LanderState;

/* Configuration & initialization */
void config_init_default(SimConfig *cfg);
bool config_load_randomized(SimConfig *cfg, const char *filename);
void state_init(LanderState *state, const SimConfig *cfg);

/* Core modules */
bool pilot_manual_init(void);
void pilot_manual_cleanup(void);
int pilot_decide_main_engine(const LanderState *state, const SimConfig *cfg);
void physics_step(LanderState *state, const SimConfig *cfg);

/* Telemetry & I/O */
FILE *telemetry_open_csv(const SimConfig *cfg);
void telemetry_log_csv(FILE *fp, const LanderState *state);
void telemetry_print_console(const LanderState *state, const SimConfig *cfg);
void telemetry_print_summary(const LanderState *state, const SimConfig *cfg);

#endif /* LANDER_H */