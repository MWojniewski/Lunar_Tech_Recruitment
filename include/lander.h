#ifndef LANDER_H
#define LANDER_H

#include <stdbool.h>
#include <stdio.h>

/* Pilot controller strategies */
typedef enum {
    PILOT_ANALYTIC,   /* Suicide burn + terminal velocity hold */
    PILOT_HYSTERESIS, /* Reactive bang-bang velocity controller */
    PILOT_NEURAL,     /* Pre-trained MLP neural network in pure C */
    PILOT_MANUAL      /* Interactive keyboard control */
} PilotType;

/* Lateral RCS thruster command for 2D extension */
typedef enum {
    RCS_OFF = 0,
    RCS_LEFT = -1,
    RCS_RIGHT = 1
} RcsState;

/* Simulation configuration and initial parameters */
typedef struct {
    double mass;              /* Spacecraft mass [kg] (default: 2000.0) */
    double initial_height;    /* Initial altitude H_0 [m] (default: 500.0) */
    double initial_velocity;  /* Initial downward speed v_0 [m/s] (default: 10.0) */
    double thrust_force;      /* Main engine thrust F_c [N] (default: 4500.0) */
    double gravity;           /* Lunar gravity g [m/s^2] (default: 1.6) */
    double max_safe_velocity; /* Max touchdown velocity [m/s] (default: 1.0) */

    double dt;                /* Physics integration time step [s] */
    double print_interval;    /* Console output interval [s] (several times/sec) */

    /* Extensions (default to 0 / disabled for baseline compliance) */
    double fault_prob;        /* Probability [0.0 - 1.0] of engine ignition failure */
    bool enable_2d;           /* Enable 2D horizontal motion & RCS thrusters */
    double initial_x;         /* Initial horizontal offset [m] */
    double initial_vx;        /* Initial horizontal velocity [m/s] */
    double rcs_force;         /* Lateral RCS thrust force [N] */

    PilotType pilot_type;     /* Selected pilot controller */
    const char *csv_filename; /* Output CSV file path */
} SimConfig;

/* Instantaneous state of the lunar lander */
typedef struct {
    double time;              /* Elapsed time [s] */
    double height;            /* Current altitude H [m] */
    double velocity;          /* Current downward velocity V [m/s] (V > 0 is down) */
    int engine_cmd;           /* Pilot command: 0 = OFF, 1 = ON */
    int engine_actual;        /* Actual engine state after fault check: 0 or 1 */

    /* 2D extension state variables */
    double x;                 /* Horizontal position [m] */
    double vx;                /* Horizontal velocity [m/s] */
    RcsState rcs_state;       /* Lateral thruster state */
} LanderState;

/* Configuration & initialization */
void config_init_default(SimConfig *cfg);
void state_init(LanderState *state, const SimConfig *cfg);

/* Core modules */
int pilot_decide_main_engine(const LanderState *state, const SimConfig *cfg);
RcsState pilot_decide_rcs(const LanderState *state, const SimConfig *cfg);
void physics_step(LanderState *state, const SimConfig *cfg);

/* Telemetry & I/O */
FILE *telemetry_open_csv(const SimConfig *cfg);
void telemetry_log_csv(FILE *fp, const LanderState *state, const SimConfig *cfg);
void telemetry_print_console(const LanderState *state, const SimConfig *cfg);
void telemetry_print_summary(const LanderState *state, const SimConfig *cfg);

#endif /* LANDER_H */