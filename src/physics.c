#include "lander.h"
#include <stdlib.h>

void config_init_default(SimConfig *cfg) {
    cfg->mass = 2000.0;
    cfg->initial_height = 500.0;
    cfg->initial_velocity = 10.0;
    cfg->thrust_force = 4500.0;
    cfg->gravity = 1.6;
    cfg->max_safe_velocity = 1.0;

    cfg->dt = 0.01;
    cfg->print_interval = 0.2; /* 5 times per second */

    cfg->fault_prob = 0.0;     /* No faults by default */
    cfg->enable_2d = false;    /* 1D vertical model by default */
    cfg->initial_x = 0.0;
    cfg->initial_vx = 0.0;
    cfg->rcs_force = 500.0;

    cfg->pilot_type = PILOT_ANALYTIC;
    cfg->csv_filename = "results/baseline.csv";
}

void state_init(LanderState *state, const SimConfig *cfg) {
    state->time = 0.0;
    state->height = cfg->initial_height;
    state->velocity = cfg->initial_velocity;
    state->engine_cmd = 0;
    state->engine_actual = 0;

    state->x = cfg->initial_x;
    state->vx = cfg->initial_vx;
    state->rcs_state = RCS_OFF;
}

void physics_step(LanderState *state, const SimConfig *cfg) {
    /* 1. Evaluate stochastic engine fault (if enabled) */
    if (state->engine_cmd == 1 && cfg->fault_prob > 0.0) {
        double roll = (double)rand() / (double)RAND_MAX;
        state->engine_actual = (roll < cfg->fault_prob) ? 0 : 1;
    } else {
        state->engine_actual = state->engine_cmd;
    }

    /* 2. Vertical acceleration (downward is positive: a = g - F_thrust / m) */
    double thrust_acc = state->engine_actual ? (cfg->thrust_force / cfg->mass) : 0.0;
    double acc_y = cfg->gravity - thrust_acc;

    /* 3. Update vertical velocity and height (exact kinematic integration over dt) */
    double prev_velocity = state->velocity;
    state->velocity += acc_y * cfg->dt;
    double avg_velocity = 0.5 * (prev_velocity + state->velocity);
    state->height -= avg_velocity * cfg->dt;

    /* 4. Optional 2D horizontal physics */
    if (cfg->enable_2d) {
        /* TODO : Implement lateral acceleration and wind/angle perturbations */
        double acc_x = ((int)state->rcs_state * cfg->rcs_force) / cfg->mass;
        double prev_vx = state->vx;
        state->vx += acc_x * cfg->dt;
        state->x += 0.5 * (prev_vx + state->vx) * cfg->dt;
    }

    state->time += cfg->dt;

    /* Clamp height to surface level */
    if (state->height < 0.0) {
        state->height = 0.0;
    }
}