#include "lander.h"

static int pilot_analytic(const LanderState *state, const SimConfig *cfg) {
    double target_v = 0.4; /* Safe target touchdown speed (< 1.0 m/s) */
    double max_upward_decel = (cfg->thrust_force / cfg->mass) - cfg->gravity;

    /* Increase safety margin if engine ignition faults are expected */
    double effective_decel = max_upward_decel * (1.0 - cfg->fault_prob * 1.2);
    if (effective_decel <= 0.05) {
        effective_decel = 0.05;
    }

    /* Near the surface: maintain gentle descent rate */
    if (state->height <= 2.0) {
        return (state->velocity > target_v) ? 1 : 0;
    }

    /* Calculate stopping distance required to decelerate from current V to target_v */
    double v_sq_diff = (state->velocity * state->velocity) - (target_v * target_v);
    double stopping_distance = (v_sq_diff > 0.0) ? (v_sq_diff / (2.0 * effective_decel)) : 0.0;

    /* Small buffer (0.5m) for discrete time-step quantization */
    if (state->height <= stopping_distance + 0.5 && state->velocity > target_v) {
        return 1;
    }
    return 0;
}

static int pilot_hysteresis(const LanderState *state, const SimConfig *cfg) {
    (void)cfg;
    /* Reference velocity profile proportional to altitude */
    double desired_v = 0.5 + 0.025 * state->height;
    double hysteresis_band = 0.2;

    if (state->velocity > desired_v + hysteresis_band) {
        return 1;
    } else if (state->velocity < desired_v - hysteresis_band) {
        return 0;
    }
    return state->engine_cmd;
}

static int pilot_neural(const LanderState *state, const SimConfig *cfg) {
    /* TODO : Implement forward pass of MLP neural network using weights.h
     * Inputs: normalized [height, velocity, x, vx]
     * Output: sigmoid activation > 0.5 ? 1 : 0
     */
    (void)cfg;
    return (state->velocity > 0.8) ? 1 : 0;
}

int pilot_decide_main_engine(const LanderState *state, const SimConfig *cfg) {
    switch (cfg->pilot_type) {
        case PILOT_ANALYTIC:
            return pilot_analytic(state, cfg);
        case PILOT_HYSTERESIS:
            return pilot_hysteresis(state, cfg);
        case PILOT_NEURAL:
            return pilot_neural(state, cfg);
        case PILOT_MANUAL:
            /* TODO : Read non-blocking spacebar input from terminal */
            return state->engine_cmd;
        default:
            return 0;
    }
}

RcsState pilot_decide_rcs(const LanderState *state, const SimConfig *cfg) {
    if (!cfg->enable_2d) {
        return RCS_OFF;
    }
    /* TODO : Implement controller for horizontal stabilization */
    (void)state;
    return RCS_OFF;
}