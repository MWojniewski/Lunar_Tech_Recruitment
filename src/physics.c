#include "lander.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    double min;
    double max;
    bool set;
} ValueBounds;

static char *trim_whitespace(char *text) {
    while (isspace((unsigned char)*text)) {
        text++;
    }

    char *end = text + strlen(text);
    while (end > text && isspace((unsigned char)end[-1])) {
        *--end = '\0';
    }
    return text;
}

static bool assign_bounds(ValueBounds *bounds, double min, double max) {
    if (bounds->set || !isfinite(min) || !isfinite(max) || min > max) {
        return false;
    }
    bounds->min = min;
    bounds->max = max;
    bounds->set = true;
    return true;
}

static double random_between(const ValueBounds *bounds) {
    if (bounds->min == bounds->max) {
        return bounds->min;
    }
    return bounds->min +
           ((double)rand() / (double)RAND_MAX) * (bounds->max - bounds->min);
}

void config_init_default(SimConfig *cfg) {
    cfg->mass = 2000.0;
    cfg->initial_height = 500.0;
    cfg->initial_velocity = 10.0;
    cfg->thrust_force = 4500.0;
    cfg->gravity = 1.6;
    cfg->max_safe_velocity = 1.0;

    cfg->dt = 0.01;
    cfg->print_interval = 0.2; /* 5 times per second */

    cfg->fault_prob = 0.001;    /* engine ignition failure probability */
    cfg->enable_2d = false;    /* 1D vertical model by default */
    cfg->initial_x = 0.0;
    cfg->initial_vx = 0.0;
    cfg->rcs_force = 500.0;

    cfg->pilot_type = PILOT_ANALYTIC;
    cfg->csv_filename = "results/baseline.csv";
}

bool config_load_randomized(SimConfig *cfg, const char *filename) {
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        perror("Failed to open simulation bounds file");
        return false;
    }

    ValueBounds mass = {0};
    ValueBounds height = {0};
    ValueBounds velocity = {0};
    ValueBounds thrust = {0};
    char line[256];
    unsigned int line_number = 0;
    bool valid = true;

    while (fgets(line, sizeof(line), fp)) {
        line_number++;
        char *entry = trim_whitespace(line);
        if (*entry == '\0' || *entry == '#') {
            continue;
        }

        char name[64];
        double min;
        double max;
        char extra;
        if (sscanf(entry, " %63[^=] = %lf , %lf %c",
                   name, &min, &max, &extra) != 3) {
            fprintf(stderr, "Invalid bounds entry at %s:%u\n",
                    filename, line_number);
            valid = false;
            break;
        }

        char *trimmed_name = trim_whitespace(name);
        ValueBounds *target = NULL;
        if (strcmp(trimmed_name, "mass") == 0) {
            target = &mass;
        } else if (strcmp(trimmed_name, "initial_height") == 0) {
            target = &height;
        } else if (strcmp(trimmed_name, "initial_velocity") == 0) {
            target = &velocity;
        } else if (strcmp(trimmed_name, "thrust_force") == 0) {
            target = &thrust;
        } else {
            fprintf(stderr, "Unknown bounds key '%s' at %s:%u\n",
                    trimmed_name, filename, line_number);
            valid = false;
            break;
        }

        if (!assign_bounds(target, min, max)) {
            fprintf(stderr, "Invalid or duplicate bounds key '%s' at %s:%u\n",
                    trimmed_name, filename, line_number);
            valid = false;
            break;
        }
    }

    if (ferror(fp)) {
        perror("Failed while reading simulation bounds file");
        valid = false;
    }
    fclose(fp);

    if (!valid || !mass.set || !height.set || !velocity.set || !thrust.set ||
        mass.min <= 0.0 || height.min < 0.0 || velocity.min < 0.0 ||
        thrust.min <= 0.0) {
        fprintf(stderr,
                "Bounds file must define mass, initial_height, "
                "initial_velocity, and thrust_force with valid ranges\n");
        return false;
    }

    cfg->mass = random_between(&mass);
    cfg->initial_height = random_between(&height);
    cfg->initial_velocity = random_between(&velocity);
    cfg->thrust_force = random_between(&thrust);
    return true;
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