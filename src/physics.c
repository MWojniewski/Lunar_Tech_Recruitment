#include "lander.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI 6.28318530717958647692

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

/* Box-Muller transform for standard normal distribution N(0, 1) */
static double random_normal(void) {
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(TWO_PI * u2);
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

    cfg->randomize = false;      /* default: no randomization */
    cfg->fault_prob = 0.0;       /* engine ignition failure probability */
    cfg->enable_fuel = false;    /* constant mass by default for baseline */
    cfg->initial_fuel = 300.0;   /* 300 kg propellant */
    cfg->fuel_burn_rate = 6.0;   /* 6 kg/s mass flow rate at full thrust */
    cfg->sensor_noise = 0.0;     /* 0.0 = ideal sensors, 1.0 = nominal Gaussian noise */

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
    ValueBounds fuel = {0};
    ValueBounds burn_rate = {0};
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
        } else if (strcmp(trimmed_name, "initial_fuel") == 0) {
            target = &fuel;
        } else if (strcmp(trimmed_name, "fuel_burn_rate") == 0) {
            target = &burn_rate;
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

    if (fuel.set) {
        if (fuel.min <= 0.0) {
            fprintf(stderr, "Bounds key 'initial_fuel' must be > 0\n");
            return false;
        }
        cfg->initial_fuel = random_between(&fuel);
    }
    if (burn_rate.set) {
        if (burn_rate.min <= 0.0) {
            fprintf(stderr, "Bounds key 'fuel_burn_rate' must be > 0\n");
            return false;
        }
        cfg->fuel_burn_rate = random_between(&burn_rate);
    }

    return true;
}

void state_init(LanderState *state, const SimConfig *cfg) {
    state->time = 0.0;
    state->height = cfg->initial_height;
    state->velocity = cfg->initial_velocity;
    state->mass = cfg->mass;
    state->fuel = cfg->enable_fuel ? cfg->initial_fuel : 0.0;

    state->engine_cmd = 0;
    state->engine_actual = 0;
    state->engine_switches = 0;

    state->measured_height = cfg->initial_height;
    state->measured_velocity = cfg->initial_velocity;
}

void physics_step(LanderState *state, const SimConfig *cfg) {
    /* 1. Engine can fire ONLY if pilot commands ON and fuel is available */
    bool fuel_available = (!cfg->enable_fuel) || (state->fuel > 0.0);

    if (state->engine_cmd == 1 && fuel_available) {
        if (cfg->fault_prob > 0.0) {
            double roll = (double)rand() / (double)RAND_MAX;
            state->engine_actual = (roll < cfg->fault_prob) ? 0 : 1;
        } else {
            state->engine_actual = 1;
        }
    } else {
        state->engine_actual = 0;
    }

    /* 2. Variable mass & thrust acceleration calculation */
    double thrust_acc = 0.0;
    if (state->engine_actual == 1) {
        if (cfg->enable_fuel) {
            double requested_dm = cfg->fuel_burn_rate * cfg->dt;
            double actual_dm = (state->fuel < requested_dm) ? state->fuel : requested_dm;
            double thrust_fraction = actual_dm / requested_dm;

            double mid_mass = state->mass - 0.5 * actual_dm;
            thrust_acc = (cfg->thrust_force * thrust_fraction) / mid_mass;

            state->fuel -= actual_dm;
            if (state->fuel < 1e-9) {
                state->fuel = 0.0;
            }
            state->mass -= actual_dm;
        } else {
            thrust_acc = cfg->thrust_force / state->mass;
        }
    }

    /* 3. Vertical kinematics (downward is positive: a = g - a_thrust) */
    double acc_y = cfg->gravity - thrust_acc;
    double prev_velocity = state->velocity;
    state->velocity += acc_y * cfg->dt;
    double avg_velocity = 0.5 * (prev_velocity + state->velocity);
    state->height -= avg_velocity * cfg->dt;

    state->time += cfg->dt;

    /* Clamp true height to surface level */
    if (state->height < 0.0) {
        state->height = 0.0;
    }

    /* 4. Sensor measurements (with optional Gaussian noise) */
    if (cfg->sensor_noise > 0.0) {
        double sigma_h = 1.2 * cfg->sensor_noise;
        double sigma_v = 0.20 * cfg->sensor_noise;

        state->measured_height = state->height + sigma_h * random_normal();
        if (state->measured_height < 0.0) {
            state->measured_height = 0.0;
        }
        state->measured_velocity = state->velocity + sigma_v * random_normal();
    } else {
        state->measured_height = state->height;
        state->measured_velocity = state->velocity;
    }
}