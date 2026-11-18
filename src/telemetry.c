#include "lander.h"
#include <stdio.h>

FILE *telemetry_open_csv(const SimConfig *cfg) {
    if (!cfg->csv_filename) {
        return NULL;
    }
    FILE *fp = fopen(cfg->csv_filename, "w");
    if (!fp) {
        perror("Failed to open CSV output file");
        return NULL;
    }

    fprintf(fp, "time,height,velocity,engine_state\n");
    return fp;
}

void telemetry_log_csv(FILE *fp, const LanderState *state) {
    if (!fp) {
        return;
    }
    fprintf(fp, "%.4f,%.4f,%.4f,%d\n",
            state->time, state->height, state->velocity, state->engine_actual);
}

void telemetry_print_console(const LanderState *state, const SimConfig *cfg) {
    const char *engine_str = state->engine_actual ? "ON" : "OFF";

    if (cfg->pilot_type == PILOT_MANUAL) {
        if (cfg->enable_fuel) {
            printf("H=%6.1fm | V=%5.1fm/s | Fuel=%5.1fkg | Engine: %s\n",
                   state->measured_height, state->measured_velocity,
                   state->fuel, engine_str);
        } else {
            printf("H=%6.1fm | V=%5.1fm/s | Engine: %s\n",
                   state->measured_height, state->measured_velocity, engine_str);
        }
    } else {
        printf("H=%.1fm, V=%.1fm/s, %s\n",
               state->height, state->velocity, engine_str);
    }
}

void telemetry_print_summary(const LanderState *state, const SimConfig *cfg) {
    bool vertical_ok = (state->velocity >= 0.0 && state->velocity < cfg->max_safe_velocity);
    printf("\n--- TOUCHDOWN SUMMARY ---\n");
    printf("Elapsed time:       %.2f s\n", state->time);
    printf("Touchdown velocity: %.2f m/s (limit: < %.1f m/s)\n",
           state->velocity, cfg->max_safe_velocity);
    printf("Engine switches:    %d\n", state->engine_switches);
    if (cfg->enable_fuel) {
        printf("Fuel burn rate:     %.2f kg/s\n", cfg->fuel_burn_rate);
        printf("Remaining fuel:     %.1f / %.1f kg (final mass: %.1f kg)\n",
               state->fuel, cfg->initial_fuel, state->mass);
    }

    if (vertical_ok) {
        printf("STATUS: SUCCESS! The ship has landed safely.\n");
    } else {
        printf("STATUS: CRASH! Impact velocity exceeded structural limits.\n");
    }
}