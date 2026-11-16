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

    if (cfg->enable_2d) {
        fprintf(fp, "time,height,velocity,engine_state,x,vx,rcs_state\n");
    } else {
        fprintf(fp, "time,height,velocity,engine_state\n");
    }
    return fp;
}

void telemetry_log_csv(FILE *fp, const LanderState *state, const SimConfig *cfg) {
    if (!fp) {
        return;
    }
    if (cfg->enable_2d) {
        fprintf(fp, "%.4f,%.4f,%.4f,%d,%.4f,%.4f,%d\n",
                state->time, state->height, state->velocity, state->engine_actual,
                state->x, state->vx, (int)state->rcs_state);
    } else {
        fprintf(fp, "%.4f,%.4f,%.4f,%d\n",
                state->time, state->height, state->velocity, state->engine_actual);
    }
}

void telemetry_print_console(const LanderState *state, const SimConfig *cfg) {
    const char *engine_str = state->engine_actual ? "ON" : "OFF";
    if (cfg->enable_2d) {
        printf("H=%.1fm, V=%.1fm/s, X=%.1fm, VX=%.1fm/s, %s\n",
               state->height, state->velocity, state->x, state->vx, engine_str);
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

    if (vertical_ok) {
        printf("STATUS: SUCCESS! The ship has landed safely.\n");
    } else {
        printf("STATUS: CRASH! Impact velocity exceeded structural limits.\n");
    }
}