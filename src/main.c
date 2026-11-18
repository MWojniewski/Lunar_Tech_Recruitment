#include "lander.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void print_usage(const char *prog_name) {
    printf("Usage: %s [csv_filename] [options]\n", prog_name);
    printf("Options:\n");
    printf("  --output <file>       Output CSV path (default: results/baseline.csv)\n");
    printf("  --pilot <type>        analytic | hysteresis | neural | manual\n");
    printf("  --fault-prob <0..1>   Engine ignition failure probability\n");
    printf("  --randomize           Randomize initial conditions from config/bounds.cfg\n");
    printf("  --mode-2d             Enable 2D horizontal dynamics\n");
    printf("  --help                Show this help message\n");
}

static bool parse_args(int argc, char **argv, SimConfig *cfg) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return false;
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            cfg->csv_filename = argv[++i];
        } else if (strcmp(argv[i], "--fault-prob") == 0 && i + 1 < argc) {
            char *end;
            double probability = strtod(argv[++i], &end);
            if (*argv[i] == '\0' || *end != '\0' ||
                !isfinite(probability) || probability < 0.0 || probability > 1.0) {
                fprintf(stderr, "Invalid fault probability: expected a value in [0.0, 1.0]\n");
                return false;
            }
            cfg->fault_prob = probability;
        } else if (strcmp(argv[i], "--mode-2d") == 0) {
            cfg->enable_2d = true;
        } else if (strcmp(argv[i], "--pilot") == 0 && i + 1 < argc) {
            const char *p = argv[++i];
            if (strcmp(p, "analytic") == 0) cfg->pilot_type = PILOT_ANALYTIC;
            else if (strcmp(p, "hysteresis") == 0) cfg->pilot_type = PILOT_HYSTERESIS;
            else if (strcmp(p, "neural") == 0) cfg->pilot_type = PILOT_NEURAL;
            else if (strcmp(p, "manual") == 0) cfg->pilot_type = PILOT_MANUAL;
        } else if (strcmp(argv[i], "--randomize") == 0) {
            cfg->randomize = true;
        } else if (argv[i][0] != '-') {
            /* Support direct positional filename argument: ./lunar_lander output.csv */
            cfg->csv_filename = argv[i];
        }
    }
    return true;
}

int main(int argc, char **argv) {
    srand((unsigned int)time(NULL));

    SimConfig cfg;
    config_init_default(&cfg);
    if (!parse_args(argc, argv, &cfg)) {
        return 0;
    }
    if (cfg.randomize) {
        if (!config_load_randomized(&cfg, "config/bounds.cfg")) {
            return 1;
        }
    }

    LanderState state;
    state_init(&state, &cfg);

    FILE *csv_fp = telemetry_open_csv(&cfg);
    if (!csv_fp) {
        return 1;
    }

    double next_print_time = 0.0;
    const double max_sim_time = 1000.0; /* Safety guard against infinite loops */

    /* Log initial state at t = 0 */
    state.engine_cmd = pilot_decide_main_engine(&state, &cfg);
    state.engine_actual = state.engine_cmd;
    telemetry_log_csv(csv_fp, &state, &cfg);
    telemetry_print_console(&state, &cfg);
    next_print_time += cfg.print_interval;

    /* Main simulation loop */
    while (state.height > 0.0 && state.time < max_sim_time) {
        state.engine_cmd = pilot_decide_main_engine(&state, &cfg);
        state.rcs_state = pilot_decide_rcs(&state, &cfg);

        physics_step(&state, &cfg);
        telemetry_log_csv(csv_fp, &state, &cfg);

        if (state.time + 1e-9 >= next_print_time || state.height <= 0.0) {
            telemetry_print_console(&state, &cfg);
            next_print_time += cfg.print_interval;
        }
    }

    telemetry_print_summary(&state, &cfg);
    fclose(csv_fp);
    return 0;
}