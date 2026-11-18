#define _POSIX_C_SOURCE 200809L

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
    printf("  --pilot <type>        analytic | hysteresis | manual\n");
    printf("  --fault-prob <0..1>   Engine ignition failure probability\n");
    printf("  --fuel [kg]           Enable variable mass & fuel burn (default: 300 kg)\n");
    printf("  --fuel-rate <kg/s>    Set fuel burn rate when engine is ON (default: 6.0 kg/s)\n");
    printf("  --noise <scale>       Gaussian sensor noise scale (e.g. 1.0)\n");
    printf("  --randomize           Randomize initial conditions from config/bounds.cfg\n");
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
        } else if (strcmp(argv[i], "--fuel") == 0) {
            cfg->enable_fuel = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                char *end;
                double fuel_val = strtod(argv[i + 1], &end);
                if (*end != '\0' || !isfinite(fuel_val) || fuel_val <= 0.0) {
                    fprintf(stderr, "Invalid fuel mass: expected a positive number [kg]\n");
                    return false;
                }
                cfg->initial_fuel = fuel_val;
                i++;
            }
        } else if (strcmp(argv[i], "--fuel-rate") == 0 && i + 1 < argc) {
            char *end;
            double rate_val = strtod(argv[++i], &end);
            if (*argv[i] == '\0' || *end != '\0' ||
                !isfinite(rate_val) || rate_val <= 0.0) {
                fprintf(stderr, "Invalid fuel burn rate: expected a positive number [kg/s]\n");
                return false;
            }
            cfg->fuel_burn_rate = rate_val;
            cfg->enable_fuel = true;
        } else if (strcmp(argv[i], "--noise") == 0 && i + 1 < argc) {
            char *end;
            double noise_val = strtod(argv[++i], &end);
            if (*argv[i] == '\0' || *end != '\0' || !isfinite(noise_val) || noise_val < 0.0) {
                fprintf(stderr, "Invalid noise scale: expected a non-negative number\n");
                return false;
            }
            cfg->sensor_noise = noise_val;
        } else if (strcmp(argv[i], "--pilot") == 0 && i + 1 < argc) {
            const char *p = argv[++i];
            if (strcmp(p, "analytic") == 0) {
                cfg->pilot_type = PILOT_ANALYTIC;
            } else if (strcmp(p, "hysteresis") == 0) {
                cfg->pilot_type = PILOT_HYSTERESIS;
            } else if (strcmp(p, "manual") == 0) {
                cfg->pilot_type = PILOT_MANUAL;
            } else {
                fprintf(stderr, "Unknown pilot type '%s' (expected analytic, hysteresis, or manual)\n", p);
                return false;
            }
        } else if (strcmp(argv[i], "--randomize") == 0) {
            cfg->randomize = true;
        } else if (argv[i][0] != '-') {
            /* Support direct positional filename argument: ./lunar_lander output.csv */
            cfg->csv_filename = argv[i];
        } else {
            fprintf(stderr, "Unknown option '%s'\n", argv[i]);
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

static void sleep_real_time_dt(double dt) {
    if (dt <= 0.0) {
        return;
    }
    time_t sec = (time_t)dt;
    long nsec = (long)((dt - (double)sec) * 1e9);
    struct timespec req = {sec, nsec};
    nanosleep(&req, NULL);
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
    if (cfg.enable_fuel && cfg.initial_fuel >= cfg.mass) {
        fprintf(stderr, "Invalid configuration: initial fuel (%.1f kg) must be less than total mass (%.1f kg)\n",
                cfg.initial_fuel, cfg.mass);
        return 1;
    }

    if (cfg.pilot_type == PILOT_MANUAL) {
        if (!pilot_manual_init()) {
            return 1;
        }
        printf("=== MANUAL LANDING MODE ===\n");
        printf("Controls: [SPACE] Toggle Engine ON/OFF | [1/W] Engine ON | [0/S] Engine OFF\n");
        printf("Goal: Touch down at H = 0.0m with vertical speed V < %.1f m/s.\n\n", cfg.max_safe_velocity);
    }

    LanderState state;
    state_init(&state, &cfg);

    FILE *csv_fp = telemetry_open_csv(&cfg);
    if (!csv_fp) {
        pilot_manual_cleanup();
        return 1;
    }

    double next_print_time = 0.0;
    const double max_sim_time = 1000.0; /* Safety guard against infinite loops */

    /* Log initial state at t = 0 */
    telemetry_log_csv(csv_fp, &state);
    telemetry_print_console(&state, &cfg);
    next_print_time += cfg.print_interval;

    /* Main simulation loop */
    while (state.height > 0.0 && state.time < max_sim_time) {
        int new_cmd = pilot_decide_main_engine(&state, &cfg);
        if (state.time > 0.0 && new_cmd != state.engine_cmd) {
            state.engine_switches++;
        }
        state.engine_cmd = new_cmd;

        physics_step(&state, &cfg);
        telemetry_log_csv(csv_fp, &state);

        if (state.time + 1e-9 >= next_print_time || state.height <= 0.0) {
            telemetry_print_console(&state, &cfg);
            next_print_time += cfg.print_interval;
        }

        if (cfg.pilot_type == PILOT_MANUAL) {
            sleep_real_time_dt(cfg.dt);
        }
    }

    pilot_manual_cleanup();
    telemetry_print_summary(&state, &cfg);
    fclose(csv_fp);
    return 0;
}