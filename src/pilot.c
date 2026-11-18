#define _POSIX_C_SOURCE 200809L

#include "lander.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

static struct termios orig_termios;
static bool terminal_raw_active = false;

void pilot_manual_cleanup(void) {
    if (terminal_raw_active) {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
        terminal_raw_active = false;
    }
}

static void handle_exit_signal(int sig) {
    (void)sig;
    pilot_manual_cleanup();
    _exit(130);
}

bool pilot_manual_init(void) {
    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "Error: --pilot manual requires an interactive terminal (TTY)\n");
        return false;
    }

    if (tcgetattr(STDIN_FILENO, &orig_termios) != 0) {
        perror("Failed to read terminal attributes");
        return false;
    }

    struct termios raw = orig_termios;
    raw.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        perror("Failed to configure non-blocking terminal mode");
        return false;
    }

    terminal_raw_active = true;
    atexit(pilot_manual_cleanup);

    struct sigaction sa;
    sa.sa_handler = handle_exit_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    return true;
}

static int pilot_analytic(const LanderState *state, const SimConfig *cfg) {
    double target_v = 0.4; /* Safe target touchdown speed (< 1.0 m/s) */
    double h = state->measured_height;
    double v = state->measured_velocity;

    /* Instantaneous upward deceleration using current spacecraft mass m(t) */
    double max_upward_decel = (cfg->thrust_force / state->mass) - cfg->gravity;
    double effective_decel = max_upward_decel * (1.0 - cfg->fault_prob * 1.2);
    if (effective_decel <= 0.05) {
        effective_decel = 0.05;
    }

    if (h <= 2.0) {
        return (v > target_v) ? 1 : 0;
    }

    double v_sq_diff = (v * v) - (target_v * target_v);
    double stopping_distance = (v_sq_diff > 0.0) ? (v_sq_diff / (2.0 * effective_decel)) : 0.0;

    if (h <= stopping_distance + 0.5 && v > target_v) {
        return 1;
    }
    return 0;
}

static int pilot_hysteresis(const LanderState *state, const SimConfig *cfg) {
    double desired_v = (0.5 * cfg->max_safe_velocity) + 0.025 * state->measured_height;
    double hysteresis_band = 0.4;

    if (state->measured_velocity > desired_v + hysteresis_band) {
        return 1;
    } else if (state->measured_velocity < desired_v - hysteresis_band) {
        return 0;
    }
    return state->engine_cmd;
}

static int pilot_manual(const LanderState *state, const SimConfig *cfg) {
    (void)cfg;
    int cmd = state->engine_cmd;

    /* Drain all pending key presses without blocking */
    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        struct timeval timeout = {0, 0};
        int ready = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
        if (ready <= 0) {
            break;
        }

        char ch = '\0';
        if (read(STDIN_FILENO, &ch, 1) <= 0) {
            break;
        }

        if (ch == ' ') {
            cmd = (cmd == 0) ? 1 : 0;
        } else if (ch == '1' || ch == 'w' || ch == 'W') {
            cmd = 1;
        } else if (ch == '0' || ch == 's' || ch == 'S') {
            cmd = 0;
        }
    }

    return cmd;
}

int pilot_decide_main_engine(const LanderState *state, const SimConfig *cfg) {
    switch (cfg->pilot_type) {
        case PILOT_ANALYTIC:
            return pilot_analytic(state, cfg);
        case PILOT_HYSTERESIS:
            return pilot_hysteresis(state, cfg);
        case PILOT_MANUAL:
            return pilot_manual(state, cfg);
        default:
            return 0;
    }
}