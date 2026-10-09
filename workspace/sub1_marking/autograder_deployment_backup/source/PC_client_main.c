#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "simulink_wrapper.h"

#ifndef MODEL_NAME
#define MODEL_NAME UCT_KDeploy
#endif

// Macro concatenation magic to dynamically construct <model>_initialize, <model>_step, <model>_terminate
#define CONCAT_EXPAND(a, b) a ## b
#define CONCAT(a, b) CONCAT_EXPAND(a, b)

#define MODEL_INIT CONCAT(MODEL_NAME, _initialize)
#define MODEL_STEP CONCAT(MODEL_NAME, _step)
#define MODEL_TERM CONCAT(MODEL_NAME, _terminate)

#define STRINGIFY_EXPAND(x) #x
#define STRINGIFY(x) STRINGIFY_EXPAND(x)
#define MODEL_HEADER STRINGIFY(MODEL_NAME.h)

// Include model header dynamically
#include MODEL_HEADER

// Globals from simulink_wrapper
extern int sim_fd;
static volatile int keep_running = 1;

void handle_sigint(int sig) {
    (void)sig;
    printf("\n[PC Main] Caught SIGINT. Requesting graceful shutdown...\n");
    keep_running = 0;
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    // Register SIGINT handler
    signal(SIGINT, handle_sigint);

    printf("[PC Main] Starting standalone client for model: %s\n", STRINGIFY(MODEL_NAME));

    // Initialize the model
    MODEL_INIT();

    printf("[PC Main] Model initialized. Entering step loop...\n");

    int has_connected = 0;
    while (keep_running) {
        MODEL_STEP();

        if (sim_fd != -1) {
            has_connected = 1;
        } else if (has_connected) {
            // We were connected but lost connection
            printf("[PC Main] Connection lost. Exiting loop.\n");
            break;
        } else {
            // We completed a step but never successfully connected (e.g., connection failed)
            printf("[PC Main] Failed to connect to Python simulator on port 8000. Exiting.\n");
            break;
        }
    }

    printf("[PC Main] Step loop terminated. Cleaning up model and socket...\n");

    // Terminate the model
    MODEL_TERM();

    // Clean up socket resources
    simulink_ext_cleanup();

    printf("[PC Main] Standalone client exited successfully.\n");
    return 0;
}
