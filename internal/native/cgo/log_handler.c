#include <stddef.h>
#include <stdlib.h>
#include "log.h"

/* Log handler */
jetkvm_log_handler_t *log_handler = NULL;

static int runtime_log_stderr = 0;

static void init_runtime_log_output() {
    const char *stderr_env = getenv("JETKVM_NATIVE_C_LOG_STDERR");
    if (stderr_env && stderr_env[0] != '\0') {
        runtime_log_stderr = 1;
    }
}

void log_message(int level, const char *filename, const char *funcname, const int line, const char *message) {
    init_runtime_log_output();
    if (runtime_log_stderr) {
        const char *level_name = "INFO";
        switch (level) {
            case LEVEL_TRACE: level_name = "TRACE"; break;
            case LEVEL_DEBUG: level_name = "DEBUG"; break;
            case LEVEL_INFO: level_name = "INFO"; break;
            case LEVEL_WARN: level_name = "WARN"; break;
            case LEVEL_ERROR: level_name = "ERROR"; break;
            case LEVEL_FATAL: level_name = "FATAL"; break;
            case LEVEL_PANIC: level_name = "PANIC"; break;
        }
        fprintf(stderr, "[C][%s] %s:%d %s: %s\n",
                level_name,
                filename ? filename : "unknown",
                line,
                funcname ? funcname : "unknown",
                message ? message : "");
        fflush(stderr);
    }
    if (log_handler != NULL) {
        log_handler(level, filename, funcname, line, message);
    }
}

void log_set_handler(jetkvm_log_handler_t *handler) {
    log_handler = handler;
}
