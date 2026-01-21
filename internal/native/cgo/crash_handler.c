#define _GNU_SOURCE
#include "crash_handler.h"
#include "log.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#if defined(__has_include)
#if __has_include(<execinfo.h>)
#include <execinfo.h>
#define JETKVM_HAVE_EXECINFO 1
#endif
#endif

#ifndef JETKVM_HAVE_EXECINFO
#if defined(__GLIBC__)
#include <execinfo.h>
#define JETKVM_HAVE_EXECINFO 1
#endif
#endif

static const char *signal_name(int sig) {
    switch (sig) {
        case SIGSEGV: return "SIGSEGV";
        case SIGABRT: return "SIGABRT";
        case SIGILL: return "SIGILL";
        case SIGFPE: return "SIGFPE";
        case SIGBUS: return "SIGBUS";
        default: return "UNKNOWN";
    }
}

static void crash_handler(int sig, siginfo_t *info, void *ucontext) {
    (void)info;
    (void)ucontext;

    char msg[128];
    int len = snprintf(msg, sizeof(msg),
                       "JETKVM native crash: signal %d (%s)\n",
                       sig, signal_name(sig));
    if (len > 0) {
        (void)write(STDERR_FILENO, msg, (size_t)len);
    }

#ifdef JETKVM_HAVE_EXECINFO
    void *bt[64];
    int count = backtrace(bt, (int)(sizeof(bt) / sizeof(bt[0])));
    if (count > 0) {
        backtrace_symbols_fd(bt, count, STDERR_FILENO);
    }
#else
    const char *no_bt = "JETKVM native crash: backtrace not available\n";
    (void)write(STDERR_FILENO, no_bt, strlen(no_bt));
#endif

    _exit(128 + sig);
}

static void install_crash_handler(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = crash_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_RESETHAND;

    (void)sigaction(SIGSEGV, &sa, NULL);
    (void)sigaction(SIGABRT, &sa, NULL);
    (void)sigaction(SIGILL, &sa, NULL);
    (void)sigaction(SIGFPE, &sa, NULL);
    (void)sigaction(SIGBUS, &sa, NULL);
}

void jetkvm_crash_handler_init(void) {
    static int initialized = 0;
    if (initialized) {
        return;
    }
    initialized = 1;

    const char *env = getenv("JETKVM_NATIVE_CRASH_BACKTRACE");
    if (!env || env[0] == '\0' || env[0] == '0') {
        return;
    }

    install_crash_handler();
    log_info("native crash handler enabled");
}
