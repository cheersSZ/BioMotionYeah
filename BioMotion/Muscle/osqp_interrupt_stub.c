// OSQP interrupt handler stub for iOS
// iOS doesn't need Ctrl-C handling

#include "osqp_configure.h"

void osqp_start_interrupt_listener(void) {}
void osqp_end_interrupt_listener(void) {}
int osqp_is_interrupted(void) { return 0; }
