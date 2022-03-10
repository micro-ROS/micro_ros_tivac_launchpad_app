#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

#define STACK_SIZE 1000
#define HEAP_SIZE 14000

// Utils

#define CHECK_AND_CONTINUE(X) if (!ret || !(X)){ret = false;}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
	static volatile int64_t init = -1; \
	if (init == -1) { init = uxr_millis();} \
	if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)