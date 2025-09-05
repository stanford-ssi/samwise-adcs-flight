# Task Scheduler & State Machine

Manages ADCS operational states and executes tasks based on timing and state transitions.

**Location:** `src/scheduler/scheduler.{h,cpp}`

## States
```cpp
static const sched_state_t *all_states[] = {
    &init_state, &test_state, &detumble_state, &slewing_state, &cool_down_state
};
```

## Functions
```cpp
void sched_init(slate_t *slate);        // Initialize scheduler and tasks
void sched_dispatch(slate_t *slate);    // Main dispatch loop
```

## State Structure
```cpp
typedef struct {
    const char *name;                           // State name
    sched_task_t **task_list;                  // Active tasks 
    size_t num_tasks;                          // Task count
    sched_state_t *(*get_next_state)(slate_t *); // Transition logic
} sched_state_t;
```

## Task Structure
```cpp
typedef struct {
    const char *name;                    // Task name
    void (*task_init)(slate_t *);       // Init function
    void (*task_func)(slate_t *);       // Main function
    uint32_t dispatch_period_ms;        // Period
    absolute_time_t next_dispatch;      // Next run time
} sched_task_t;
```

## Implementation
- **Initialization:** Enumerate all tasks across states, call task init functions, set initial timers
- **Dispatch:** Check task timers, execute ready tasks, check for state transitions
- **Timing:** Period-based scheduling with absolute time tracking

Used by main loop to coordinate all system operations across different flight modes.