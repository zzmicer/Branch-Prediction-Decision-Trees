// Author: Hongliang Gao;   Created: Jan 27 2011
// Description: framework header for cbp3.
// DO NOT MODIFY. THIS FILE WILL NOT BE SUBMITTED WITH YOUR PREDICTOR.

#ifndef CBP3_FRAMEWORK_H_INCLUDED
#define CBP3_FRAMEWORK_H_INCLUDED

#define FETCHQ_SIZE 128
#define ROB_SIZE 256

// interface functions with the predictor

void PredictorInit();        // initialize predictors (e.g., allocating arrays)
void PredictorReset();       // reset predictor state at the beginning of a run
void PredictorRunACycle();   // called once at the end of each cycle to calculate predictions
void PredictorRunEnd();      // called at the end of a run
void PredictorExit();        // called at the end of simulation

class cbp3_queue_entry_t;
struct cbp3_cycle_activity_t;

// if the marker is set, the framework will rewind to the beginning of the trace
extern bool rewind_marked;

const cbp3_cycle_activity_t *get_cycle_info();            // what happened in the cycle
const cbp3_queue_entry_t    *fetch_entry(uint8_t e);      // access fetch queue
const cbp3_queue_entry_t    *rob_entry(uint8_t e);        // access rob
const int                    rename(uint8_t reg);         // access rename table
const int                    rename_flags();              // access rename table (for flags)
const uint32_t               reg_val(uint8_t reg);        // access architecture reg file

bool  report_pred(uint8_t ptr, bool in_rob, uint32_t pred);


class cbp3_queue_entry_t {
    public:
        cbp3_queue_entry_t () { reset(); }
        void reset();

        bool valid;

        // dynamic info of the uop
        cbp3_uop_dynamic_t uop;

        // cycle info of each stage
        uint32_t cycle_fetch;          // fetch
        uint32_t cycle_allocate;       // allocation from fetch queue to rob
        uint32_t cycle_agu;            // address generation
        uint32_t cycle_std;            // store data is ready
        uint32_t cycle_exe;            // execution
        uint32_t cycle_retire;         // retire

        uint32_t cycle_last_pred;      // cycle of the last prediction reported for this uop
        uint32_t last_pred;            // last prediction of a branch
};

struct cbp3_cycle_activity_t {
    // number of uops processed at each stage
    uint8_t  num_fetch;
    uint8_t  num_allocate;
    uint8_t  num_exe;
    uint8_t  num_retire;
    uint8_t  num_agu;
    uint8_t  num_std;

    // fetchq/rob pointers of each uop
    uint8_t  fetch_q[PIPE_WIDTH];
    uint8_t  allocate_q[PIPE_WIDTH]; // rob ptr of each allocated uop
    uint8_t  exe_q[15];
    uint8_t  retire_q[PIPE_WIDTH];
    uint8_t  agu_q[2];
    uint8_t  std_q[8];

    // current cycle number (due to trace warmup, it does not start from 0)
    uint32_t cycle;
};

#endif
