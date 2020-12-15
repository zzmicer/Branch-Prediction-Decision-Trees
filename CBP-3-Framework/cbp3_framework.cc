// Author: Hongliang Gao;   Created: Jan 27 2011
// Description: implementation of the framework (driver) for cbp3.
// DO NOT MODIFY. THIS FILE WILL NOT BE SUBMITTED WITH YOUR PREDICTOR
 
#include <stdio.h>
#include <stdlib.h>
#include <cassert>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <time.h>

using namespace std;
#include "cbp3_def.h"
#include "cbp3_reader.h"
#include "cbp3_framework.h"
#include "predictor.h"

uint32_t num_run = 0;
void run(int sim_len);

int main ( int argc, char *argv[] )
{
    time_t time_start = time(NULL);

// process command inputs
    char tfile[500];
    int sim_len = -1; // to the end of trace
    if ( argc != 3 && argc != 5) {
        printf( "usage: %s -t /path-to/trace [-u num-uops-to-simulate]\n", argv[0]);
        exit (1);
    }else {
        int targ = -1, uarg = -1;
        bool wrong = false;
        for (int i = 1; i <= argc - 2; i += 2) {
            if (strcmp(argv[i], "-t") == 0) targ = i + 1;
            else if (strcmp(argv[i], "-u") == 0) uarg = i + 1;
            else 
                wrong = true;
        }
        if (targ == -1 || wrong)  {
            printf( "usage: %s -t /path-to/trace [-u num-uops-to-simulate]\n", argv[0]);
            exit (1);
        }
        
        if (uarg != -1) {
            sim_len = atoi(argv[uarg]);
            if (sim_len <= 0) sim_len = -1;
        }
        assert(strlen(argv[targ]) < 500);
        strcpy(tfile, argv[targ]);
    }

// load the trace into memory
    printf("********************  CBP3  Start ***********************\n");
    printf("Trace:            %s\n", tfile);
    if (sim_len == -1)
        printf("Uops to simulate: Whole Trace\n\n");
    else
        printf("Uops to simulate: %i\n\n", sim_len);

    // check whether trace is compressed by bzip2
    bool bzip2 = false;
    if (strlen(tfile) > 3)
        bzip2 = (tfile[strlen(tfile) - 1] == '2' && 
                tfile[strlen(tfile) - 2] == 'z' && 
                tfile[strlen(tfile) - 3] == 'b');
    if (bzip2) {
        printf("Trace is compressed by bzip2.\n");
    }

    char cmd[600];
    sprintf (cmd, "%s %s", bzip2 ? "bzip2 -dc" : "cat", tfile);
    FILE *tracefp = popen (cmd, "r");
    if (!tracefp) {
        printf("ERROR: could not open trace: %s\n", tfile);
        exit (1);
    }
    printf("\nLoading trace...\n");
    ReaderLoadTrace(tracefp, true, false, 17, 16);
    printf("Trace loaded.\n\n");
    fclose(tracefp);

// initialize predictors
    PredictorInit();
    
    printf("Reader max_mem: %.2f M\n", (double)ReaderMaxMem()/(1024.0*1024.0));
    printf("Reader info_mem: %.2f M\n", (double)ReaderTraceInfoSize()/(1024.0*1024.0));

// keep running if rewind_marked is set
// the length of each run is decided by sim_len
    while (num_run == 0 || rewind_marked) {
        time_t run_start = time(NULL);
        run(sim_len);
        printf("Simulation Time: %ld Seconds\n", (time(NULL) - run_start));
        ReaderRewind();
    }

// the end...
    printf("\n********************  CBP3  End   ***********************\n");
    printf("Total Simulation Time: %ld Seconds\n", (time(NULL) - time_start));
    PredictorExit();

    return 1;
}

// oracle info about an uop from the reader
cbp3_uop_dynamic_t uop_info[FETCHQ_SIZE + ROB_SIZE];

// live uop info that will be availabe to the predictor
// depending on the stage of the uop, fields are copied from uop_info array
cbp3_queue_entry_t live_uop[FETCHQ_SIZE + ROB_SIZE];

uint32_t reg_value[NUM_REGS];      // architectural reg file filled at retire
bool     reg_valid[NUM_REGS];      

// renaming table
// it is indexed by logical reg number and pointing to a rob entry that holds the producer uop of this reg
int      rename_table[NUM_REGS];
int      rename_table_flags;

bool     rewind_marked;
uint32_t cycle;
cbp3_cycle_activity_t cycle_info;

uint8_t  allocate_fetchq;    // fetch q ptr of the uop that will be allocated next
bool     first_fetch;

// stats
uint32_t total_cycle;
uint32_t num_insts;
uint32_t num_uops;
uint32_t num_cond_br;
uint32_t num_ind_br;
uint32_t penalty_cond_br;    // misprediction penalty cycles of conditional branches
uint32_t penalty_ind_br;     // misprediction penalty cycles of indirect branches
uint32_t msp_cond_br;        // number of mispredictions of conditional branches
uint32_t msp_ind_br;         // number of mispredictions of indirect branches

void reset() {
    for (int i = 0; i < (FETCHQ_SIZE + ROB_SIZE); i++) {
        live_uop[i].reset();
        uop_info[i].reset();
    }
    for (int i = 0; i < NUM_REGS; i++) {
        reg_value[i] = 0;
        reg_valid[i] = false;
        rename_table[i] = -1; // -1 means producer is not in rob
        rename_table_flags = -1;
    }

    rewind_marked = false;
    cycle = 0;
    allocate_fetchq = 0;
    first_fetch = true;

    total_cycle = 0;
    num_insts = 0;
    num_uops = 0;
    num_cond_br = 0;
    num_ind_br = 0;
    penalty_cond_br = 0;
    penalty_ind_br = 0;
    msp_cond_br = 0;
    msp_ind_br = 0;

    PredictorReset();
}

// for debug: compare two uops
bool uop_check(cbp3_uop_dynamic_t *u1, cbp3_uop_dynamic_t *u2) {
    for (int i = 0; i < UOP_SRCS_MAX; i++) {
        if (u1->srcs[i] != u2->srcs[i]) return false;
        if (u1->src_data[i] != u2->src_data[i]) return false;
        if (u1->srcs_static[i] != u2->srcs_static[i]) return false;
    }
    for (int i = 0; i < 4; i++)
        if (u1->ldst_data[i] != u2->ldst_data[i]) return false;
    if (u1->dst != u2->dst ||
        u1->dst_data != u2->dst_data ||
        u1->wflags != u2->wflags ||
        u1->rflags != u2->rflags)
        return false;
    if (u1->vaddr != u2->vaddr ||
        u1->br_target != u2->br_target ||
        u1->br_taken != u2->br_taken ||
        u1->uop_id != u2->uop_id ||
        u1->inst_size != u2->inst_size)
        return false;
    if (u1->pc != u2->pc ||
        u1->type != u2->type ||
        u1->opsize != u2->opsize ||
        u1->dst_static != u2->dst_static)
        return false;
    return true;
}

void run(int sim_len) {
    num_run ++;
    printf("\n*********  RUN  %i   *********\n", num_run);
    reset();
    uint16_t num_stages[NUM_STAGES];
    
    while (ReaderRunACycle(num_stages) && // trace does not end
            (sim_len == -1 ? 1 : num_uops < (uint32_t)sim_len)) { // in the simulation range
        // get info from the reader
        const cbp3_cycle_info_t *reader = ReaderInfo();

        cycle = reader->cycle;
        // fill cycle_info
        cycle_info.cycle = reader->cycle;
        cycle_info.num_fetch = num_stages[0];
        cycle_info.num_allocate = num_stages[1];
        cycle_info.num_exe = num_stages[2];
        cycle_info.num_retire = num_stages[3];
        cycle_info.num_agu = num_stages[4];
        cycle_info.num_std = num_stages[5];

        // fill in different fields into live_uop array for each stage

        for (uint32_t i = 0; i < cycle_info.num_retire; i ++) {
            uint8_t q = reader->retire_q[i];
            cycle_info.retire_q[i] = q;
            cbp3_queue_entry_t &uop = live_uop[q + FETCHQ_SIZE];
            // fill reg file
            if (!reg_is_constant(uop.uop.dst)) {
                reg_valid[uop.uop.dst] = true;
                reg_value[uop.uop.dst] = uop.uop.dst_data;
            }

            uop.cycle_retire = cycle;
            
            uint16_t type = uop.uop.type;
            if (type & IS_EOM)            num_insts ++; // the uop is end of an inst
            num_uops ++;
            if (type & IS_BR_CONDITIONAL) num_cond_br ++;
            if (type & IS_BR_INDIRECT) num_ind_br ++;

            //assert(uop_check(&live_uop[q + FETCHQ_SIZE].uop, &uop_info[q + FETCHQ_SIZE]));

            // uop retired, fix rename_table
            if (!reg_is_constant(uop.uop.dst) && rename_table[uop.uop.dst] == q)
                rename_table[uop.uop.dst] = -1;
            // need to check all fp regs because of fxchg swapping regs
            // reg 35 is MM0
            for (int k = 35; k < 35 + 8; k++) {
                if (rename_table[k] == q) {
                    rename_table[k] = -1;
                }
            }
            if ((type & IS_WFLAGS) && rename_table_flags == q)
                rename_table_flags = -1;
        }

        for (uint32_t i = 0; i < cycle_info.num_exe; i ++) {
            uint8_t q = reader->exe_q[i];
            cycle_info.exe_q[i] = q;
            cbp3_queue_entry_t &uop = live_uop[q + FETCHQ_SIZE];
            cbp3_uop_dynamic_t &reader_uop = uop_info[q + FETCHQ_SIZE];

            assert(uop.valid);
            for (int s = 0; s < UOP_SRCS_MAX; s++)
                uop.uop.src_data[s] = reader_uop.src_data[s];
            uop.uop.dst_data = reader_uop.dst_data;
            uop.uop.wflags = reader_uop.wflags;
            uop.uop.rflags = reader_uop.rflags;
            for (int s = 0; s < 4; s++)
                uop.uop.ldst_data[s] = reader_uop.ldst_data[s];

            // calculate mispred info
            uint16_t type = uop.uop.type;
            if (type & IS_BR_CONDITIONAL) {
                bool msp = (uop.uop.br_taken != (uop.last_pred > 0));
                if (!uop.cycle_last_pred) { // no prediction provided
                    msp = true;
                    penalty_cond_br += (cycle - uop.cycle_fetch);
                }else {
                    assert(uop.cycle_last_pred < cycle);
                    penalty_cond_br += ((msp ? cycle : uop.cycle_last_pred) - uop.cycle_fetch);
                }
                msp_cond_br += msp;
            }
            if (type & IS_BR_INDIRECT) {
                bool msp = (uop.uop.br_target != uop.last_pred);
                if (!uop.cycle_last_pred) {
                    msp = true;
                    penalty_ind_br += (cycle - uop.cycle_fetch);
                }else {
                    assert(uop.cycle_last_pred < cycle);
                    penalty_ind_br += ((msp ? cycle : uop.cycle_last_pred) - uop.cycle_fetch);
                }
                msp_ind_br += msp;
            }

            uop.cycle_exe = cycle;
        }

        for (uint32_t i = 0; i < cycle_info.num_agu; i ++) {
            uint8_t q = reader->agu_q[i];
            cycle_info.agu_q[i] = q;
            cbp3_queue_entry_t &uop = live_uop[q + FETCHQ_SIZE];
            cbp3_uop_dynamic_t &reader_uop = uop_info[q + FETCHQ_SIZE];
            assert(uop.valid);
            uop.uop.vaddr = reader_uop.vaddr;
            uop.cycle_agu = cycle;
        }

        for (uint32_t i = 0; i < cycle_info.num_std; i ++) {
            uint8_t q = reader->std_q[i];
            cycle_info.std_q[i] = q;
            cbp3_queue_entry_t &uop = live_uop[q + FETCHQ_SIZE];
            assert(uop.valid);
            uop.cycle_std = cycle;
        }

        for (uint32_t i = 0; i < cycle_info.num_allocate; i ++) {
            uint8_t q = reader->allocate_q[i];
            // must allocate from fetch q entry pointed by allocate_fetchq
            cbp3_queue_entry_t &uop = live_uop[allocate_fetchq];
            // rob entry ptr is from trace
            cycle_info.allocate_q[i] = q;
            cbp3_queue_entry_t &uop_rob = live_uop[q + FETCHQ_SIZE];

            // copy uop_info from fetch queue position to its corresponding rob position
            uop_info[q + FETCHQ_SIZE] = uop_info[allocate_fetchq];
            cbp3_uop_dynamic_t &reader_uop = uop_info[q + FETCHQ_SIZE];

            allocate_fetchq = (allocate_fetchq + 1) % FETCHQ_SIZE;
            assert(uop.valid && !uop_rob.valid);
            // move uop from fetch queue to rob
            uop_rob = uop;
            // invalid the fetch queue entry
            uop.reset();

            // fill fields that are available at allocation stage
            uop_rob.uop.opsize = reader_uop.opsize;
            uop_rob.uop.dst_static = reader_uop.dst_static;
            uop_rob.uop.dst = reader_uop.dst;
            for (int s = 0; s < UOP_SRCS_MAX; s++) {
                uop_rob.uop.srcs_static[s] = reader_uop.srcs_static[s];
                uop_rob.uop.srcs[s] = reader_uop.srcs[s];
            }

            uop_rob.cycle_allocate = cycle;

            if (uop_rob.uop.type & (1 << 13)) {
                int tmp = rename_table[uop_rob.uop.srcs[0]];
                rename_table[uop_rob.uop.srcs[0]] = rename_table[uop_rob.uop.srcs[1]];
                rename_table[uop_rob.uop.srcs[1]] = tmp;
            }else if (!reg_is_constant(uop_rob.uop.dst)) {
                rename_table[uop_rob.uop.dst] = q;
            }

            if (uop_rob.uop.type & IS_WFLAGS)
                rename_table_flags = q;
        }
 
        for (uint32_t i = 0; i < cycle_info.num_fetch; i ++) {
            uint8_t q = reader->fetch_q[i];
            cycle_info.fetch_q[i] = q;
            if (first_fetch) {
                allocate_fetchq = q; // allocate will start from this fetch q
                first_fetch = false;
            }
            cbp3_queue_entry_t &uop = live_uop[q];
            // put oracle uop info in an array and fill into live_uop later
            uop_info[q] = reader->uopinfo[i];
            cbp3_uop_dynamic_t &reader_uop = uop_info[q];
            // uop_id is not supposed to be used in predictors
            reader_uop.uop_id = 0;
            
            assert(!uop.valid);
            uop.valid = true;
            // fill fields that are available at fetch stage
            uop.uop.uop_id = reader_uop.uop_id;
            uop.uop.inst_size = reader_uop.inst_size;
            uop.uop.pc = reader_uop.pc;
            uop.uop.type = reader_uop.type;
            // yes, br_target and direction is available for br history update
            uop.uop.br_target = reader_uop.br_target;
            uop.uop.br_taken = reader_uop.br_taken;
            uop.cycle_fetch = cycle;
        }

        PredictorRunACycle();

        // release rob entries of retired uops
        for (uint32_t i = 0; i < cycle_info.num_retire; i ++) {
            uint8_t q = reader->retire_q[i];
            live_uop[q + FETCHQ_SIZE].reset();
        }

        total_cycle++;

    };

    if (sim_len == -1) { 
        // a complete run, check trace signature
        // if the check failed, something was wrong in the reader
        bool pass = ReaderTraceCheck();
        printf("Trace signature self checking: %s\n\n", pass ? "passed!" : "failed!");
        if (!pass) exit(1);
    }
    // print out stats of a run
    printf("Total cycles:                         %d\n\n", total_cycle);
    printf("Num_Inst:                             %d\n", num_insts);
    printf("Num_Uops:                             %d\n\n", num_uops);

    printf("Num_cond_br:                          %d\n", num_cond_br);
    printf("Mispred_cond_br:                      %d\n", msp_cond_br);
    printf("Mispred_penalty_cond_br:              %d\n", penalty_cond_br);
    printf("Conditional_MPKI:                     %.4f\n", (double)msp_cond_br/(double)num_insts*1000);
    printf("Conditional_MR:                       %.4f\n", (double)msp_cond_br/(double)num_cond_br);
    printf("Final Score Run%i_Conditional_MPPKI:   %.4f\n\n", num_run, (double)penalty_cond_br/(double)num_insts*1000);

    printf("Num_ind_br:                           %d\n", num_ind_br);
    printf("Mispred_ind_br:                       %d\n", msp_ind_br);
    printf("Mispred_penalty_cond_br:              %d\n", penalty_ind_br);
    printf("Indirect_MPKI:                        %.4f\n", (double)msp_ind_br/(double)num_insts*1000);
    printf("Indirect_MR:                          %.4f\n", (double)msp_ind_br/(double)num_ind_br);
    printf("Final Score Run%i_Indirect_MPPKI:      %.4f\n\n", num_run, (double)penalty_ind_br/(double)num_insts*1000);

    PredictorRunEnd();
}

const cbp3_cycle_activity_t *get_cycle_info() {
    return &cycle_info;
}

const cbp3_queue_entry_t *fetch_entry(uint8_t e) {
    assert(e < FETCHQ_SIZE);
    return &live_uop[e];
}

const cbp3_queue_entry_t *rob_entry(uint8_t e) {
    return &live_uop[e + FETCHQ_SIZE];
}

const int rename(uint8_t reg) {
    assert(reg < NUM_REGS);
    return rename_table[reg];
}

const int rename_flags() {
    return rename_table_flags;
}

const uint32_t reg_val(uint8_t reg) {
    assert(reg < NUM_REGS);
    return reg_valid[reg] ? reg_value[reg] : 0;
}

bool  report_pred(uint8_t ptr, bool in_rob, uint32_t pred) {
    if (!in_rob)
        assert(ptr < FETCHQ_SIZE);

    // get the uop from fetchq or rob
    cbp3_queue_entry_t * uop = &live_uop[in_rob ? (ptr + FETCHQ_SIZE) : ptr];
    if (!uop->valid)
        return false;

    if (uop->cycle_exe) // uop has been executed
        return false;
    
    // record the prediction
    uop->cycle_last_pred = cycle;
    uop->last_pred = pred;
    return true;
}

void cbp3_queue_entry_t::reset() {
    uop.reset();
    valid = false;
    cycle_fetch = 0;
    cycle_allocate = 0;
    cycle_agu = 0;
    cycle_std = 0;
    cycle_exe = 0;
    cycle_retire = 0;
    cycle_last_pred = 0;
    last_pred = 0;
}


