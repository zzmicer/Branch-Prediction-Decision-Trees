// Author: Hongliang Gao;   Created: Jan 27 2011
// Description: sample predictors for cbp3.

#include <stdio.h>
#include <cassert>
#include <string.h>
#include <inttypes.h>
#include <queue>


using namespace std;
#include "cbp3_def.h"
#include "cbp3_framework.h"

// this file includes two sample predictors
// one is a 64 KB gshare conditional predictor
// the other is a 64 KB indirect predictor indexed by (pc ^ history)

// rewind_marked is also used to show how to rewind the reader for multiple runs
// the predictor will use gshare in the first run
// the indirect predictor will be used in the second run

// NOTE: rewind_marked is only provided to help tuning work. the final
// submitted code should only include one run.


#define GSHARE_SIZE 6 // 256K 2-bit counters = 64 KB cost
#define IND_SIZE 4    // 16K 32-bit targets  = 64 KB cost
#define GA_SIZE 48

// predictor tables
int8_t   *gtable;
uint32_t *indtable;
queue<uint8_t> gaqueue;


// two branch history registers:
// the framework provids real branch results at fetch stage to simplify branch history
// update for predicting later branches. however, they are not available until execution stage
// in a real machine. therefore, you can only use them to update predictors at or after the
// branch is executed.
// in this sample code, we update predictors at retire stage where uops are processed
// in order to enable easy regneration of branch history.

// cost: depending on predictor size
uint32_t brh_fetch;
uint32_t brh_retire;

// count number of runs
uint32_t runs;

void printQueue(queue<uint8_t> q)
{
	//printing content of queue 
	while (!q.empty()){
		printf("%d",q.front());
		printf(" ");
		q.pop();
	}
	printf("\n");
}

void PredictorInit() {
    runs = 0;
    gtable = new int8_t[1 << GSHARE_SIZE];
    indtable = new uint32_t[1 << IND_SIZE];
    assert(gtable && indtable);
}

void PredictorReset() {
    // this function is called before EVERY run
    // it is used to reset predictors and change configurations

    if (runs == 0)
        printf("Predictor:gshare\nconfig: %i counters, %i KB cost\n", 1 << GSHARE_SIZE, (1 << GSHARE_SIZE) * 2 / 8 / 1024);
    else
        printf("Predictor:ind\nconfig: %i targets,  %i KB cost\n", 1 << IND_SIZE, (1 << IND_SIZE) * 4 / 1024);

    for (int i = 0; i < (1 << GSHARE_SIZE); i ++)
        gtable[i] = 0;
    for (int i = 0; i < (1 << IND_SIZE); i ++)
        indtable[i] = 0;
	     	
    for (int i=0;i<GA_SIZE;i++)
	gaqueue.push(0);	

    brh_fetch = 0;
    brh_retire = 0;
}

void PredictorRunACycle() {
    // get info about what uops are processed at each pipeline stage
    const cbp3_cycle_activity_t *cycle_info = get_cycle_info();

    // make prediction at fetch stage
    for (int i = 0; i < cycle_info->num_fetch; i++) {
        uint32_t fe_ptr = cycle_info->fetch_q[i];
        const cbp3_uop_dynamic_t *uop = &fetch_entry(fe_ptr)->uop;

        if (runs == 0 && uop->type & IS_BR_CONDITIONAL) {
            // get prediction	    
			gaqueue.pop();
			gaqueue.push(uop->pc & 0xFF);
            uint32_t gidx = (brh_fetch ^ uop->pc) & ((1 << GSHARE_SIZE) - 1);
            bool gpred = (gtable[gidx] >= 0);
		
        printf("PC@%u", uop->pc);
        printf("\n");
        printf("GTABLE@");
        for (int i = 0; i < (1 << GSHARE_SIZE); i++){
			printf("%d",gtable[i]);
			printf(" ");
	    }
	    printf("\n");
	    printf("GA@");
	    printQueue(gaqueue);
	    printf("TARGET@%d",uop->br_taken);
	    printf("\n");   
	    
        
        assert(report_pred(fe_ptr, false, gpred));
        }else if (runs == 1 && uop->type & IS_BR_INDIRECT) {
            uint32_t gidx = (brh_fetch ^ uop->pc) & ((1 << IND_SIZE) - 1);
            uint32_t gpred = indtable[gidx];

            assert(report_pred(fe_ptr, false, gpred));
        }

        // update fetch branch history
        if (uop->type & IS_BR_CONDITIONAL)
            brh_fetch = (brh_fetch << 1) | (uop->br_taken ? 1 : 0);
        else if (uop_is_branch(uop->type))
            brh_fetch = (brh_fetch << 1) | 1;
    }

    for (int i = 0; i < cycle_info->num_retire; i++) {
        uint32_t rob_ptr = cycle_info->retire_q[i];
        const cbp3_uop_dynamic_t *uop = &rob_entry(rob_ptr)->uop;

        if (runs == 0 && uop->type & IS_BR_CONDITIONAL) {
            uint32_t gidx = (brh_retire ^ uop->pc) & ((1 << GSHARE_SIZE) - 1);

            // update predictor
            bool t = uop->br_taken;
            if (t && gtable[gidx] < 1)
                gtable[gidx] ++;
            else if (!t && gtable[gidx] > -2)
                gtable[gidx] --;
        }else if (runs == 1 && uop->type & IS_BR_INDIRECT) {
            uint32_t gidx = (brh_retire ^ uop->pc) & ((1 << IND_SIZE) - 1);
            indtable[gidx] = uop->br_target;
        }

        // update retire branch history
        if (uop->type & IS_BR_CONDITIONAL)
            brh_retire = (brh_retire << 1) | (uop->br_taken ? 1 : 0);
        else if (uop_is_branch(uop->type))
            brh_retire = (brh_retire << 1) | 1;
    
    
    }
}

void PredictorRunEnd() {
    runs ++;
    if (runs < 2) // set rewind_marked to indicate that we want more runs
        rewind_marked = true;
}

void PredictorExit() {
    delete [] gtable;
    delete [] indtable;
}
