#ifndef KPCP_H
#define KPCP_H

// L2 SPP
//#define L2_PF_DEBUG_PRINT
#ifdef L2_PF_DEBUG_PRINT
#define L2_PF_DEBUG(x) x
#else
#define L2_PF_DEBUG(x)
#endif

#define L2_ST_SET 1
#define L2_ST_WAY 256
#define L2_ST_PRIME 1
#define L2_PT_SET 512
#define L2_PT_WAY 4
#define L2_PT_PRIME 509
#define CDELTA_MAX 16
#define CSIG_MAX 16
#define L2_GHR_TRACK 8
#define L2_GHR_ON
#define SIG_SHIFT  3
#define SIG_LENGTH 12
#define SIG_MASK ((1 << SIG_LENGTH) - 1)

#define BAD_MAX 7

class SIGNATURE_TABLE {
  public:

    int valid,
        tag,
        last_block,
        signature,
        lru,
        l2_pf[64],
        used[64],
        delta[64],
        depth[64],
        dirty[64],
        first_hit;

    SIGNATURE_TABLE() {
        valid = 0;
        tag = 0;
        last_block = 0;
        signature = 0;
        lru = 0;
        
        for (uint32_t i=0; i<64; i++) {
            l2_pf[i] = 0;
            used[i] = 0;
            delta[i] = 0;
            depth[i] = 0;
            dirty[i] = 0;
        }

        first_hit = 0;
    };
};

class PATTERN_TABLE {
  public:
    int delta,
        c_delta,
        c_sig;

    PATTERN_TABLE() {
        delta = 0;
        c_delta = 0;
        c_sig = 0;
    };
};

class GLOBAL_HISTORY_REGISTER {
  public:
    int signature,
        path_conf,
        last_block,
        oop_delta,
        lru;

    GLOBAL_HISTORY_REGISTER() {
        signature = 0;
        path_conf = 0;
        last_block = 0;
        oop_delta = 0;
    };
};

extern SIGNATURE_TABLE L2_ST[NUM_CPUS][L2_ST_SET][L2_ST_WAY];
extern PATTERN_TABLE L2_PT[NUM_CPUS][L2_PT_SET][L2_PT_WAY];
extern GLOBAL_HISTORY_REGISTER L2_GHR[NUM_CPUS][L2_GHR_TRACK];
extern int L2_ST_access[NUM_CPUS], L2_ST_hit[NUM_CPUS], L2_ST_invalid[NUM_CPUS], L2_ST_miss[NUM_CPUS];
extern int L2_PT_access[NUM_CPUS], L2_PT_hit[NUM_CPUS], L2_PT_invalid[NUM_CPUS], L2_PT_miss[NUM_CPUS];
extern int l2_sig_dist[NUM_CPUS][1<<SIG_LENGTH];

unsigned int get_new_signature(unsigned int old_signature, int curr_delta);
int L2_ST_update(uint32_t cpu, uint64_t addr);
int L2_ST_check(uint32_t cpu, uint64_t addr);
void L2_PT_update(uint32_t cpu, int signature, int delta);
void notify_sampler(uint32_t cpu, int64_t address, int dirty, int useful);

#endif
