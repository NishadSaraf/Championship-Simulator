#ifndef DRC_H
#define DRC_H

#include "memory_class.h"

// DRC configuration
#define DRC_CHANNEL_WIDTH 16 // 16B
#define DRC_WQ_SIZE 48
#define DRC_RQ_SIZE 48

#define DRC_CHANNELS 4      // default: assuming one DIMM per one channel 4GB * 1 => 4GB off-chip memory
#define LOG2_DRC_CHANNELS 2
#define DRC_RANKS 2         // 128MB * 2 ranks => 256MB per DIMM
#define LOG2_DRC_RANKS 1
#define DRC_BANKS 16         // 8MB * 16 banks => 128MB per rank
#define LOG2_DRC_BANKS 4
#define DRC_ROWS 4096      // 2KB * 4K rows => 8MB per bank
#define LOG2_DRC_ROWS 12
#define DRC_COLUMNS 32      // 64B * 32 column chunks (Assuming 1B DRAM cell * 8 chips * 8 transactions = 64B size of column chunks) => 2KB per row
#define LOG2_DRC_COLUMNS 5
#define DRC_SETS_PER_ROW 28
#define DRC_WAY 1

// the data bus must wait this amount of time when switching between reads and writes, and vice versa
#define DRC_DBUS_TURN_AROUND_TIME ((15*CPU_FREQ)/2000) // 7.5 ns 
#define DRC_TAG_CHECK_TIME ((15*CPU_FREQ)/2000) // 7.5 ns 
extern uint32_t DRC_MTPS, DRC_TAG_SIZE, DRC_BLOCK_SIZE, DRC_TAG_RETURN_TIME, DRC_DATA_RETURN_TIME;

// these values control when to send out a burst of writes
#define DRC_WRITE_HIGH_WM    (DRC_WQ_SIZE*3/4)
#define DRC_WRITE_LOW_WM     (DRC_WQ_SIZE*1/4)
#define MIN_DRC_WRITES_PER_SWITCH (DRC_WQ_SIZE*1/4)

// DRC
class DRC_MEMORY_CONTROLLER : public MEMORY {
  public:
    const string NAME;
    const uint32_t MSHR_SIZE;

    DRAM_ARRAY drc_array[DRC_CHANNELS][DRC_RANKS][DRC_BANKS][DRC_ROWS];
    PACKET_QUEUE MSHR{NAME + "_MSHR", MSHR_SIZE};

    uint64_t dbus_cycle_available[DRC_CHANNELS], dbus_congested, num_valid, num_zero_reuse;
    uint64_t bank_cycle_available[DRC_CHANNELS][DRC_RANKS][DRC_BANKS];
    uint8_t  write_mode[DRC_CHANNELS]; 
    uint8_t do_write;
    uint32_t processed_writes;
    uint32_t num_writes_processed[DRC_CHANNELS],
             scheduled_reads, scheduled_writes;
    int fill_level;

    BANK_REQUEST bank_request[DRC_CHANNELS][DRC_RANKS][DRC_BANKS];

    // constructor
    DRC_MEMORY_CONTROLLER(string v1, uint32_t v2) : NAME (v1), MSHR_SIZE (v2) {
        dbus_congested = 0;
        num_valid = 0;
        num_zero_reuse = 0;

        scheduled_reads = 0;
        scheduled_writes = 0;
        for (uint32_t i=0; i<DRC_CHANNELS; i++) {
            dbus_cycle_available[i] = 0;
            write_mode[i] = 0;
            do_write = 0;
            processed_writes = 0;
            num_writes_processed[i] = 0;

            for (uint32_t j=0; j<DRC_RANKS; j++) {
                for (uint32_t k=0; k<DRC_BANKS; k++) {
                    bank_cycle_available[i][j][k] = 0;

                    for (uint32_t l=0; l<DRC_ROWS; l++) {
                        /*
                        // allocate memory for DRAM cache blocks
                        drc_array[i][j][k][l].block = new BLOCK* [DRC_SETS_PER_ROW];
                        for (uint32_t set=0; set<DRC_COLUMNS; set++)
                            drc_array[i][j][k][l].block[set] = new BLOCK [DRC_WAY];
                        */
                    }
                }
            }
        }

        // set associative DRC
        DRC_TAG_SIZE = 4 * DRC_WAY;
        DRC_BLOCK_SIZE = 64;

        // alloy DRC
        DRC_TAG_SIZE = 4 + 64;
        DRC_BLOCK_SIZE = 4 + 64;

        fill_level = FILL_DRC;
    };

    // destructor
    ~DRC_MEMORY_CONTROLLER() {
        for (uint32_t i=0; i<DRC_CHANNELS; i++) {
            for (uint32_t j=0; j<DRC_RANKS; j++) {
                for (uint32_t k=0; k<DRC_BANKS; k++) {
                    for (uint32_t l=0; l<DRC_ROWS; l++) {
                        if (drc_array[i][j][k][l].block != NULL) {
                            for (uint32_t set=0; set<DRC_SETS_PER_ROW; set++) {
                                if (drc_array[i][j][k][l].block[set] != NULL)
                                    delete[] drc_array[i][j][k][l].block[set];
                            }
                            delete[] drc_array[i][j][k][l].block;
                        }
                    }
                }
            }
        }
    };

    // queues
    PACKET_QUEUE WQ{NAME + "_WQ", DRC_WQ_SIZE}, // write queue
                 RQ{NAME + "_RQ", DRC_RQ_SIZE}; // read queue

    // functions
    virtual int  add_rq(PACKET *packet);
    virtual int  add_wq(PACKET *packet);
    virtual int  add_pq(PACKET *packet);
    virtual void return_data(PACKET *packet);
    virtual void operate();
    virtual void increment_WQ_FULL();
    virtual uint32_t get_occupancy(uint8_t queue_type);
    virtual uint32_t get_size(uint8_t queue_type);

    //void schedule(uint32_t channel, PACKET_QUEUE *queue), process(uint32_t channel, PACKET_QUEUE *queue),
    void schedule(PACKET_QUEUE *queue), process(PACKET_QUEUE *queue),
         update_schedule_cycle(PACKET_QUEUE *queue),
         update_process_cycle(PACKET_QUEUE *queue),
         reset_remain_requests(PACKET_QUEUE *queue),
         update_replacement_state(uint32_t cpu, uint32_t set, uint32_t way, uint64_t full_addr, uint64_t ip, uint64_t victim_addr, uint32_t type, uint8_t hit),
         fill_cache(BLOCK *block, uint32_t set, uint32_t way, PACKET *packet),
         add_mshr(PACKET *packet);

    uint32_t drc_get_channel(uint64_t address),
             drc_get_rank   (uint64_t address),
             drc_get_bank   (uint64_t address),
             drc_get_row    (uint64_t address),
             drc_get_column (uint64_t address),
             drc_check_hit  (PACKET_QUEUE *queue, uint64_t address, uint32_t cpu, uint32_t channel, uint32_t rank, uint32_t bank, uint32_t row),
             find_victim(uint32_t cpu, uint64_t instr_id, uint32_t set, const BLOCK *current_set, uint64_t ip, uint64_t full_addr, uint32_t type);
    uint64_t get_bank_earliest_cycle();
    int check_drc_queue(PACKET_QUEUE *queue, PACKET *packet),
        check_mshr(PACKET *packet);
};

#endif
