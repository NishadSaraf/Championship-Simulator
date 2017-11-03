#include "drc_controller.h"
#include "dram_controller.h"

// initialized in main.cc
uint64_t last_drc_read_mode, last_drc_write_mode, drc_blocks;
uint32_t DRC_MTPS, DRC_TAG_SIZE, DRC_BLOCK_SIZE, DRC_TAG_RETURN_TIME, DRC_DATA_RETURN_TIME;

void DRC_MEMORY_CONTROLLER::reset_remain_requests(PACKET_QUEUE *queue)
{
    for (uint32_t i=0; i<queue->SIZE; i++) {
        if (queue->entry[i].scheduled) {

            uint64_t op_addr = queue->entry[i].address;
            uint32_t op_cpu = queue->entry[i].cpu,
                     op_channel = drc_get_channel(op_addr), 
                     op_rank = drc_get_rank(op_addr), 
                     op_bank = drc_get_bank(op_addr), 
                     op_row = drc_get_row(op_addr);

#ifdef DEBUG_PRINT
            //uint32_t op_column = drc_get_column(op_addr);
#endif

            // update open row
            if ((bank_request[op_channel][op_rank][op_bank].cycle_available - tCAS) <= current_core_cycle[op_cpu])
                bank_request[op_channel][op_rank][op_bank].open_row = op_row;
            else
                bank_request[op_channel][op_rank][op_bank].open_row = UINT32_MAX;

            // this bank is ready for another DRC request
            bank_request[op_channel][op_rank][op_bank].request_index = -1;
            bank_request[op_channel][op_rank][op_bank].row_buffer_hit = 0;
            bank_request[op_channel][op_rank][op_bank].working = 0;
            bank_request[op_channel][op_rank][op_bank].cycle_available = current_core_cycle[op_cpu];
            if (bank_request[op_channel][op_rank][op_bank].is_write) {
                scheduled_writes--;
                bank_request[op_channel][op_rank][op_bank].is_write = 0;
            }
            else if (bank_request[op_channel][op_rank][op_bank].is_read) {
                scheduled_reads--;
                bank_request[op_channel][op_rank][op_bank].is_read = 0;
            }

            queue->entry[i].scheduled = 0;
            queue->entry[i].event_cycle = current_core_cycle[op_cpu];

            DP ( if (warmup_complete[op_cpu]) {
            cout << queue->NAME << " instr_id: " << queue->entry[i].instr_id << " swrites: " << scheduled_writes << " sreads: " << scheduled_reads << endl; });

        }
    }
    
    update_schedule_cycle(&RQ);
    update_schedule_cycle(&WQ);
    update_process_cycle(&RQ);
    update_process_cycle(&WQ);

#ifdef SANITY_CHECK
    if (queue->is_WQ) {
        if (scheduled_writes != 0)
            assert(0);
    }
    else {
        if (scheduled_reads != 0)
            assert(0);
    }
#endif
}

void DRC_MEMORY_CONTROLLER::operate()
{
    if (do_write == 0) {
        if (WQ.occupancy >= DRC_WRITE_HIGH_WM) {
            do_write = 1;
            processed_writes = 0;
            last_drc_read_mode = current_core_cycle[0];
            //cout << "DRC is now WRITE mode cycle: " << current_core_cycle[0] << endl;

            // reset scheduled RQ requests
            reset_remain_requests(&RQ);

            for (uint32_t i=0; i<DRC_CHANNELS; i++) {
                // add data bus turnaround time
                dbus_cycle_available[i] += DRC_DBUS_TURN_AROUND_TIME;
            }
        }
    } else {
        if (WQ.occupancy == 0)
            do_write = 0;
        else if (RQ.occupancy && (WQ.occupancy < DRC_WRITE_LOW_WM))
            do_write = 0;
        else if (RQ.occupancy && (MIN_DRC_WRITES_PER_SWITCH <= processed_writes))
            do_write = 0;

        if (do_write == 0) {
            processed_writes = 0;
            last_drc_write_mode = current_core_cycle[0];
            //cout << "DRC is now READ  mode cycle: " << current_core_cycle[0] << endl;

            // reset scheduled WQ requests
            reset_remain_requests(&WQ);

            for (uint32_t i=0; i<DRC_CHANNELS; i++) {
                // add data bus turnaround time
                dbus_cycle_available[i] += DRC_DBUS_TURN_AROUND_TIME;
            }
        }
    }

    // do FR-FCFS scheduling

    // handle write
    // schedule new entry
    //for (uint32_t channel=0; channel<DRC_CHANNELS; channel++) {
        //if (write_mode[channel] && (WQ.next_schedule_index < WQ.SIZE)) {
        if (do_write && (WQ.next_schedule_index < WQ.SIZE)) {
            if (WQ.next_schedule_cycle <= current_core_cycle[WQ.entry[WQ.next_schedule_index].cpu])
                schedule(&WQ);
                //schedule(channel, &WQ);
        }
    //}

    // process DRC requests
    //for (uint32_t channel=0; channel<DRC_CHANNELS; channel++) {
        //if (write_mode[channel] && (WQ.next_process_index < WQ.SIZE)) {
        if (do_write && (WQ.next_process_index < WQ.SIZE)) {
            if (WQ.next_process_cycle <= current_core_cycle[WQ.entry[WQ.next_process_index].cpu])
                //process(channel, &WQ);
                process(&WQ);
        }
    //}

    // handle read
    // schedule new entry
    //for (uint32_t channel=0; channel<DRC_CHANNELS; channel++) {
        //if ((write_mode[channel] == 0) && (RQ.next_schedule_index < RQ.SIZE)) {
        if ((do_write == 0) && (RQ.next_schedule_index < RQ.SIZE)) {
            if (RQ.next_schedule_cycle <= current_core_cycle[RQ.entry[RQ.next_schedule_index].cpu])
                schedule(&RQ);
                //schedule(channel, &RQ);
        }
    //}

    // process DRC requests
    //for (uint32_t channel=0; channel<DRC_CHANNELS; channel++) {
        //if ((write_mode[channel] == 0) && (RQ.next_process_index < RQ.SIZE)) {
        if ((do_write == 0) && (RQ.next_process_index < RQ.SIZE)) {
            if (RQ.next_process_cycle <= current_core_cycle[RQ.entry[RQ.next_process_index].cpu])
                process(&RQ);
                //process(channel, &RQ);
        }
    //}
}

void DRC_MEMORY_CONTROLLER::schedule(PACKET_QUEUE *queue)
{
    uint64_t drc_addr;
    uint32_t drc_channel, drc_rank, drc_bank, drc_row;
    uint8_t  row_buffer_hit = 0;

    int oldest_index = -1;
    uint64_t oldest_cycle = UINT64_MAX;

    // first, search for the oldest open row hit
    for (uint32_t i=0; i<queue->SIZE; i++) {

        // empty entry
        drc_addr = queue->entry[i].address;
        if (drc_addr == 0)
            continue;

        // already scheduled
        if (queue->entry[i].scheduled) { 
            //cout << queue->NAME << " index: " << i << " is already scheduled" << endl;
            continue;
        }

        drc_channel = drc_get_channel(drc_addr);
        drc_rank = drc_get_rank(drc_addr);
        drc_bank = drc_get_bank(drc_addr);

        // bank is busy
        if (bank_request[drc_channel][drc_rank][drc_bank].working) {

            //DP ( if (warmup_complete[0]) {
            //cout << queue->NAME << " " << __func__ << " instr_id: " << queue->entry[i].instr_id << " bank is busy";
            //cout << " swrites: " << scheduled_writes << " sreads: " << scheduled_reads;
            //cout << " write: " << +bank_request[drc_channel][drc_rank][drc_bank].is_write << " read: " << +bank_request[drc_channel][drc_rank][drc_bank].is_read << hex;
            //cout << " address: " << queue->entry[i].address << dec << " channel: " << drc_channel << " rank: " << drc_rank << " bank: " << drc_bank << endl; });
            //cout << queue->NAME << " index: " << i << " rank: " << drc_rank << " bank: " << drc_bank << " is busy" << endl;

            continue;
        }

        // check open row
        drc_row = drc_get_row(drc_addr);
        if (bank_request[drc_channel][drc_rank][drc_bank].open_row != drc_row) {

            /*
            DP ( if (warmup_complete[0]) {
            cout << queue->NAME << " " << __func__ << " instr_id: " << queue->entry[i].instr_id << " row is inactive";
            cout << " swrites: " << scheduled_writes << " sreads: " << scheduled_reads;
            cout << " write: " << +bank_request[drc_channel][drc_rank][drc_bank].is_write << " read: " << +bank_request[drc_channel][drc_rank][drc_bank].is_read << hex;
            cout << " address: " << queue->entry[i].address << dec << " channel: " << drc_channel << " rank: " << drc_rank << " bank: " << drc_bank << endl; });
            */
            //cout << queue->NAME << " index: " << i << " event_cycle: " << queue->entry[i].event_cycle << " is selected" << endl;

            continue;
        }

        // select the oldest entry
        if (queue->entry[i].event_cycle < oldest_cycle) {
            oldest_cycle = queue->entry[i].event_cycle;
            oldest_index = i;
            row_buffer_hit = 1;
        }	  
    }

    if (oldest_index == -1) { // no matching open_row (row buffer miss)

        oldest_cycle = UINT64_MAX;
        for (uint32_t i=0; i<queue->SIZE; i++) {

            // empty entry
            drc_addr = queue->entry[i].address;
            if (drc_addr == 0)
                continue;
            
            // already scheduled
            if (queue->entry[i].scheduled) {
                //cout << queue->NAME << " index: " << i << " is already scheduled" << endl;
                continue;
            }

            drc_channel = drc_get_channel(drc_addr);
            drc_rank = drc_get_rank(drc_addr);
            drc_bank = drc_get_bank(drc_addr);

            // bank is busy
            if (bank_request[drc_channel][drc_rank][drc_bank].working) { 
                //cout << queue->NAME << " index: " << i << " rank: " << drc_rank << " bank: " << drc_bank << " is busy" << endl;
                continue;
            }

            // select the oldest entry
            if (queue->entry[i].event_cycle < oldest_cycle) {
                oldest_cycle = queue->entry[i].event_cycle;
                oldest_index = i;
                //cout << queue->NAME << " index: " << i << " event_cycle: " << queue->entry[i].event_cycle << " is selected" << endl;
            }
        }
    }

    // at this point, the scheduler knows which channel, rank, bank, row to access and if the request is a row buffer hit or miss
    if (oldest_index != -1) { // scheduler might not find anything if all requests are already scheduled or all banks are busy

        uint64_t LATENCY = 0;
        if (row_buffer_hit)
            LATENCY = tCAS;
            //LATENCY = tCAS + DRC_TAG_CHECK_TIME;
        else 
            LATENCY = tRP + tRCD + tCAS;
            //LATENCY = tRP + tRCD + tCAS + DRC_TAG_CHECK_TIME;

        uint64_t op_addr = queue->entry[oldest_index].address;
        uint32_t op_cpu = queue->entry[oldest_index].cpu,
                 op_channel = drc_get_channel(op_addr), 
                 op_rank = drc_get_rank(op_addr), 
                 op_bank = drc_get_bank(op_addr), 
                 op_row = drc_get_row(op_addr);
#ifdef DEBUG_PRINT
        uint32_t op_column = drc_get_column(op_addr);
#endif
        // this bank is now busy
        bank_request[op_channel][op_rank][op_bank].working = 1;
        bank_request[op_channel][op_rank][op_bank].cycle_available = current_core_cycle[op_cpu] + LATENCY;

        bank_request[op_channel][op_rank][op_bank].request_index = oldest_index;
        bank_request[op_channel][op_rank][op_bank].row_buffer_hit = row_buffer_hit;
        if (queue->is_WQ) {
            bank_request[op_channel][op_rank][op_bank].is_write = 1;
            bank_request[op_channel][op_rank][op_bank].is_read = 0;
            scheduled_writes++;
        } else {
            bank_request[op_channel][op_rank][op_bank].is_write = 0;
            bank_request[op_channel][op_rank][op_bank].is_read = 1;
            scheduled_reads++;
        }

        // update open row
        bank_request[op_channel][op_rank][op_bank].open_row = op_row;
        queue->entry[oldest_index].scheduled = 1;
        queue->entry[oldest_index].event_cycle = current_core_cycle[op_cpu] + LATENCY;

        DP (if (warmup_complete[op_cpu]) {
        cout << "[" << queue->NAME << "] " <<  __func__ << " instr_id: " << queue->entry[oldest_index].instr_id;
        cout << " row buffer: " << (row_buffer_hit ? (int)bank_request[op_channel][op_rank][op_bank].open_row : -1) << hex;
        cout << " address: " << queue->entry[oldest_index].address << " full_addr: " << queue->entry[oldest_index].full_addr << dec;
        cout << " index: " << oldest_index << " occupancy: " << queue->occupancy;
        cout << " ch: " << op_channel << " rank: " << op_rank << " bank: " << op_bank; // wrong from here
        cout << " row: " << op_row << " current: " << current_core_cycle[op_cpu] << " event: " << queue->entry[oldest_index].event_cycle << endl; });

        update_schedule_cycle(queue);
        update_process_cycle(queue);
    }
}

//void DRC_MEMORY_CONTROLLER::process(uint32_t channel, PACKET_QUEUE *queue)
void DRC_MEMORY_CONTROLLER::process(PACKET_QUEUE *queue)
{
    uint32_t request_index = queue->next_process_index;

    // sanity check
    if (request_index == queue->SIZE)
        assert(0);

    uint64_t op_addr = queue->entry[request_index].address;
    uint32_t op_cpu = queue->entry[request_index].cpu,
             op_channel = drc_get_channel(op_addr), 
             op_rank = drc_get_rank(op_addr), 
             op_bank = drc_get_bank(op_addr),
             op_row = drc_get_row(op_addr),
             op_column = drc_get_column(op_addr);

    uint64_t op_instr_id = queue->entry[request_index].instr_id,
             op_ip = queue->entry[request_index].ip,
             op_full_addr = queue->entry[request_index].full_addr;
    uint8_t  op_type = queue->entry[request_index].type;

#ifdef SANITY_CHECK
    if (bank_request[op_channel][op_rank][op_bank].request_index != (int)request_index) {
        cout << queue->NAME << " " << "instr_id: " << op_instr_id << " cpu: " << op_cpu << " op_addr: " << hex << op_addr << dec;
        cout << " ch: " << op_channel << " rank: " << op_rank << " bank: " << op_bank << " row: " << op_row << " column: " << op_column;
        cout << " cycle: " << current_core_cycle[op_cpu] << endl;
        assert(0);
    }
#endif

    // for alloy style cache, you can skip tag checking part because tag and data are co-allocated
    // just uncomment this line and it will skip the tag checking
    // queue->entry[request_index].drc_tag_read = 1;

    // ================================================= FIRST PROCESS TAG ============================================================= //
    // paid all DRC access latency, ready to read tags
    if ((queue->entry[request_index].drc_tag_read == 0) && (bank_request[op_channel][op_rank][op_bank].cycle_available <= current_core_cycle[op_cpu])) {
        // check if data bus is available
        if (dbus_cycle_available[op_channel] <= current_core_cycle[op_cpu]) {

            // update data bus cycle time
            dbus_cycle_available[op_channel] = current_core_cycle[op_cpu] + DRC_TAG_RETURN_TIME;

            DP ( if (warmup_complete[op_cpu]) {
            cout << "[" << queue->NAME << "] " <<  __func__ << " tag read dbus_free";
            cout << " address: " << hex << queue->entry[request_index].address << " full_addr: " << queue->entry[request_index].full_addr << dec;
            cout << " occupancy: " << queue->occupancy << " channel: " << op_channel << " rank: " << op_rank << " bank: " << op_bank;
            cout << " row: " << op_row << " column: " << op_column;
            cout << " current_cycle: " << current_core_cycle[op_cpu] << " event_cycle: " << queue->entry[request_index].event_cycle << endl; });

            // update queue entry
            queue->entry[request_index].event_cycle = dbus_cycle_available[op_channel] + tCAS; // we need another DRC access with tCAS latency to process actual data
            queue->entry[request_index].drc_tag_read = 1;

            update_process_cycle(queue);

        } else { // data bus is busy, the available bank cycle time is fast-forwarded for faster simulation
            bank_request[op_channel][op_rank][op_bank].cycle_available = dbus_cycle_available[op_channel];
            dbus_congested++;

            DP ( if (warmup_complete[op_cpu]) {
            cout << "[" << queue->NAME << "] " <<  __func__ << " tag read dbus_occupied" << hex;
            cout << " address: " << queue->entry[request_index].address << " full_addr: " << queue->entry[request_index].full_addr << dec;
            cout << " occupancy: " << queue->occupancy << " channel: " << op_channel << " rank: " << op_rank << " bank: " << op_bank;
            cout << " row: " << op_row << " column: " << op_column;
            cout << " current_cycle: " << current_core_cycle[op_cpu] << " event_cycle: " << bank_request[op_channel][op_rank][op_bank].cycle_available << endl; });

            return; // request cannot be processed at this cycle
        }

        return; // processed tags
    }
    // ================================================================================================================================= //


    // processed DRC tag and paid all DRC access latency, data is ready to be processed
    if (queue->entry[request_index].drc_tag_read && (bank_request[op_channel][op_rank][op_bank].cycle_available <= current_core_cycle[op_cpu])) {

        // check if this is a hit
        uint32_t set = op_addr % DRC_SETS_PER_ROW,
                 way = drc_check_hit(queue, op_addr, op_cpu, op_channel, op_rank, op_bank, op_row);
        if (way < DRC_WAY)
            bank_request[op_channel][op_rank][op_bank].drc_hit = 1;
        else
            bank_request[op_channel][op_rank][op_bank].drc_hit = 0;

        if (queue->is_WQ) { // code is little bit different here because DRC write always needs to use DBUS but DRC read miss does not occupy dbus and send request to the lower level
            if (dbus_cycle_available[op_channel] <= current_core_cycle[op_cpu]) { // first, check if data bus is available
                if (bank_request[op_channel][op_rank][op_bank].drc_hit) { // write hit

                    // update data bus cycle time
                    dbus_cycle_available[op_channel] = current_core_cycle[op_cpu] + DRC_DATA_RETURN_TIME;

                    if (bank_request[op_channel][op_rank][op_bank].row_buffer_hit)
                        queue->ROW_BUFFER_HIT++;
                    else
                        queue->ROW_BUFFER_MISS++;

                    HIT[op_type]++;
                    ACCESS[op_type]++;

                    DP (if (warmup_complete[op_cpu]) {
                    cout << "[DRC_WQ] " << __func__ << " write hit address: " << hex << op_addr << dec << " cpu: " << op_cpu << " channel: " << op_channel;
                    cout << " rank: " << op_rank << " bank: " << op_bank << " row: " << op_row << " set: " << (op_addr % DRC_SETS_PER_ROW) << endl; }); 

                    // this bank is ready for another DRC request
                    bank_request[op_channel][op_rank][op_bank].request_index = -1;
                    bank_request[op_channel][op_rank][op_bank].row_buffer_hit = 0;
                    bank_request[op_channel][op_rank][op_bank].working = 0;
                    bank_request[op_channel][op_rank][op_bank].is_write = 0;
                    bank_request[op_channel][op_rank][op_bank].is_read = 0;
                    bank_request[op_channel][op_rank][op_bank].drc_hit = 0;

                    num_writes_processed[op_channel]++;
                    processed_writes++;
                    scheduled_writes--;

                    // remove the oldest entry
                    queue->remove_queue(&queue->entry[request_index]);

                    update_process_cycle(queue);
                } else { // write miss

                    // find victim
                    uint32_t victim_way = find_victim(op_cpu, op_instr_id, set, drc_array[op_channel][op_rank][op_bank][op_row].block[set], op_ip, op_full_addr, op_type);

                    BLOCK *op_block = &drc_array[op_channel][op_rank][op_bank][op_row].block[set][victim_way];
#ifdef DRC_BYPASS
                    if (victim_way == DRC_WAY) {
                        cerr << "DRC bypassing for writebacks is not allowed!" << endl;
                        assert(0);
                    }
#endif
                    uint8_t  do_fill = 1;

                    // is this dirty?
                    if (op_block->dirty) {

                        // check if the lower level WQ has enough room to keep this writeback request
                        if (lower_level) { 
                            if (lower_level->get_occupancy(2) == lower_level->get_size(2)) {

                                // lower level WQ is full, cannot replace this victim
                                do_fill = 0;
                                lower_level->increment_WQ_FULL();
                                STALL[op_type]++;

                                DP ( if (warmup_complete[op_cpu]) {
                                cout << "[" << NAME << "] " << __func__ << "do_fill: " << +do_fill;
                                cout << " lower level wq is full!" << " fill_addr: " << hex << op_addr;
                                cout << " victim_addr: " << op_block->address << dec << endl; });

                                return; // request cannot be processed at this cycle
                            } else {
                                PACKET writeback_packet;

                                writeback_packet.fill_level = fill_level << 1;
                                writeback_packet.cpu = op_cpu;
                                writeback_packet.address = op_block->address;
                                writeback_packet.full_addr = op_block->full_addr;
                                writeback_packet.data = op_block->data;
                                writeback_packet.instr_id = op_instr_id;
                                writeback_packet.ip = 0;
                                writeback_packet.type = WRITEBACK;
                                writeback_packet.event_cycle = current_core_cycle[op_cpu];

                                lower_level->add_wq(&writeback_packet);
                            }
                        }
                    }

                    if (do_fill) {
                        // update replacement policy
                        update_replacement_state(op_cpu, set, victim_way, op_full_addr, op_ip, op_full_addr, op_type, 0);

                        // check if this block was used or not
                        if (op_block->valid && (op_block->used == 0))
                            num_zero_reuse++; // block is evicted without a single reuse

                        fill_cache(op_block, set, victim_way, &queue->entry[request_index]);

                        // mark dirty
                        op_block->dirty = 1; 

                        // check fill level
                        if (queue->entry[request_index].fill_level < fill_level) {
                            if (queue->entry[request_index].instruction) 
                                upper_level_icache[op_cpu]->return_data(&queue->entry[request_index]);
                            else // data
                                upper_level_dcache[op_cpu]->return_data(&queue->entry[request_index]);
                        }

                        if (bank_request[op_channel][op_rank][op_bank].row_buffer_hit)
                            queue->ROW_BUFFER_HIT++;
                        else
                            queue->ROW_BUFFER_MISS++;
                        
                        MISS[op_type]++;
                        ACCESS[op_type]++;

                        DP (if (warmup_complete[op_cpu]) {
                        cout << "[DRC_WQ] " << __func__ << " write miss address: " << hex << op_addr << dec << " cpu: " << op_cpu << " channel: " << op_channel;
                        cout << " rank: " << op_rank << " bank: " << op_bank << " row: " << op_row << " set: " << set << endl; }); 

                        // this bank is ready for another DRC request
                        bank_request[op_channel][op_rank][op_bank].request_index = -1;
                        bank_request[op_channel][op_rank][op_bank].row_buffer_hit = 0;
                        bank_request[op_channel][op_rank][op_bank].working = 0;
                        bank_request[op_channel][op_rank][op_bank].is_write = 0;
                        bank_request[op_channel][op_rank][op_bank].is_read = 0;
                        bank_request[op_channel][op_rank][op_bank].drc_hit = 0;

                        num_writes_processed[op_channel]++;
                        processed_writes++;
                        scheduled_writes--;

                        // remove the oldest entry
                        queue->remove_queue(&queue->entry[request_index]);

                        update_process_cycle(queue);
                    }
                }
            } else { // data bus is busy, the available bank cycle time is fast-forwarded for faster simulation
                bank_request[op_channel][op_rank][op_bank].cycle_available = dbus_cycle_available[op_channel];
                dbus_congested++;

                DP ( if (warmup_complete[op_cpu]) {
                cout << "[" << queue->NAME << "] " <<  __func__ << " dbus_occupied" << hex;
                cout << " address: " << queue->entry[request_index].address << " full_addr: " << queue->entry[request_index].full_addr << dec;
                cout << " occupancy: " << queue->occupancy << " channel: " << op_channel << " rank: " << op_rank << " bank: " << op_bank;
                cout << " row: " << op_row << " column: " << op_column;
                cout << " current_cycle: " << current_core_cycle[op_cpu] << " event_cycle: " << bank_request[op_channel][op_rank][op_bank].cycle_available << endl; });

                return; // request cannot be processed at this cycle
            }
        } else { // DRC read
            if (bank_request[op_channel][op_rank][op_bank].drc_hit) { // read hit

                // check if data bus is available
                if (dbus_cycle_available[op_channel] <= current_core_cycle[op_cpu]) {

                    // update data bus cycle time
                    dbus_cycle_available[op_channel] = current_core_cycle[op_cpu] + DRC_DATA_RETURN_TIME;
                    queue->entry[request_index].event_cycle = dbus_cycle_available[op_channel]; 

                    DP ( if (warmup_complete[op_cpu]) {
                    cout << "[" << queue->NAME << "] " <<  __func__ << " read hit" << hex;
                    cout << " address: " << queue->entry[request_index].address << " full_addr: " << queue->entry[request_index].full_addr << dec;
                    cout << " occupancy: " << queue->occupancy << " channel: " << op_channel << " rank: " << op_rank << " bank: " << op_bank;
                    cout << " row: " << op_row << " column: " << op_column;
                    cout << " current_cycle: " << current_core_cycle[op_cpu] << " event_cycle: " << queue->entry[request_index].event_cycle << endl; });

                    // send data back to the core cache hierarchy
                    upper_level_dcache[op_cpu]->return_data(&queue->entry[request_index]);

                    if (bank_request[op_channel][op_rank][op_bank].row_buffer_hit)
                        queue->ROW_BUFFER_HIT++;
                    else
                        queue->ROW_BUFFER_MISS++;

                    // this bank is ready for another DRC request
                    bank_request[op_channel][op_rank][op_bank].request_index = -1;
                    bank_request[op_channel][op_rank][op_bank].row_buffer_hit = 0;
                    bank_request[op_channel][op_rank][op_bank].working = 0;
                    bank_request[op_channel][op_rank][op_bank].is_write = 0;
                    bank_request[op_channel][op_rank][op_bank].is_read = 0;
                    bank_request[op_channel][op_rank][op_bank].drc_hit = 0;

                    drc_array[op_channel][op_rank][op_bank][op_row].block[set][way].used = 1;

                    scheduled_reads--;

                    HIT[op_type]++;
                    ACCESS[op_type]++;

                    // remove the oldest entry
                    queue->remove_queue(&queue->entry[request_index]);

                    update_process_cycle(queue);
                } else { // data bus is busy, the available bank cycle time is fast-forwarded for faster simulation
                    bank_request[op_channel][op_rank][op_bank].cycle_available = dbus_cycle_available[op_channel];
                    dbus_congested++;

                    DP ( if (warmup_complete[op_cpu]) {
                    cout << "[" << queue->NAME << "] " <<  __func__ << " read hit dbus_occupied" << hex;
                    cout << " address: " << queue->entry[request_index].address << " full_addr: " << queue->entry[request_index].full_addr << dec;
                    cout << " occupancy: " << queue->occupancy << " channel: " << op_channel << " rank: " << op_rank << " bank: " << op_bank;
                    cout << " row: " << op_row << " column: " << op_column;
                    cout << " current_cycle: " << current_core_cycle[op_cpu] << " event_cycle: " << bank_request[op_channel][op_rank][op_bank].cycle_available << endl; });

                    return; // request cannot be processed at this cycle
                }
            } else { // read miss

                DP ( if (warmup_complete[op_cpu]) {
                cout << "[" << NAME << "] " << __func__ << " read miss";
                cout << " instr_id: " << op_instr_id << " address: " << hex << op_addr;
                cout << " full_addr: " << op_full_addr << dec;
                cout << " cycle: " << queue->entry[request_index].event_cycle << endl; });

                // check mshr
                uint8_t miss_handled = 1;
                int mshr_index = check_mshr(&queue->entry[request_index]);

                if ((mshr_index == -1) && (MSHR.occupancy < MSHR_SIZE)) { // this is a new miss

                    // add it to mshr (read miss)
                    add_mshr(&queue->entry[request_index]);

                    // before we add this packet to DRAM RQ, we must reset scheduled bit 
                    // since this packet was once scheduled while it was sitting in DRC RQ
                    queue->entry[request_index].scheduled = 0;
                    queue->entry[request_index].drc_tag_read = 0;

                    // add it to the next level's read queue
                    if (lower_level)
                        lower_level->add_rq(&queue->entry[request_index]);
                } else {
                    if ((mshr_index == -1) && (MSHR.occupancy == MSHR_SIZE)) { // not enough MSHR resource
                        // cannot handle miss request until one of MSHRs is available
                        miss_handled = 0;
                        STALL[op_type]++;
                    } else if (mshr_index != -1) { // already in-flight miss

                        // mark merged consumer
                        if (op_type == RFO) {

                            if (queue->entry[request_index].tlb_access) {
                                uint32_t sq_index = queue->entry[request_index].sq_index;
                                MSHR.entry[mshr_index].store_merged = 1;
                                MSHR.entry[mshr_index].sq_index_depend_on_me[sq_index] = 1;
                                for (uint32_t i=0; i<SQ_SIZE; i++) {
                                    if (queue->entry[request_index].sq_index_depend_on_me[i])
                                        MSHR.entry[mshr_index].sq_index_depend_on_me[i] = 1;
                                    else
                                        continue;
                                }
                            }

                            if (queue->entry[request_index].load_merged) {
                                //uint32_t lq_index = queue->entry[request_index].lq_index; 
                                MSHR.entry[mshr_index].load_merged = 1;
                                //MSHR.entry[mshr_index].lq_index_depend_on_me[lq_index] = 1;
                                for (uint32_t i=0; i<LQ_SIZE; i++) {
                                    if (queue->entry[request_index].lq_index_depend_on_me[i])
                                        MSHR.entry[mshr_index].lq_index_depend_on_me[i] = 1;
                                }
                            }
                        } else {
                            if (queue->entry[request_index].instruction) {
                                uint32_t rob_index = queue->entry[request_index].rob_index;
                                MSHR.entry[mshr_index].instr_merged = 1;
                                MSHR.entry[mshr_index].rob_index_depend_on_me[rob_index] = 1;

                                DP (if (warmup_complete[MSHR.entry[mshr_index].cpu]) {
                                cout << "[INSTR_MERGED] " << __func__ << " cpu: " << MSHR.entry[mshr_index].cpu << " instr_id: " << MSHR.entry[mshr_index].instr_id;
                                cout << " merged rob_index: " << rob_index << " instr_id: " << op_instr_id << endl; });

                                if (queue->entry[request_index].instr_merged) {
                                    for (uint32_t i=0; i<ROB_SIZE; i++) {
                                        if (queue->entry[request_index].rob_index_depend_on_me[i]) {
                                            MSHR.entry[mshr_index].rob_index_depend_on_me[i] = 1;

                                            DP (if (warmup_complete[MSHR.entry[mshr_index].cpu]) {
                                            cout << "[INSTR_MERGED] " << __func__ << " cpu: " << MSHR.entry[mshr_index].cpu << " instr_id: " << MSHR.entry[mshr_index].instr_id;
                                            cout << " merged rob_index: " << i << " instr_id: N/A" << endl; });
                                        }
                                    }
                                }
                            } else {
                                uint32_t lq_index = queue->entry[request_index].lq_index;
                                MSHR.entry[mshr_index].load_merged = 1;
                                MSHR.entry[mshr_index].lq_index_depend_on_me[lq_index] = 1;

                                DP (if (warmup_complete[op_cpu]) {
                                cout << "[DATA_MERGED] " << __func__ << " cpu: " << op_cpu << " instr_id: " << op_instr_id;
                                cout << " merged rob_index: " << queue->entry[request_index].rob_index << " instr_id: " << op_instr_id << " lq_index: " << queue->entry[request_index].lq_index << endl; });

                                for (uint32_t i=0; i<LQ_SIZE; i++) {
                                    if (queue->entry[request_index].lq_index_depend_on_me[i]) 
                                        MSHR.entry[mshr_index].lq_index_depend_on_me[i] = 1;
                                }

                                if (queue->entry[request_index].store_merged) {
                                    //uint32_t sq_index = queue->entry[request_index].sq_index;
                                    MSHR.entry[mshr_index].store_merged = 1;
                                    //MSHR.entry[mshr_index].sq_index_depend_on_me[sq_index] = 1;
                                    for (uint32_t i=0; i<SQ_SIZE; i++) {
                                        if (queue->entry[request_index].sq_index_depend_on_me[i])
                                            MSHR.entry[mshr_index].sq_index_depend_on_me[i] = 1;
                                        else
                                            continue;
                                    }
                                }
                            }
                        }

                        // update fill_level
                        if (queue->entry[request_index].fill_level < MSHR.entry[mshr_index].fill_level)
                            MSHR.entry[mshr_index].fill_level = queue->entry[request_index].fill_level;

                        // update request
                        if (MSHR.entry[mshr_index].type == PREFETCH) {
                            uint8_t  prior_returned = MSHR.entry[mshr_index].returned;
                            uint64_t prior_event_cycle = MSHR.entry[mshr_index].event_cycle;
                            memcpy(&MSHR.entry[mshr_index], &queue->entry[request_index], sizeof(PACKET));
                            
                            // in case request is already returned, we should keep event_cycle and retunred variables
                            MSHR.entry[mshr_index].returned = prior_returned;
                            MSHR.entry[mshr_index].event_cycle = prior_event_cycle;
                        }

                        MSHR_MERGED[queue->entry[request_index].type]++;

                        DP ( if (warmup_complete[op_cpu]) {
                        cout << "[" << NAME << "] " << __func__ << " mshr merged";
                        cout << " instr_id: " << queue->entry[request_index].instr_id << " prior_id: " << MSHR.entry[mshr_index].instr_id; 
                        cout << " address: " << hex << queue->entry[request_index].address;
                        cout << " full_addr: " << queue->entry[request_index].full_addr << dec;
                        cout << " cycle: " << queue->entry[request_index].event_cycle << endl; });
                    } else { // WE SHOULD NOT REACH HERE
                        cerr << "[" << NAME << "] MSHR errors" << endl;
                        assert(0);
                    }
                }

                if (miss_handled) {
                    DP ( if (warmup_complete[op_cpu]) {
                    cout << "[" << queue->NAME << "] " <<  __func__ << " read miss" << hex;
                    cout << " address: " << queue->entry[request_index].address << " full_addr: " << queue->entry[request_index].full_addr << dec;
                    cout << " occupancy: " << queue->occupancy << " channel: " << op_channel << " rank: " << op_rank << " bank: " << op_bank;
                    cout << " row: " << op_row << " column: " << op_column;
                    cout << " current_cycle: " << current_core_cycle[op_cpu] << " event_cycle: " << queue->entry[request_index].event_cycle << endl; });

                    if (bank_request[op_channel][op_rank][op_bank].row_buffer_hit)
                        queue->ROW_BUFFER_HIT++;
                    else
                        queue->ROW_BUFFER_MISS++;

                    MISS[op_type]++;
                    ACCESS[op_type]++;

                    // this bank is ready for another DRC request
                    bank_request[op_channel][op_rank][op_bank].request_index = -1;
                    bank_request[op_channel][op_rank][op_bank].row_buffer_hit = 0;
                    bank_request[op_channel][op_rank][op_bank].working = 0;
                    bank_request[op_channel][op_rank][op_bank].is_write = 0;
                    bank_request[op_channel][op_rank][op_bank].is_read = 0;
                    bank_request[op_channel][op_rank][op_bank].drc_hit = 0;

                    scheduled_reads--;

                    // remove this entry from RQ
                    queue->remove_queue(&queue->entry[request_index]);

                    update_process_cycle(queue);
                }
            }
        }
    }

    // TODO: Maybe we can handle the second oldest request something like that? 
    // If the bank is busy.. Was there some kind of job before? (Check DRC scheduling paper)
    // How many times do we stuck by "waited enough but bank is still busy" scenario?

    // TODO: sometime channel is idle since tCAS takes 18 cycles and data transfer across channel takes only 16 cycles
    // if we use a wider channel the channel idle problem becomes more critical
}

int DRC_MEMORY_CONTROLLER::add_rq(PACKET *packet)
{
    // check for the latest wirtebacks in the write queue
    //int wq_index = WQ.check_queue(packet);
    int wq_index = check_drc_queue(&WQ, packet);
    if (wq_index != -1) {
        
        // check fill level
        if (packet->fill_level < fill_level) {

            packet->data = WQ.entry[wq_index].data;
            if (packet->instruction) 
                upper_level_icache[packet->cpu]->return_data(packet);
            else // data
                upper_level_dcache[packet->cpu]->return_data(packet);
        }

        DP ( if (packet->cpu) {
        cout << "[" << NAME << "_RQ] " << __func__ << " instr_id: " << packet->instr_id << " found recent writebacks";
        cout << hex << " read: " << packet->address << " writeback: " << WQ.entry[wq_index].address << dec << endl; });

        ACCESS[1]++;
        HIT[1]++;

        WQ.FORWARD++;
        RQ.ACCESS++;
        //assert(0);

        return -1;
    }

    // check for duplicates in the read queue
    int index = check_drc_queue(&RQ, packet);
    if (index != -1)
        return index; // merged index

    // search for the empty index
    for (index=0; index<DRC_RQ_SIZE; index++) {
        if (RQ.entry[index].address == 0) {
            
            memcpy(&RQ.entry[index], packet, sizeof(PACKET));
            RQ.occupancy++;

#ifdef DEBUG_PRINT
            uint32_t channel = drc_get_channel(packet->address),
                     rank = drc_get_rank(packet->address),
                     bank = drc_get_bank(packet->address),
                     row = drc_get_row(packet->address),
                     column = drc_get_column(packet->address); 
#endif

            DP ( if(warmup_complete[packet->cpu]) {
            cout << "[" << NAME << "_RQ] " <<  __func__ << " instr_id: " << packet->instr_id << " address: " << hex << packet->address;
            cout << " full_addr: " << packet->full_addr << dec << " ch: " << channel;
            cout << " rank: " << rank << " bank: " << bank << " row: " << row << " col: " << column;
            cout << " occupancy: " << RQ.occupancy << " current: " << current_core_cycle[packet->cpu] << " event: " << packet->event_cycle << endl; });

            break;
        }
    }

    update_schedule_cycle(&RQ);

    return -1;
}

int DRC_MEMORY_CONTROLLER::add_wq(PACKET *packet)
{
    // check for duplicates in the write queue
    int index = check_drc_queue(&WQ, packet);
    if (index != -1)
        return index; // merged index

    // search for the empty index
    for (index=0; index<DRC_WQ_SIZE; index++) {
        if (WQ.entry[index].address == 0) {
            
            memcpy(&WQ.entry[index], packet, sizeof(PACKET));
            WQ.occupancy++;

#ifdef DEBUG_PRINT
            uint32_t channel = drc_get_channel(packet->address),
                     rank = drc_get_rank(packet->address),
                     bank = drc_get_bank(packet->address),
                     row = drc_get_row(packet->address),
                     column = drc_get_column(packet->address); 
#endif

            DP ( if(warmup_complete[packet->cpu]) {
            cout << "[" << NAME << "_WQ] " <<  __func__ << " instr_id: " << packet->instr_id << " address: " << hex << packet->address;
            cout << " full_addr: " << packet->full_addr << dec << " ch: " << channel;
            cout << " rank: " << rank << " bank: " << bank << " row: " << row << " col: " << column;
            cout << " occupancy: " << WQ.occupancy << " current: " << current_core_cycle[packet->cpu] << " event: " << packet->event_cycle << endl; });

            break;
        }
    }

    update_schedule_cycle(&WQ);

    return -1;
}

int DRC_MEMORY_CONTROLLER::add_pq(PACKET *packet)
{
    return -1;
}

void DRC_MEMORY_CONTROLLER::return_data(PACKET *packet)
{
    // check MSHR information
    int mshr_index = check_mshr(packet);

    // sanity check
    if (mshr_index == -1) {
        cerr << "[" << NAME << "_MSHR] " << __func__ << " instr_id: " << packet->instr_id << " cannot find a matching entry!";
        cerr << " full_addr: " << hex << packet->full_addr;
        cerr << " address: " << packet->address << dec;
        cerr << " event: " << packet->event_cycle << " current: " << current_core_cycle[packet->cpu] << endl;
        assert(0);
    }

    // MSHR holds the most updated information about this request
    // no need to do memcpy
    MSHR.num_returned++;
    MSHR.entry[mshr_index].returned = COMPLETED;
    MSHR.entry[mshr_index].data = packet->data;

    // ADD LATENCY
    MSHR.entry[mshr_index].event_cycle = current_core_cycle[packet->cpu];
    packet->event_cycle = current_core_cycle[packet->cpu];
    /*
    if (MSHR.entry[mshr_index].event_cycle < current_core_cycle[packet->cpu])
        MSHR.entry[mshr_index].event_cycle = current_core_cycle[packet->cpu] + LATENCY;
    else
        MSHR.entry[mshr_index].event_cycle += LATENCY;
    */

    // we need to immediately return these data to upper level memory
    // if we don't, it takes a very long time to 
    // 1) change the DRC mode from read to write
    // 2) fill in missing cache blocks in DRC
    // 3) finally return data to upper level cache
    // which eventually stalls the main core pipeline

    // FIXME: need to consider DRC dbus congestion
    // first add it to the wq and make one more function that can return data to on-chip caches

    // check fill level
    if (packet->fill_level < fill_level) {

        if (packet->instruction) 
            upper_level_icache[packet->cpu]->return_data(packet);
        else // data
            upper_level_dcache[packet->cpu]->return_data(packet);

        // since it is now returned to the upper level memory we must reset the fill_level
        // if we don't, when the DRC mode changes to the write mode and this block gets filled in the DRC
        // it will try to retun data again to the upper level memory
        packet->fill_level = FILL_DRC;
    }

    // before we add returned packet to DRC WQ, we must reset scheduled bit in this packet
    // since this packet once scheduled while it was sitting in DRC RQ to be processed
    packet->scheduled = 0;

    // add it to the WQ
    add_wq(packet); 
    update_schedule_cycle(&WQ);

    // even though it is not filled in the DRC yet, we can safely release MSHR resource
    MSHR.remove_queue(&MSHR.entry[mshr_index]);

    DP (if (warmup_complete[packet->cpu]) {
    cout << "[" << NAME << "_MSHR] " <<  __func__ << " instr_id: " << MSHR.entry[mshr_index].instr_id;
    cout << " address: " << hex << MSHR.entry[mshr_index].address << " full_addr: " << MSHR.entry[mshr_index].full_addr;
    cout << " data: " << MSHR.entry[mshr_index].data << dec << " num_returned: " << MSHR.num_returned;
    cout << " index: " << mshr_index << " occupancy: " << MSHR.occupancy;
    cout << " event: " << MSHR.entry[mshr_index].event_cycle << " current: " << current_core_cycle[packet->cpu] << " next: " << MSHR.next_fill_cycle << endl; });
}

// FIXME: we should have two schedule cycles and process cycles for DRC
void DRC_MEMORY_CONTROLLER::update_schedule_cycle(PACKET_QUEUE *queue)
{
    // update next_schedule_cycle
    uint64_t min_cycle = UINT64_MAX;
    uint32_t min_index = queue->SIZE;
    for (uint32_t i=0; i<queue->SIZE; i++) {
        DP (if (warmup_complete[queue->entry[min_index].cpu]) {
        cout << "[" << queue->NAME << "] " <<  __func__ << " instr_id: " << queue->entry[i].instr_id;
        cout << " index: " << i << " address: " << hex << queue->entry[i].address << dec << " scheduled: " << +queue->entry[i].scheduled;
        cout << " event: " << queue->entry[i].event_cycle << " min_cycle: " << min_cycle << endl;
        });

        if (queue->entry[i].address && (queue->entry[i].scheduled == 0) && (queue->entry[i].event_cycle < min_cycle)) {
            min_cycle = queue->entry[i].event_cycle;
            min_index = i;
        }
    }
    
    queue->next_schedule_cycle = min_cycle;
    queue->next_schedule_index = min_index;
    if (min_index < queue->SIZE) {

        DP (if (warmup_complete[queue->entry[min_index].cpu]) {
        cout << "[" << queue->NAME << "] " <<  __func__ << " instr_id: " << queue->entry[min_index].instr_id << " min_index: " << min_index;
        cout << " next_schedule_index: " << queue->next_schedule_index;
        cout << " address: " << hex << queue->entry[min_index].address << " full_addr: " << queue->entry[min_index].full_addr;
        cout << " data: " << queue->entry[min_index].data << dec;
        cout << " event: " << queue->entry[min_index].event_cycle << " current: " << current_core_cycle[queue->entry[min_index].cpu] << " next: " << queue->next_schedule_cycle << endl; });
    }
}

void DRC_MEMORY_CONTROLLER::update_process_cycle(PACKET_QUEUE *queue)
{
    // update next_process_cycle
    uint64_t min_cycle = UINT64_MAX;
    uint32_t min_index = queue->SIZE;
    for (uint32_t i=0; i<queue->SIZE; i++) {
        if (queue->entry[i].scheduled && (queue->entry[i].event_cycle < min_cycle)) {
            min_cycle = queue->entry[i].event_cycle;
            min_index = i;
        }
    }
    
    queue->next_process_cycle = min_cycle;
    queue->next_process_index = min_index;
    if (min_index < queue->SIZE) {

        DP (if (warmup_complete[queue->entry[min_index].cpu]) {
        cout << "[" << queue->NAME << "] " <<  __func__ << " instr_id: " << queue->entry[min_index].instr_id;
        cout << " address: " << hex << queue->entry[min_index].address << " full_addr: " << queue->entry[min_index].full_addr;
        cout << " data: " << queue->entry[min_index].data << dec << " num_returned: " << queue->num_returned;
        cout << " event: " << queue->entry[min_index].event_cycle << " current: " << current_core_cycle[queue->entry[min_index].cpu] << " next: " << queue->next_process_cycle << endl; });
    }
}

int DRC_MEMORY_CONTROLLER::check_drc_queue(PACKET_QUEUE *queue, PACKET *packet)
{
    // search write queue
    for (uint32_t index=0; index<queue->SIZE; index++) {
        if (queue->entry[index].address == packet->address) {
            
            DP ( if (warmup_complete[packet->cpu]) {
            cout << "[" << queue->NAME << "] " << __func__ << " same entry instr_id: " << packet->instr_id << " prior_id: " << queue->entry[index].instr_id;
            cout << " address: " << hex << packet->address << " full_addr: " << packet->full_addr << dec << endl; });

            return index;
        }
    }

    DP ( if (warmup_complete[packet->cpu]) {
    cout << "[" << queue->NAME << "] " << __func__ << " new address: " << hex << packet->address;
    cout << " full_addr: " << packet->full_addr << dec << endl; });

    DP ( if (warmup_complete[packet->cpu] && (queue->occupancy == queue->SIZE)) {
    cout << "[" << queue->NAME << "] " << __func__ << " mshr is full";
    cout << " instr_id: " << packet->instr_id << " mshr occupancy: " << queue->occupancy;
    cout << " address: " << hex << packet->address;
    cout << " full_addr: " << packet->full_addr << dec;
    cout << " cycle: " << current_core_cycle[packet->cpu] << endl; });

    return -1;
}

uint32_t DRC_MEMORY_CONTROLLER::drc_get_column(uint64_t address)
{
    if (LOG2_DRC_COLUMNS == 0)
        return 0;

    int shift = 0;

    return (uint32_t) (address >> shift) & (DRC_COLUMNS - 1);
}

uint32_t DRC_MEMORY_CONTROLLER::drc_get_channel(uint64_t address)
{
    if (LOG2_DRC_CHANNELS == 0)
        return 0;

    int shift = LOG2_DRC_COLUMNS;

    return (uint32_t) (address >> shift) & (DRC_CHANNELS - 1);
}

uint32_t DRC_MEMORY_CONTROLLER::drc_get_bank(uint64_t address)
{
    if (LOG2_DRC_BANKS == 0)
        return 0;

    int shift = LOG2_DRC_COLUMNS + LOG2_DRC_CHANNELS;

    return (uint32_t) (address >> shift) & (DRC_BANKS - 1);
}

uint32_t DRC_MEMORY_CONTROLLER::drc_get_rank(uint64_t address)
{
    if (LOG2_DRC_RANKS == 0)
        return 0;

    int shift = LOG2_DRC_COLUMNS + LOG2_DRC_CHANNELS + LOG2_DRC_BANKS;

    return (uint32_t) (address >> shift) & (DRC_RANKS - 1);
}

uint32_t DRC_MEMORY_CONTROLLER::drc_get_row(uint64_t address)
{
    if (LOG2_DRC_ROWS == 0)
        return 0;

    int shift = LOG2_DRC_COLUMNS + LOG2_DRC_CHANNELS + LOG2_DRC_BANKS + LOG2_DRC_RANKS;

    return (uint32_t) (address >> shift) & (DRC_ROWS - 1);
}

uint32_t DRC_MEMORY_CONTROLLER::get_occupancy(uint8_t queue_type)
{
    if (queue_type == 0)
        return MSHR.occupancy;
    else if (queue_type == 1)
        return RQ.occupancy;
    else if (queue_type == 2)
        return WQ.occupancy;

    return 0;
}

uint32_t DRC_MEMORY_CONTROLLER::get_size(uint8_t queue_type)
{
    if (queue_type == 0)
        return MSHR.SIZE;
    else if (queue_type == 1)
        return RQ.SIZE;
    else if (queue_type == 2)
        return WQ.SIZE;

    return 0;
}

void DRC_MEMORY_CONTROLLER::increment_WQ_FULL()
{
    WQ.FULL++;
}

uint32_t DRC_MEMORY_CONTROLLER::drc_check_hit(PACKET_QUEUE *queue, uint64_t address, uint32_t cpu, uint32_t channel, uint32_t rank, uint32_t bank, uint32_t row)
{
    // determine DRC set index for this row
    // TODO: check if this modulo works okay or not
    uint32_t set = address % DRC_SETS_PER_ROW; 

    if (drc_array[channel][rank][bank][row].block == NULL) { // never accessed this row, dynamically allocate DRC blocks
        drc_array[channel][rank][bank][row].block = new BLOCK* [DRC_SETS_PER_ROW];
        for (uint32_t i=0; i<DRC_SETS_PER_ROW; i++) {
            drc_array[channel][rank][bank][row].block[i] = new BLOCK[DRC_WAY];
            drc_blocks += DRC_WAY;
        }

        return DRC_WAY; // never accessed this row, this is a miss
    }

    for (uint32_t i=0; i<DRC_WAY; i++) {
        if (drc_array[channel][rank][bank][row].block[set][i].tag == address) {
            DP ( if (warmup_complete[cpu]) {
            cout << "[" << queue->NAME << "] " << __func__ << " found matching address: " << hex << address << dec << " set: " << set << " way: " << i << endl; });

            return i;
        }
    }

    return DRC_WAY; // cannot find any matching tag, this is a miss
}

void DRC_MEMORY_CONTROLLER::fill_cache(BLOCK *block, uint32_t set, uint32_t way, PACKET *packet)
{
    if (block->valid == 0) {
        block->valid = 1;
        num_valid++;
    }
    // block->prefetch = packet->prefetch; // FIXME: looks like this needs to be fixed
    block->used = 0;
    block->dirty = 0;

    block->tag = packet->address;
    block->address = packet->address;
    block->full_addr = packet->full_addr;
    block->data = packet->data;
    block->cpu = packet->cpu;
    block->instr_id = packet->instr_id;

    DP ( if (warmup_complete[packet->cpu]) {
    cout << "[" << NAME << "] " << __func__ << " set: " << set << " way: " << way;
    cout << " lru: " << block->lru << " instr_id: " << packet->instr_id << " type: " << +packet->type << " tag: " << hex << block->tag << " full_addr: " << block->full_addr;
    cout << " data: " << block->data << dec << endl; });
}

// FIXME: IMPLEMENT YOUR OWN DRC REPLACEMENT ALGORITHM HERE
void DRC_MEMORY_CONTROLLER::update_replacement_state(uint32_t cpu, uint32_t set, uint32_t way, uint64_t full_addr, uint64_t ip, uint64_t victim_addr, uint32_t type, uint8_t hit)
{

}

// FIXME: IMPLEMENT YOUR OWN DRC REPLACEMENT ALGORITHM HERE
uint32_t DRC_MEMORY_CONTROLLER::find_victim(uint32_t cpu, uint64_t instr_id, uint32_t set, const BLOCK *current_set, uint64_t ip, uint64_t full_addr, uint32_t type)
{
    return 0;
}

int DRC_MEMORY_CONTROLLER::check_mshr(PACKET *packet)
{
    // search mshr
    for (uint32_t index=0; index<MSHR_SIZE; index++) {
        if (MSHR.entry[index].address == packet->address) {
            
            DP ( if (warmup_complete[packet->cpu]) {
            cout << "[" << NAME << "_MSHR] " << __func__ << " same entry instr_id: " << packet->instr_id << " prior_id: " << MSHR.entry[index].instr_id;
            cout << " address: " << hex << packet->address;
            cout << " full_addr: " << packet->full_addr << dec << endl; });

            return index;
        }
    }

    DP ( if (warmup_complete[packet->cpu]) {
    cout << "[" << NAME << "_MSHR] " << __func__ << " new address: " << hex << packet->address;
    cout << " full_addr: " << packet->full_addr << dec << endl; });

    DP ( if (warmup_complete[packet->cpu] && (MSHR.occupancy == MSHR_SIZE)) { 
    cout << "[" << NAME << "_MSHR] " << __func__ << " mshr is full";
    cout << " instr_id: " << packet->instr_id << " mshr occupancy: " << MSHR.occupancy;
    cout << " address: " << hex << packet->address;
    cout << " full_addr: " << packet->full_addr << dec;
    cout << " cycle: " << current_core_cycle[packet->cpu] << endl; });

    return -1;
}

void DRC_MEMORY_CONTROLLER::add_mshr(PACKET *packet)
{
    uint32_t index = 0;

    // search mshr
    for (index=0; index<MSHR_SIZE; index++) {
        if (MSHR.entry[index].address == 0) {
            
            memcpy(&MSHR.entry[index], packet, sizeof(PACKET));
            MSHR.entry[index].returned = INFLIGHT;
            MSHR.occupancy++;

            DP ( if (warmup_complete[packet->cpu]) {
            cout << "[" << NAME << "_MSHR] " << __func__ << " instr_id: " << packet->instr_id;
            cout << " address: " << hex << packet->address << " full_addr: " << packet->full_addr << dec;
            cout << " index: " << index << " occupancy: " << MSHR.occupancy << endl; });

            break;
        }
    }
}
