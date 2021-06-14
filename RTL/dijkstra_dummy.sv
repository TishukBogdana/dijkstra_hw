// dummy dijkstra algorithm implementation
//! Add flush
// add error processing
`include "dijkstra_params.svh"

module dijkstra_dummy (
	input  logic clk,    // Clock
	input  logic rst_n,  // Asynchronous reset active low
	// Single port RAM interface
	input  logic [VIRTEX_DWIDTH-1:0]    weights_ram_data_i [MAX_VIRTEX_NUM-1:0],
	output logic [VIRTEX_AWIDTH-1:0]    weights_ram_addr_o,
	output logic                        weights_ram_cs_o,
	// Control logic
	input  logic                        dj_start_i,
	input  logic [VIRTEX_NUM_WIDTH-1:0] dj_virt_initial_i, // The initial virtex
	input  logic [VIRTEX_NUM_WIDTH-1:0] dj_virt_num_i,     // Virtex number
	output logic                        dj_rdy_o, 
    // Result 
    output logic                        result_rdy_o,
    output logic                        error_o,
    output logic [VIRTEX_DWIDTH-1:0]    dist_vect_o    [MAX_VIRTEX_NUM-1:0],
    output logic [VIRTEX_NUM_WIDTH-1:0] route_vect_o   [MAX_VIRTEX_NUM-1:0]
);

localparam STATES_NUM = 3;
localparam STATE_IDLE = 0;
localparam STATE_CALC = 1;
localparam STATE_CMP  = 2;

// Registered external values
logic                        dj_start_ff; 
logic [VIRTEX_NUM_WIDTH-1:0] dj_virt_initial_ff;
logic [VIRTEX_NUM_WIDTH  :0] dj_num_ff; 

// Control logic
logic                        dj_start;
logic [STATES_NUM-1      :0] dj_state_ff;
logic [STATES_NUM-1      :0] dj_state_en;
logic [STATES_NUM-1      :0] dj_state_next;
logic                        dj_finish;
logic                        cmp_rdy;
logic                        calc_rdy;
logic [VIRTEX_NUM_WIDTH  :0] dj_proc_cnt_ff; 

// Dataflow
logic [VIRTEX_NUM_WIDTH-1:0] virt_curr_num_next; 
logic [VIRTEX_NUM_WIDTH-1:0] virt_curr_num_ff; 
logic [VIRTEX_DWIDTH   -1:0] virt_curr_weight_ff;
logic                        weights_rdata_en;
logic                        rf_upd_en;
logic                        cmp_src_sel_ff;
logic                  [2:0] calc_proc_ff;  // counter of calculation phase steps
logic [VIRTEX_NUM_WIDTH-1:0] virt_cnt_ff;   // Virtex processign counter
logic [VIRTEX_NUM_WIDTH-1:0] cmp_cnt_ff;    // Compare log cpunter
logic                        weight_lower     [MAX_VIRTEX_NUM-1:0]; // Route vector
logic [VIRTEX_DWIDTH   -1:0] weights_rdata_ff [MAX_VIRTEX_NUM-1:0]; // Weights registered data from RAM
logic [VIRTEX_DWIDTH   -1:0] weights_sum_ff   [MAX_VIRTEX_NUM-1:0]; // Weights after summators
logic [VIRTEX_DWIDTH   -1:0] dist_rf_next     [MAX_VIRTEX_NUM-1:0];
logic [VIRTEX_DWIDTH   -1:0] dist_rf_ff       [MAX_VIRTEX_NUM-1:0]; // Distance register file
logic [VIRTEX_NUM_WIDTH-1:0] route_vect_next  [MAX_VIRTEX_NUM-1:0];
logic [VIRTEX_NUM_WIDTH-1:0] route_vect_ff    [MAX_VIRTEX_NUM-1:0]; // Route vector
logic [VIRTEX_NUM_WIDTH-1:0] visit_vect_ff    [MAX_VIRTEX_NUM-1:0]; // Visited virtexes vector

logic [VIRTEX_DWIDTH   -1:0] weight_cmp_next  [MAX_VIRTEX_NUM-1:0]; 
logic                        weight_cmpbuf_lo [MAX_VIRTEX_NUM/2-1:0]; 
logic [VIRTEX_DWIDTH   -1:0] weight_cmpbuf_ff [MAX_VIRTEX_NUM/2-1:0]; // Distance comparison buffer
logic [VIRTEX_NUM_WIDTH-1:0] idx_cmpbuf_ff    [MAX_VIRTEX_NUM/2-1:0]; // Virtex number buffer


// Init Dijkstra
always_ff @(posedge clk or negedge rst_n)
    if (~rst_n)
        dj_start_ff <= '0;
    else
        dj_start_ff <= dj_start_i;

always_ff @(posedge clk or negedge rst_n)
    if (~rst_n) begin 
        dj_virt_initial_ff <= '0;
        dj_num_ff          <= '0;
    end else if (dj_start_i) begin
        dj_virt_initial_ff <= dj_virt_initial_i;
        dj_num_ff          <= dj_virt_num_i;
    end

// Control logic
assign dj_start = dj_start_ff & dj_state_ff[STATE_IDLE];

assign dj_state_en = dj_start | dj_finish | calc_rdy | cmp_rdy;

assign calc_rdy = calc_proc_ff[2];

assign cmp_rdy = ~(|cmp_cnt_ff);

assign dj_finish =  (virt_cnt_ff == (dj_num_ff -1)) & dj_state_ff[STATE_CMP];

assign dj_state_next[STATE_IDLE] = dj_state_ff[STATE_CMP]  & dj_finish;
assign dj_state_next[STATE_CALC] = dj_start |  (dj_state_ff[STATE_CMP] & cmp_rdy & ~dj_finish) ;
assign dj_state_next[STATE_CMP]  = dj_state_ff[STATE_CALC] & calc_rdy;

always_ff @(posedge clk or negedge rst_n)
    if (~rst_n)
        dj_state_ff <= 1;
    else if (dj_state_en)
        dj_state_ff <= dj_state_next;
 
 always_ff @(posedge clk or negedge rst_n )
     if (~rst_n)
         calc_proc_ff <= '0;
     else 
         calc_proc_ff <= ( dj_state_en & dj_state_next[STATE_CALC] ) ? 2'b01 : calc_proc_ff << 1'b1;       
         
 always_ff @(posedge clk or negedge rst_n )
     if (~rst_n)
         virt_cnt_ff <= '0;
     else
         virt_cnt_ff <= dj_start ? '0 : cmp_rdy ? virt_cnt_ff + 1'b1 : virt_cnt_ff;       
                     
  always_ff @(posedge clk or negedge rst_n )
      if (~rst_n)
         cmp_src_sel_ff <= '0;
      else
         cmp_src_sel_ff <= dj_state_next[STATE_CMP] ? 1'b1 :1'b0;
        
   always_ff @(posedge clk or negedge rst_n )
       if(~rst_n)
            cmp_cnt_ff <= '1;
       else
            cmp_cnt_ff <= dj_state_next[STATE_CMP] ? 4'b1000 : (dj_state_ff[STATE_CMP] & ~cmp_rdy ? cmp_cnt_ff >> 1 : '1);
 
// Dataflow
assign weights_rdata_en = calc_proc_ff[0];
assign rf_upd_en = dj_start | calc_proc_ff[2];
        
for (genvar idx = 0; idx < MAX_VIRTEX_NUM; idx = idx + 1) begin : g_dataflow
    always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
            weights_rdata_ff[idx]  <= '0;
        else if (weights_rdata_en)
            weights_rdata_ff[idx]  <= weights_ram_data_i[idx];
            
    always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
            weights_sum_ff[idx]  <= '0;
        else
            weights_sum_ff[idx]  <= weights_rdata_ff[idx] + virt_curr_weight_ff;
            
    always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
            dist_rf_ff[idx]  <= '0;
        else if (rf_upd_en)
            dist_rf_ff[idx]  <= dist_rf_next[idx];
            
            
    always_ff @(posedge clk or negedge rst_n)
        if (~rst_n)
            route_vect_ff[idx]  <= '0;
        else if (rf_upd_en)
            route_vect_ff[idx]  <= route_vect_next[idx];
            
    always_ff @(posedge clk or negedge rst_n)
                if (~rst_n)
                    visit_vect_ff[idx]  <= '0;
                else if (rf_upd_en)
                    visit_vect_ff[idx]  <= dj_start ? '0 : (virt_curr_num_ff == idx) ? '1: visit_vect_ff[idx];
            
    assign weight_lower[idx] = ( weights_sum_ff[idx] < dist_rf_ff[idx] ) & ~(weights_rdata_ff[idx]);
    assign dist_rf_next[idx] = dj_start ? ( (idx == virt_curr_num_ff) ? '0 : '1)
                                        : ( weight_lower[idx] ? weights_sum_ff[idx] : dist_rf_ff[idx] );
   
    assign route_vect_next[idx] = dj_start ? virt_curr_num_ff
                                        : ( weight_lower[idx] ? idx : dist_rf_ff[idx] );   
    //! check timings                                    
    assign weight_cmp_next[idx] = cmp_src_sel_ff ? (dist_rf_ff[idx] | {VIRTEX_DWIDTH{visit_vect_ff[idx]}})
                                                 : (idx < MAX_VIRTEX_NUM/2) ?   weight_cmpbuf_ff[idx] : '1;                            
                                        
end

for (genvar idx = 0; idx < MAX_VIRTEX_NUM; idx = idx + 1) begin : g_min_find

    assign weight_cmpbuf_lo[idx] = (weight_cmp_next[2*idx] < weight_cmp_next[2*idx + 1]);

    always_ff @(posedge clk) begin
        weight_cmpbuf_ff[idx] <= weight_cmpbuf_lo[idx] ? weight_cmp_next[2*idx]     : weight_cmp_next[2*idx + 1];
        idx_cmpbuf_ff[idx]    <= weight_cmpbuf_lo[idx] ? (cmp_src_sel_ff ? 2*idx    : idx_cmpbuf_ff[2*idx])
                                                       : (cmp_src_sel_ff ? 2*idx +1 : idx_cmpbuf_ff[2*idx +1]);
    end
end

// output assignments
assign weights_ram_addr_o = virt_curr_num_ff;
assign weights_ram_cs_o   = cmp_rdy | dj_start ;
assign dj_rdy_o           = dj_state_ff[STATE_IDLE];
assign result_rdy_o       = dj_finish;

// ------------

assign virt_curr_num_next = dj_start ? dj_virt_initial_ff : (cmp_rdy ? idx_cmpbuf_ff[0] : virt_curr_num_ff);

always_ff @(posedge clk or negedge rst_n)
    if (~rst_n)
        virt_curr_num_ff <= '0;
    else 
        virt_curr_num_ff <= virt_curr_num_next;
        
always_ff @(posedge clk or negedge rst_n)
    if (~rst_n)
        virt_curr_weight_ff <= '0;
    else //  check timings!
        virt_curr_weight_ff <= dist_rf_ff[virt_curr_num_next];
endmodule