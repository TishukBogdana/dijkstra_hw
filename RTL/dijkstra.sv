// Dijkstra alogorithm hardware implementation
`include "dijkstra_params.svh" 

module dijkstra (
	input  logic clk,    // Clock
	input  logic rst_n,  // Asynchronous reset active low
	// Single port RAM interface
	input  logic [VIRTEX_DWIDTH-1:0]    weights_ram_data_i [PIPE_WIDTH-1:0],
	output logic [VIRTEX_AWIDTH-1:0]    weights_ram_addr_o,
	output logic                        weights_ram_cs_o,
	// Control logic
	input  logic                        accel_start_i,
	input  logic [VIRTEX_NUM_WIDTH-1:0] accel_virt_initial_i, // The initial virtex
	input  logic [VIRTEX_NUM_WIDTH-1:0] accel_virt_num_i,     // Virtex number
	output logic                        accel_rdy_o, 
    // Result 
    output logic                        result_rdy_o,
    output logic                        error_o,
    output logic [VIRTEX_DWIDTH-1:0]    dist_vect_o    [PIPE_WIDTH-1:0],
    output logic [VIRTEX_NUM_WIDTH-1:0] route_vect_o   [PIPE_WIDTH-1:0]
);

localparam ACC_STATE_IDLE = 0;
localparam ACC_STATE_CALC = 1;
localparam ACC_STATE_RES  = 2;
localparam ACC_NUM_STATES = 3;

localparam PROC_STATE_RDY  = 0;
localparam PROC_STATE_CALC = 1;
localparam PROC_STATE_CMP  = 2;
localparam PROC_NUM_STATES = 3;

logic                         accel_state_en;
logic [ACC_NUM_STATES-1:0]    accel_state_next;
logic [ACC_NUM_STATES-1:0]    accel_state_ff;
logic                         proc_state_en;
logic [PROC_NUM_STATES-1:0]   proc_states_next;
logic [PROC_NUM_STATES-1:0]   proc_state_ff;

logic                         proc_data_en;
logic [VIRTEX_DWIDTH-1:0]     rams_data_ff [PIPE_WIDTH-1:0];
logic [VIRTEX_DWIDTH-1:0]     sum_data_ff  [PIPE_WIDTH-1:0];
logic [VIRTEX_NUM_WIDTH-1:0]  v_curr_num_ff;
logic [VIRTEX_DWIDTH-1:0]     v_curr_weight_ff;

logic [VIRTEX_NUM_WIDTH-1:0]  virtex_num_ff;
logic [ITER_NUM_WIDTH  -1:0]  iter_cnt_next;
logic [ITER_NUM_WIDTH  -1:0]  iter_cnt_ff;

logic [VIRTEX_NUM_WIDTH-1:0]  virtex_cnt_next;
logic [VIRTEX_NUM_WIDTH-1:0]  virtex_cnt_ff;
logic                         virtex_cnt_en;
logic                         res_rdy;
logic                         res_sent;
logic                         rd_rdy;

logic [ITER_NUM_WIDTH  -1:0]  rd_cnt_next;
logic [ITER_NUM_WIDTH  -1:0]  rd_cnt_ff;
logic [ITER_NUM_WIDTH  -1:0]  upd_cnt_next;
logic [ITER_NUM_WIDTH  -1:0]  upd_cnt_ff;

// Internal register file 
logic [PIPE_WIDTH                  -1:0] data_upd_vec;
logic [VIRTEX_DWIDTH*PIPE_WIDTH    -1:0] dist_vect_upd;
logic [VIRTEX_DWIDTH*PIPE_WIDTH    -1:0] dist_vect_ff    [MAX_VIRTEX_NUM / PIPE_WIDTH -1:0];
logic [VIRTEX_NUM_WIDTH*PIPE_WIDTH -1:0] route_vect_upd;
logic [VIRTEX_NUM_WIDTH*PIPE_WIDTH -1:0] route_vect_ff   [MAX_VIRTEX_NUM / PIPE_WIDTH -1:0];
// -------------- Global state ------------------ 
 
always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		virtex_curr_ff <= 0;
		virtex_num_ff  <= 0;
	end else begin
		virtex_curr_ff <= accel_virt_initial_i; //FIXME		
		virtex_num_ff  <= accel_virt_num_i;
	end
end

always_ff @(posedge clk or negedge rst_n) 
	if(~rst_n) 
		accel_start_ff <= 0;
	else begin
		accel_start_ff <= accel_start_i; //FIXME		


assign accel_state_en = accel_start_i | res_rdy | res_sent;

assign res_sent = ~(|iter_cnt_ff);
assign res_rdy  = virtex_cnt_en & (virtex_cnt_next == virtex_num_ff); // fixme

assign accel_state_next[ACC_STATE_IDLE] = accel_state_ff[ACC_STATE_RES]  & res_sent;
assign accel_state_next[ACC_STATE_CALC] = accel_state_ff[ACC_STATE_IDLE] & accel_start_i;
assign accel_state_next[ACC_STATE_RES]  = accel_state_ff[ACC_STATE_CALC] & res_rdy;

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		accel_state_ff <= 1;
	end else begin
		if (accel_state_en)
			accel_state_ff <= accel_state_next;
	end
end

assign iter_cnt_next = accel_start_i ? ( accel_virt_num_i >> PIPE_SHIFT )
                                     : ( accel_state_ff[ACC_STATE_RES] ? iter_cnt_ff - 1 : iter_cnt_ff );

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		iter_cnt_ff  <= 0;
	end else begin
		iter_cnt_ff  <= iter_cnt_next;
	end
end

assign virtex_cnt_en   = accel_start_i | accel_state_ff[ACC_STATE_CALC];
assign virtex_cnt_next = accel_start_i ? '0
									   : ( proc_state_ff[PROC_STATE_RDY] ? ( virtex_cnt_ff + 1 ) : virtex_cnt_ff  );

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		virtex_cnt_ff <= 0;
	end else begin
		if (virtex_cnt_en)
			virtex_cnt_ff <= virtex_cnt_next;
	end
end


// ---------------------------------------------
// Virtex processing
// ---------------------------------------------

// Virtex weights are signed, so if weight is negative - this means that there is no link betveen two virtexes

assign proc_state_en = accel_start_i | rd_rdy | cmp_rdy;

assign proc_states_next[PROC_STATE_RDY]  = proc_state_ff[PROC_STATE_CMP]  & cmp_rdy;
assign proc_states_next[PROC_STATE_CALC] = proc_state_ff[PROC_STATE_RDY]  & accel_start_i;
assign proc_states_next[PROC_STATE_CMP]  = proc_state_ff[PROC_STATE_CALC] & rd_rdy;

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		proc_state_ff <= 1;
	end else begin
		if (proc_state_en)
		proc_state_ff <= proc_states_next;
	end
end

assign rd_rdy = (rd_cnt_ff == virtex_num_ff);

assign rd_cnt_next = rd_rdy ? '0 
							: ( proc_state_ff[PROC_STATE_CALC] ? rd_cnt_ff + 1 : rd_cnt_ff); 

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		rd_cnt_ff <= '0;
	end else begin
		rd_cnt_ff <= rd_cnt_next;
	end
end

assign rf_upd_en = ( ( rd_cnt_ff > 1 ) | proc_state_ff[PROC_STATE_CMP] );
assign upd_cnt_next = ( upd_cnt_ff == ( virtex_num_ff -1 ) ) ? '0 
							                                 : ( ( ( rd_cnt_ff > 1 ) | proc_state_ff[PROC_STATE_CMP] ) ? upd_cnt_ff + 1
							                                                                                           : upd_cnt_ff ); 

always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		upd_cnt_ff <= '0;
	end else begin
		upd_cnt_ff <= upd_cnt_next;
	end
end

assign proc_data_en = proc_state_ff[PROC_STATE_CALC];

for (genvar idx = 0; idx < PIPE_WIDTH ; idx = idx + 1) begin : g_virt_proc
	always_ff @(posedge clk ) 
		if(proc_data_en) begin
			rams_data_ff[idx] <= weights_ram_data_i[idx];
			sum_data_ff [idx] <= rams_data_ff[idx] + v_curr_weight_ff;
        end

    assign data_upd_vec[idx] = ( sum_data_ff [idx] < dist_vect_ff[(idx + 1) * VIRTEX_DWIDTH -1 -: VIRTEX_DWIDTH] );
	assign dist_vect_upd[(idx + 1) * VIRTEX_DWIDTH -1 -: VIRTEX_DWIDTH] = data_upd_vec[idx] 
	                                                                    ? sum_data_ff [idx] 
	                                                                    : dist_vect_ff[(idx + 1) * VIRTEX_DWIDTH -1 -: VIRTEX_DWIDTH];

	assign route_vect_upd[(idx + 1) * VIRTEX_NUM_WIDTH -1 -: VIRTEX_NUM_WIDTH] = data_upd_vec[idx] 
																			   ? virtex_curr_ff 
	                                                                           : route_vect_ff[(idx + 1) * VIRTEX_NUM_WIDTH -1 -: VIRTEX_NUM_WIDTH];

	assign route_vect_init[idx] = virtex_curr_ff;

	assign dist_vect_init[idx] = (idx == (virtex_curr_ff - {(virtex_curr_ff >> PIPE_SHIFT ) , {PIPE_WIDTH{0}}}) ? '0 : '1;

end :  g_virt_proc

for (genvar idx = 0; idx < MAX_VIRTEX_NUM/PIPE_WIDTH ; idx = idx + 1) begin : g_virt_upd
	always_ff @(posedge clk or negedge rst_n) 
    	if (~rst_n) begin
    		dist_vect_ff[idx]  <= '1;
    		route_vect_ff[idx] <= '0;
    	end else if( rf_upd_en & (idx == upd_cnt_ff) ) begin
			dist_vect_ff[idx]  <= dist_vect_upd;
			route_vect_ff[idx] <= route_vect_upd;
		end else if( accel_start_ff ) begin
			dist_vect_ff[idx]  <= (idx == (virtex_curr_ff >> PIPE_SHIFT) ) dist_vect_init : '1;
			route_vect_ff[idx] <= route_vect_init;
		end 
end : g_virt_upd

endmodule


