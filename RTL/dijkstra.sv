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


// -------------- Global state ------------------
 
always_ff @(posedge clk or negedge rst_n) begin 
	if(~rst_n) begin
		virtex_curr_ff <= 0;
		virtex_num_ff  <= 0;
	end else begin
		virtex_curr_ff <= accel_virt_initial_i;
		virtex_num_ff  <= accel_virt_num_i;
	end
end

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

assign 

for (genvar idx = 0; idx < PIPE_WIDTH ; idx = idx + 1) begin : g_virt_proc
	always_ff @(posedge clk ) 
		if(proc_data_en) begin
			rams_data_ff[idx] <= weights_ram_data_i[idx];
			sum_data_ff [idx] <= rams_data_ff[idx] + v_curr_weight_ff;
        end

end :  g_virt_proc

endmodule


