// Dijkstra testbench
`timescale 1ns/1ps 

module dijkstra_tb;

    // IO
	logic clk;    // Clock
	logic rst_n;  // Asynchronous reset active low
	// Single port RAM interface
	logic [VIRTEX_DWIDTH-1:0]    weights_ram_data [MAX_VIRTEX_NUM-1:0];
	logic [VIRTEX_AWIDTH-1:0]    weights_ram_addr;
	logic                        weights_ram_cs;
	// Control logic
	logic                        dj_start;
	logic [VIRTEX_NUM_WIDTH-1:0] dj_virt_initial; // The initial virtex
	logic [VIRTEX_NUM_WIDTH-1:0] dj_virt_num;     // Virtex number
	logic                        dj_rdy; 
    // Result 
    logic                        result_rdy;
    logic                        error;
    logic [VIRTEX_DWIDTH-1:0]    dist_vect    [MAX_VIRTEX_NUM-1:0];
    logic [VIRTEX_NUM_WIDTH-1:0] route_vect   [MAX_VIRTEX_NUM-1:0];
    
    
    logic [MAX_VIRTEX_NUM -1:0][VIRTEX_DWIDTH-1:0] virt_mem [MAX_VIRTEX_NUM -1:0];
    
    int vir_num;
   
dijkstra_dummy i_dijkstra (
	.clk                  (clk),    
	.rst_n                (rst_n),  
	// Single port RAM interface
	.weights_ram_data_i   (weights_ram_data),
	.weights_ram_addr_o   (weights_ram_addr),
	.weights_ram_cs_o     (weights_ram_cs),
	// Control logic
	.dj_start_i           (dj_start),
	.dj_virt_initial_i    (dj_virt_initial), 
	.dj_virt_num_i        (dj_virt_num),     
	.dj_rdy_o             (dj_rdy), 
    // Result 
    .result_rdy_o         (result_rdy),
    .error_o              (error),
    .dist_vect_o          (dist_vect) ,
    .route_vect_o         (route_vect)
);

//dijkstra i_dijkstra(
//	.clk(clk),    
//	.rst_n(rst_n), 
//	// Single port RAM interface
//    .weights_ram_data_i(weights_ram_data),
//	.weights_ram_addr_o(weights_ram_addr),
//	.weights_ram_cs_o(weights_ram_cs),
//	// Control logic
//	.accel_start_i(accel_start),
//	.accel_virt_initial_i(accel_virt_initial), 
//	.accel_virt_num_i(accel_virt_num),     
//	.accel_rdy_o(accel_rdy), 
//    // Result 
//    .result_rdy_o(result_rdy),
//    .error_o(error),
//    .dist_vect_o(dist_vect),
//    .route_vect_o(route_vect)
//);

initial begin
    clk = 0;
    rst_n = 1;
    dj_virt_num  = 5;
    dj_virt_initial = 0;
//    virt_mem = {{{'1},{32'hA},{32'h1E},{32'h32},{32'hA},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{32'hA},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{32'h28},{32'h14},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{32'h1E},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}},
//                {{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1},{'1}}};

    for (int i = 0; i < MAX_VIRTEX_NUM; i++)
          for (int j = 0; j < MAX_VIRTEX_NUM; j++)
            virt_mem[i][j] = '1;
    virt_mem [0][1] = 32'hA;
    virt_mem [0][2] = 32'h1E;
    virt_mem [0][3] = 32'h32;
    virt_mem [0][4] = 32'hA;
    virt_mem [2][4] = 32'hA;
    virt_mem [3][1] = 32'h28;
    virt_mem [3][2] = 32'h14;
    virt_mem [4][2] = 32'hA;
    virt_mem [4][3] = 32'h1E;
    
    
      #1 rst_n = 0;
      #7 rst_n = 1;
      
      #2 dj_start = 1;
      #7 dj_start = 0;
      
        
end

for(genvar idx = 0; idx < MAX_VIRTEX_NUM; idx = idx + 1)
    always_ff @(posedge clk)
        if(weights_ram_cs)
            weights_ram_data[idx] <= virt_mem[weights_ram_addr][idx];
        
always 
    #5 clk = ~clk;

endmodule 