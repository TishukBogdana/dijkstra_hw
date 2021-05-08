parameter PIPE_WIDTH        = 8;
parameter PIPE_SHIFT        = $clog2(PIPE_WIDTH);
parameter MAX_VIRTEX_NUM    = 64;
parameter VIRTEX_DWIDTH     = 32;
parameter VIRTEX_NUM_WIDTH  = $clog2(MAX_VIRTEX_NUM);
parameter ITER_NUM_WIDTH    = $clog2(MAX_VIRTEX_NUM / PIPE_WIDTH);
parameter VIRTEX_AWIDTH     = $clog2(MAX_VIRTEX_NUM * MAX_VIRTEX_NUM / PIPE_WIDTH);