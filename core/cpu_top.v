`include "e203_defines.v"

module cpu_top(
    input clk,
    input rst,

    input rst_n
  
);

  wire test_mode = 1'b0;

  wire ifu2biu_icb_cmd_valid;
  wire ifu2biu_icb_cmd_ready;
  wire [31:0] ifu2biu_icb_cmd_addr;

  wire ifu2biu_icb_rsp_valid;
  wire ifu2biu_icb_rsp_ready;
  wire ifu2biu_icb_rsp_err = 1'b0;
  wire [31:0] ifu2biu_icb_rsp_rdata;


  wire ifu_o_valid;
  wire ifu_o_ready;
  wire [`E203_INSTR_SIZE-1:0] ifu_o_ir;
  wire [`E203_PC_SIZE-1:0] ifu_o_pc;
  wire ifu_o_pc_vld; 
  wire ifu_o_misalgn; 
  wire ifu_o_buserr; 
  wire [`E203_RFIDX_WIDTH-1:0] ifu_o_rs1idx;
  wire [`E203_RFIDX_WIDTH-1:0] ifu_o_rs2idx;
  wire ifu_o_prdt_taken;
  wire ifu_o_muldiv_b2b;

  wire wfi_halt_ifu_req;
  wire wfi_halt_ifu_ack;
  wire pipe_flush_ack;
  wire pipe_flush_req;
  wire [`E203_PC_SIZE-1:0] pipe_flush_add_op1;  
  wire [`E203_PC_SIZE-1:0] pipe_flush_add_op2;  
  `ifdef E203_TIMING_BOOST//}
  wire [`E203_PC_SIZE-1:0] pipe_flush_pc;  
  `endif//}

  wire oitf_empty;
  wire [`E203_XLEN-1:0] rf2ifu_x1;
  wire [`E203_XLEN-1:0] rf2ifu_rs1;
  wire dec2ifu_rden;
  wire dec2ifu_rs1en;
  wire [`E203_RFIDX_WIDTH-1:0] dec2ifu_rdidx;
  wire dec2ifu_mulhsu;
  wire dec2ifu_div   ;
  wire dec2ifu_rem   ;
  wire dec2ifu_divu  ;
  wire dec2ifu_remu  ;


  wire itcm_nohold;

  wire [31:0] pc_rtvec = 32'b0;
  

  e203_ifu u_e203_ifu(
    .inspect_pc   (inspect_pc),

    .ifu_active      (ifu_active),
    .pc_rtvec        (pc_rtvec),  

    .itcm_nohold     (itcm_nohold),

  `ifdef E203_HAS_MEM_ITF //{
    .ifu2biu_icb_cmd_valid  (ifu2biu_icb_cmd_valid),
    .ifu2biu_icb_cmd_ready  (1'b1),
    .ifu2biu_icb_cmd_addr   (ifu2biu_icb_cmd_addr ),
    
    .ifu2biu_icb_rsp_valid  (ifu2biu_icb_rsp_valid),
    .ifu2biu_icb_rsp_ready  (ifu2biu_icb_rsp_ready),
    .ifu2biu_icb_rsp_err    (ifu2biu_icb_rsp_err  ),
    .ifu2biu_icb_rsp_rdata  (ifu2biu_icb_rsp_rdata),

  `endif//}


    .ifu_o_valid            (ifu_o_valid         ),
    .ifu_o_ready            (ifu_o_ready         ),
    .ifu_o_ir               (ifu_o_ir            ),
    .ifu_o_pc               (ifu_o_pc            ),
    .ifu_o_pc_vld           (ifu_o_pc_vld        ),
    .ifu_o_misalgn          (ifu_o_misalgn       ), 
    .ifu_o_buserr           (ifu_o_buserr        ), 
    .ifu_o_rs1idx           (ifu_o_rs1idx        ),
    .ifu_o_rs2idx           (ifu_o_rs2idx        ),
    .ifu_o_prdt_taken       (ifu_o_prdt_taken    ),
    .ifu_o_muldiv_b2b       (ifu_o_muldiv_b2b    ),

    .ifu_halt_req           (wfi_halt_ifu_req),
    .ifu_halt_ack           (wfi_halt_ifu_ack),
    .pipe_flush_ack         (pipe_flush_ack      ),
    .pipe_flush_req         (pipe_flush_req      ),
    .pipe_flush_add_op1     (pipe_flush_add_op1  ),  
    .pipe_flush_add_op2     (pipe_flush_add_op2  ),  
  `ifdef E203_TIMING_BOOST//}
    .pipe_flush_pc          (pipe_flush_pc),  
  `endif//}

                                 
    .oitf_empty             (oitf_empty   ),
    .rf2ifu_x1              (rf2ifu_x1    ),
    .rf2ifu_rs1             (rf2ifu_rs1   ),
    .dec2ifu_rden           (dec2ifu_rden ),
    .dec2ifu_rs1en          (dec2ifu_rs1en),
    .dec2ifu_rdidx          (dec2ifu_rdidx),
    .dec2ifu_mulhsu         (dec2ifu_mulhsu),
    .dec2ifu_div            (dec2ifu_div   ),
    .dec2ifu_rem            (dec2ifu_rem   ),
    .dec2ifu_divu           (dec2ifu_divu  ),
    .dec2ifu_remu           (dec2ifu_remu  ),

    .clk                    (clk  ),
    .rst_n                  (rst_n         ) 
  );

  

  wire                         lsu_o_valid; 
  wire                         lsu_o_ready; 
  wire [`E203_XLEN-1:0]        lsu_o_wbck_wdat;
  wire [`E203_ITAG_WIDTH -1:0] lsu_o_wbck_itag;
  wire                         lsu_o_wbck_err ; 
  wire                         lsu_o_cmt_buserr ; 
  wire                         lsu_o_cmt_ld;
  wire                         lsu_o_cmt_st;
  wire [`E203_ADDR_SIZE -1:0]  lsu_o_cmt_badaddr;

  wire                         agu_icb_cmd_valid; 
  wire                         agu_icb_cmd_ready; 
  wire [`E203_ADDR_SIZE-1:0]   agu_icb_cmd_addr; 
  wire                         agu_icb_cmd_read;   
  wire [`E203_XLEN-1:0]        agu_icb_cmd_wdata; 
  wire [`E203_XLEN/8-1:0]      agu_icb_cmd_wmask; 
  wire                         agu_icb_cmd_lock;
  wire                         agu_icb_cmd_excl;
  wire [1:0]                   agu_icb_cmd_size;
  wire                         agu_icb_cmd_back2agu; 
  wire                         agu_icb_cmd_usign;
  wire [`E203_ITAG_WIDTH -1:0] agu_icb_cmd_itag;
  wire                         agu_icb_rsp_valid; 
  wire                         agu_icb_rsp_ready; 
  wire                         agu_icb_rsp_err  ; 
  wire                         agu_icb_rsp_excl_ok  ; 
  wire [`E203_XLEN-1:0]        agu_icb_rsp_rdata;

  wire commit_mret;
  wire commit_trap;
  wire excp_active;

  e203_exu u_e203_exu(
    .excp_active            (excp_active),
    .commit_mret            (commit_mret),
    .commit_trap            (commit_trap),
    .test_mode              (test_mode),
    .core_wfi               (core_wfi),
    .tm_stop                (tm_stop),
    .itcm_nohold            (itcm_nohold),
    .core_cgstop            (core_cgstop),
    .tcm_cgstop             (tcm_cgstop),
    .exu_active             (exu_active),

    .core_mhartid           (0),
    .dbg_irq_r              (0),
    .lcl_irq_r              (0    ),
    .ext_irq_r              (0    ),
    .sft_irq_r              (0    ),
    .tmr_irq_r              (0    ),
    .evt_r                  (0    ),

    .cmt_dpc                (cmt_dpc        ),
    .cmt_dpc_ena            (cmt_dpc_ena    ),
    .cmt_dcause             (cmt_dcause     ),
    .cmt_dcause_ena         (cmt_dcause_ena ),

    .wr_dcsr_ena     (wr_dcsr_ena    ),
    .wr_dpc_ena      (wr_dpc_ena     ),
    .wr_dscratch_ena (wr_dscratch_ena),


                                     
    .wr_csr_nxt      (wr_csr_nxt    ),
                                     
    .dcsr_r          (dcsr_r         ),
    .dpc_r           (dpc_r          ),
    .dscratch_r      (dscratch_r     ),

    .dbg_mode               (0  ),
    .dbg_halt_r             (0),
    .dbg_step_r             (0),
    .dbg_ebreakm_r          (0),
    .dbg_stopcycle          (0),

    .i_valid                (ifu_o_valid         ),
    .i_ready                (ifu_o_ready         ),
    .i_ir                   (ifu_o_ir            ),
    .i_pc                   (ifu_o_pc            ),
    .i_pc_vld               (ifu_o_pc_vld        ),
    .i_misalgn              (ifu_o_misalgn       ), 
    .i_buserr               (ifu_o_buserr        ), 
    .i_rs1idx               (ifu_o_rs1idx        ),
    .i_rs2idx               (ifu_o_rs2idx        ),
    .i_prdt_taken           (ifu_o_prdt_taken    ),
    .i_muldiv_b2b           (ifu_o_muldiv_b2b    ),

    .wfi_halt_ifu_req       (wfi_halt_ifu_req),
    .wfi_halt_ifu_ack       (wfi_halt_ifu_ack),

    .pipe_flush_ack         (pipe_flush_ack      ),
    .pipe_flush_req         (pipe_flush_req      ),
    .pipe_flush_add_op1     (pipe_flush_add_op1  ),  
    .pipe_flush_add_op2     (pipe_flush_add_op2  ),  
  `ifdef E203_TIMING_BOOST//}
    .pipe_flush_pc          (pipe_flush_pc),  
  `endif//}

    .lsu_o_valid            (lsu_o_valid   ),
    .lsu_o_ready            (lsu_o_ready   ),
    .lsu_o_wbck_wdat        (lsu_o_wbck_wdat    ),
    .lsu_o_wbck_itag        (lsu_o_wbck_itag    ),
    .lsu_o_wbck_err         (lsu_o_wbck_err     ),
    .lsu_o_cmt_buserr       (lsu_o_cmt_buserr     ),
    .lsu_o_cmt_ld           (lsu_o_cmt_ld),
    .lsu_o_cmt_st           (lsu_o_cmt_st),
    .lsu_o_cmt_badaddr      (lsu_o_cmt_badaddr     ),

    .agu_icb_cmd_valid      (agu_icb_cmd_valid   ),
    .agu_icb_cmd_ready      (agu_icb_cmd_ready   ),
    .agu_icb_cmd_addr       (agu_icb_cmd_addr    ),
    .agu_icb_cmd_read       (agu_icb_cmd_read    ),
    .agu_icb_cmd_wdata      (agu_icb_cmd_wdata   ),
    .agu_icb_cmd_wmask      (agu_icb_cmd_wmask   ),
    .agu_icb_cmd_lock       (agu_icb_cmd_lock    ),
    .agu_icb_cmd_excl       (agu_icb_cmd_excl    ),
    .agu_icb_cmd_size       (agu_icb_cmd_size    ),
    .agu_icb_cmd_back2agu   (agu_icb_cmd_back2agu),
    .agu_icb_cmd_usign      (agu_icb_cmd_usign   ),
    .agu_icb_cmd_itag       (agu_icb_cmd_itag    ),
    .agu_icb_rsp_valid      (agu_icb_rsp_valid   ),
    .agu_icb_rsp_ready      (agu_icb_rsp_ready   ),
    .agu_icb_rsp_err        (agu_icb_rsp_err     ),
    .agu_icb_rsp_excl_ok    (agu_icb_rsp_excl_ok ),
    .agu_icb_rsp_rdata      (agu_icb_rsp_rdata   ),

    .oitf_empty             (oitf_empty   ),
    .rf2ifu_x1              (rf2ifu_x1    ),
    .rf2ifu_rs1             (rf2ifu_rs1   ),
    .dec2ifu_rden           (dec2ifu_rden ),
    .dec2ifu_rs1en          (dec2ifu_rs1en),
    .dec2ifu_rdidx          (dec2ifu_rdidx),
    .dec2ifu_mulhsu         (dec2ifu_mulhsu),
    .dec2ifu_div            (dec2ifu_div   ),
    .dec2ifu_rem            (dec2ifu_rem   ),
    .dec2ifu_divu           (dec2ifu_divu  ),
    .dec2ifu_remu           (dec2ifu_remu  ),


    .clk_aon                (clk),
    .clk                    (clk),
    .rst_n                  (rst_n  ) 
  );

  wire                         lsu2biu_icb_cmd_valid;
  wire                         lsu2biu_icb_cmd_ready;
  wire [`E203_ADDR_SIZE-1:0]   lsu2biu_icb_cmd_addr; 
  wire                         lsu2biu_icb_cmd_read; 
  wire [`E203_XLEN-1:0]        lsu2biu_icb_cmd_wdata;
  wire [`E203_XLEN/8-1:0]      lsu2biu_icb_cmd_wmask;
  wire                         lsu2biu_icb_cmd_lock;
  wire                         lsu2biu_icb_cmd_excl;
  wire [1:0]                   lsu2biu_icb_cmd_size;

  wire                         lsu2biu_icb_rsp_valid;
  wire                         lsu2biu_icb_rsp_ready;
  wire                         lsu2biu_icb_rsp_err  ;
  wire                         lsu2biu_icb_rsp_excl_ok;
  wire [`E203_XLEN-1:0]        lsu2biu_icb_rsp_rdata;

  e203_lsu u_e203_lsu(
    .excp_active         (excp_active),
    .commit_mret            (commit_mret),
    .commit_trap         (commit_trap),
    .lsu_active          (lsu_active),
    .lsu_o_valid         (lsu_o_valid   ),
    .lsu_o_ready         (lsu_o_ready   ),
    .lsu_o_wbck_wdat     (lsu_o_wbck_wdat    ),
    .lsu_o_wbck_itag     (lsu_o_wbck_itag    ),
    .lsu_o_wbck_err      (lsu_o_wbck_err     ),
    .lsu_o_cmt_buserr    (lsu_o_cmt_buserr     ),
    .lsu_o_cmt_ld        (lsu_o_cmt_ld),
    .lsu_o_cmt_st        (lsu_o_cmt_st),
    .lsu_o_cmt_badaddr   (lsu_o_cmt_badaddr     ),
                        
    .agu_icb_cmd_valid   (agu_icb_cmd_valid ),
    .agu_icb_cmd_ready   (agu_icb_cmd_ready ),
    .agu_icb_cmd_addr    (agu_icb_cmd_addr  ),
    .agu_icb_cmd_read    (agu_icb_cmd_read  ),
    .agu_icb_cmd_wdata   (agu_icb_cmd_wdata ),
    .agu_icb_cmd_wmask   (agu_icb_cmd_wmask ),
    .agu_icb_cmd_lock    (agu_icb_cmd_lock  ),
    .agu_icb_cmd_excl    (agu_icb_cmd_excl  ),
    .agu_icb_cmd_size    (agu_icb_cmd_size  ),
   
    .agu_icb_cmd_back2agu(agu_icb_cmd_back2agu ),
    .agu_icb_cmd_usign   (agu_icb_cmd_usign),
    .agu_icb_cmd_itag    (agu_icb_cmd_itag),
  
    .agu_icb_rsp_valid   (agu_icb_rsp_valid ),
    .agu_icb_rsp_ready   (agu_icb_rsp_ready ),
    .agu_icb_rsp_err     (agu_icb_rsp_err   ),
    .agu_icb_rsp_excl_ok (agu_icb_rsp_excl_ok),
    .agu_icb_rsp_rdata   (agu_icb_rsp_rdata),

    .biu_icb_cmd_valid  (lsu2biu_icb_cmd_valid),
    .biu_icb_cmd_ready  (lsu2biu_icb_cmd_ready),
    .biu_icb_cmd_addr   (lsu2biu_icb_cmd_addr ),
    .biu_icb_cmd_read   (lsu2biu_icb_cmd_read ),
    .biu_icb_cmd_wdata  (lsu2biu_icb_cmd_wdata),
    .biu_icb_cmd_wmask  (lsu2biu_icb_cmd_wmask),
    .biu_icb_cmd_lock   (lsu2biu_icb_cmd_lock ),
    .biu_icb_cmd_excl   (lsu2biu_icb_cmd_excl ),
    .biu_icb_cmd_size   (lsu2biu_icb_cmd_size ),
    
    .biu_icb_rsp_valid  (lsu2biu_icb_rsp_valid),
    .biu_icb_rsp_ready  (lsu2biu_icb_rsp_ready),
    .biu_icb_rsp_err    (1'b0  ),
    .biu_icb_rsp_excl_ok(1'b1),
    .biu_icb_rsp_rdata  (lsu2biu_icb_rsp_rdata),

    .clk           (clk ),
    .rst_n         (rst_n        ) 
  );

  wire i_dcache_read = lsu2biu_icb_cmd_valid && lsu2biu_icb_cmd_read;
  wire i_dcache_write = lsu2biu_icb_cmd_valid && (~lsu2biu_icb_cmd_read);

  wire [127:0] i_m_readdata;
  wire [31:0] o_m_addr;
  wire [3:0] o_m_byte_en;
  wire [127:0] o_m_writedata;
  wire o_m_read, o_m_write;


  cache  u_cache (
    .clk                      ( clk                              ),
    .rst                      ( rst                              ),
    .i_icache_addr            ( ifu2biu_icb_cmd_addr             ),
    .i_icache_read            ( ifu2biu_icb_cmd_valid                    ),
    .i_dcache_addr            ( lsu2biu_icb_cmd_addr        ),
    .i_dcache_byte_en         ( lsu2biu_icb_cmd_wmask      ),
    .i_dcache_writedata       ( lsu2biu_icb_cmd_wdata     ),
    .i_dcache_read            ( i_dcache_read                    ),
    .i_dcache_write           ( i_dcache_write                   ),
    .i_m_readdata             ( i_m_readdata              ),
    .i_m_readdata_valid       ( 1'b1               ),
    .i_m_waitrequest          ( 1'b0                  ),

    .o_icache_readdata        ( ifu2biu_icb_rsp_rdata  ),
    .o_icache_readdata_valid  ( ifu2biu_icb_rsp_valid          ),
    .o_icache_waitrequest     ( ifu2biu_icb_cmd_ready             ),
    .o_dcache_readdata        ( lsu2biu_icb_rsp_rdata      ),
    .o_dcache_readdata_valid  ( lsu2biu_icb_rsp_valid          ),
    .o_dcache_waitrequest     ( lsu2biu_icb_cmd_ready             ),
    .o_m_addr                 ( o_m_addr                   ),
    .o_m_byte_en              ( o_m_byte_en                ),
    .o_m_writedata            ( o_m_writedata            ),
    .o_m_read                 ( o_m_read                         ),
    .o_m_write                ( o_m_write                        )
);

mem memory(.clk(clk),
           .we(o_m_write),
           .a(o_m_addr),
           .wd(o_m_writedata),
           .rd(i_m_readdata));


endmodule                                      
                                               
                                               
                                               
