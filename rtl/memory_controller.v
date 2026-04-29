/*
 * Memory Controller
 *
 * Top-level wrapper integrating:
 * - AXI4 Dual-Port RAM (axi_dp_ram.v)
 * - AXI DMA (axi_dma.v)
 * - AXI4 Address Decoder & Router (axi4_decoder.v)
 * - DMA Register Handler (dma_reg_handler.v)
 *
 * Architecture:
 * - Single external AXI4 slave port
 * - Address partitioning:
 *   * 0x00000-0x0FFFF → RAM port A (direct data access)
 *   * 0x10000-0x10FFF → DMA control registers
 * - DMA uses RAM port B for internal memory operations
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module memory_controller #
(
    parameter AXI_DATA_WIDTH = 32,
    parameter AXI_ADDR_WIDTH = 16,
    parameter ID_WIDTH = 8,
    parameter RAM_ADDR_WIDTH = 16,
    parameter STRB_WIDTH = (AXI_DATA_WIDTH / 8)
)
(
    input wire clk,
    input wire rst,

    // External AXI4 Slave Port
    // Write Address Channel
    input wire [ID_WIDTH-1:0]    s_axi_awid,
    input wire [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input wire [7:0]             s_axi_awlen,
    input wire [2:0]             s_axi_awsize,
    input wire [1:0]             s_axi_awburst,
    input wire [3:0]             s_axi_awcache,
    input wire [2:0]             s_axi_awprot,
    input wire                   s_axi_awvalid,
    output wire                  s_axi_awready,

    // Write Data Channel
    input wire [AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input wire [STRB_WIDTH-1:0]     s_axi_wstrb,
    input wire                      s_axi_wlast,
    input wire                      s_axi_wvalid,
    output wire                     s_axi_wready,

    // Write Response Channel
    output wire [ID_WIDTH-1:0]   s_axi_bid,
    output wire [1:0]            s_axi_bresp,
    output wire                  s_axi_bvalid,
    input wire                   s_axi_bready,

    // Read Address Channel
    input wire [ID_WIDTH-1:0]    s_axi_arid,
    input wire [AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input wire [7:0]             s_axi_arlen,
    input wire [2:0]             s_axi_arsize,
    input wire [1:0]             s_axi_arburst,
    input wire [3:0]             s_axi_arcache,
    input wire [2:0]             s_axi_arprot,
    input wire                   s_axi_arvalid,
    output wire                  s_axi_arready,

    // Read Data Channel
    output wire [ID_WIDTH-1:0]   s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output wire [1:0]            s_axi_rresp,
    output wire                  s_axi_rlast,
    output wire                  s_axi_rvalid,
    input wire                   s_axi_rready
);

    // ===== Internal Signals =====

    // Decoder to RAM (port A)
    wire [ID_WIDTH-1:0]    decoder_ram_awid;
    wire [AXI_ADDR_WIDTH-1:0] decoder_ram_awaddr;
    wire [7:0]             decoder_ram_awlen;
    wire [2:0]             decoder_ram_awsize;
    wire [1:0]             decoder_ram_awburst;
    wire [3:0]             decoder_ram_awcache;
    wire [2:0]             decoder_ram_awprot;
    wire                   decoder_ram_awvalid;
    wire                   decoder_ram_awready;

    wire [AXI_DATA_WIDTH-1:0] decoder_ram_wdata;
    wire [STRB_WIDTH-1:0]     decoder_ram_wstrb;
    wire                      decoder_ram_wlast;
    wire                      decoder_ram_wvalid;
    wire                      decoder_ram_wready;

    wire [ID_WIDTH-1:0]    decoder_ram_bid;
    wire [1:0]             decoder_ram_bresp;
    wire                   decoder_ram_bvalid;
    wire                   decoder_ram_bready;

    wire [ID_WIDTH-1:0]    decoder_ram_arid;
    wire [AXI_ADDR_WIDTH-1:0] decoder_ram_araddr;
    wire [7:0]             decoder_ram_arlen;
    wire [2:0]             decoder_ram_arsize;
    wire [1:0]             decoder_ram_arburst;
    wire [3:0]             decoder_ram_arcache;
    wire [2:0]             decoder_ram_arprot;
    wire                   decoder_ram_arvalid;
    wire                   decoder_ram_arready;

    wire [ID_WIDTH-1:0]    decoder_ram_rid;
    wire [AXI_DATA_WIDTH-1:0] decoder_ram_rdata;
    wire [1:0]             decoder_ram_rresp;
    wire                   decoder_ram_rlast;
    wire                   decoder_ram_rvalid;
    wire                   decoder_ram_rready;

    // Decoder to Register Handler
    wire [7:0]            decoder_reg_write_addr;
    wire [AXI_DATA_WIDTH-1:0] decoder_reg_write_data;
    wire [STRB_WIDTH-1:0]     decoder_reg_write_strb;
    wire                      decoder_reg_write_valid;
    wire                      decoder_reg_write_done;

    wire [7:0]            decoder_reg_read_addr;
    wire                  decoder_reg_read_valid;
    wire [AXI_DATA_WIDTH-1:0] decoder_reg_read_data;
    wire                      decoder_reg_read_done;

    // Register Handler to DMA
    wire [15:0]          reg_handler_dma_read_addr;
    wire [15:0]          reg_handler_dma_read_len;
    wire [7:0]           reg_handler_dma_read_tag;
    wire                 reg_handler_dma_read_valid;

    wire [15:0]          reg_handler_dma_write_addr;
    wire [15:0]          reg_handler_dma_write_len;
    wire [7:0]           reg_handler_dma_write_tag;
    wire                 reg_handler_dma_write_valid;

    wire                 reg_handler_dma_enable;
    wire                 reg_handler_dma_abort;

    // DMA Status to Register Handler
    wire                 dma_read_status_valid;
    wire [7:0]           dma_read_status_error;
    wire                 dma_write_status_valid;
    wire [7:0]           dma_write_status_error;

    // DMA Master to RAM (port B)
    wire [ID_WIDTH-1:0]    dma_axi_awid;
    wire [AXI_ADDR_WIDTH-1:0] dma_axi_awaddr;
    wire [7:0]             dma_axi_awlen;
    wire [2:0]             dma_axi_awsize;
    wire [1:0]             dma_axi_awburst;
    wire [3:0]             dma_axi_awcache;
    wire [2:0]             dma_axi_awprot;
    wire                   dma_axi_awvalid;
    wire                   dma_axi_awready;

    wire [AXI_DATA_WIDTH-1:0] dma_axi_wdata;
    wire [STRB_WIDTH-1:0]     dma_axi_wstrb;
    wire                      dma_axi_wlast;
    wire                      dma_axi_wvalid;
    wire                      dma_axi_wready;

    wire [ID_WIDTH-1:0]    dma_axi_bid;
    wire [1:0]             dma_axi_bresp;
    wire                   dma_axi_bvalid;
    wire                   dma_axi_bready;

    wire [ID_WIDTH-1:0]    dma_axi_arid;
    wire [AXI_ADDR_WIDTH-1:0] dma_axi_araddr;
    wire [7:0]             dma_axi_arlen;
    wire [2:0]             dma_axi_arsize;
    wire [1:0]             dma_axi_arburst;
    wire [3:0]             dma_axi_arcache;
    wire [2:0]             dma_axi_arprot;
    wire                   dma_axi_arvalid;
    wire                   dma_axi_arready;

    wire [ID_WIDTH-1:0]    dma_axi_rid;
    wire [AXI_DATA_WIDTH-1:0] dma_axi_rdata;
    wire [1:0]             dma_axi_rresp;
    wire                   dma_axi_rlast;
    wire                   dma_axi_rvalid;
    wire                   dma_axi_rready;

    // ===== Instantiate Modules =====

    // 1. AXI4 Address Decoder & Router
    axi4_decoder #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .ID_WIDTH(ID_WIDTH),
        .STRB_WIDTH(STRB_WIDTH)
    ) decoder_inst (
        .clk(clk),
        .rst(rst),

        // External AXI4 Slave
        .s_axi_awid(s_axi_awid),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awlen(s_axi_awlen),
        .s_axi_awsize(s_axi_awsize),
        .s_axi_awburst(s_axi_awburst),
        .s_axi_awcache(s_axi_awcache),
        .s_axi_awprot(s_axi_awprot),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),

        .s_axi_wdata(s_axi_wdata),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wlast(s_axi_wlast),
        .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready),

        .s_axi_bid(s_axi_bid),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),

        .s_axi_arid(s_axi_arid),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_arlen(s_axi_arlen),
        .s_axi_arsize(s_axi_arsize),
        .s_axi_arburst(s_axi_arburst),
        .s_axi_arcache(s_axi_arcache),
        .s_axi_arprot(s_axi_arprot),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),

        .s_axi_rid(s_axi_rid),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rlast(s_axi_rlast),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),

        // Routed to RAM
        .m_axi_ram_awid(decoder_ram_awid),
        .m_axi_ram_awaddr(decoder_ram_awaddr),
        .m_axi_ram_awlen(decoder_ram_awlen),
        .m_axi_ram_awsize(decoder_ram_awsize),
        .m_axi_ram_awburst(decoder_ram_awburst),
        .m_axi_ram_awcache(decoder_ram_awcache),
        .m_axi_ram_awprot(decoder_ram_awprot),
        .m_axi_ram_awvalid(decoder_ram_awvalid),
        .m_axi_ram_awready(decoder_ram_awready),

        .m_axi_ram_wdata(decoder_ram_wdata),
        .m_axi_ram_wstrb(decoder_ram_wstrb),
        .m_axi_ram_wlast(decoder_ram_wlast),
        .m_axi_ram_wvalid(decoder_ram_wvalid),
        .m_axi_ram_wready(decoder_ram_wready),

        .m_axi_ram_bid(decoder_ram_bid),
        .m_axi_ram_bresp(decoder_ram_bresp),
        .m_axi_ram_bvalid(decoder_ram_bvalid),
        .m_axi_ram_bready(decoder_ram_bready),

        .m_axi_ram_arid(decoder_ram_arid),
        .m_axi_ram_araddr(decoder_ram_araddr),
        .m_axi_ram_arlen(decoder_ram_arlen),
        .m_axi_ram_arsize(decoder_ram_arsize),
        .m_axi_ram_arburst(decoder_ram_arburst),
        .m_axi_ram_arcache(decoder_ram_arcache),
        .m_axi_ram_arprot(decoder_ram_arprot),
        .m_axi_ram_arvalid(decoder_ram_arvalid),
        .m_axi_ram_arready(decoder_ram_arready),

        .m_axi_ram_rid(decoder_ram_rid),
        .m_axi_ram_rdata(decoder_ram_rdata),
        .m_axi_ram_rresp(decoder_ram_rresp),
        .m_axi_ram_rlast(decoder_ram_rlast),
        .m_axi_ram_rvalid(decoder_ram_rvalid),
        .m_axi_ram_rready(decoder_ram_rready),

        // Routed to Register Handler
        .m_axi_reg_addr(decoder_reg_write_addr),
        .m_axi_reg_wdata(decoder_reg_write_data),
        .m_axi_reg_wstrb(decoder_reg_write_strb),
        .m_axi_reg_write_valid(decoder_reg_write_valid),
        .m_axi_reg_write_done(decoder_reg_write_done),

        .m_axi_reg_read_addr(decoder_reg_read_addr),
        .m_axi_reg_read_valid(decoder_reg_read_valid),
        .m_axi_reg_rdata(decoder_reg_read_data),
        .m_axi_reg_read_done(decoder_reg_read_done)
    );

    // 2. DMA Register Handler
    dma_reg_handler #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .STRB_WIDTH(STRB_WIDTH)
    ) reg_handler_inst (
        .clk(clk),
        .rst(rst),

        // Register interface from decoder
        .reg_write_addr(decoder_reg_write_addr),
        .reg_write_data(decoder_reg_write_data),
        .reg_write_strb(decoder_reg_write_strb),
        .reg_write_valid(decoder_reg_write_valid),
        .reg_write_done(decoder_reg_write_done),

        .reg_read_addr(decoder_reg_read_addr),
        .reg_read_valid(decoder_reg_read_valid),
        .reg_read_data(decoder_reg_read_data),
        .reg_read_done(decoder_reg_read_done),

        // DMA descriptor outputs
        .dma_read_addr(reg_handler_dma_read_addr),
        .dma_read_len(reg_handler_dma_read_len),
        .dma_read_tag(reg_handler_dma_read_tag),
        .dma_read_valid(reg_handler_dma_read_valid),

        .dma_write_addr(reg_handler_dma_write_addr),
        .dma_write_len(reg_handler_dma_write_len),
        .dma_write_tag(reg_handler_dma_write_tag),
        .dma_write_valid(reg_handler_dma_write_valid),

        // DMA status inputs
        .dma_read_done(dma_read_status_valid),
        .dma_read_error(dma_read_status_error),
        .dma_write_done(dma_write_status_valid),
        .dma_write_error(dma_write_status_error),

        // Control outputs
        .dma_enable(reg_handler_dma_enable),
        .dma_abort(reg_handler_dma_abort)
    );

    // 3. AXI DMA Controller
    axi_dma #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_STRB_WIDTH(STRB_WIDTH),
        .AXI_ID_WIDTH(ID_WIDTH),
        .AXI_MAX_BURST_LEN(16),
        .AXIS_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXIS_KEEP_ENABLE(1),
        .AXIS_LAST_ENABLE(1),
        .AXIS_ID_ENABLE(0),
        .AXIS_DEST_ENABLE(0),
        .AXIS_USER_ENABLE(0),
        .LEN_WIDTH(16),
        .TAG_WIDTH(8),
        .ENABLE_SG(0),
        .ENABLE_UNALIGNED(0)
    ) dma_inst (
        .clk(clk),
        .rst(rst),

        // Read descriptor
        .s_axis_read_desc_addr(reg_handler_dma_read_addr),
        .s_axis_read_desc_len(reg_handler_dma_read_len),
        .s_axis_read_desc_tag(reg_handler_dma_read_tag),
        .s_axis_read_desc_valid(reg_handler_dma_read_valid),
        .s_axis_read_desc_ready(),

        // Write descriptor
        .s_axis_write_desc_addr(reg_handler_dma_write_addr),
        .s_axis_write_desc_len(reg_handler_dma_write_len),
        .s_axis_write_desc_tag(reg_handler_dma_write_tag),
        .s_axis_write_desc_valid(reg_handler_dma_write_valid),
        .s_axis_write_desc_ready(),

        // Read status
        .m_axis_read_desc_status_valid(dma_read_status_valid),
        .m_axis_read_desc_status_len(dma_read_status_error),
        .m_axis_read_desc_status_tag(),
        .m_axis_read_desc_status_id(),
        .m_axis_read_desc_status_dest(),
        .m_axis_read_desc_status_user(),
        .m_axis_read_desc_status_error(dma_read_status_error),
        .m_axis_read_desc_status_ready(1'b1),

        // Write status
        .m_axis_write_desc_status_valid(dma_write_status_valid),
        .m_axis_write_desc_status_len(),
        .m_axis_write_desc_status_tag(),
        .m_axis_write_desc_status_id(),
        .m_axis_write_desc_status_dest(),
        .m_axis_write_desc_status_user(),
        .m_axis_write_desc_status_error(dma_write_status_error),
        .m_axis_write_desc_status_ready(1'b1),

        // Read data stream (unused in loopback)
        .m_axis_read_data_tdata(),
        .m_axis_read_data_tkeep(),
        .m_axis_read_data_tlast(),
        .m_axis_read_data_tid(),
        .m_axis_read_data_tdest(),
        .m_axis_read_data_tuser(),
        .m_axis_read_data_tvalid(),
        .m_axis_read_data_tready(1'b1),

        // Write data stream (unused in loopback)
        .s_axis_write_data_tdata(32'h0),
        .s_axis_write_data_tkeep(4'hF),
        .s_axis_write_data_tlast(1'b0),
        .s_axis_write_data_tid(8'h0),
        .s_axis_write_data_tdest(8'h0),
        .s_axis_write_data_tuser(1'b0),
        .s_axis_write_data_tvalid(1'b0),
        .s_axis_write_data_tready(),

        // AXI4 Master
        .m_axi_awid(dma_axi_awid),
        .m_axi_awaddr(dma_axi_awaddr),
        .m_axi_awlen(dma_axi_awlen),
        .m_axi_awsize(dma_axi_awsize),
        .m_axi_awburst(dma_axi_awburst),
        .m_axi_awcache(dma_axi_awcache),
        .m_axi_awprot(dma_axi_awprot),
        .m_axi_awvalid(dma_axi_awvalid),
        .m_axi_awready(dma_axi_awready),

        .m_axi_wdata(dma_axi_wdata),
        .m_axi_wstrb(dma_axi_wstrb),
        .m_axi_wlast(dma_axi_wlast),
        .m_axi_wvalid(dma_axi_wvalid),
        .m_axi_wready(dma_axi_wready),

        .m_axi_bid(dma_axi_bid),
        .m_axi_bresp(dma_axi_bresp),
        .m_axi_bvalid(dma_axi_bvalid),
        .m_axi_bready(dma_axi_bready),

        .m_axi_arid(dma_axi_arid),
        .m_axi_araddr(dma_axi_araddr),
        .m_axi_arlen(dma_axi_arlen),
        .m_axi_arsize(dma_axi_arsize),
        .m_axi_arburst(dma_axi_arburst),
        .m_axi_arcache(dma_axi_arcache),
        .m_axi_arprot(dma_axi_arprot),
        .m_axi_arvalid(dma_axi_arvalid),
        .m_axi_arready(dma_axi_arready),

        .m_axi_rid(dma_axi_rid),
        .m_axi_rdata(dma_axi_rdata),
        .m_axi_rresp(dma_axi_rresp),
        .m_axi_rlast(dma_axi_rlast),
        .m_axi_rvalid(dma_axi_rvalid),
        .m_axi_rready(dma_axi_rready),

        // Control signals
        .read_enable(reg_handler_dma_enable),
        .write_enable(reg_handler_dma_enable),
        .write_abort(reg_handler_dma_abort)
    );

    // 4. AXI Dual-Port RAM
    axi_dp_ram #(
        .DATA_WIDTH(AXI_DATA_WIDTH),
        .ADDR_WIDTH(RAM_ADDR_WIDTH),
        .STRB_WIDTH(STRB_WIDTH),
        .ID_WIDTH(ID_WIDTH),
        .A_PIPELINE_OUTPUT(0),
        .B_PIPELINE_OUTPUT(0),
        .A_INTERLEAVE(0),
        .B_INTERLEAVE(0)
    ) ram_inst (
        // Port A clock and reset
        .a_clk(clk),
        .a_rst(rst),

        // Port B clock and reset
        .b_clk(clk),
        .b_rst(rst),

        // Port A - External AXI4 Slave (write channels)
        .s_axi_a_awid(decoder_ram_awid),
        .s_axi_a_awaddr(decoder_ram_awaddr),
        .s_axi_a_awlen(decoder_ram_awlen),
        .s_axi_a_awsize(decoder_ram_awsize),
        .s_axi_a_awburst(decoder_ram_awburst),
        .s_axi_a_awlock(1'b0),
        .s_axi_a_awcache(decoder_ram_awcache),
        .s_axi_a_awprot(decoder_ram_awprot),
        .s_axi_a_awvalid(decoder_ram_awvalid),
        .s_axi_a_awready(decoder_ram_awready),

        .s_axi_a_wdata(decoder_ram_wdata),
        .s_axi_a_wstrb(decoder_ram_wstrb),
        .s_axi_a_wlast(decoder_ram_wlast),
        .s_axi_a_wvalid(decoder_ram_wvalid),
        .s_axi_a_wready(decoder_ram_wready),

        .s_axi_a_bid(decoder_ram_bid),
        .s_axi_a_bresp(decoder_ram_bresp),
        .s_axi_a_bvalid(decoder_ram_bvalid),
        .s_axi_a_bready(decoder_ram_bready),

        // Port A - External AXI4 Slave (read channels)
        .s_axi_a_arid(decoder_ram_arid),
        .s_axi_a_araddr(decoder_ram_araddr),
        .s_axi_a_arlen(decoder_ram_arlen),
        .s_axi_a_arsize(decoder_ram_arsize),
        .s_axi_a_arburst(decoder_ram_arburst),
        .s_axi_a_arlock(1'b0),
        .s_axi_a_arcache(decoder_ram_arcache),
        .s_axi_a_arprot(decoder_ram_arprot),
        .s_axi_a_arvalid(decoder_ram_arvalid),
        .s_axi_a_arready(decoder_ram_arready),

        .s_axi_a_rid(decoder_ram_rid),
        .s_axi_a_rdata(decoder_ram_rdata),
        .s_axi_a_rresp(decoder_ram_rresp),
        .s_axi_a_rlast(decoder_ram_rlast),
        .s_axi_a_rvalid(decoder_ram_rvalid),
        .s_axi_a_rready(decoder_ram_rready),

        // Port B - DMA AXI4 Master (write channels)
        .s_axi_b_awid(dma_axi_awid),
        .s_axi_b_awaddr(dma_axi_awaddr),
        .s_axi_b_awlen(dma_axi_awlen),
        .s_axi_b_awsize(dma_axi_awsize),
        .s_axi_b_awburst(dma_axi_awburst),
        .s_axi_b_awlock(1'b0),
        .s_axi_b_awcache(dma_axi_awcache),
        .s_axi_b_awprot(dma_axi_awprot),
        .s_axi_b_awvalid(dma_axi_awvalid),
        .s_axi_b_awready(dma_axi_awready),

        .s_axi_b_wdata(dma_axi_wdata),
        .s_axi_b_wstrb(dma_axi_wstrb),
        .s_axi_b_wlast(dma_axi_wlast),
        .s_axi_b_wvalid(dma_axi_wvalid),
        .s_axi_b_wready(dma_axi_wready),

        .s_axi_b_bid(dma_axi_bid),
        .s_axi_b_bresp(dma_axi_bresp),
        .s_axi_b_bvalid(dma_axi_bvalid),
        .s_axi_b_bready(dma_axi_bready),

        // Port B - DMA AXI4 Master (read channels)
        .s_axi_b_arid(dma_axi_arid),
        .s_axi_b_araddr(dma_axi_araddr),
        .s_axi_b_arlen(dma_axi_arlen),
        .s_axi_b_arsize(dma_axi_arsize),
        .s_axi_b_arburst(dma_axi_arburst),
        .s_axi_b_arlock(1'b0),
        .s_axi_b_arcache(dma_axi_arcache),
        .s_axi_b_arprot(dma_axi_arprot),
        .s_axi_b_arvalid(dma_axi_arvalid),
        .s_axi_b_arready(dma_axi_arready),

        .s_axi_b_rid(dma_axi_rid),
        .s_axi_b_rdata(dma_axi_rdata),
        .s_axi_b_rresp(dma_axi_rresp),
        .s_axi_b_rlast(dma_axi_rlast),
        .s_axi_b_rvalid(dma_axi_rvalid),
        .s_axi_b_rready(dma_axi_rready)
    );

endmodule

`default_nettype wire
