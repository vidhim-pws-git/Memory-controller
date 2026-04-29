/*
 * AXI4 Address Decoder & Router
 *
 * Partitions address space and routes transactions to either RAM or DMA registers
 * - Address[19:16] = 0x0 → RAM port A (0x00000-0x0FFFF)
 * - Address[19:16] = 0x1 → DMA registers (0x10000-0x10FFF)
 * - Else → Error response (DECERR)
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module axi4_decoder #
(
    parameter AXI_DATA_WIDTH = 32,
    parameter AXI_ADDR_WIDTH = 16,
    parameter ID_WIDTH = 8,
    parameter STRB_WIDTH = AXI_DATA_WIDTH / 8
)
(
    input wire clk,
    input wire rst,

    // External AXI4 Slave Port (from top-level master)
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
    input wire                   s_axi_rready,

    // ===== Routed AXI4 to RAM (Port A) =====
    // Write Address Channel
    output wire [ID_WIDTH-1:0]    m_axi_ram_awid,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_ram_awaddr,
    output wire [7:0]             m_axi_ram_awlen,
    output wire [2:0]             m_axi_ram_awsize,
    output wire [1:0]             m_axi_ram_awburst,
    output wire [3:0]             m_axi_ram_awcache,
    output wire [2:0]             m_axi_ram_awprot,
    output wire                   m_axi_ram_awvalid,
    input wire                    m_axi_ram_awready,

    // Write Data Channel
    output wire [AXI_DATA_WIDTH-1:0] m_axi_ram_wdata,
    output wire [STRB_WIDTH-1:0]     m_axi_ram_wstrb,
    output wire                      m_axi_ram_wlast,
    output wire                      m_axi_ram_wvalid,
    input wire                       m_axi_ram_wready,

    // Write Response Channel
    input wire [ID_WIDTH-1:0]    m_axi_ram_bid,
    input wire [1:0]             m_axi_ram_bresp,
    input wire                   m_axi_ram_bvalid,
    output wire                  m_axi_ram_bready,

    // Read Address Channel
    output wire [ID_WIDTH-1:0]    m_axi_ram_arid,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_ram_araddr,
    output wire [7:0]             m_axi_ram_arlen,
    output wire [2:0]             m_axi_ram_arsize,
    output wire [1:0]             m_axi_ram_arburst,
    output wire [3:0]             m_axi_ram_arcache,
    output wire [2:0]             m_axi_ram_arprot,
    output wire                   m_axi_ram_arvalid,
    input wire                    m_axi_ram_arready,

    // Read Data Channel
    input wire [ID_WIDTH-1:0]    m_axi_ram_rid,
    input wire [AXI_DATA_WIDTH-1:0] m_axi_ram_rdata,
    input wire [1:0]             m_axi_ram_rresp,
    input wire                   m_axi_ram_rlast,
    input wire                   m_axi_ram_rvalid,
    output wire                  m_axi_ram_rready,

    // ===== Simplified AXI4 to Register Handler =====
    // Simplified write interface
    output wire [7:0]            m_axi_reg_addr,
    output wire [AXI_DATA_WIDTH-1:0] m_axi_reg_wdata,
    output wire [STRB_WIDTH-1:0]     m_axi_reg_wstrb,
    output wire                      m_axi_reg_write_valid,
    input wire                       m_axi_reg_write_done,

    // Simplified read interface
    output wire [7:0]            m_axi_reg_read_addr,
    output wire                  m_axi_reg_read_valid,
    input wire [AXI_DATA_WIDTH-1:0] m_axi_reg_rdata,
    input wire                      m_axi_reg_read_done
);

    // Address decode: check upper 4 bits [19:16]
    // Assumes AXI_ADDR_WIDTH >= 16
    wire [3:0] addr_select_aw = s_axi_awaddr[19:16];
    wire [3:0] addr_select_ar = s_axi_araddr[19:16];

    wire aw_is_ram = (addr_select_aw == 4'h0);
    wire aw_is_reg = (addr_select_aw == 4'h1);
    wire aw_is_err = ~aw_is_ram & ~aw_is_reg;

    wire ar_is_ram = (addr_select_ar == 4'h0);
    wire ar_is_reg = (addr_select_ar == 4'h1);
    wire ar_is_err = ~ar_is_ram & ~ar_is_reg;

    // ===== Write Path Routing =====

    // Route write address channel
    assign m_axi_ram_awid = s_axi_awid;
    assign m_axi_ram_awaddr = s_axi_awaddr;
    assign m_axi_ram_awlen = s_axi_awlen;
    assign m_axi_ram_awsize = s_axi_awsize;
    assign m_axi_ram_awburst = s_axi_awburst;
    assign m_axi_ram_awcache = s_axi_awcache;
    assign m_axi_ram_awprot = s_axi_awprot;
    assign m_axi_ram_awvalid = s_axi_awvalid & aw_is_ram;
    
    // Register handler write address
    assign m_axi_reg_addr = s_axi_awaddr[7:0];

    // Ready signal mux: only respond if destination is ready
    assign s_axi_awready = (aw_is_ram & m_axi_ram_awready) | (aw_is_reg & m_axi_reg_write_done);

    // Route write data channel (pass through to RAM always; register handler will ignore if not targeted)
    assign m_axi_ram_wdata = s_axi_wdata;
    assign m_axi_ram_wstrb = s_axi_wstrb;
    assign m_axi_ram_wlast = s_axi_wlast;
    
    // Store target during write address phase
    reg write_target_is_ram;
    always @(posedge clk) begin
        if (rst) begin
            write_target_is_ram <= 1'b0;
        end else if (s_axi_awvalid & s_axi_awready) begin
            write_target_is_ram <= aw_is_ram;
        end
    end

    assign m_axi_ram_wvalid = s_axi_wvalid & write_target_is_ram;
    
    // Register write path
    assign m_axi_reg_wdata = s_axi_wdata;
    assign m_axi_reg_wstrb = s_axi_wstrb;
    assign m_axi_reg_write_valid = s_axi_wvalid & ~write_target_is_ram;
    
    assign s_axi_wready = (write_target_is_ram & m_axi_ram_wready) | (~write_target_is_ram & m_axi_reg_write_done);

    // Route write response channel
    assign s_axi_bid = m_axi_ram_bid;
    assign s_axi_bresp = m_axi_ram_bresp;
    assign s_axi_bvalid = m_axi_ram_bvalid;
    assign m_axi_ram_bready = s_axi_bready;

    // ===== Read Path Routing =====

    // Route read address channel
    assign m_axi_ram_arid = s_axi_arid;
    assign m_axi_ram_araddr = s_axi_araddr;
    assign m_axi_ram_arlen = s_axi_arlen;
    assign m_axi_ram_arsize = s_axi_arsize;
    assign m_axi_ram_arburst = s_axi_arburst;
    assign m_axi_ram_arcache = s_axi_arcache;
    assign m_axi_ram_arprot = s_axi_arprot;
    assign m_axi_ram_arvalid = s_axi_arvalid & ar_is_ram;
    
    // Register handler read address
    assign m_axi_reg_read_addr = s_axi_araddr[7:0];

    // Ready signal mux
    assign s_axi_arready = (ar_is_ram & m_axi_ram_arready) | (ar_is_reg & m_axi_reg_read_done);

    // Store target during read address phase
    reg read_target_is_ram;
    always @(posedge clk) begin
        if (rst) begin
            read_target_is_ram <= 1'b0;
        end else if (s_axi_arvalid & s_axi_arready) begin
            read_target_is_ram <= ar_is_ram;
        end
    end

    assign m_axi_reg_read_valid = s_axi_arvalid & ar_is_reg;

    // Route read data channel
    assign s_axi_rid = read_target_is_ram ? m_axi_ram_rid : {ID_WIDTH{1'b0}};
    assign s_axi_rdata = read_target_is_ram ? m_axi_ram_rdata : m_axi_reg_rdata;
    assign s_axi_rresp = read_target_is_ram ? m_axi_ram_rresp : 2'b00;
    assign s_axi_rlast = read_target_is_ram ? m_axi_ram_rlast : 1'b1;
    assign s_axi_rvalid = read_target_is_ram ? m_axi_ram_rvalid : m_axi_reg_read_done;
    
    assign m_axi_ram_rready = s_axi_rready & read_target_is_ram;

endmodule

`default_nettype wire
