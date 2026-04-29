/*
 * DMA Register Handler
 *
 * Provides AXI4-compatible register interface for DMA control
 * - Stores read/write descriptors (address, length, tag)
 * - Generates descriptor valid pulses on control register writes
 * - Captures and returns DMA status (done, error codes)
 * - Manages control signals (enable, abort)
 *
 * Register Map (offsets from 0x10000):
 * 0x00: READ_ADDR      - DMA read source address
 * 0x04: READ_LEN       - DMA read length (bytes)
 * 0x08: READ_TAG       - DMA read operation tag
 * 0x0C: READ_CTRL      - Read control (write 0x01 to trigger)
 * 0x10: WRITE_ADDR     - DMA write destination address
 * 0x14: WRITE_LEN      - DMA write length (bytes)
 * 0x18: WRITE_TAG      - DMA write operation tag
 * 0x1C: WRITE_CTRL     - Write control (write 0x01 to trigger)
 * 0x20: READ_STATUS    - Read status (RO): error_code[15:8], done[0]
 * 0x24: WRITE_STATUS   - Write status (RO): error_code[15:8], done[0]
 * 0x28: CONTROL        - Global control: abort[1], enable[0]
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module dma_reg_handler #
(
    parameter AXI_DATA_WIDTH = 32,
    parameter STRB_WIDTH = (AXI_DATA_WIDTH / 8)
)
(
    input wire clk,
    input wire rst,

    // Simplified AXI4 register interface (from decoder)
    // Write path
    input wire [7:0]            reg_write_addr,
    input wire [AXI_DATA_WIDTH-1:0] reg_write_data,
    input wire [STRB_WIDTH-1:0] reg_write_strb,
    input wire                  reg_write_valid,
    output wire                 reg_write_done,

    // Read path
    input wire [7:0]            reg_read_addr,
    input wire                  reg_read_valid,
    output wire [AXI_DATA_WIDTH-1:0] reg_read_data,
    output wire                 reg_read_done,

    // DMA Descriptor Outputs (to axi_dma)
    // Read descriptor
    output wire [15:0]          dma_read_addr,
    output wire [15:0]          dma_read_len,
    output wire [7:0]           dma_read_tag,
    output wire                 dma_read_valid,

    // Write descriptor
    output wire [15:0]          dma_write_addr,
    output wire [15:0]          dma_write_len,
    output wire [7:0]           dma_write_tag,
    output wire                 dma_write_valid,

    // DMA Status Inputs (from axi_dma)
    input wire                  dma_read_done,
    input wire [7:0]            dma_read_error,
    input wire                  dma_write_done,
    input wire [7:0]            dma_write_error,

    // Control Outputs
    output wire                 dma_enable,
    output wire                 dma_abort
);

    // ===== Internal Register Storage =====
    reg [15:0] read_addr_reg;
    reg [15:0] read_len_reg;
    reg [7:0]  read_tag_reg;
    reg [7:0]  read_ctrl_reg;

    reg [15:0] write_addr_reg;
    reg [15:0] write_len_reg;
    reg [7:0]  write_tag_reg;
    reg [7:0]  write_ctrl_reg;

    reg [7:0]  control_reg;

    // Status capture registers
    reg        read_status_done;
    reg [7:0]  read_status_error;
    reg        write_status_done;
    reg [7:0]  write_status_error;

    // Descriptor valid pulse generators
    reg        read_ctrl_prev;
    reg        write_ctrl_prev;
    wire       read_ctrl_pulse = read_ctrl_reg[0] & ~read_ctrl_prev;
    wire       write_ctrl_pulse = write_ctrl_reg[0] & ~write_ctrl_prev;

    // ===== Write Path Logic =====
    always @(posedge clk) begin
        if (rst) begin
            read_addr_reg <= 16'h0;
            read_len_reg <= 16'h0;
            read_tag_reg <= 8'h0;
            read_ctrl_reg <= 8'h0;
            write_addr_reg <= 16'h0;
            write_len_reg <= 16'h0;
            write_tag_reg <= 8'h0;
            write_ctrl_reg <= 8'h0;
            control_reg <= 8'h0;
        end else if (reg_write_valid) begin
            case (reg_write_addr)
                8'h00: read_addr_reg <= reg_write_data[15:0];
                8'h04: read_len_reg <= reg_write_data[15:0];
                8'h08: read_tag_reg <= reg_write_data[7:0];
                8'h0C: read_ctrl_reg <= reg_write_data[7:0];
                8'h10: write_addr_reg <= reg_write_data[15:0];
                8'h14: write_len_reg <= reg_write_data[15:0];
                8'h18: write_tag_reg <= reg_write_data[7:0];
                8'h1C: write_ctrl_reg <= reg_write_data[7:0];
                8'h28: control_reg <= reg_write_data[7:0];
                default: begin
                    // Status registers are read-only; no write action
                end
            endcase
        end
    end

    // ===== Status Capture Logic =====
    always @(posedge clk) begin
        if (rst) begin
            read_status_done <= 1'b0;
            read_status_error <= 8'h0;
            write_status_done <= 1'b0;
            write_status_error <= 8'h0;
        end else begin
            // Capture DMA done signals
            if (dma_read_done) begin
                read_status_done <= 1'b1;
                read_status_error <= dma_read_error;
            end
            if (dma_write_done) begin
                write_status_done <= 1'b1;
                write_status_error <= dma_write_error;
            end

            // Clear status on descriptor trigger (optional: clear when new command issued)
            if (read_ctrl_pulse) begin
                read_status_done <= 1'b0;
                read_status_error <= 8'h0;
            end
            if (write_ctrl_pulse) begin
                write_status_done <= 1'b0;
                write_status_error <= 8'h0;
            end
        end
    end

    // ===== Descriptor Output Generation =====
    assign dma_read_addr = read_addr_reg;
    assign dma_read_len = read_len_reg;
    assign dma_read_tag = read_tag_reg;
    assign dma_read_valid = read_ctrl_pulse;

    assign dma_write_addr = write_addr_reg;
    assign dma_write_len = write_len_reg;
    assign dma_write_tag = write_tag_reg;
    assign dma_write_valid = write_ctrl_pulse;

    // ===== Control Output Generation =====
    assign dma_enable = control_reg[0];
    assign dma_abort = control_reg[1];

    // ===== Read Path Logic =====
    reg [AXI_DATA_WIDTH-1:0] read_data_mux;

    always @(*) begin
        case (reg_read_addr)
            8'h00: read_data_mux = {{(AXI_DATA_WIDTH-16){1'b0}}, read_addr_reg};
            8'h04: read_data_mux = {{(AXI_DATA_WIDTH-16){1'b0}}, read_len_reg};
            8'h08: read_data_mux = {{(AXI_DATA_WIDTH-8){1'b0}}, read_tag_reg};
            8'h0C: read_data_mux = {{(AXI_DATA_WIDTH-8){1'b0}}, read_ctrl_reg};
            8'h10: read_data_mux = {{(AXI_DATA_WIDTH-16){1'b0}}, write_addr_reg};
            8'h14: read_data_mux = {{(AXI_DATA_WIDTH-16){1'b0}}, write_len_reg};
            8'h18: read_data_mux = {{(AXI_DATA_WIDTH-8){1'b0}}, write_tag_reg};
            8'h1C: read_data_mux = {{(AXI_DATA_WIDTH-8){1'b0}}, write_ctrl_reg};
            8'h20: read_data_mux = {{(AXI_DATA_WIDTH-16){1'b0}}, read_status_error, 7'b0, read_status_done};
            8'h24: read_data_mux = {{(AXI_DATA_WIDTH-16){1'b0}}, write_status_error, 7'b0, write_status_done};
            8'h28: read_data_mux = {{(AXI_DATA_WIDTH-8){1'b0}}, control_reg};
            default: read_data_mux = {AXI_DATA_WIDTH{1'b0}};
        endcase
    end

    assign reg_read_data = read_data_mux;

    // ===== Handshake Logic =====
    // Write path: always ready (no buffering)
    assign reg_write_done = reg_write_valid;

    // Read path: always ready (combinatorial read)
    assign reg_read_done = reg_read_valid;

    // ===== Control Pulse Detection =====
    always @(posedge clk) begin
        if (rst) begin
            read_ctrl_prev <= 1'b0;
            write_ctrl_prev <= 1'b0;
        end else begin
            read_ctrl_prev <= read_ctrl_reg[0];
            write_ctrl_prev <= write_ctrl_reg[0];
        end
    end

endmodule

`default_nettype wire
