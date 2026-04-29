# Memory Controller Wrapper Design Specification

## Parameters

```
AXI_DATA_WIDTH = 32
AXI_ADDR_WIDTH = 16
ID_WIDTH = 8
RAM_ADDR_WIDTH = 16 (64KB dual-port RAM)
REGISTER_BASE_ADDR = 0x10000 (upper 4 bits [19:16] = 0x1)
```

---

## Register Map (Address Offsets from REGISTER_BASE_ADDR)

| Offset | Name | Access | Width | Description |
|--------|------|--------|-------|-------------|
| 0x00 | READ_ADDR | RW | 16 | DMA read source address |
| 0x04 | READ_LEN | RW | 16 | DMA read length (bytes) |
| 0x08 | READ_TAG | RW | 8 | DMA read operation tag |
| 0x0C | READ_CTRL | RW | 8 | Read control: [7:1]=reserved, [0]=valid (write-strobe trigger) |
| 0x10 | WRITE_ADDR | RW | 16 | DMA write destination address |
| 0x14 | WRITE_LEN | RW | 16 | DMA write length (bytes) |
| 0x18 | WRITE_TAG | RW | 8 | DMA write operation tag |
| 0x1C | WRITE_CTRL | RW | 8 | Write control: [7:1]=reserved, [0]=valid (write-strobe trigger) |
| 0x20 | READ_STATUS | RO | 16 | Read status: [15:8]=error_code, [7:1]=reserved, [0]=done |
| 0x24 | WRITE_STATUS | RO | 16 | Write status: [15:8]=error_code, [7:1]=reserved, [0]=done |
| 0x28 | CONTROL | RW | 8 | Global control: [7:2]=reserved, [1]=abort, [0]=enable |

**Total register space:** ~0x2C bytes (placed at 0x10000–0x1002C)

---

## Module Interfaces

### 1. memory_controller.v (Top-Level)

**Parameters:**
```verilog
parameter AXI_DATA_WIDTH = 32,
parameter AXI_ADDR_WIDTH = 16,
parameter ID_WIDTH = 8,
parameter RAM_ADDR_WIDTH = 16
```

**Ports:**
```
// Clock & Reset
input clk, input rst

// External AXI4 Slave (to decoder)
input [ID_WIDTH-1:0] s_axi_awid
input [AXI_ADDR_WIDTH-1:0] s_axi_awaddr
input [7:0] s_axi_awlen
input [2:0] s_axi_awsize
input [1:0] s_axi_awburst
input s_axi_awvalid
output s_axi_awready
// ... (full AXI4 write address/write data/write response channels)

input [ID_WIDTH-1:0] s_axi_arid
input [AXI_ADDR_WIDTH-1:0] s_axi_araddr
input [7:0] s_axi_arlen
input [2:0] s_axi_arsize
input [1:0] s_axi_arburst
input s_axi_arvalid
output s_axi_arready
// ... (full AXI4 read address/read data channels)
```

**Internal connections:**
- Instantiate: axi4_decoder, dma_reg_handler, axi_dma, axi_dp_ram
- Route decoder outputs → RAM port A or register handler
- Route register handler → DMA descriptor inputs
- Route DMA status outputs → register handler
- DMA AXI4 master (m_axi_*) → RAM port B

---

### 2. axi4_decoder.v (Address Decoder & Router)

**Role:** Partition address space; route transactions to RAM or register handler

**Ports:**
```
input clk, input rst

// External AXI4 slave (from top-level)
input [ID_WIDTH-1:0] s_axi_*id, s_axi_*addr, s_axi_*valid, etc.
output s_axi_*ready, s_axi_*resp, etc.

// Routed AXI4 to RAM (port A)
output [ID_WIDTH-1:0] m_axi_ram_*id
output [AXI_ADDR_WIDTH-1:0] m_axi_ram_*addr (lower 16 bits of input)
output m_axi_ram_*valid
input m_axi_ram_*ready
input [31:0] m_axi_ram_rdata
input [1:0] m_axi_ram_rresp
// ... (similar for other AXI4 channels)

// Routed AXI4 to Register Handler
output [7:0] m_axi_reg_addr (bits [7:0] of input, within reg space)
output [31:0] m_axi_reg_wdata, m_axi_reg_rdata
output m_axi_reg_write_en, m_axi_reg_read_en
input m_axi_reg_write_valid, m_axi_reg_read_valid
```

**Logic:**
- If addr[19:16] == 4'h0: route to RAM (pass through fully)
- Else if addr[19:16] == 4'h1: route to register handler
- Else: return error response (DECERR)

---

### 3. dma_reg_handler.v (DMA Register Interface)

**Role:** Convert AXI4 register writes/reads ↔ DMA descriptor/status signals

**Ports:**
```
input clk, input rst

// AXI4 register interface (from decoder) - simplified reg protocol
input [7:0] reg_addr
input [31:0] reg_wdata
output [31:0] reg_rdata
input reg_write_en, reg_read_en
output reg_write_done, reg_read_done

// DMA Descriptor outputs (to axi_dma)
output [15:0] dma_read_addr, dma_read_len
output [7:0] dma_read_tag
output dma_read_valid

output [15:0] dma_write_addr, dma_write_len
output [7:0] dma_write_tag
output dma_write_valid

// DMA Status inputs (from axi_dma)
input dma_read_done, dma_write_done
input [7:0] dma_read_error, dma_write_error

// Control signals
output dma_enable, dma_abort
input [1:0] dma_status_rd, dma_status_wr // feedback for status polling
```

**Register storage:** 8 internal registers (READ_ADDR, READ_LEN, READ_TAG, WRITE_ADDR, WRITE_LEN, WRITE_TAG, CONTROL)

**Read Map:**
- 0x20: READ_STATUS ← {dma_read_error[7:0], 7'b0, dma_read_done}
- 0x24: WRITE_STATUS ← {dma_write_error[7:0], 7'b0, dma_write_done}
- 0x00–0x1C: Return corresponding stored registers

**Write Map:**
- 0x00–0x1C: Store register value
- Write to READ_CTRL[0]=1 or WRITE_CTRL[0]=1 triggers descriptor valid pulse for that channel
- 0x28: CONTROL ← {enable, abort, reserved}

---

## Connection Logic (memory_controller.v)

```
Instantiate:
  - axi4_decoder (AXI4 split logic)
  - dma_reg_handler (register + descriptor logic)
  - axi_dma (from existing IP)
  - axi_dp_ram (from existing IP)

Connect decoder outputs:
  → If RAM: drive RAM port A with full AXI4 signals
  → If REG: drive reg_handler with simplified write/read handshake

Connect reg_handler outputs:
  → dma_read_addr/len/tag/valid → axi_dma descriptor input
  → dma_write_addr/len/tag/valid → axi_dma descriptor input

Connect axi_dma:
  → descriptor inputs ← reg_handler outputs
  → status outputs → reg_handler inputs
  → AXI4 master (m_axi_*) → RAM port B

Connect RAM:
  → s_axi_a_* (port A) ← decoder AXI4 routed signals
  → s_axi_b_* (port B) ← axi_dma AXI4 master signals
```

---

## Data Flow Example

### External master writes to RAM at 0x1000:
1. AXI4 write: AWADDR=0x1000, WDATA=0xDEADBEEF
2. Decoder: addr[19:16]=0 → route to RAM port A
3. RAM port A: Write 0xDEADBEEF to address 0x1000
4. Response back to master

### External master submits read descriptor:
1. AXI4 write: AWADDR=0x10000 (READ_ADDR), WDATA=0x2000
2. Decoder: addr[19:16]=1 → route to reg_handler
3. reg_handler: Store READ_ADDR←0x2000
4. AXI4 write: AWADDR=0x10004 (READ_LEN), WDATA=0x0100 (256 bytes)
5. reg_handler: Store READ_LEN←0x0100
6. AXI4 write: AWADDR=0x1000C (READ_CTRL), WDATA=0x01 (trigger)
7. reg_handler: Pulse dma_read_valid for one cycle
8. axi_dma: Accepts descriptor, starts DMA read operation

### DMA operation completes:
1. axi_dma: Sets read_done signal
2. reg_handler: Latches read_done in READ_STATUS register
3. External master reads: AXI4 read at 0x10020 (READ_STATUS)
4. reg_handler: Returns {error_code, 7'b0, done_flag}
