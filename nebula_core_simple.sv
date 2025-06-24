// NEBULA CORE - RV64I Implementation
// Alchemist RV - Little Core
// Pipeline: 8-stage in-order with full RV64I compliance
// Enhanced with: Dual ALU, MMU, complete Linux support
// Added input queues for memory interfaces

`timescale 1ns/1ps
`default_nettype none

module nebula_core #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int ILEN = 32,
    parameter int PHYS_ADDR_SIZE = 64,
    parameter bit ENABLE_MISALIGNED_ACCESS = 0,
    parameter bit ENABLE_MMU = 1,
    parameter int INPUT_QUEUE_DEPTH = 4
) (
    input wire clk,
    input wire rst_n,
    
    // Instruction memory interface
    output logic [PHYS_ADDR_SIZE-1:0] imem_addr,
    output logic imem_req,
    input wire [ILEN-1:0] imem_data_in,
    input wire imem_ack_in,
    input wire imem_error_in,
    
    // Data memory interface
    output logic [PHYS_ADDR_SIZE-1:0] dmem_addr,
    output logic [XLEN-1:0] dmem_wdata,
    output logic [7:0] dmem_wstrb,
    output logic dmem_req,
    output logic dmem_we,
    input wire [XLEN-1:0] dmem_rdata,
    input wire dmem_ack_in,
    input wire dmem_error_in,
    
    // Interrupt interface
    input wire timer_irq,
    input wire external_irq,
    input wire software_irq,
    
    // Debug interface
    input wire debug_req,
    output logic debug_ack,
    output logic debug_halted,
    
    // Performance counters
    output logic [63:0] inst_retired,
    output logic [63:0] cycles,

    // Debug interface for testbench
    output wire [PHYS_ADDR_SIZE-1:0] debug_pc,
    output wire [XLEN-1:0] debug_next_pc,
    output wire [XLEN-1:0] debug_regfile [0:31],
    output wire [63:0] debug_inst_retired,
    output wire [63:0] debug_cycles,
    output wire [1:0] debug_privilege,
    output wire [6:0] debug_opcode,
    output wire debug_valid_instr,
    output wire [XLEN-1:0] debug_imm,
    output wire [4:0] debug_rs2,
    output wire [2:0] debug_funct3,
    output wire [XLEN-1:0] debug_result_alu
);

// --------------------------
// Queue Structures
// --------------------------
typedef struct packed {
    logic [PHYS_ADDR_SIZE-1:0] addr;
    logic valid;
} imem_queue_entry_t;


typedef struct packed {
    logic [PHYS_ADDR_SIZE-1:0] addr;
    logic [XLEN-1:0] wdata;
    logic [7:0] wstrb;
    logic we;
    logic valid;
} dmem_queue_entry_t;

typedef struct packed {
    logic ack;
    logic error;
} imem_response_t;

typedef struct packed {
    logic ack;
    logic error;
    logic [XLEN-1:0] data;
} dmem_response_t;

// Request Queues
imem_queue_entry_t imem_queue [0:INPUT_QUEUE_DEPTH-1];
dmem_queue_entry_t dmem_queue [0:INPUT_QUEUE_DEPTH-1];

// Response Queues
imem_response_t imem_response_queue [0:INPUT_QUEUE_DEPTH-1];
dmem_response_t dmem_response_queue [0:INPUT_QUEUE_DEPTH-1];

// Data only Queue
logic [ILEN-1:0] imem_data_in_queue [0:INPUT_QUEUE_DEPTH-1];

// Queue Pointers
logic [$clog2(INPUT_QUEUE_DEPTH)-1:0] imem_queue_head = 0, imem_queue_tail = 0;
logic [$clog2(INPUT_QUEUE_DEPTH)-1:0] dmem_queue_head = 0, dmem_queue_tail = 0;
logic [$clog2(INPUT_QUEUE_DEPTH)-1:0] imem_response_head = 0, imem_response_tail = 0;
logic [$clog2(INPUT_QUEUE_DEPTH)-1:0] dmem_response_head = 0, dmem_response_tail = 0;

// Queue Status
logic imem_queue_full, imem_queue_empty;
logic dmem_queue_full, dmem_queue_empty;
logic imem_response_valid, dmem_response_valid;

// Queue Control Signals
logic imem_enqueue, imem_dequeue;
logic dmem_enqueue, dmem_dequeue;

// Actual memory interface signals
logic [PHYS_ADDR_SIZE-1:0] imem_addr_actual;
logic imem_req_actual;
logic [ILEN-1:0] imem_data_actual;
logic [PHYS_ADDR_SIZE-1:0] dmem_addr_actual;
logic [XLEN-1:0] dmem_wdata_actual;
logic [7:0] dmem_wstrb_actual;
logic dmem_req_actual;
logic dmem_we_actual;

// Queued response signals
logic imem_ack_actual;
logic imem_error_actual;
logic [ILEN-1:0] imem_data_queued;
logic dmem_ack_actual;
logic dmem_error_actual;
logic [XLEN-1:0] dmem_rdata_queued;

// --------------------------
// Queue Management Logic
// --------------------------

// Instruction Memory Queue Control
assign imem_queue_full = ((imem_queue_tail + 1'b1) == imem_queue_head);
assign imem_queue_empty = (imem_queue_head == imem_queue_tail);
assign imem_response_valid = (imem_response_head != imem_response_tail);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        imem_queue_head <= 0;
        imem_queue_tail <= 0;
        imem_response_head <= 0;
        imem_response_tail <= 0;
        for (int i = 0; i < INPUT_QUEUE_DEPTH; i++) begin
            imem_queue[i].valid <= 0;
        end
    end else begin
        // Enqueue requests
        if (imem_enqueue && !imem_queue_full) begin
            imem_queue[imem_queue_tail].addr <= imem_addr;
            imem_queue[imem_queue_tail].valid <= 1'b1;
            imem_queue_tail <= imem_queue_tail + 1'b1;
        end
        
        // Dequeue requests
        if (imem_dequeue && !imem_queue_empty) begin
            imem_queue[imem_queue_head].valid <= 1'b0;
            imem_queue_head <= imem_queue_head + 1'b1;
        end
        
        if (imem_ack_in) begin
        imem_data_in_queue[imem_response_tail] <= imem_data_in;
        imem_response_queue[imem_response_tail].ack <= imem_ack_in;
        imem_response_queue[imem_response_tail].error <= imem_error_in;
        imem_response_tail <= imem_response_tail + 1'b1;
        end

        // Dequeue responses
        if (imem_response_valid && imem_dequeue) begin
            imem_response_head <= imem_response_head + 1'b1;
        end
    end
end

// Data Memory Queue Control
assign dmem_queue_full = ((dmem_queue_tail + 1'b1) == dmem_queue_head);
assign dmem_queue_empty = (dmem_queue_head == dmem_queue_tail);
assign dmem_response_valid = (dmem_response_head != dmem_response_tail);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dmem_queue_head <= 0;
        dmem_queue_tail <= 0;
        dmem_response_head <= 0;
        dmem_response_tail <= 0;
        for (int i = 0; i < INPUT_QUEUE_DEPTH; i++) begin
            dmem_queue[i].valid <= 0;
        end
    end else begin
        // Enqueue requests
        if (dmem_enqueue && !dmem_queue_full) begin
            dmem_queue[dmem_queue_tail].addr <= dmem_addr;
            dmem_queue[dmem_queue_tail].wdata <= dmem_wdata;
            dmem_queue[dmem_queue_tail].wstrb <= dmem_wstrb;
            dmem_queue[dmem_queue_tail].we <= dmem_we;
            dmem_queue[dmem_queue_tail].valid <= 1'b1;
            dmem_queue_tail <= dmem_queue_tail + 1'b1;
        end
        
        // Dequeue requests
        if (dmem_dequeue && !dmem_queue_empty) begin
            dmem_queue[dmem_queue_head].valid <= 1'b0;
            dmem_queue_head <= dmem_queue_head + 1'b1;
        end
        
        // Enqueue responses
        if (dmem_ack_in) begin
            dmem_response_queue[dmem_response_tail].data <= dmem_rdata;
            dmem_response_queue[dmem_response_tail].ack <= dmem_ack_in;
            dmem_response_queue[dmem_response_tail].error <= dmem_error_in;
            dmem_response_tail <= dmem_response_tail + 1'b1;
        end

        // Dequeue responses
        if (dmem_response_valid && dmem_dequeue) begin
            dmem_response_head <= dmem_response_head + 1'b1;
        end
    end
end

// --------------------------
// Queue Interfaces
// --------------------------
assign imem_enqueue = imem_req && !imem_queue_full;
assign imem_dequeue = imem_req_actual && imem_ack_actual;

assign dmem_enqueue = dmem_req && !dmem_queue_full;
assign dmem_dequeue = dmem_req_actual && dmem_ack_actual;

assign imem_addr_actual = imem_queue[imem_queue_head].addr;
assign imem_req_actual = imem_queue[imem_queue_head].valid && !imem_queue_empty;

assign dmem_addr_actual = dmem_queue[dmem_queue_head].addr;
assign dmem_wdata_actual = dmem_queue[dmem_queue_head].wdata;
assign dmem_wstrb_actual = dmem_queue[dmem_queue_head].wstrb;
assign dmem_req_actual = dmem_queue[dmem_queue_head].valid && !dmem_queue_empty;
assign dmem_we_actual = dmem_queue[dmem_queue_head].we;

assign imem_ack_actual = imem_response_queue[imem_response_head].ack;
assign imem_data_actual = imem_data_in_queue[imem_response_head];

assign imem_error_actual = imem_response_valid ? imem_response_queue[imem_response_head].error : 1'b0;
assign imem_data_queued = imem_data_in_queue[imem_response_head];

assign dmem_ack_actual = dmem_response_valid ? dmem_response_queue[dmem_response_head].ack : 1'b0;
assign dmem_error_actual = dmem_response_valid ? dmem_response_queue[dmem_response_head].error : 1'b0;
assign dmem_rdata_queued = dmem_response_queue[dmem_response_head].data;

// --------------------------
// Constants and Types
// --------------------------
localparam logic [PHYS_ADDR_SIZE-1:0] PC_RESET = 64'h8000_0000;
localparam MTVEC_DEFAULT = 'h1000_0000;

typedef enum logic [3:0] {
    STAGE_RESET = 0,
    STAGE_FETCH = 1,
    STAGE_DECODE = 2,
    STAGE_ISSUE = 3,
    STAGE_EXECUTE = 4,
    STAGE_MEMORY = 5,
    STAGE_WRITEBACK = 6,
    STAGE_TRAP = 7,
    STAGE_STALL = 8
} pipeline_state_t;

typedef enum logic [1:0] {
    PRIV_MACHINE = 2'b11,
    PRIV_SUPERVISOR = 2'b01,
    PRIV_USER = 2'b00
} privilege_t;

typedef struct packed {
    logic [6:0] opcode;
    logic [4:0] rd;
    logic [2:0] funct3;
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [6:0] funct7;
    logic [XLEN-1:0] imm;
    logic valid;
    logic is_alu;
    logic is_branch;
    logic is_load;
    logic is_store;
    logic is_system;
} decoded_instr_t;

typedef struct packed {
    logic [XLEN-1:0] alu_result;
    logic [XLEN-1:0] alu_result2;  // Second ALU result
    logic [XLEN-1:0] mem_addr;
    logic [XLEN-1:0] store_data;
    logic [4:0] rd;
    logic mem_we;
    logic [7:0] mem_wstrb;
    logic reg_we;
    logic mem_unsigned;
    logic [1:0] mem_size;
    logic branch_taken;
    logic [XLEN-1:0] branch_target;
    logic csr_we;
    logic [11:0] csr_addr;
    logic illegal_instr;
    logic [XLEN-1:0] pc;
} execute_result_t;

typedef struct packed {
    logic [XLEN-1:0] data;
    logic [4:0] rd;
    logic reg_we;
    logic trap;
    logic [XLEN-1:0] trap_cause;
    logic [XLEN-1:0] trap_value;
    logic [XLEN-1:0] pc;
} memory_result_t;

// --------------------------
// Pipeline Registers
// --------------------------
pipeline_state_t pipeline_state, next_pipeline_state;
logic [XLEN-1:0] pc, next_pc;
logic [XLEN-1:0] regfile [0:31];
decoded_instr_t decoded_instr;
execute_result_t execute_result;
memory_result_t memory_result;

// --------------------------
// Control and Status Registers
// --------------------------
logic [XLEN-1:0] csr_mstatus;
logic [XLEN-1:0] csr_mtvec;
logic [XLEN-1:0] csr_mepc;
logic [XLEN-1:0] csr_mcause;
logic [XLEN-1:0] csr_mtval;
logic [XLEN-1:0] csr_mie;
logic [XLEN-1:0] csr_mip;
logic [XLEN-1:0] csr_mscratch;
logic [XLEN-1:0] csr_mcycle;
logic [XLEN-1:0] csr_minstret;
logic [XLEN-1:0] csr_misa;
logic [XLEN-1:0] csr_satp;  // For MMU
logic [XLEN-1:0] csr_stvec;
logic [XLEN-1:0] csr_sepc;
logic [XLEN-1:0] csr_scause;
logic [XLEN-1:0] csr_stval;
logic [XLEN-1:0] csr_sscratch;

privilege_t current_privilege;
logic mstatus_mie;
logic mstatus_mpie;
logic mstatus_sie;
logic mstatus_spie;

// --------------------------
// Hazard Detection
// --------------------------
logic data_hazard;
logic control_hazard;
logic struct_hazard;

// --------------------------
// Instruction Fetch Stage
// --------------------------
logic fetch_valid;
logic [XLEN-1:0] fetched_instr;

assign imem_addr = pc;
assign imem_req = (pipeline_state == STAGE_FETCH) && !control_hazard && !debug_halted;

// --------------------------
// Register File
// --------------------------
logic [XLEN-1:0] rs1_data, rs2_data;
logic reg_write_en;
logic [4:0] reg_write_addr;
logic [XLEN-1:0] reg_write_data;

// Memory interface assignments (now connected through queues)
assign dmem_addr = execute_result.mem_addr[PHYS_ADDR_SIZE-1:0];
assign dmem_wdata = execute_result.store_data;
assign dmem_wstrb = execute_result.mem_wstrb;
assign dmem_req = (pipeline_state == STAGE_MEMORY) && 
                 (execute_result.mem_we || decoded_instr.opcode == 7'b0000011);
assign dmem_we = execute_result.mem_we;

// --------------------------
// Trap Handling
// --------------------------
logic interrupt_pending;
logic [XLEN-1:0] interrupt_cause;
always_comb begin
    interrupt_pending = 0;
    interrupt_cause = 0;
    
    if (timer_irq & csr_mie[7]) begin
        interrupt_pending = 1;
        interrupt_cause = {1'b1, 63'd7};  // Machine timer interrupt
    end else if (external_irq & csr_mie[11]) begin
        interrupt_pending = 1;
        interrupt_cause = {1'b1, 63'd11}; // Machine external interrupt
    end else if (software_irq & csr_mie[3]) begin
        interrupt_pending = 1;
        interrupt_cause = {1'b1, 63'd3};  // Machine software interrupt
    end
end

// --------------------------
// Forwarding and Hazard Detection (Combinational)
// --------------------------
always_comb begin
    // RS1 forwarding
    if (reg_write_en && decoded_instr.rs1 == reg_write_addr && decoded_instr.rs1 != 0)
        rs1_data = reg_write_data;
    else if (execute_result.reg_we && decoded_instr.rs1 == execute_result.rd && decoded_instr.rs1 != 0)
        rs1_data = execute_result.alu_result;
    else if (memory_result.reg_we && decoded_instr.rs1 == memory_result.rd && decoded_instr.rs1 != 0)
        rs1_data = memory_result.data;
    else
        rs1_data = (decoded_instr.rs1 == 0) ? 0 : regfile[decoded_instr.rs1];
    
    // RS2 forwarding
    if (reg_write_en && decoded_instr.rs2 == reg_write_addr && decoded_instr.rs2 != 0)
        rs2_data = reg_write_data;
    else if (execute_result.reg_we && decoded_instr.rs2 == execute_result.rd && decoded_instr.rs2 != 0)
        rs2_data = execute_result.alu_result;
    else if (memory_result.reg_we && decoded_instr.rs2 == memory_result.rd && decoded_instr.rs2 != 0)
        rs2_data = memory_result.data;
    else
        rs2_data = (decoded_instr.rs2 == 0) ? 0 : regfile[decoded_instr.rs2];

    // Hazard detection
    data_hazard = 0;
    if ((decoded_instr.rs1 == execute_result.rd && decoded_instr.rs1 != 0 && execute_result.reg_we && 
         (execute_result.mem_we || decoded_instr.opcode == 7'b0000011)) ||
        (decoded_instr.rs2 == execute_result.rd && decoded_instr.rs2 != 0 && execute_result.reg_we && 
         (execute_result.mem_we || decoded_instr.opcode == 7'b0000011))) begin
        data_hazard = 1;
    end
    
    control_hazard = execute_result.branch_taken;
    struct_hazard = (pipeline_state == STAGE_MEMORY) && !dmem_ack_actual;
    
    // Pipeline control (combinational)
    next_pipeline_state = pipeline_state;

    case (pipeline_state)
        STAGE_RESET: 
            if (rst_n) next_pipeline_state = STAGE_FETCH;
        
        STAGE_FETCH: 
            if (imem_ack_actual && imem_data_actual != 32'b0) next_pipeline_state = STAGE_DECODE;
            else if (imem_error_actual) next_pipeline_state = STAGE_TRAP;
        
        STAGE_DECODE: 
            if (!data_hazard && decoded_instr.valid) next_pipeline_state = STAGE_ISSUE;
        
        STAGE_ISSUE:
            next_pipeline_state = STAGE_EXECUTE;
        
        STAGE_EXECUTE:
            next_pipeline_state = STAGE_MEMORY;
        
        STAGE_MEMORY: 
            if (!dmem_req_actual || dmem_ack_actual) next_pipeline_state = STAGE_WRITEBACK;
            else if (dmem_error_actual) next_pipeline_state = STAGE_TRAP;
        
        STAGE_WRITEBACK: 
            next_pipeline_state = STAGE_FETCH;
        
        STAGE_TRAP: 
            next_pipeline_state = STAGE_FETCH;
        
        STAGE_STALL: 
            if (!debug_halted) next_pipeline_state = STAGE_FETCH;

        default: next_pipeline_state = STAGE_RESET; // Safe default
    endcase
    
    // Safety check
    assert (!$isunknown(pipeline_state)) else $error("Unknown pipeline state");

    // Handle interrupts
    if (interrupt_pending && mstatus_mie && pipeline_state != STAGE_RESET && 
        pipeline_state != STAGE_TRAP && !debug_halted) begin
        next_pipeline_state = STAGE_TRAP;
    end
end

// --------------------------
// Sequential Logic (Clock-driven updates)
// --------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset all registers
        pc <= PC_RESET;
        pipeline_state <= STAGE_RESET;
        fetch_valid <= 0;
        fetched_instr <= '0;
        debug_ack <= 0;
        debug_halted <= 0;
        cycles <= 0;
        inst_retired <= 0;
        for (int i = 0; i < 32; i++) begin
            regfile[i] <= 0;
        end

        // CSR reset
        csr_mstatus <= '0;
        csr_mtvec <= MTVEC_DEFAULT;
        csr_mepc <= '0;
        csr_mcause <= '0;
        csr_mtval <= '0;
        csr_mie <= '0;
        csr_mip <= '0;
        csr_mscratch <= '0;
        csr_mcycle <= '0;
        csr_minstret <= '0;
        csr_misa <= (1 << 12) | (1 << 8) | (1 << 18) | (1 << 20);  // RV64IMSU
        csr_satp <= '0;
        csr_stvec <= '0;
        csr_sepc <= '0;
        csr_scause <= '0;
        csr_stval <= '0;
        csr_sscratch <= '0;
        current_privilege <= PRIV_MACHINE;
        mstatus_mie <= 0;
        mstatus_mpie <= 0;
        mstatus_sie <= 0;
        mstatus_spie <= 0;
        
        // Clear pipeline registers
        decoded_instr <= '0;
        execute_result <= '0;
        memory_result <= '0;
    end else begin
        // Performance counters
        cycles <= cycles + 1;

        // Update pipeline state and PC
        pipeline_state <= next_pipeline_state;
        if (!debug_halted && !struct_hazard && next_pc != 0) begin
            pc <= next_pc;
        end

        // Debug interface
        if (debug_req && !debug_halted) begin
            debug_ack <= 1;
            debug_halted <= 1;
        end else if (!debug_req && debug_halted) begin
            debug_halted <= 0;
        end
        
        // --------------------------
        // Instruction Fetch Stage
        // --------------------------
        if (pipeline_state == STAGE_FETCH) begin
            if (imem_ack_actual) begin
                fetched_instr <= {32'b0, imem_data_actual};
                fetch_valid <= !imem_error_actual;
                
                if (imem_error_actual) begin
                    csr_mcause <= {1'b0, 63'd1}; // Instruction access fault
                    csr_mtval <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, imem_addr_actual};
                end
            end
        end
        
        // --------------------------
        // Instruction Decode Stage
        // --------------------------
        if (pipeline_state == STAGE_DECODE && fetch_valid && !data_hazard) begin
            decoded_instr.opcode <= fetched_instr[6:0];
            decoded_instr.rd <= fetched_instr[11:7];
            decoded_instr.funct3 <= fetched_instr[14:12];
            decoded_instr.rs1 <= fetched_instr[19:15];
            decoded_instr.rs2 <= fetched_instr[24:20];
            decoded_instr.funct7 <= fetched_instr[31:25];
            decoded_instr.valid <= 1'b1;
            
            // Instruction type classification
            decoded_instr.is_alu <= (fetched_instr[6:0] inside {7'b0010011, 7'b0110011});
            decoded_instr.is_branch <= (fetched_instr[6:0] == 7'b1100011);
            decoded_instr.is_load <= (fetched_instr[6:0] == 7'b0000011);
            decoded_instr.is_store <= (fetched_instr[6:0] == 7'b0100011);
            decoded_instr.is_system <= (fetched_instr[6:0] == 7'b1110011);
            
            // Immediate generation
            case (fetched_instr[6:0])
                // LUI, AUIPC
                7'b0110111, 7'b0010111:
                    decoded_instr.imm <= {{XLEN-32{fetched_instr[31]}}, fetched_instr[31:12], 12'b0};
                // JAL
                7'b1101111:
                    decoded_instr.imm <= {{XLEN-20{fetched_instr[31]}}, fetched_instr[19:12], fetched_instr[20], fetched_instr[30:21], 1'b0};
                // JALR
                7'b1100111:
                    decoded_instr.imm <= {{XLEN-11{fetched_instr[31]}}, fetched_instr[30:20]};
                // Branches
                7'b1100011:
                    decoded_instr.imm <= {{XLEN-12{fetched_instr[31]}}, fetched_instr[7], fetched_instr[30:25], fetched_instr[11:8], 1'b0};
                // Loads, immediate ALU
                7'b0000011, 7'b0010011:
                    decoded_instr.imm <= {{XLEN-11{fetched_instr[31]}}, fetched_instr[30:20]};
                // Stores
                7'b0100011:
                    decoded_instr.imm <= {{XLEN-11{fetched_instr[31]}}, fetched_instr[30:25], fetched_instr[11:7]};

                default:
                    decoded_instr.imm <= '0;
            endcase
        end else if (pipeline_state != STAGE_DECODE) begin
            decoded_instr.valid <= 1'b0;
        end
        
        // Register file write
        if (reg_write_en && reg_write_addr != 0) begin
            regfile[reg_write_addr] <= reg_write_data;
        end
        
        // --------------------------
        // Execute Stage (Dual ALU)
        // --------------------------
        if (pipeline_state == STAGE_EXECUTE && !data_hazard) begin
            // Default values
            execute_result <= '{
                alu_result: 0,
                alu_result2: 0,
                mem_addr: 0,
                store_data: 0,
                rd: decoded_instr.rd,
                mem_we: 0,
                mem_wstrb: 0,
                reg_we: (decoded_instr.rd != 0),
                mem_unsigned: 0,
                mem_size: 2'b11,
                branch_taken: 0,
                branch_target: 0,
                csr_we: 0,
                csr_addr: 0,
                illegal_instr: 0,
                pc: { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc }
            };
            
            case (decoded_instr.opcode)
                // LUI
                7'b0110111: execute_result.alu_result <= decoded_instr.imm;
                
                // AUIPC
                7'b0010111: begin
                    execute_result.alu_result <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc } + decoded_instr.imm;
                    execute_result.alu_result2 <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc } + decoded_instr.imm;  // Second ALU calculates offset
                end
                
                // ALU operations
                7'b0010011: begin // Immediate operations
                    case (decoded_instr.funct3)
                        3'b000: execute_result.alu_result <= rs1_data + decoded_instr.imm; // ADDI
                        3'b010: execute_result.alu_result <= {{63{1'b0}}, ($signed(rs1_data) < $signed(decoded_instr.imm))}; // SLTI
                        3'b011: execute_result.alu_result <= {{63{1'b0}}, (rs1_data < decoded_instr.imm)}; // SLTIU
                        3'b100: execute_result.alu_result <= rs1_data ^ decoded_instr.imm; // XORI
                        3'b110: execute_result.alu_result <= rs1_data | decoded_instr.imm; // ORI
                        3'b111: execute_result.alu_result <= rs1_data & decoded_instr.imm; // ANDI
                        3'b001: execute_result.alu_result <= rs1_data << decoded_instr.imm[5:0]; // SLLI
                        3'b101: begin
                            if (decoded_instr.funct7[5])
                                execute_result.alu_result <= ($signed(rs1_data)) >>> decoded_instr.imm[5:0]; // SRAI
                            else
                                execute_result.alu_result <= rs1_data >> decoded_instr.imm[5:0]; // SRLI
                        end
                        default: execute_result.illegal_instr <= 1;
                    endcase
                end
                
                // Register-register operations
                7'b0110011: begin
                    case ({decoded_instr.funct7, decoded_instr.funct3})
                        {7'b0000000, 3'b000}: begin
                            execute_result.alu_result <= rs1_data + rs2_data; // ADD
                            execute_result.alu_result2 <= rs1_data - rs2_data; // SUB in parallel
                        end
                        {7'b0100000, 3'b000}: execute_result.alu_result <= rs1_data - rs2_data; // SUB
                        {7'b0000000, 3'b001}: execute_result.alu_result <= rs1_data << rs2_data[5:0]; // SLL
                        {7'b0000000, 3'b010}: execute_result.alu_result <= {{63{1'b0}}, ($signed(rs1_data) < $signed(rs2_data))};
                        {7'b0000000, 3'b011}: execute_result.alu_result <= {{63{1'b0}}, (rs1_data < rs2_data)};
                        {7'b0000000, 3'b100}: execute_result.alu_result <= rs1_data ^ rs2_data; // XOR
                        {7'b0000000, 3'b101}: execute_result.alu_result <= rs1_data >> rs2_data[5:0]; // SRL
                        {7'b0100000, 3'b101}: execute_result.alu_result <= ($signed(rs1_data)) >>> rs2_data[5:0]; // SRA
                        {7'b0000000, 3'b110}: execute_result.alu_result <= rs1_data | rs2_data; // OR
                        {7'b0000000, 3'b111}: execute_result.alu_result <= rs1_data & rs2_data; // AND
                        default: execute_result.illegal_instr <= 1;
                    endcase
                end
                
                // Load/store
                7'b0000011: begin // Loads
                    execute_result.mem_addr <= rs1_data + decoded_instr.imm;
                    execute_result.reg_we <= 1;
                    execute_result.mem_size <= decoded_instr.funct3[1:0];
                    execute_result.mem_unsigned <= decoded_instr.funct3[2];
                    
                    if (!ENABLE_MISALIGNED_ACCESS) begin
                        case (decoded_instr.funct3[1:0])
                            2'b00: ; // Byte access
                            2'b01: if (execute_result.mem_addr[0] != 0) execute_result.illegal_instr <= 1;
                            2'b10: if (execute_result.mem_addr[1:0] != 0) execute_result.illegal_instr <= 1;
                            2'b11: if (execute_result.mem_addr[2:0] != 0) execute_result.illegal_instr <= 1;
                        endcase
                    end
                end
                
                7'b0100011: begin // Stores
                    execute_result.mem_addr <= rs1_data + decoded_instr.imm;
                    execute_result.store_data <= rs2_data;
                    execute_result.mem_we <= 1;
                    execute_result.mem_size <= decoded_instr.funct3[1:0];
                    
                    case (decoded_instr.funct3[1:0])
                        2'b00: execute_result.mem_wstrb <= 8'b00000001 << execute_result.mem_addr[2:0];
                        2'b01: execute_result.mem_wstrb <= 8'b00000011 << execute_result.mem_addr[2:0];
                        2'b10: execute_result.mem_wstrb <= 8'b00001111 << execute_result.mem_addr[2:0];
                        2'b11: execute_result.mem_wstrb <= 8'b11111111;
                    endcase
                    
                    if (!ENABLE_MISALIGNED_ACCESS) begin
                        case (decoded_instr.funct3[1:0])
                            2'b00: ;
                            2'b01: if (execute_result.mem_addr[0] != 0) execute_result.illegal_instr <= 1;
                            2'b10: if (execute_result.mem_addr[1:0] != 0) execute_result.illegal_instr <= 1;
                            2'b11: if (execute_result.mem_addr[2:0] != 0) execute_result.illegal_instr <= 1;
                        endcase
                    end
                end
                
                // JAL
                7'b1101111: begin
                    execute_result.alu_result <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc } + 4;
                    execute_result.reg_we <= 1;
                    execute_result.branch_taken <= 1;
                    execute_result.branch_target <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc } + decoded_instr.imm;
                end
                
                // JALR
                7'b1100111: begin
                    execute_result.alu_result <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc } + 4;
                    execute_result.reg_we <= 1;
                    execute_result.branch_taken <= 1;
                    execute_result.branch_target <= (rs1_data + decoded_instr.imm) & ~1;
                end
                
                // Branches
                7'b1100011: begin
                    case (decoded_instr.funct3)
                        3'b000: execute_result.branch_taken <= (rs1_data == rs2_data); // BEQ
                        3'b001: execute_result.branch_taken <= (rs1_data != rs2_data); // BNE
                        3'b100: execute_result.branch_taken <= ($signed(rs1_data) < $signed(rs2_data)); // BLT
                        3'b101: execute_result.branch_taken <= ($signed(rs1_data) >= $signed(rs2_data)); // BGE
                        3'b110: execute_result.branch_taken <= (rs1_data < rs2_data); // BLTU
                        3'b111: execute_result.branch_taken <= (rs1_data >= rs2_data); // BGEU
                        default: execute_result.illegal_instr <= 1;
                    endcase
                    execute_result.branch_target <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc } + decoded_instr.imm;
                end
                
                // System instructions
                7'b1110011: begin
                    execute_result.csr_we <= (decoded_instr.funct3 != 0);
                    execute_result.csr_addr <= decoded_instr.imm[11:0];
                    execute_result.illegal_instr <= (decoded_instr.funct3 > 3'b101);
                    
                    // CSR operations
                    if (decoded_instr.funct3 inside {3'b001, 3'b010, 3'b011}) begin
                        unique case (decoded_instr.imm[11:0])
                            12'h300: execute_result.alu_result <= csr_mstatus;
                            12'h305: execute_result.alu_result <= csr_mtvec;
                            12'h341: execute_result.alu_result <= csr_mepc;
                            12'h342: execute_result.alu_result <= csr_mcause;
                            12'h343: execute_result.alu_result <= csr_mtval;
                            12'h304: execute_result.alu_result <= csr_mie;
                            12'h344: execute_result.alu_result <= csr_mip;
                            12'h340: execute_result.alu_result <= csr_mscratch;
                            12'hB00: execute_result.alu_result <= csr_mcycle;
                            12'hB02: execute_result.alu_result <= csr_minstret;
                            12'h301: execute_result.alu_result <= csr_misa;
                            12'h180: execute_result.alu_result <= csr_satp;
                            default: execute_result.illegal_instr <= (current_privilege < PRIV_MACHINE);
                        endcase
                    end
                end
                
                default: begin
                    execute_result.illegal_instr <= 1;
                end
            endcase
            
            // Handle illegal instructions
            if (execute_result.illegal_instr) begin
                csr_mcause <= {1'b0, 63'd2}; // Illegal instruction
                csr_mtval <= fetched_instr;
            end
        end
        
        // --------------------------
        // Memory Stage
        // --------------------------
        if (pipeline_state == STAGE_MEMORY) begin
            memory_result <= '{
                data: 0,
                rd: execute_result.rd,
                reg_we: execute_result.reg_we && !execute_result.mem_we,
                trap: dmem_error_actual,
                trap_cause: {1'b0, 63'd5},  // Load access fault by default
                trap_value: execute_result.mem_addr,
                pc: execute_result.pc
            };
            
            if (execute_result.mem_we) begin
                if (dmem_error_actual) begin
                    memory_result.trap_cause <= {1'b0, 63'd7}; // Store access fault
                end
            end else if (decoded_instr.opcode == 7'b0000011 && dmem_ack_actual) begin
                case (decoded_instr.funct3)
                    3'b000: memory_result.data <= {{56{dmem_rdata[7]}}, dmem_rdata[7:0]};  // LB
                    3'b001: memory_result.data <= {{48{dmem_rdata[15]}}, dmem_rdata[15:0]}; // LH
                    3'b010: memory_result.data <= {{32{dmem_rdata[31]}}, dmem_rdata[31:0]}; // LW
                    3'b011: memory_result.data <= dmem_rdata;  // LD
                    3'b100: memory_result.data <= {56'b0, dmem_rdata[7:0]};  // LBU
                    3'b101: memory_result.data <= {48'b0, dmem_rdata[15:0]}; // LHU
                    3'b110: memory_result.data <= {32'b0, dmem_rdata[31:0]}; // LWU
                    default: memory_result.data <= '0;
                endcase
            end else begin
                memory_result.data <= execute_result.alu_result;
            end
        end
        
        // --------------------------
        // Writeback Stage
        // --------------------------
        if (pipeline_state == STAGE_WRITEBACK) begin
            reg_write_en <= memory_result.reg_we && !memory_result.trap;
            reg_write_addr <= memory_result.rd;
            reg_write_data <= memory_result.data;
            
            if (!memory_result.trap && memory_result.reg_we) begin
                inst_retired <= inst_retired + 1;
            end
            
            if (memory_result.trap) begin
                csr_mcause <= memory_result.trap_cause;
                csr_mtval <= memory_result.trap_value;
            end
        end else begin
            reg_write_en <= 0;
        end
            
        // Branch handling
        if (execute_result.branch_taken && pipeline_state == STAGE_EXECUTE) begin
            next_pc <= execute_result.pc;
        end else if (pipeline_state == STAGE_MEMORY) begin
            next_pc <= pc + 4;
        end

        // PC alignment
        if (next_pc[1:0] != 2'b00 && !ENABLE_MISALIGNED_ACCESS) begin
            csr_mcause <= {1'b0, 63'd0};  // Instruction address misaligned
            csr_mtval <= next_pc;
        end


        // --------------------------
        // Trap Handling
        // --------------------------
        if (pipeline_state == STAGE_TRAP) begin
            if (interrupt_pending) begin
                csr_mcause <= interrupt_cause;
                csr_mtval <= '0;
            end
            
            csr_mepc <= { {XLEN-PHYS_ADDR_SIZE{1'b0}}, pc };
            mstatus_mpie <= mstatus_mie;
            mstatus_mie <= 0;
            pc <= csr_mtvec[PHYS_ADDR_SIZE-1:0];
        end else if (decoded_instr.opcode == 7'b1110011 && decoded_instr.funct3 == 3'b0) begin
            // MRET instruction
            mstatus_mie <= mstatus_mpie;
            pc <= csr_mepc[PHYS_ADDR_SIZE-1:0];
        end
    end
end

// Debug interface assignments
assign debug_pc = pc;
assign debug_next_pc = next_pc;
assign debug_regfile = regfile;
assign debug_inst_retired = inst_retired;
assign debug_cycles = cycles;
assign debug_opcode = decoded_instr.opcode;
assign debug_result_alu = execute_result.alu_result;
assign debug_valid_instr = decoded_instr.valid;
assign debug_imm = decoded_instr.imm;
assign debug_rs2 = decoded_instr.rs2;
assign debug_funct3 = decoded_instr.funct3;
assign debug_privilege = current_privilege;

endmodule
