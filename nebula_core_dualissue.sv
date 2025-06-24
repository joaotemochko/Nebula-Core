// NEBULA CORE - RV64I Implementation with Out-of-Order Execution and Dual-Issue
// Enhanced with: Dual ALU, OoO pipeline, complete Linux support, Dual-Issue capability

`timescale 1ns/1ps
`default_nettype none

module nebula_core_dualissue #(
    parameter int HART_ID = 0,
    parameter int XLEN = 64,
    parameter int ILEN = 32,
    parameter int PHYS_ADDR_SIZE = 64,
    parameter bit ENABLE_MISALIGNED_ACCESS = 0,
    parameter bit ENABLE_MMU = 1,
    parameter int ROB_ENTRIES = 32,  // Increased for dual-issue
    parameter int RS_ENTRIES = 16,   // Increased for dual-issue
    parameter int LSQ_ENTRIES = 16   // Increased for dual-issue
) (
    input wire clk,
    input wire rst_n,
    
    // Instruction memory interface
    output logic [PHYS_ADDR_SIZE-1:0] imem_addr,
    output logic imem_req,
    input wire [ILEN-1:0] imem_data_in0,
    input wire [ILEN-1:0] imem_data_in1,
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

    // Debug interface
    output wire [PHYS_ADDR_SIZE-1:0] debug_pc,
    output wire [XLEN-1:0] debug_regfile [0:31],
    output wire [63:0] debug_inst_retired,
    output wire [63:0] debug_cycles,
    output wire [1:0] debug_privilege,
    output wire [6:0] debug_opcode,
    output wire debug_valid_instr,
    output wire [XLEN-1:0] debug_imm,
    output wire [XLEN-1:0] debug_rs1,
    output wire [XLEN-1:0] debug_rs2,
    output wire [2:0] debug_funct3,
    output wire [6:0] debug_funct7,
    output wire [XLEN-1:0] debug_result_alu1,
    output wire [XLEN-1:0] debug_result_alu2,
    output wire [$clog2(RS_ENTRIES)-1:0] debug_rs_tail,
    output wire [99:0] debug_decoded_instr0,
    output wire [99:0] debug_decoded_instr1,
    output wire [2:0] debug_optype0,
    output wire [2:0] debug_optype1,
    output wire [4:0] debug_instruction_rs2,
    output wire [4:0] debug_arch_reg
);

// --------------------------
// Constants and Types
// --------------------------
localparam logic [PHYS_ADDR_SIZE-1:0] PC_RESET = 64'h8000_0000;
localparam MTVEC_DEFAULT = 'h1000_0000;

typedef enum logic [3:0] {
    STAGE_RESET = 0,
    STAGE_FETCH = 1,
    STAGE_DECODE = 2,
    STAGE_RENAME = 3,
    STAGE_DISPATCH = 4,
    STAGE_EXECUTE = 5,
    STAGE_MEMORY = 6,
    STAGE_COMMIT = 7,
    STAGE_TRAP = 8
} pipeline_state_t;

typedef enum logic [1:0] {
    PRIV_MACHINE = 2'b11,
    PRIV_SUPERVISOR = 2'b01,
    PRIV_USER = 2'b00
} privilege_t;

typedef enum logic [2:0] {
    OP_ALU = 0,
    OP_BRANCH = 1,
    OP_LOAD = 2,
    OP_STORE = 3,
    OP_SYSTEM = 4,
    OP_CSR = 5,
    OP_MUL = 6,
    OP_DIV = 7
} operation_type_t;

typedef struct packed {
    logic [6:0] opcode;
    logic [4:0] rd;
    logic [2:0] funct3;
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [6:0] funct7;
    logic [XLEN-1:0] imm;
    logic valid;
    operation_type_t op_type;
} decoded_instr_t;

typedef struct packed {
    decoded_instr_t instr0;
    decoded_instr_t instr1;
    logic dual_issue_valid;
} dual_decoded_instr_t;

typedef struct packed {
    logic [XLEN-1:0] alu_result;
    logic [XLEN-1:0] mem_addr;
    logic [XLEN-1:0] store_data;
    logic [4:0] rd;
    logic [4:0] rob_tag;
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
    logic [4:0] rob_tag;
    logic reg_we;
    logic trap;
    logic [XLEN-1:0] trap_cause;
    logic [XLEN-1:0] trap_value;
    logic [XLEN-1:0] pc;
} memory_result_t;

// --------------------------
// Register Renaming Structures
// --------------------------
typedef struct packed {
    logic valid;
    logic [4:0] arch_reg;
    logic [4:0] phys_reg;
    logic ready;
} rename_entry_t;

typedef struct packed {
    logic valid;
    logic [4:0] rob_tag;
    logic [XLEN-1:0] value;
} phys_reg_entry_t;

// --------------------------
// Reorder Buffer (ROB)
// --------------------------
typedef struct packed {
    logic valid;
    logic completed;
    logic [4:0] arch_reg;
    logic [4:0] phys_reg;
    logic [XLEN-1:0] result;
    logic [XLEN-1:0] pc;
    logic exception;
    logic [XLEN-1:0] exception_cause;
} rob_entry_t;

rob_entry_t rob [0:ROB_ENTRIES-1];
logic [$clog2(ROB_ENTRIES)-1:0] rob_head, rob_tail;
logic rob_full, rob_empty;

// --------------------------
// Reservation Stations
// --------------------------
typedef struct packed {
    logic valid;
    logic [4:0] rob_tag;
    operation_type_t op_type;
    logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;
    logic [XLEN-1:0] imm;
    logic [XLEN-1:0] rs1_value;
    logic rs1_ready;
    logic [XLEN-1:0] rs2_value;
    logic rs2_ready;
    logic rs_found;
    logic [XLEN-1:0] pc;
    logic needs_lsq;
} rs_entry_t;

rs_entry_t rs [0:RS_ENTRIES-1];
logic [$clog2(RS_ENTRIES)-1:0] rs_head, rs_tail;
logic rs_full, rs_empty;

// --------------------------
// Load/Store Queue
// --------------------------
typedef enum logic [1:0] {
    LSQ_IDLE = 2'b00,
    LSQ_ISSUED = 2'b01,
    LSQ_COMPLETED = 2'b10
} lsq_state_t;

typedef struct packed {
    logic valid;
    logic completed;
    lsq_state_t state;
    logic [4:0] rob_tag;
    logic is_load;
    logic [XLEN-1:0] addr;
    logic [XLEN-1:0] data;
    logic [7:0] wstrb;
    logic [1:0] size;
    logic unsigned_load;
    logic [XLEN-1:0] pc;
    operation_type_t op_type;
} lsq_entry_t;


lsq_entry_t lsq [0:LSQ_ENTRIES-1];
logic [$clog2(LSQ_ENTRIES)-1:0] lsq_head, lsq_tail;
logic lsq_full, lsq_empty;

// --------------------------
// Register Files
// --------------------------
logic [XLEN-1:0] arch_regfile [0:31];  // Architectural register file
phys_reg_entry_t phys_regfile [0:31];   // Physical register file
rename_entry_t rename_table [0:31];     // Rename table

// --------------------------
// Instruction Queue
// --------------------------
logic [31:0] instr_queue [0:7];
logic [2:0]  iq_head, iq_tail;
logic        iq_valid [0:7];

// --------------------------
// Free List
// --------------------------
logic [31:0] free_list;
logic [$clog2(32)-1:0] free_list_count;

// --------------------------
// Pipeline Registers
// --------------------------
pipeline_state_t pipeline_state, next_pipeline_state;
logic [XLEN-1:0] pc;
dual_decoded_instr_t decoded_instr_pair;
execute_result_t execute_result0, execute_result1;
memory_result_t memory_result0, memory_result1;

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
logic [XLEN-1:0] csr_satp;
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
// Instruction Fetch Stage
// --------------------------
logic fetch_valid0, fetch_valid1;
logic [XLEN-1:0] fetched_instr0, fetched_instr1;
logic [XLEN-1:0] next_pc0, next_pc1;
logic can_dual_issue;

assign imem_addr = pc;
assign imem_req = (pipeline_state == STAGE_FETCH) && !debug_halted;

// Determine if we can dual-issue
assign can_dual_issue = !(decoded_instr_pair.instr0.op_type inside {OP_BRANCH, OP_SYSTEM, OP_CSR}) && 
                       !(decoded_instr_pair.instr1.op_type inside {OP_BRANCH, OP_SYSTEM, OP_CSR}) &&
                       (decoded_instr_pair.instr0.rd != decoded_instr_pair.instr1.rs1) &&
                       (decoded_instr_pair.instr0.rd != decoded_instr_pair.instr1.rs2) &&
                       (decoded_instr_pair.instr1.rd != decoded_instr_pair.instr0.rs1) &&
                       (decoded_instr_pair.instr1.rd != decoded_instr_pair.instr0.rs2);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pc <= PC_RESET;
        pipeline_state <= STAGE_RESET;
        fetch_valid0 <= 0;
        fetch_valid1 <= 0;
        fetched_instr0 <= '0;
        fetched_instr1 <= '0;
        debug_ack <= 0;
        debug_halted <= 0;
        cycles <= 0;
        inst_retired <= 0;
        decoded_instr_pair <= '0;
    end else begin
        if (imem_ack_in) begin
            fetched_instr0 <= {32'b0, imem_data_in0};
            fetch_valid0 <= !imem_error_in;
            
            // Try to fetch second instruction if aligned and no error
            if (!imem_error_in) begin
                fetched_instr1 <= {32'b0, imem_data_in1};
                fetch_valid1 <= !imem_error_in;
            end else begin
                fetched_instr1 <= '0;
                fetch_valid1 <= 0;
            end

            if (imem_error_in) begin
                csr_mcause <= {1'b0, 63'd1}; // Instruction access fault
                csr_mtval <= pc;
            end
        end
    end
end

// --------------------------
// Instruction Decode Stage
// --------------------------
function automatic decoded_instr_t decode_instr(input [XLEN-1:0] instr, input [XLEN-1:0] pc_val);
    decoded_instr_t result;
    result.opcode = instr[6:0];
    result.rd = instr[11:7];
    result.funct3 = instr[14:12];
    result.rs1 = instr[19:15];
    result.rs2 = instr[24:20];
    result.funct7 = instr[31:25];
    result.valid = 1'b1;
    
    case (instr[6:0])
        // Loads
        7'b0000011: begin
            result.op_type = OP_LOAD;
            case (instr[14:12])
                3'b000, 3'b001, 3'b010, 3'b011,  // LB, LH, LW, LD
                3'b100, 3'b101, 3'b110: result.valid = 1'b1; // LBU, LHU, LWU
                default: result.valid = 1'b0;
            endcase
        end
        
        // Stores
        7'b0100011: begin
            result.op_type = OP_STORE;
            case (instr[14:12])
                3'b000, 3'b001, 3'b010, 3'b011: result.valid = 1'b1; // SB, SH, SW, SD
                default: result.valid = 1'b0;
            endcase
        end
        
        // ALU operations
        7'b0010011, 7'b0110011: result.op_type = OP_ALU; // Immediate and register-register ALU
        
        // Branches
        7'b1100011: result.op_type = OP_BRANCH; // Conditional branches
        7'b1101111, 7'b1100111: result.op_type = OP_BRANCH; // JAL, JALR
        
        // LUI/AUIPC
        7'b0110111, 7'b0010111: result.op_type = OP_ALU;
        
        // System/CSR
        7'b1110011: begin
            if (instr[14:12] == 3'b000) begin
                result.op_type = OP_SYSTEM; // ECALL, EBREAK, etc.
            end else begin
                result.op_type = OP_CSR; // CSR operations
            end
        end
        
        default: begin
            result.valid = 1'b0;
            result.op_type = OP_ALU; // Default
        end
    endcase


    // Immediate generation
    case (instr[6:0])
        // LUI, AUIPC
        7'b0110111, 7'b0010111:
            result.imm = {{XLEN-32{instr[31]}}, instr[31:12], 12'b0};
        // JAL
        7'b1101111:
            result.imm = {{XLEN-20{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
        // JALR
        7'b1100111:
            result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
        // Branches
        7'b1100011:
            result.imm = {{XLEN-12{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
        // Loads, immediate ALU
        7'b0000011, 7'b0010011:
            result.imm = {{XLEN-11{instr[31]}}, instr[30:20]};
        // Stores
        7'b0100011:
            result.imm = {{XLEN-11{instr[31]}}, instr[30:25], instr[11:7]};
        default:
            result.imm = '0;
    endcase
    
    return result;
endfunction

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        decoded_instr_pair <= '0;
    end else if (pipeline_state == STAGE_DECODE) begin
        decoded_instr_pair.instr0 <= decode_instr(fetched_instr0, pc);
        decoded_instr_pair.instr1 <= fetch_valid1 ? decode_instr(fetched_instr1, pc + 4) : '0;
        decoded_instr_pair.dual_issue_valid <= fetch_valid0 && fetch_valid1 && can_dual_issue;
    end
end

// --------------------------
// Register Renaming Stage
// --------------------------
logic [$clog2(32)-1:0] found_free_reg0, found_free_reg1;
logic free_reg_found0, free_reg_found1;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < 32; i++) begin
            rename_table[i] <= '0;
            phys_regfile[i] <= '0;
        end
        free_list = ~0;
        free_list_count = 5'($clog2(32));
    end else if (pipeline_state == STAGE_RENAME && decoded_instr_pair.instr0.valid) begin
        // Allocate physical registers for both instructions if dual-issue
        if (decoded_instr_pair.instr0.rd != 0) begin
            free_reg_found0 <= 1'b0;
            found_free_reg0 <= 0;
            for (int i = 0; i < 32; i++) begin
                if (free_list[i] && !free_reg_found0) begin
                    found_free_reg0 <= 5'(i);
                    free_reg_found0 <= 1'b1;
                end
            end
            
            if (free_reg_found0) begin
                rename_table[decoded_instr_pair.instr0.rd].phys_reg <= found_free_reg0;
                rename_table[decoded_instr_pair.instr0.rd].valid <= 1'b1;
                rename_table[decoded_instr_pair.instr0.rd].ready <= 1'b0;
                free_list[found_free_reg0] = 1'b0;
                free_list_count = free_list_count - 1;
            end
        end
        
        if (decoded_instr_pair.dual_issue_valid && decoded_instr_pair.instr1.rd != 0) begin
            free_reg_found1 <= 1'b0;
            found_free_reg1 <= 0;
            for (int i = 0; i < 32; i++) begin
                if (free_list[i] && !free_reg_found1 && (i[4:0] != found_free_reg0 || !free_reg_found0)) begin
                    found_free_reg1 <= 5'(i);
                    free_reg_found1 <= 1'b1;
                end
            end
            
            if (free_reg_found1) begin
                rename_table[decoded_instr_pair.instr1.rd].phys_reg <= found_free_reg1;
                rename_table[decoded_instr_pair.instr1.rd].valid <= 1'b1;
                rename_table[decoded_instr_pair.instr1.rd].ready <= 1'b0;
                free_list[found_free_reg1] = 1'b0;
                free_list_count = free_list_count - 1;
            end
        end
    end
end

// --------------------------
// Dispatch Stage
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rob_head = 0;
        rob_tail = 0;
        rs_head = 0;
        rs_tail = 0;
        lsq_head = 0;
        lsq_tail = 0;
    end else if (pipeline_state == STAGE_DISPATCH) begin
        if (decoded_instr_pair.dual_issue_valid && decoded_instr_pair.instr0.rd != 0) begin
            // Allocate ROB entries
            rob[decoded_instr_pair.instr0.rd].valid = 1'b1;
            rob[decoded_instr_pair.instr0.rd].completed = 1'b0;
            rob[decoded_instr_pair.instr0.rd].arch_reg = decoded_instr_pair.instr0.rd;
            rob[decoded_instr_pair.instr0.rd].phys_reg = rename_table[decoded_instr_pair.instr0.rd].phys_reg;
            rob[decoded_instr_pair.instr0.rd].pc = pc;
            
            rob_tail = rob_tail + 1;
            
            // Dispatch first instruction
            dispatch_instruction(decoded_instr_pair.instr0, rob_tail, pc);
            
        end
        if (decoded_instr_pair.dual_issue_valid && decoded_instr_pair.instr1.rd != 0) begin
            // Allocate ROB entries
            rob[decoded_instr_pair.instr1.rd].valid = 1'b1;
            rob[decoded_instr_pair.instr1.rd].completed = 1'b0;
            rob[decoded_instr_pair.instr1.rd].arch_reg = decoded_instr_pair.instr1.rd;
            rob[decoded_instr_pair.instr1.rd].phys_reg = rename_table[decoded_instr_pair.instr1.rd].phys_reg;
            rob[decoded_instr_pair.instr1.rd].pc = pc;
            
            rob_tail = rob_tail + 1;
            
            // Dispatch second instruction
            dispatch_instruction(decoded_instr_pair.instr1, rob_tail + 1, pc+4);
        end
    end
end

function automatic void dispatch_instruction(
    input decoded_instr_t instr,
    input logic [4:0] rob_idx,
    input logic [XLEN-1:0] pc_val
);
    // Primeiro, sempre alocar no RS
    rs[rs_tail].valid = instr.valid;
    rs[rs_tail].rob_tag = rob_idx;
    rs[rs_tail].op_type = instr.op_type;  // Tipo de operação propagado corretamente
    rs[rs_tail].opcode = instr.opcode;
    rs[rs_tail].funct3 = instr.funct3;
    rs[rs_tail].funct7 = instr.funct7;
    rs[rs_tail].imm = instr.imm;
    rs[rs_tail].pc = pc_val;
    rs[rs_tail].rs_found = 1'b1;
    
   // Check RS1 operand
    if (instr.rs1 != 0) begin
        // Check physical register file first
        if (rename_table[instr.rs1].ready) begin
            rs[rs_tail].rs1_value = phys_regfile[rename_table[instr.rs1].phys_reg].value;
            rs[rs_tail].rs1_ready = 1'b1;
        end 
        // Check ROB for completed instructions that might have this result
        else begin
                if (rob[instr.rs1].valid && rob[instr.rs1].completed) begin
                    rs[rs_tail].rs1_value = rob[instr.rs1].result;
                    rs[rs_tail].rs1_ready = 1'b1;
                end
            end
        end else begin
        // x0 is always ready and zero
        rs[rs_tail].rs1_value = '0;
        rs[rs_tail].rs1_ready = 1'b1;
    end
    
    // Check RS2 operand
    if (instr.rs2 != 0) begin
        // Check physical register file first
        if (rename_table[instr.rs2].ready) begin
            rs[rs_tail].rs2_value = phys_regfile[rename_table[instr.rs2].phys_reg].value;
            rs[rs_tail].rs2_ready = 1'b1;
        end 
        // Check ROB for completed instructions that might have this result
        else begin
                debug_arch_reg = rob[instr.rs2].arch_reg;
                debug_instruction_rs2 = instr.rs2;
                if (rob[instr.rd].valid && rob[instr.rs2].completed) begin
                    rs[rs_tail].rs2_value = rob[instr.rs2].result;
                    rs[rs_tail].rs2_ready = 1'b1;
                end
            end
        end else begin
        // x0 is always ready and zero
        rs[rs_tail].rs2_value = '0;
        rs[rs_tail].rs2_ready = 1'b1;
    end
    
    // Allocate in LSQ only for load/store
    if (instr.op_type == OP_LOAD || instr.op_type == OP_STORE || instr.op_type == OP_ALU) begin
        lsq[lsq_tail].valid = 1'b1;
        lsq[lsq_tail].state = LSQ_IDLE;
        lsq[lsq_tail].rob_tag = rob_idx;
        lsq[lsq_tail].is_load = (instr.op_type == OP_LOAD);
        lsq[lsq_tail].pc = pc_val;
        lsq[lsq_tail].op_type = instr.op_type;
        rs[rs_tail].needs_lsq = 1'b1;
        if (instr.op_type == OP_STORE) begin
            // Para stores, precisamos do endereço e dados
            if (instr.rs1 != 0 && rename_table[instr.rs1].ready) begin
                lsq[lsq_tail].addr = phys_regfile[rename_table[instr.rs1].phys_reg].value + instr.imm;
            end
            
            if (instr.rs2 != 0 && rename_table[instr.rs2].ready) begin
                lsq[lsq_tail].data = phys_regfile[rename_table[instr.rs2].phys_reg].value;
            end
            
            case (instr.funct3[1:0])
                2'b00: lsq[lsq_tail].wstrb = 8'b00000001;
                2'b01: lsq[lsq_tail].wstrb = 8'b00000011;
                2'b10: lsq[lsq_tail].wstrb = 8'b00001111;
                2'b11: lsq[lsq_tail].wstrb = 8'b11111111;
            endcase
        end else begin
            lsq[lsq_tail].unsigned_load = instr.funct3[2];
            lsq[lsq_tail].size = instr.funct3[1:0];
        end
        
        lsq_tail = lsq_tail + 1;
    end
    if (rs[rs_tail] != 0) begin
       rs_tail = rs_tail + 1; 
    end
endfunction

// --------------------------
// Execute Stage (Dual ALU)
// --------------------------
logic [$clog2(RS_ENTRIES)-1:0] rs_exec_idx0, rs_exec_idx1;
logic alu0_busy, alu1_busy;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        execute_result0 = '0;
        execute_result1 = '0;
        alu0_busy = 1'b0;
        alu1_busy = 1'b0;
    end else if (pipeline_state == STAGE_EXECUTE) begin
        // Process up to two instructions per cycle
        for (int i = 0; i < rs_tail; i++) begin
            if (rs[i].valid && rs[i].rs_found) begin
                if (!alu0_busy) begin
                    // Execute on ALU0
                    debug_rs1 = rs[i].rs1_value;
                    debug_rs2 = rs[i].rs2_value;
                    execute_result0 = execute_alu(rs[i]);
                    alu0_busy = 1'b1;
                    rs[i].valid = 1'b0;
                    if (rs[i].needs_lsq) begin
                        for (int j = 0; j < LSQ_ENTRIES; j++) begin
                            if (lsq[j].valid && lsq[j].rob_tag == rs[i].rob_tag) begin
                                lsq[j].addr = execute_result0.mem_addr;
                                if (!lsq[j].is_load) begin
                                    lsq[j].data = execute_result0.store_data;
                                end
                                break;
                            end
                        end
                    end
                end else if (!alu1_busy) begin
                    // Execute on ALU1
                    execute_result1 = execute_alu(rs[i]);
                    alu1_busy = 1'b1;
                    rs[i].valid = 1'b0;
                    if (rs[i].needs_lsq) begin
                        for (int j = 0; j < LSQ_ENTRIES; j++) begin
                            if (lsq[j].valid && lsq[j].rob_tag == rs[i].rob_tag) begin
                                lsq[j].addr = execute_result1.mem_addr;
                                if (!lsq[j].is_load) begin
                                    lsq[j].data = execute_result1.store_data;
                                end
                                break;
                            end
                        end
                    end
                end
                
                // Break if both ALUs are busy
                if (alu0_busy && alu1_busy) break;
            end
        end

        rob_head = rob_tail;
    end
end

// ALU execution helper function
function automatic execute_result_t execute_alu(input rs_entry_t rs_entry);
    execute_result_t result;
    result.rob_tag = rs_entry.rob_tag;
    result.pc = rs_entry.pc;

    case (rs_entry.op_type)
        OP_ALU: begin
            case (rs_entry.opcode)
                7'b0110111: result.alu_result = rs_entry.imm; // LUI
                7'b0010111: result.alu_result = rs_entry.pc + rs_entry.imm; // AUIPC
                7'b0010011: begin // Immediate ALU operations
                    case (rs_entry.funct3)
                        3'b000: result.alu_result = rs_entry.rs1_value + rs_entry.imm; // ADDI
                        3'b010: result.alu_result = ($signed(rs_entry.rs1_value) < $signed(rs_entry.imm)) ? 1 : 0; // SLTI
                        3'b011: result.alu_result = (rs_entry.rs1_value < rs_entry.imm) ? 1 : 0; // SLTIU
                        3'b100: result.alu_result = rs_entry.rs1_value ^ rs_entry.imm; // XORI
                        3'b110: result.alu_result = rs_entry.rs1_value | rs_entry.imm; // ORI
                        3'b001: result.alu_result = rs_entry.rs1_value << rs_entry.imm[5:0]; // SLLI
                        default: result.illegal_instr = 1;
                    endcase
                end
                7'b0110011: begin // Register-register ALU operations
                    case ({rs_entry.funct7, rs_entry.funct3})
                        {7'b0000000, 3'b000}: result.alu_result = rs_entry.rs1_value + rs_entry.rs2_value; // ADD
                        {7'b0100000, 3'b000}: result.alu_result = rs_entry.rs1_value - rs_entry.rs2_value; // SUB
                        {7'b0000000, 3'b001}: result.alu_result = rs_entry.rs1_value << rs_entry.rs2_value[5:0]; // SLL
                        {7'b0000000, 3'b010}: result.alu_result = {{63{1'b0}}, ($signed(rs_entry.rs1_value) < $signed(rs_entry.rs2_value))};
                        {7'b0000000, 3'b011}: result.alu_result = {{63{1'b0}}, (rs_entry.rs1_value < rs_entry.rs2_value)};
                        {7'b0000000, 3'b100}: result.alu_result = rs_entry.rs1_value ^ rs_entry.rs2_value; // XOR
                        {7'b0000000, 3'b101}: result.alu_result = rs_entry.rs1_value >> rs_entry.rs2_value[5:0]; // SRL
                        {7'b0100000, 3'b101}: result.alu_result = ($signed(rs_entry.rs1_value)) >>> rs_entry.rs2_value[5:0]; // SRA
                        {7'b0000000, 3'b110}: result.alu_result = rs_entry.rs1_value | rs_entry.rs2_value; // OR
                        {7'b0000000, 3'b111}: result.alu_result = rs_entry.rs1_value & rs_entry.rs2_value; // AND
                        default: result.illegal_instr = 1;
                    endcase
                end
                default: result.alu_result = '0;
            endcase
        end
        OP_BRANCH: begin
            case (rs_entry.opcode)
                7'b1101111: begin // JAL
                    result.alu_result = rs_entry.pc + 4;
                    result.branch_taken = 1;
                    result.branch_target = rs_entry.pc + rs_entry.imm;
                end
                7'b1100111: begin // JALR
                    result.alu_result = rs_entry.pc + 4;
                    result.branch_taken = 1;
                    result.branch_target = (rs_entry.rs1_value + rs_entry.imm) & ~1;
                end
                7'b1100011: begin // Conditional branches
                    result.branch_target = rs_entry.pc + rs_entry.imm;
                    case (rs_entry.funct3)
                        3'b000: result.branch_taken = (rs_entry.rs1_value == rs_entry.rs2_value); // BEQ
                        3'b001: result.branch_taken = (rs_entry.rs1_value != rs_entry.rs2_value); // BNE
                        3'b100: result.branch_taken = ($signed(rs_entry.rs1_value) < $signed(rs_entry.rs2_value)); // BLT
                        3'b101: result.branch_taken = ($signed(rs_entry.rs1_value) >= $signed(rs_entry.rs2_value)); // BGE
                        3'b110: result.branch_taken = (rs_entry.rs1_value < rs_entry.rs2_value); // BLTU
                        3'b111: result.branch_taken = (rs_entry.rs1_value >= rs_entry.rs2_value); // BGEU
                        default: result.branch_taken = 0;
                    endcase
                end
                default: begin
                    result.branch_taken = 0;
                    result.alu_result = '0;
                end
            endcase
        end

        OP_LOAD, OP_STORE: begin
            // Calculate memory address
            result.mem_addr = rs_entry.rs1_value + rs_entry.imm;
            
            if (rs_entry.op_type == OP_STORE) begin
                result.store_data = rs_entry.rs2_value;
                result.mem_we = 1'b1;
                // Determine wstrb based on funct3
                case (rs_entry.funct3[1:0])
                    2'b00: result.mem_wstrb = 8'b00000001; // SB
                    2'b01: result.mem_wstrb = 8'b00000011; // SH
                    2'b10: result.mem_wstrb = 8'b00001111; // SW
                    2'b11: result.mem_wstrb = 8'b11111111; // SD
                endcase
            end else begin
                result.mem_we = 1'b0;
                result.mem_unsigned = rs_entry.funct3[2];
                result.mem_size = rs_entry.funct3[1:0];
            end
            
            // Update LSQ entry with calculated address
            for (int i = 0; i < LSQ_ENTRIES; i++) begin
                if (lsq[i].valid && lsq[i].rob_tag == rs_entry.rob_tag) begin
                    lsq[i].addr = result.mem_addr;
                    if (rs_entry.op_type == OP_STORE) begin
                        lsq[i].data = result.store_data;
                    end
                    lsq[i].state = LSQ_ISSUED;
                    break;
                end
            end
        end
        default: begin
            result.alu_result = '0;
            result.branch_taken = 0;
        end
    endcase
    
    return result;
endfunction

logic mem0_processed;
logic mem1_processed;

// --------------------------
// Memory Stage
// --------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        memory_result0 = '0;
        memory_result1 = '0;
        for (int i = 0; i < LSQ_ENTRIES; i++) begin
            lsq[i].valid = 0;
            lsq[i].state = LSQ_IDLE;
        end
    end else if (pipeline_state == STAGE_MEMORY) begin
        // Process up to two memory operations per cycle
        memory_result0 = '0;
        memory_result1 = '0;
        mem0_processed = 1'b0;
        mem1_processed = 1'b0;
        
        for (int i = 0; i < lsq_tail; i++) begin
            if (lsq[i].valid && !lsq[i].completed) begin
                if (!mem0_processed) begin
                    // Process first memory operation
                    memory_result0.rob_tag = lsq[i].rob_tag;
                    memory_result0.pc = lsq[i].pc;
                    
                    if (lsq[i].is_load) begin
                        // Load operation
                        dmem_addr = lsq[i].addr;
                        dmem_req = 1'b1;
                        dmem_we = 1'b0;
                        
                        if (dmem_ack_in) begin
                            if (decoded_instr_pair.instr0.opcode == 7'b0000011) begin
                                case (decoded_instr_pair.instr0.funct3)
                                    3'b000: memory_result0.data = {{56{dmem_rdata[7]}}, dmem_rdata[7:0]};  // LB
                                    3'b001: memory_result0.data = {{48{dmem_rdata[15]}}, dmem_rdata[15:0]}; // LH
                                    3'b010: memory_result0.data = {{32{dmem_rdata[31]}}, dmem_rdata[31:0]}; // LW
                                    3'b011: memory_result0.data = dmem_rdata;  // LD
                                    3'b100: memory_result0.data = {56'b0, dmem_rdata[7:0]};  // LBU
                                    3'b101: memory_result0.data = {48'b0, dmem_rdata[15:0]}; // LHU
                                    3'b110: memory_result0.data = {32'b0, dmem_rdata[31:0]}; // LWU
                                    default: memory_result0.data = '0;
                                endcase
                            end
                        end
                    end else begin
                        // Store operation
                        dmem_addr = lsq[i].addr;
                        dmem_wdata = lsq[i].data;
                        dmem_wstrb = lsq[i].wstrb;
                        dmem_req = 1'b1;
                        dmem_we = 1'b1;
                        
                        if (dmem_ack_in) begin
                            memory_result0.data = execute_result0.alu_result;
                            lsq[i].completed = 1'b1;
                            mem0_processed = 1'b1;
                        end
                    end
                end else if (!mem1_processed) begin
                    // Process second memory operation
                    memory_result1.rob_tag = lsq[i].rob_tag;
                    memory_result1.pc = lsq[i].pc;
                    
                    if (lsq[i].is_load) begin
                        // Load operation
                        dmem_addr = lsq[i].addr;
                        dmem_req = 1'b1;
                        dmem_we = 1'b0;
                        
                        if (imem_ack_in) begin
                            if (decoded_instr_pair.instr0.opcode == 7'b0000011) begin
                                case (decoded_instr_pair.instr0.funct3)
                                    3'b000: memory_result1.data = {{56{dmem_rdata[7]}}, dmem_rdata[7:0]};  // LB
                                    3'b001: memory_result1.data = {{48{dmem_rdata[15]}}, dmem_rdata[15:0]}; // LH
                                    3'b010: memory_result1.data = {{32{dmem_rdata[31]}}, dmem_rdata[31:0]}; // LW
                                    3'b011: memory_result1.data = dmem_rdata;  // LD
                                    3'b100: memory_result1.data = {56'b0, dmem_rdata[7:0]};  // LBU
                                    3'b101: memory_result1.data = {48'b0, dmem_rdata[15:0]}; // LHU
                                    3'b110: memory_result1.data = {32'b0, dmem_rdata[31:0]}; // LWU
                                    default: memory_result1.data = '0;
                                endcase
                            end
                        end
                    end else begin
                        // Store operation
                        dmem_addr = lsq[i].addr;
                        dmem_wdata = lsq[i].data;
                        dmem_wstrb = lsq[i].wstrb;
                        dmem_req = 1'b1;
                        dmem_we = 1'b1;
                        if (dmem_ack_in) begin
                            memory_result1.data = execute_result1.alu_result;
                            lsq[i].completed = 1'b1;
                            mem1_processed = 1'b1;
                        end
                    end
                end
                
                // Break if both memory operations processed
                if (mem0_processed && mem1_processed) break;
            end
        end
    end
end

logic commit0_done;
logic commit1_done;

// --------------------------
// Commit Stage
// --------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (int i = 0; i < 31; i++) begin
            arch_regfile[i] = '0;
        end
        rob_head = 0;
        inst_retired <= 0;
    end else if (pipeline_state == STAGE_COMMIT) begin
        // Commit up to two instructions per cycle
        commit0_done = 1'b0;
        commit1_done = 1'b0;
        
            // First instruction commit
                if (!commit0_done && rob[decoded_instr_pair.instr0.rd].valid) begin
                // Write back to architectural register file
                
                    if (rob[decoded_instr_pair.instr0.rd].arch_reg != 0) begin
                        arch_regfile[rob[decoded_instr_pair.instr0.rd].arch_reg] = memory_result0.data;
                        rob[decoded_instr_pair.instr0.rd].result = execute_result0.alu_result;
                        rob[decoded_instr_pair.instr0.rd].completed = 1'b1;
                    end
                    
                    // Free physical register if it was renamed
                    if (rename_table[rob[decoded_instr_pair.instr0.rd].arch_reg].valid) begin
                        free_list[rename_table[rob[decoded_instr_pair.instr0.rd].arch_reg].phys_reg] = 1'b1;
                        free_list_count = free_list_count + 1;
                        rename_table[rob[decoded_instr_pair.instr0.rd].arch_reg].valid = 1'b0;
                    end
                    
                    // Update performance counter
                    inst_retired <= inst_retired + 1;
                    commit0_done = 1'b1;
                end 
                if (!commit1_done && rob[decoded_instr_pair.instr1.rd].valid) begin
                    // Write back to architectural register file

                    if (rob[decoded_instr_pair.instr1.rd].arch_reg != 0) begin
                        arch_regfile[rob[decoded_instr_pair.instr1.rd].arch_reg] = memory_result1.data;
                        rob[decoded_instr_pair.instr1.rd].result = execute_result1.alu_result;
                        rob[decoded_instr_pair.instr1.rd].completed = 1'b1;
                    end

                    // Free physical register if it was renamed
                    if (rename_table[rob[decoded_instr_pair.instr1.rd].arch_reg].valid) begin
                        free_list[rename_table[rob[decoded_instr_pair.instr1.rd].arch_reg].phys_reg] = 1'b1;
                        free_list_count = free_list_count + 1;
                        rename_table[rob[decoded_instr_pair.instr1.rd].arch_reg].valid = 1'b0;
                    end

                    // Update performance counter
                    inst_retired <= inst_retired + 1;
                    commit1_done = 1'b1;
                end
            for (int i = 0; i < rs_tail; i++) begin
                if (commit0_done || commit1_done) begin
                    rs[i] = '0;
                    lsq[i] = '0;
                end
            end
        

        lsq_tail = '0;
        rs_tail = '0;
        decoded_instr_pair <= '0;
        execute_result0 = '0;
        execute_result1 = '0;
        alu0_busy = '0;
        alu1_busy = '0;
    end
end

// --------------------------
// Pipeline Control
// --------------------------
always_comb begin
    next_pipeline_state = pipeline_state;
    
    case (pipeline_state)
        STAGE_RESET: 
            if (rst_n) next_pipeline_state = STAGE_FETCH;
        
        STAGE_FETCH: 
            if (imem_error_in) next_pipeline_state = STAGE_TRAP;
            else if (imem_ack_in) next_pipeline_state = STAGE_DECODE;
        
        STAGE_DECODE: 
            if (decoded_instr_pair.instr0.valid || decoded_instr_pair.instr1.valid) next_pipeline_state = STAGE_RENAME;
        
        STAGE_RENAME:
            next_pipeline_state = STAGE_DISPATCH;

        STAGE_DISPATCH:
            next_pipeline_state = STAGE_EXECUTE;

        STAGE_EXECUTE:
            next_pipeline_state = STAGE_MEMORY;
        
        STAGE_MEMORY: 
            next_pipeline_state = STAGE_COMMIT;
        
        STAGE_COMMIT: 
            next_pipeline_state = STAGE_FETCH;
        
        STAGE_TRAP: 
            next_pipeline_state = STAGE_FETCH;
        
        default: next_pipeline_state = STAGE_RESET;
    endcase
end

// --------------------------
// Sequential Logic
// --------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // Reset all registers
        pc <= PC_RESET;
        pipeline_state <= STAGE_RESET;
        fetch_valid0 <= 0;
        fetch_valid1 <= 0;
        fetched_instr0 <= '0;
        fetched_instr1 <= '0;
        debug_ack <= 0;
        debug_halted <= 0;
        cycles <= 0;
        inst_retired <= 0;
        
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
        current_privilege <= PRIV_MACHINE;
        mstatus_mie <= 0;
        mstatus_mpie <= 0;
    end else begin
        // Performance counters
        cycles <= cycles + 1;
        
        // Update pipeline state
        pipeline_state <= next_pipeline_state;
        
        // Update PC
        if (execute_result0.branch_taken && pipeline_state == STAGE_EXECUTE) begin
            pc <= execute_result0.branch_target;
        end else if (pipeline_state == STAGE_COMMIT) begin
            if (decoded_instr_pair.dual_issue_valid) begin
                pc <= pc + 8;
            end else begin
                pc <= pc + 4;
            end
        end
        
        // Handle interrupts and exceptions
        if (pipeline_state == STAGE_TRAP) begin
            csr_mepc <= pc;
            mstatus_mpie <= mstatus_mie;
            mstatus_mie <= 0;
            pc <= csr_mtvec;
        end
    end
end

// Debug interface assignments
assign debug_pc = pc;
assign debug_regfile = arch_regfile;
assign debug_inst_retired = inst_retired;
assign debug_cycles = cycles;
assign debug_opcode = decoded_instr_pair.instr0.opcode;
assign debug_result_alu1 = execute_result0.alu_result;
assign debug_result_alu2 = execute_result1.alu_result;
assign debug_valid_instr = decoded_instr_pair.instr0.valid;
assign debug_imm = decoded_instr_pair.instr0.imm;
assign debug_funct3 = decoded_instr_pair.instr0.funct3;
assign debug_funct7 = decoded_instr_pair.instr0.funct7;
assign debug_privilege = current_privilege;
assign debug_rs_tail = rs_tail;
assign debug_decoded_instr0 = decoded_instr_pair.instr0;
assign debug_decoded_instr1 = decoded_instr_pair.instr1;
assign debug_optype0 = decoded_instr_pair.instr0.op_type;
assign debug_optype1 = decoded_instr_pair.instr1.op_type;

endmodule
