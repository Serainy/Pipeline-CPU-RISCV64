`ifndef __CORE_SV
`define __CORE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "include/defines.sv"
`endif

// `include "common.svh"
// `include "defines.svh"

module csr
	import defines::*;
	import common::*;(
    input  logic clk,
    input  logic rst,
    input  logic csr_we,
    input  logic [11:0] 			csr_addr_r, 
    input  logic [11:0] 			csr_addr_w, 
    input  logic [`REG_WIDTH-1:0]  	csr_wdata, 
    output logic [`REG_WIDTH-1:0]  	csr_rdata,
    output CSR_reg 					csr_reg,

    input  logic  					is_mret,

	output logic [`REG_WIDTH-1:0]  	csr_pc,
	input  logic [`REG_WIDTH-1:0]  	pc,
	input  logic 					ecall, 
	input  logic					hand_in
	);
    localparam logic [11:0]
        ADDR_MSTATUS_A  = 12'h300,
        ADDR_MTVEC_A    = 12'h305,
        ADDR_MIP_A      = 12'h344,
        ADDR_MIE_A      = 12'h304,
        ADDR_MSCRATCH_A = 12'h340,
        ADDR_MCAUSE_A   = 12'h342,
        ADDR_MTVAL_A    = 12'h343,
        ADDR_MEPC_A     = 12'h341,
        ADDR_MCYCLE_A   = 12'hB00,
        ADDR_SATP_A     = 12'h180,
		ADDR_MODE_A 	= 12'h7C0;
    localparam logic [3:0]
        ADDR_MSTATUS  	= 'h1,
        ADDR_MTVEC    	= 'h2,
        ADDR_MIP      	= 'h3,
        ADDR_MIE      	= 'h4,
        ADDR_MSCRATCH 	= 'h5,
        ADDR_MCAUSE   	= 'h6,
        ADDR_MTVAL    	= 'h7,
        ADDR_MEPC     	= 'h8,
        ADDR_MCYCLE   	= 'h9,
        ADDR_SATP     	= 'hA,
		ADDR_MODE 	  	= 'hB;
	logic [3:0] used_r, used_w;
	always_comb
	begin
		case(csr_addr_r)
			ADDR_MSTATUS_A: used_r = ADDR_MSTATUS;
			ADDR_MTVEC_A: 	used_r = ADDR_MTVEC;
			ADDR_MIP_A: 	used_r = ADDR_MIP;
			ADDR_MIE_A: 	used_r = ADDR_MIE;
			ADDR_MSCRATCH_A:used_r = ADDR_MSCRATCH;
			ADDR_MCAUSE_A: 	used_r = ADDR_MCAUSE;
			ADDR_MTVAL_A: 	used_r = ADDR_MTVAL;
			ADDR_MEPC_A: 	used_r = ADDR_MEPC;
			ADDR_MCYCLE_A: 	used_r = ADDR_MCYCLE;
			ADDR_SATP_A: 	used_r = ADDR_SATP;
			ADDR_MODE_A: 	used_r = ADDR_MODE;
			default: 		used_r = 'h0;
		endcase
		case(csr_addr_w)
			ADDR_MSTATUS_A: used_w = ADDR_MSTATUS;
			ADDR_MTVEC_A: 	used_w = ADDR_MTVEC;
			ADDR_MIP_A: 	used_w = ADDR_MIP;
			ADDR_MIE_A: 	used_w = ADDR_MIE;
			ADDR_MSCRATCH_A:used_w = ADDR_MSCRATCH;
			ADDR_MCAUSE_A: 	used_w = ADDR_MCAUSE;
			ADDR_MTVAL_A: 	used_w = ADDR_MTVAL;
			ADDR_MEPC_A: 	used_w = ADDR_MEPC;
			ADDR_MCYCLE_A: 	used_w = ADDR_MCYCLE;
			ADDR_SATP_A: 	used_w = ADDR_SATP;
			ADDR_MODE_A: 	used_w = ADDR_MODE;
			default: 		used_w = 'h0;
		endcase
	end

    logic [`REG_WIDTH-1:0] csr_regs [11:0];
	logic [`REG_WIDTH-1:0] csr_regs_nxt [11:0]; 
	logic [`REG_WIDTH-1:0] csr_pcpc;
	always @(posedge clk, posedge rst)
	begin
		if(rst)
			csr_pc <= PCINIT;
		else if(ecall)
			csr_pc <= csr_pcpc;
	end
    
    always_comb begin
		csr_pcpc = PCINIT;
		for (int i = 0; i < 12; i = i + 1) begin
			csr_regs_nxt[i] = csr_regs[i];
		end
		if(ecall)
		begin
			csr_regs_nxt[ADDR_MEPC] 			= pc;
			csr_pcpc 							= csr_regs_nxt[ADDR_MTVEC];
			csr_regs_nxt[ADDR_MCAUSE] 			= 64'h0000000000000008;
			csr_regs_nxt[ADDR_MSTATUS][7] 		= csr_regs_nxt[ADDR_MSTATUS][3];
			csr_regs_nxt[ADDR_MSTATUS][3] 		= 0;
			csr_regs_nxt[ADDR_MSTATUS][12:11] 	= csr_regs_nxt[ADDR_MODE][1:0];
			
			csr_regs_nxt[ADDR_MODE] 		= 3;
		end
        else if (csr_we)  
		begin
			if(is_mret)
			begin
				csr_regs_nxt[ADDR_MODE][1:0]  = csr_regs_nxt[ADDR_MSTATUS][12:11];
				csr_regs_nxt[ADDR_MSTATUS][3] = csr_regs_nxt[ADDR_MSTATUS][7];
				csr_regs_nxt[ADDR_MSTATUS][7] = 1;
				csr_regs_nxt[ADDR_MSTATUS][12:11] = 2'b00;
			end
			else
			begin
				csr_regs_nxt[used_w] = csr_wdata;
			end
        end
    end

    always @(posedge clk or posedge rst) begin
        if(rst) 
		begin
			for (int i = 0; i < 12; i = i + 1) begin
				csr_regs[i] <= '0;
			end
			csr_regs[ADDR_MODE] <= 3;
		end
        else 
		begin
            csr_regs[ADDR_MSTATUS]  <= csr_regs_nxt[ADDR_MSTATUS];
			csr_regs[ADDR_MTVEC]    <= csr_regs_nxt[ADDR_MTVEC];
			csr_regs[ADDR_MIP]      <= csr_regs_nxt[ADDR_MIP];
			csr_regs[ADDR_MIE]      <= csr_regs_nxt[ADDR_MIE];
			csr_regs[ADDR_MSCRATCH] <= csr_regs_nxt[ADDR_MSCRATCH];
			csr_regs[ADDR_MCAUSE]   <= csr_regs_nxt[ADDR_MCAUSE];
			csr_regs[ADDR_MTVAL]    <= csr_regs_nxt[ADDR_MTVAL];
			csr_regs[ADDR_MEPC]     <= csr_regs_nxt[ADDR_MEPC];
			csr_regs[ADDR_MCYCLE]   <= csr_regs_nxt[ADDR_MCYCLE];
			csr_regs[ADDR_SATP]     <= csr_regs_nxt[ADDR_SATP];
			csr_regs[ADDR_MODE]     <= csr_regs_nxt[ADDR_MODE];
        end
    end

    assign csr_rdata 		= csr_regs[used_r];


    assign csr_reg.mstatus  = csr_regs_nxt[ADDR_MSTATUS];
    assign csr_reg.mtvec    = csr_regs_nxt[ADDR_MTVEC];
    assign csr_reg.mip      = csr_regs_nxt[ADDR_MIP];
    assign csr_reg.mie      = csr_regs_nxt[ADDR_MIE];
    assign csr_reg.mscratch = csr_regs_nxt[ADDR_MSCRATCH];
    assign csr_reg.mcause   = csr_regs_nxt[ADDR_MCAUSE];
    assign csr_reg.mtval    = csr_regs_nxt[ADDR_MTVAL];
    assign csr_reg.mepc     = csr_regs_nxt[ADDR_MEPC];
    assign csr_reg.mcycle   = csr_regs_nxt[ADDR_MCYCLE];
    assign csr_reg.satp     = csr_regs_nxt[ADDR_SATP];
    assign csr_reg.mode     = csr_regs_nxt[ADDR_MODE][1:0];

	endmodule

module adder
    import defines::*;(
    input  logic [`REG_WIDTH-1:0] a,
    input  logic [`REG_WIDTH-1:0] b,
    output logic [`REG_WIDTH-1:0] y
    );
    assign y = a + b;
	endmodule

module multiplier_multicycle 
	import defines::*;(
    input  logic clk, rst, valid, hand_in,
    input  logic [`REG_WIDTH-1:0] a, b,
    output logic [`REG_WIDTH-1:0] c,
    output logic done
	);
	typedef enum logic { INIT, DOING } state_t;
	state_t state, state_nxt;
    assign done = (state_nxt == INIT) & valid;

    logic [`REG_WIDTH+1:0] count, count_nxt;
    localparam logic [`REG_WIDTH+1:0] MULT_DELAY = {1'b0, 1'b1, 64'b0};

    always_ff @(posedge clk) begin
        if (rst|hand_in) begin
            {state, count} <= '0;
        end else begin
            {state, count} <= {state_nxt, count_nxt};
        end
    end
    always_comb begin
        {state_nxt, count_nxt} = {state, count}; // default
        unique case(state)
            INIT: begin
                if (valid) begin
                    state_nxt = DOING;
                    count_nxt = MULT_DELAY;
                end
            end
            DOING: begin
                count_nxt = {1'b0, count_nxt[65:1]};
                if (count_nxt == '0) begin
                    state_nxt = INIT;
                end
            end
			default:
				if (valid) begin
                    state_nxt = DOING;
                    count_nxt = MULT_DELAY;
                end
        endcase
    end
    logic [128:0] p, p_nxt;
    always_comb begin
        p_nxt = p;
        unique case(state)
            INIT: begin
                p_nxt = {65'b0, a};
            end
            DOING: begin
                if (p_nxt[0]) begin
                    p_nxt[128:64] = p_nxt[127:64] + b;
                    // p_nxt[64:32] = p_nxt[64:32] + b;
            	end
            	p_nxt = {1'b0, p_nxt[128:1]};
            end
			default:
				p_nxt = {65'b0, a};
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst|hand_in) begin
            p <= '0;
        end else begin
            p <= p_nxt;
        end
    end
    assign c = p[63:0];
	endmodule

module divider_multicycle
	import defines::*; #(parameter WIDTH = 64)(
    input  logic clk, rst, valid_div, valid_divu, valid_rem, valid_remu, valid_w,hand_in,
    input  logic [WIDTH-1:0] a, b,
    output logic [WIDTH-1:0] c, // c = {a % b, a / b}
    output logic done
	);
	logic flag;
	assign flag = (WIDTH == 64)? ~valid_w:valid_w;
	// assign flag = valid_w;
    typedef enum logic { INIT, DOING } state_t;
	state_t state, state_nxt;
    logic [WIDTH+1:0] count, count_nxt;
    // localparam logic [WIDTH+1:0] 	DIV_DELAY = {1'b0, 1'b1, WIDTH{1'b0}};
    logic [WIDTH+1:0] 	DIV_DELAY;
	assign 						 	DIV_DELAY[WIDTH-1:0] 		= 0;
	assign							DIV_DELAY[WIDTH+1:WIDTH] 	= 1;

	logic  isUnsigned;
	assign isUnsigned = valid_divu | valid_remu;
	logic  [WIDTH-1:0] a_abs, b_abs;
	assign a_abs = (isUnsigned == 0)?((a[WIDTH-1] == 0)?a:(~a+1)):a;
	assign b_abs = (isUnsigned == 0)?((b[WIDTH-1] == 0)?b:(~b+1)):b;
	logic  isDiv_minus, isRem_minus;
	assign isDiv_minus = a[WIDTH-1] != b[WIDTH-1];
	assign isRem_minus = (a[WIDTH-1] == 1); // a < 0

    always_ff @(posedge clk) begin
        if (rst|hand_in) begin
            {state, count} <= '0;
        end else begin
            {state, count} <= {state_nxt, count_nxt};
        end
    end
    assign done = (state_nxt == INIT);
    always_comb begin
        {state_nxt, count_nxt} = {state, count}; // default
        unique case(state)
            INIT: begin
                if ((valid_div | valid_divu | valid_rem | valid_remu) & flag) begin
                    state_nxt = DOING;
                    count_nxt = DIV_DELAY;
                end
            end
            DOING: begin
                count_nxt = {1'b0, count_nxt[WIDTH+1:1]};
                if (count_nxt == '0) begin
                    state_nxt = INIT;
                end
            end
			default: begin
                if ((valid_div | valid_divu | valid_rem | valid_remu) & flag) begin
                    state_nxt = DOING;
                    count_nxt = DIV_DELAY;
                end
            end
        endcase
    end
    logic [2*WIDTH-1:0] p, p_nxt;
    always_comb begin
        p_nxt = p;
        unique case(state)
            INIT: begin
				p_nxt[WIDTH-1:0] 		= a_abs;
				p_nxt[2*WIDTH-1:WIDTH]	= 0;
                // p_nxt = {WIDTH'b0, a_abs};
            end
            DOING: begin
                p_nxt = {p_nxt[2*WIDTH-2:0], 1'b0};
                if (p_nxt[2*WIDTH-1:WIDTH] >= b_abs) begin
                    p_nxt[2*WIDTH-1:WIDTH] -= b_abs;
                    p_nxt[0] = 1'b1;
                end
            end
			default: begin
                p_nxt[WIDTH-1:0] 		= a_abs;
				p_nxt[2*WIDTH-1:WIDTH]	= 0;
            end
        endcase
    end
    always_ff @(posedge clk) begin
        if (rst|hand_in) begin
            p <= '0;
        end else begin
            p <= p_nxt;
        end
    end
	always_comb
	begin
		c = 0;
		if(isUnsigned)
		begin 
			c = (valid_divu == 1)?p[WIDTH-1:0]:p[2*WIDTH-1:WIDTH];
			if(b == 0 && valid_divu)
			// begin c = ~(WIDTH'b0); end
			begin c = ~0; end
			else if(b == 0 && valid_remu)
			begin c = a; end
		end
		else if(valid_div)
		begin
			c = (isDiv_minus)?(~p[WIDTH-1:0]+1):p[WIDTH-1:0];
			if(b == 0)
			// begin c = ~(WIDTH'b0); end
			begin c = ~0; end
		end
		else if(valid_rem)
		begin
			c = (isRem_minus)?(~p[2*WIDTH-1:WIDTH]+1):p[2*WIDTH-1:WIDTH];
			if(b == 0)
			begin c = a; end
		end
	end
	endmodule

module if_id_regs
	import defines::*;(
	input  clk, rst, hand_in,
	output logic valid_if_id_o,
	input  logic [`REG_WIDTH-1:0] pc_if_id_i,
	output reg 	 [`REG_WIDTH-1:0] pc_if_id_o,
	input  logic [`INSTR_WIDTH-1:0] instr_if_id_i,
	output reg 	 [`INSTR_WIDTH-1:0] instr_if_id_o,
	input  logic 	 				flag_clear,
	input  logic 	 				stallFlag
    );

	always@(posedge clk or posedge rst)
	begin
		if(rst)
			pc_if_id_o <= 0;
		else if(stallFlag&hand_in)
			pc_if_id_o <= pc_if_id_o;
		else if(flag_clear&hand_in)
			pc_if_id_o <= 0;
		else if(hand_in)
			pc_if_id_o <= pc_if_id_i;
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			valid_if_id_o <= 0;
		else if(stallFlag&hand_in)
			valid_if_id_o <= valid_if_id_o;
		else if(flag_clear&hand_in)
			valid_if_id_o <= 0;
		else if(hand_in)
			valid_if_id_o <= 1;
	end	
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			instr_if_id_o <= 0;
		else if(stallFlag&hand_in)
			instr_if_id_o <= instr_if_id_o;
		else if(flag_clear&hand_in)
			instr_if_id_o <= 0;
		else if(hand_in)
			instr_if_id_o <= instr_if_id_i;
	end
	endmodule

module id
    import defines::*;(
    input  logic                        rst,

    input  logic [`REG_WIDTH-1:0]       input_pc,
    input  logic [`INSTR_WIDTH-1:0]     instr,

    output logic [`REG_WIDTH-1:0]       pc,
    output logic [`REG_WIDTH-1:0]       pc_b,

    //from regs
    input  logic [`REG_WIDTH-1:0]       rs1data,
    input  logic [`REG_WIDTH-1:0]       rs2data,
    //to regs
    output logic [`REG_ADDR_WIDTH-1:0]  rs1addr,
    output logic [`REG_ADDR_WIDTH-1:0]  rs2addr,

    output logic [`REG_WIDTH-1:0]       rs1_data_o,
    output logic [`REG_WIDTH-1:0]       rs2_data_o,
    
    output logic [`REG_ADDR_WIDTH-1:0]  rdAddr,

    output logic                        memread,
    output logic                        memtoreg,
    output logic                        memwrite,

    output logic                        isNotImm,
    output logic                        regwrite,
    output logic [`REG_WIDTH-1:0]       imm,
    output logic [`ALUOP_WIDTH-1:0]     aluop,
    output logic [2:0]                  funct3,
    output logic [6:0]                  funct7,
    output logic [6:0]                  opcode,

    output logic                  		Iswaitedflag_data,
    output logic                  		Iswaitedflag_addr,
    output logic                  		Iswaitedflag_alu,

	input  logic [2:0]					forwardae, forwardbe,
	input  logic [`REG_WIDTH-1:0]    	aluout_ex_mem_o,wb_data,aluout_ex_mem_i,
    output logic                  		flag_b,

	input  logic [`REG_WIDTH-1:0]    	dreaddata_mem_wb_i,
	input  logic [2:0] 					func3_dbusdone,


	output logic 						csr_we,
	output logic 				        is_mret,
	output logic [11:0]					csr_addr,
	output logic [`REG_WIDTH-1:0]       zimm,
	output logic 				        ecall
	);

	assign      opcode  = instr[6:0];
	logic [4:0] rd      = instr[11:7];
	logic [4:0] rs1     = instr[19:15];    
	logic [4:0] rs2     = instr[24:20];
	assign 		funct3  = instr[14:12];
	assign 		funct7  = instr[31:25];
	// assign funct = {funct7[5], funct3};

	assign 		csr_we 	= (opcode == `INSTR_CSR);
	assign 		is_mret = (instr == 32'b00110000001000000000000001110011);
	assign 		csr_addr= instr[31:20];
	assign 		zimm  	= {{(`REG_WIDTH-5){1'b0}}, instr[19:15]};
	assign 		ecall = (instr == 32'b001110011);


	logic [`REG_WIDTH-1:0] a, b;
	always_comb 
		begin
			case(forwardae)
				3'b011:	a = aluout_ex_mem_i;
				3'b000:	a = rs1_data_o;
				3'b001:	a = wb_data;
				3'b010:	a = aluout_ex_mem_o;
				3'b100:	
					case(func3_dbusdone[2:0])
						3'b000:  a = {{56{dreaddata_mem_wb_i[7]}}, dreaddata_mem_wb_i[7:0]};  	// lb
						3'b100:  a = {56'b0			  	 , dreaddata_mem_wb_i[7:0]};  			// lbu
						3'b001:  a = {{48{dreaddata_mem_wb_i[15]}}, dreaddata_mem_wb_i[15:0]};	// lh
						3'b101:  a = {48'b0			  	 , dreaddata_mem_wb_i[15:0]}; 			// lhu
						3'b010:  a = {{32{dreaddata_mem_wb_i[31]}}, dreaddata_mem_wb_i[31:0]};	// lw
						3'b110:  a = {32'b0			 	 , dreaddata_mem_wb_i[31:0]}; 			// lwu
						3'b011:  a = dreaddata_mem_wb_i[63:0];					 	  			// ld
						default: a = dreaddata_mem_wb_i[63:0];
					endcase
				default:a = rs1_data_o; 
			endcase
			case(forwardbe)
				3'b011:	b = aluout_ex_mem_i;
				3'b000:	b = (isNotImm == 1'b0)? rs2_data_o : {{(`REG_WIDTH-13){instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
				3'b001:	b = wb_data;
				3'b010:	b = aluout_ex_mem_o;
				3'b100:
					case(func3_dbusdone[2:0])
						3'b000:  b = {{56{dreaddata_mem_wb_i[7]}}, dreaddata_mem_wb_i[7:0]};  	// lb
						3'b100:  b = {56'b0			  	 , dreaddata_mem_wb_i[7:0]};  			// lbu
						3'b001:  b = {{48{dreaddata_mem_wb_i[15]}}, dreaddata_mem_wb_i[15:0]};	// lh
						3'b101:  b = {48'b0			  	 , dreaddata_mem_wb_i[15:0]}; 			// lhu
						3'b010:  b = {{32{dreaddata_mem_wb_i[31]}}, dreaddata_mem_wb_i[31:0]};	// lw
						3'b110:  b = {32'b0			 	 , dreaddata_mem_wb_i[31:0]}; 			// lwu
						3'b011:  b = dreaddata_mem_wb_i[63:0];					 	  			// ld
						default: b = dreaddata_mem_wb_i[63:0];
					endcase
				default:b = (isNotImm == 1'b0)? rs2_data_o : {{(`REG_WIDTH-13){instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
			endcase 
		end
	logic  JudgeE, JudgeL, JudgeUL; 
	assign JudgeE = (a == b);
	assign JudgeL = ($signed(a) <  $signed(b));
	assign JudgeUL= (a <  b);

	always_comb
	begin  
		pc 			= input_pc;
		rs1addr     = `REG_ADDR_WIDTH'h0;
		rs2addr     = `REG_ADDR_WIDTH'h0;
		rs1_data_o  = `REG_WIDTH'h0;
		rs2_data_o  = `REG_WIDTH'h0;
		rdAddr      = `REG_ADDR_WIDTH'h0;

		memread     = 1'b0;
		memtoreg    = 1'b0;
		memwrite    = 1'b0;

		isNotImm    = 1'b0;
		imm         = `REG_WIDTH'h0;
		aluop       = 2'b00;
		regwrite    = 1'b0;

		Iswaitedflag_data= 1'b0;
		Iswaitedflag_addr= 1'b0;
		Iswaitedflag_alu = 1'b0;
		flag_b = 1'b0;
		pc_b   = 0; 
		case(opcode) 
			`INSTR_R:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					Iswaitedflag_alu = (funct7[0] == 1'b1)?1'b1:1'b0;
				end
			`INSTR_I:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b1;
					imm         = {{(`REG_WIDTH-12){instr[31]}}, instr[31:20]};
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_B:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = 5'b0;

					memtoreg   = 1'b0;
					memread    = 1'b0;
					memwrite   = 1'b0;
					isNotImm   = 1'b0;
					imm        = {{(`REG_WIDTH-13){instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
					aluop      = 2'b01;
					regwrite   = 1'b0;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					case(funct3)
						3'b000: // beq
						begin
							pc_b = (JudgeE == 1'b1)? (input_pc+imm) : input_pc;
							flag_b = (JudgeE == 1'b1);
						end
						3'b001: // bne
						begin
							pc_b = (JudgeE == 1'b0)? (input_pc+imm) : input_pc;
							flag_b = (JudgeE == 1'b0);
						end
						3'b100: // blt
						begin
							pc_b = (JudgeL == 1'b1)? (input_pc+imm) : input_pc;
							flag_b = (JudgeL == 1'b1);
						end
						3'b101: // bge
						begin
							pc_b = (JudgeL == 1'b0)? (input_pc+imm) : input_pc;
							flag_b = (JudgeL == 1'b0);
						end
						3'b110: // bltu
						begin
							pc_b = (JudgeUL == 1'b1)? (input_pc+imm) : input_pc;
							flag_b = (JudgeUL == 1'b1);
						end
						3'b111: // bgeu
						begin
							pc_b = (JudgeUL == 1'b0)? (input_pc+imm) : input_pc;
							flag_b = (JudgeUL == 1'b0);
						end
						default:
							pc = input_pc;
					endcase
				end 
			`INSTR_J:
				begin
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-21){instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_JR:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-12){instr[31]}},instr[31:20]};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_U:
				begin
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-32){instr[31]}}, instr[31:12], 12'b0};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_UPC:
				begin
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = {{(`REG_WIDTH-32){instr[31]}}, instr[31:12], 12'b0};
					aluop       = 2'b11;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_S:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = `REG_ADDR_WIDTH'h0;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b1;
					isNotImm    = 1'b1;
					imm        	= {{(`REG_WIDTH-12){instr[31]}}, instr[31:25], instr[11:7]};
					aluop       = 2'b00;
					regwrite    = 1'b0;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b1;
				end
			`INSTR_IL:
				begin
					rs1addr 	= rs1;
					rs2addr 	= `REG_ADDR_WIDTH'h0;
					rs1_data_o 	= rs1data;
					rs2_data_o 	= `REG_WIDTH'h0;
					rdAddr  	= rd;

					memtoreg   	= 1'b1;
					memread    	= 1'b1;
					memwrite   	= 1'b0;
					isNotImm   	= 1'b1;
					imm        	= {{(`REG_WIDTH-12){instr[31]}}, instr[31:20]};
					aluop      	= 2'b00;
					regwrite   	= 1'b1;
					Iswaitedflag_data= 1'b1;
					Iswaitedflag_addr= 1'b0;
           	 	end
			`INSTR_RW:
				begin
					rs1addr     = rs1;
					rs2addr     = rs2;
					rs1_data_o  = rs1data;
					rs2_data_o  = rs2data;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					Iswaitedflag_alu = (funct7[0] == 1'b1)?1'b1:1'b0;
           	 	end
			`INSTR_IW:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b1;
					imm         = {{(`REG_WIDTH-12){instr[31]}}, instr[31:20]};
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			`INSTR_CSR:
				begin
					rs1addr     = rs1;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = rs1data;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = rd;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b1;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b10;
					regwrite    = 1'b1;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
					case(funct3)
						`OR:		isNotImm    = 1'b1;
						`AND:		isNotImm    = 1'b1;
						`SLL:		isNotImm    = 1'b0;
						`SRL_SRA:	isNotImm    = 1'b1;
						`SLT:		isNotImm    = 1'b0;
						`SLTU:		isNotImm    = 1'b0;
						default:	isNotImm	= 1'b0;
					endcase
				end
			default:
				begin
					pc = input_pc;
					rs1addr     = `REG_ADDR_WIDTH'h0;
					rs2addr     = `REG_ADDR_WIDTH'h0;
					rs1_data_o  = `REG_WIDTH'h0;
					rs2_data_o  = `REG_WIDTH'h0;
					rdAddr      = `REG_ADDR_WIDTH'h0;

					memtoreg    = 1'b0;
					memread     = 1'b0;
					memwrite    = 1'b0;
					isNotImm    = 1'b0;
					imm         = `REG_WIDTH'h0;
					aluop       = 2'b00;
					regwrite    = 1'b0;
					Iswaitedflag_data= 1'b0;
					Iswaitedflag_addr= 1'b0;
				end
			endcase 
	end
	endmodule 

module id_ex_regs
	import defines::*;(
	input  clk,
	input  rst,hand_in,
	input  logic [`REG_WIDTH-1:0] 		pc_id_ex_i,
	input  logic [`REG_WIDTH-1:0] 		rs1data_id_ex_i,
	input  logic [`REG_WIDTH-1:0] 		rs2data_id_ex_i,
	input  logic [`REG_ADDR_WIDTH-1:0] 	rdAddr_id_ex_i,
	input  logic  						memread_id_ex_i,
	input  logic 						memtoreg_id_ex_i,
	input  logic  						memtowrite_id_ex_i,
	input  logic 						isNotImm_id_ex_i,
	input  logic 						regwrite_id_ex_i,
	input  logic [`REG_WIDTH-1:0]		imm_id_ex_i,
	input  logic [`ALUOP_WIDTH-1:0]		aluop_id_ex_i,
	input  logic [2:0]					funct3_id_ex_i,
	input  logic [6:0]					funct7_id_ex_i,
	input  logic [6:0]					opcode_id_ex_i,
	input  logic 						Iswaitedflag_data_id_ex_i,
	input  logic 						Iswaitedflag_addr_id_ex_i,
	input  logic 						Iswaitedflag_alu_id_ex_i,
	input  logic [`REG_ADDR_WIDTH-1:0] 	rs1addr_id_ex_i,
	input  logic [`REG_ADDR_WIDTH-1:0] 	rs2addr_id_ex_i,
	input  logic 						valid_id_ex_i,

	input  logic  						csr_we_id_ex_i,
	input  logic  						is_mret_id_ex_i,
	input  logic [11:0]				csr_addr_id_ex_i,
	input  logic [`REG_WIDTH-1:0]  		zimm_id_ex_i,
	input  logic  						ecall_id_ex_i,
	
	output logic [`REG_WIDTH-1:0] 		pc_id_ex_o,
	output logic [`REG_WIDTH-1:0] 		rs1data_id_ex_o,
	output logic [`REG_WIDTH-1:0] 		rs2data_id_ex_o,
	output logic [`REG_ADDR_WIDTH-1:0] 	rdAddr_id_ex_o,
	output logic  						memread_id_ex_o,
	output logic 						memtoreg_id_ex_o,
	output logic  						memtowrite_id_ex_o,
	output logic 						isNotImm_id_ex_o,
	output logic 						regwrite_id_ex_o,
	output logic [`REG_WIDTH-1:0]		imm_id_ex_o,
	output logic [`ALUOP_WIDTH-1:0]		aluop_id_ex_o,
	output logic [2:0]					funct3_id_ex_o,
	output logic [6:0]					funct7_id_ex_o,
	output logic [6:0]					opcode_id_ex_o,
	output logic 						Iswaitedflag_data_id_ex_o,
	output logic 						Iswaitedflag_addr_id_ex_o,
	output logic 						Iswaitedflag_alu_id_ex_o,
	output logic [`REG_ADDR_WIDTH-1:0] 	rs1addr_id_ex_o,
	output logic [`REG_ADDR_WIDTH-1:0] 	rs2addr_id_ex_o,
	output logic						valid_id_ex_o,
	input  logic						flag_j,
	input  logic						flag_b,
	input  logic						stallFlag,

	output logic  						csr_we_id_ex_o,
	output logic  						is_mret_id_ex_o,
	output logic [11:0]				csr_addr_id_ex_o,
	output logic [`REG_WIDTH-1:0]  		zimm_id_ex_o,
	output logic  						ecall_id_ex_o
    );
	logic flag_clear, flag_clear_pcvalid;
	assign flag_clear = flag_j | flag_b | stallFlag;
	assign flag_clear_pcvalid = flag_j | stallFlag;
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear_pcvalid&hand_in)
				pc_id_ex_o <= 0;
			else if(hand_in)
				pc_id_ex_o <= pc_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	pc_id_ex_o <= 0;
			else
				pc_id_ex_o <= pc_id_ex_o;
		end

	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				csr_we_id_ex_o <= 0;
			else if(hand_in)
				csr_we_id_ex_o <= csr_we_id_ex_i;
			else
				csr_we_id_ex_o <= csr_we_id_ex_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				is_mret_id_ex_o <= 0;
			else if(hand_in)
				is_mret_id_ex_o <= is_mret_id_ex_i;
			else
				is_mret_id_ex_o <= is_mret_id_ex_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				csr_addr_id_ex_o <= 0;
			else if(hand_in)
				csr_addr_id_ex_o <= csr_addr_id_ex_i;
			else
				csr_addr_id_ex_o <= csr_addr_id_ex_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				zimm_id_ex_o <= 0;
			else if(hand_in)
				zimm_id_ex_o <= zimm_id_ex_i;
			else
				zimm_id_ex_o <= zimm_id_ex_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				ecall_id_ex_o <= 0;
			else if(hand_in)
				ecall_id_ex_o <= ecall_id_ex_i;
			else
				ecall_id_ex_o <= ecall_id_ex_o;
		end	
	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				rs1data_id_ex_o <= 0;
			else if(hand_in)
				rs1data_id_ex_o <= rs1data_id_ex_i;
			else
				rs1data_id_ex_o <= rs1data_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				rs2data_id_ex_o <= 0;
			else if(hand_in)
				rs2data_id_ex_o <= rs2data_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	rs2data_id_ex_o <= 0;
			else
				rs2data_id_ex_o <= rs2data_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				rdAddr_id_ex_o <= 0;
			else if(hand_in)
				rdAddr_id_ex_o <= rdAddr_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	rdAddr_id_ex_o <= 0;
			else
				rdAddr_id_ex_o <= rdAddr_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				memread_id_ex_o <= 0;
			else if(hand_in)
				memread_id_ex_o <= memread_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	memread_id_ex_o <= 0;
			else
				memread_id_ex_o <= memread_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				memtoreg_id_ex_o <= 0;
			else if(hand_in)
				memtoreg_id_ex_o <= memtoreg_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	memtoreg_id_ex_o <= 0;
			else
				memtoreg_id_ex_o <= memtoreg_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				memtowrite_id_ex_o <= 0;
			else if(hand_in)
				memtowrite_id_ex_o <= memtowrite_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	rs2data_id_ex_o <= 0;
			else
				memtowrite_id_ex_o <= memtowrite_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				isNotImm_id_ex_o <= 0;
			else if(hand_in)
				isNotImm_id_ex_o <= isNotImm_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	isNotImm_id_ex_o <= 0;
			else
				isNotImm_id_ex_o <= isNotImm_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				regwrite_id_ex_o <= 0;
			else if(hand_in)
				regwrite_id_ex_o <= regwrite_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	regwrite_id_ex_o <= 0;
			else
				regwrite_id_ex_o <= regwrite_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				imm_id_ex_o <= 0;
			else if(hand_in)
				imm_id_ex_o <= imm_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	imm_id_ex_o <= 0;
			else
				imm_id_ex_o <= imm_id_ex_o;	
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				aluop_id_ex_o <= 0;
			else if(hand_in)
				aluop_id_ex_o <= aluop_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	aluop_id_ex_o <= 0;
			else
				aluop_id_ex_o <= aluop_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				funct3_id_ex_o <= 0;
			else if(hand_in)
				funct3_id_ex_o <= funct3_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	funct3_id_ex_o <= 0;
			else
				funct3_id_ex_o <= funct3_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				funct7_id_ex_o <= 0;
			else if(hand_in)
				funct7_id_ex_o <= funct7_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	funct7_id_ex_o <= 0;
			else
				funct7_id_ex_o <= funct7_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				opcode_id_ex_o <= 0;
			else if(hand_in)
				opcode_id_ex_o <= opcode_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	opcode_id_ex_o <= 0;
			else
				opcode_id_ex_o <= opcode_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				Iswaitedflag_data_id_ex_o <= 0;
			else if(hand_in)
				Iswaitedflag_data_id_ex_o <= Iswaitedflag_data_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	Iswaitedflag_data_id_ex_o <= 0;
			else
				Iswaitedflag_data_id_ex_o <= Iswaitedflag_data_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				Iswaitedflag_addr_id_ex_o <= 0;
			else if(hand_in)
				Iswaitedflag_addr_id_ex_o <= Iswaitedflag_addr_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	Iswaitedflag_addr_id_ex_o <= 0;
			else
				Iswaitedflag_addr_id_ex_o <= Iswaitedflag_addr_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				Iswaitedflag_alu_id_ex_o <= 0;
			else if(hand_in)
				Iswaitedflag_alu_id_ex_o <= Iswaitedflag_alu_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	Iswaitedflag_alu_id_ex_o <= 0;
			else
				Iswaitedflag_alu_id_ex_o <= Iswaitedflag_alu_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				rs1addr_id_ex_o <= 0;
			else if(hand_in)
				rs1addr_id_ex_o <= rs1addr_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	rs1addr_id_ex_o <= 0;
			else
				rs1addr_id_ex_o <= rs1addr_id_ex_o;
		end	
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear&hand_in)
				rs2addr_id_ex_o <= 0;
			else if(hand_in)
				rs2addr_id_ex_o <= rs2addr_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	rs2addr_id_ex_o <= 0;
			else
				rs2addr_id_ex_o <= rs2addr_id_ex_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_clear_pcvalid & hand_in)
				valid_id_ex_o <= 0;
			else if(hand_in)
				valid_id_ex_o <= valid_id_ex_i;
			// else if((~Iswaitedflag_alu_id_ex_i)|aludone_ex_mem_i)
			// 	valid_id_ex_o <= 0;
			else
				valid_id_ex_o <= valid_id_ex_o;
		end	
	endmodule

module ALU
    import defines::*;(
	input logic						clk, rst, hand_in,drespdataok,
	input logic [1:0]				forwardae, forwardbe,
    input logic [`REG_WIDTH-1:0]    aluout_ex_mem_o,wb_data,dreaddata_mem_wb_i,
    input logic [`REG_WIDTH-1:0]    pc_i,
    input logic [`REG_WIDTH-1:0]    rs1_data_i,
    input logic [`REG_WIDTH-1:0]    rs2_data_i,
    input logic [2:0]               funct3,
    input logic [6:0]               funct7,
    input logic [`REG_WIDTH-1:0]    imm,
    input logic [6:0]               opcode,

    input logic                     isNotImm,
    input logic [`ALUOP_WIDTH-1:0]  aluop,
    
    output logic                    branchFlag,

    output logic [`REG_WIDTH-1:0]   result,
    output logic [`REG_WIDTH-1:0]   result_ff,
    output logic [`REG_WIDTH-1:0]   pc_o,
	output logic 					done,
	output logic [`REG_WIDTH-1:0]	writedata,
	input  logic [2:0] 				func3_dbusdone,

	input  logic [`REG_WIDTH-1:0] 	csr_rdata,
    input  logic [`REG_WIDTH-1:0] 	zimm,
    output logic [`REG_WIDTH-1:0] 	csr_wdata,
    output logic [`REG_WIDTH-1:0] 	csr_result,

	input  logic [1:0]				forward_csr,
    input  logic [`REG_WIDTH-1:0] 	csr_wdata_ex_mem_o,
    input  logic [`REG_WIDTH-1:0] 	csr_wdata_mem_wb_o,

    input logic [6:0]               opcode_ex_mem_o,
    input logic [6:0]               opcode_mem_wb_o,
    input  logic [`REG_WIDTH-1:0] 	csr_result_ex_mem_o,
    input  logic [`REG_WIDTH-1:0] 	csr_result_mem_wb_o
	);

	logic [`REG_WIDTH-1:0] a, b;
	logic [`REG_WIDTH-1:0] csr_rdata_true;
	always_comb 
		begin
			case(forwardae)
				2'b00:	a = rs1_data_i;
				2'b01:	a = (opcode_mem_wb_o == `INSTR_CSR) ? csr_result_mem_wb_o:wb_data;
				2'b10:	a = (opcode_ex_mem_o == `INSTR_CSR) ? csr_result_ex_mem_o:aluout_ex_mem_o;
				2'b11:	
					begin
						case(func3_dbusdone[2:0])
							3'b000:  a = {{56{dreaddata_mem_wb_i[7]}}, dreaddata_mem_wb_i[7:0]};  	// lb
							3'b100:  a = {56'b0			  	 , dreaddata_mem_wb_i[7:0]};  			// lbu
							3'b001:  a = {{48{dreaddata_mem_wb_i[15]}}, dreaddata_mem_wb_i[15:0]};	// lh
							3'b101:  a = {48'b0			  	 , dreaddata_mem_wb_i[15:0]}; 			// lhu
							3'b010:  a = {{32{dreaddata_mem_wb_i[31]}}, dreaddata_mem_wb_i[31:0]};	// lw
							3'b110:  a = {32'b0			 	 , dreaddata_mem_wb_i[31:0]}; 			// lwu
							3'b011:  a = dreaddata_mem_wb_i[63:0];					 	  			// ld
							default: a = dreaddata_mem_wb_i[63:0];
						endcase
					end
				default:
					a = rs1_data_i; 
			endcase
			case(forwardbe)
				2'b00:	b = (isNotImm == 1'b0)? rs2_data_i : imm;
				2'b01:	b = (opcode_mem_wb_o == `INSTR_CSR) ? csr_result_mem_wb_o:wb_data;
				2'b10:	b = (opcode_ex_mem_o == `INSTR_CSR) ? csr_result_ex_mem_o:aluout_ex_mem_o;
				2'b11:
				begin
					case(func3_dbusdone[2:0])
						3'b000:  b = {{56{dreaddata_mem_wb_i[7]}}, dreaddata_mem_wb_i[7:0]};  	// lb
						3'b100:  b = {56'b0			  	 , dreaddata_mem_wb_i[7:0]};  			// lbu
						3'b001:  b = {{48{dreaddata_mem_wb_i[15]}}, dreaddata_mem_wb_i[15:0]};	// lh
						3'b101:  b = {48'b0			  	 , dreaddata_mem_wb_i[15:0]}; 			// lhu
						3'b010:  b = {{32{dreaddata_mem_wb_i[31]}}, dreaddata_mem_wb_i[31:0]};	// lw
						3'b110:  b = {32'b0			 	 , dreaddata_mem_wb_i[31:0]}; 			// lwu
						3'b011:  b = dreaddata_mem_wb_i[63:0];					 	  			// ld
						default: b = dreaddata_mem_wb_i[63:0];
					endcase
				end
				default:
					b = (isNotImm == 1'b0)? rs2_data_i : imm;
			endcase 
			case(forwardbe)
				2'b00:	writedata = rs2_data_i;
				2'b01:	writedata = (opcode_mem_wb_o == `INSTR_CSR) ? csr_result_mem_wb_o:wb_data;
				2'b10:	writedata = (opcode_ex_mem_o == `INSTR_CSR) ? csr_result_ex_mem_o:aluout_ex_mem_o;
				2'b11:
				begin
					case(func3_dbusdone[2:0])
						3'b000:  writedata = {{56{dreaddata_mem_wb_i[7]}}, dreaddata_mem_wb_i[7:0]};  	// lb
						3'b100:  writedata = {56'b0			  	 , dreaddata_mem_wb_i[7:0]};  			// lbu
						3'b001:  writedata = {{48{dreaddata_mem_wb_i[15]}}, dreaddata_mem_wb_i[15:0]};	// lh
						3'b101:  writedata = {48'b0			  	 , dreaddata_mem_wb_i[15:0]}; 			// lhu
						3'b010:  writedata = {{32{dreaddata_mem_wb_i[31]}}, dreaddata_mem_wb_i[31:0]};	// lw
						3'b110:  writedata = {32'b0			 	 , dreaddata_mem_wb_i[31:0]}; 			// lwu
						3'b011:  writedata = dreaddata_mem_wb_i[63:0];					 	  			// ld
						default: writedata = dreaddata_mem_wb_i[63:0];
					endcase
				end
				default:
					writedata = rs2_data_i;
			endcase 
			case(forward_csr)
				2'b11:	csr_rdata_true = csr_wdata_ex_mem_o;//ex
				2'b01:	csr_rdata_true = csr_wdata_mem_wb_o;//mem
				default: csr_rdata_true = csr_rdata;
			endcase
		end
	// assign a = rs1_data_i;
	// assign b = (isNotImm == 1'b0)? rs2_data_i : imm;

	logic  JudgeE, JudgeL, JudgeUL; 
	assign JudgeE = (a == b);
	assign JudgeL = ($signed(a) <  $signed(b));
	assign JudgeUL= (a <  b);
	logic  isMul;
	logic  isMulw;
	logic  isDiv;
	logic  isDivw;
	logic  isDivu;
	logic  isDivuw;
	logic  isRem;
	logic  isRemw;
	logic  isRemu;
	logic  isRemuw;
	logic  valid_w;
	assign valid_w= isDivw|isDivuw|isRemw|isRemuw;
	logic  muldone;
	logic  divdone1;
	logic  divdone2;
	logic  [`REG_WIDTH-1:0] result_mul;
	logic  [`REG_WIDTH-1:0] result_div1;
	logic  [31:0] 			result_div2;

	logic [63:0] tem64;
	logic [31:0] tem32;
	always_comb
	begin
		tem64 		= 'b0;
		tem32 		= 'b0;
		branchFlag  = 1'b0;
		result      = `REG_WIDTH'b0;
		pc_o        = pc_i;
		isMul  		= 0;
		isMulw		= 0;
		isDiv  		= 0;
		isDivw		= 0;
		isDivu 		= 0;
		isDivuw		= 0;
		isRem  		= 0;
		isRemw		= 0;
		isRemu 		= 0;
		isRemuw 	= 0;

		csr_wdata 	= 0;
		csr_result 	= 0;
		case(aluop)
			2'b10:
				begin
					case(funct3)
						`ADD://Actually ADD or SUB or MUL
							begin
								if(opcode == `INSTR_I) // ADDI
								begin 
									result = a + b;
								end
								else if(opcode == `INSTR_IW)  // ADDIW
								begin
									// ADD IMM to 32 signed to 64
									tem64 = a + b;
									tem32 = tem64[31:0];
									result = {{32{tem32[31]}}, tem32};
								end
								else if(opcode == `INSTR_RW && funct7[0] == 1'b0) // ADDW OR SUBW
								begin
									tem64 = (funct7[5] == 1'b0)? (a + b):(a - b);
									tem32 = tem64[31:0];
									result = {{32{tem32[31]}}, tem32};
								end
								else if(opcode == `INSTR_RW && funct7[0] == 1'b1) // MULW
								begin
									isMul = 1;
									isMulw= 1;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b0) // ADD OR SUB
								begin 
									result = (funct7[5] == 1'b0)? (a + b):(a - b);
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1) // MUL
								begin 
									isMul = 1;
								end
							end
						`XOR://Actually XOR or DIV
							begin
								if(opcode == `INSTR_I || ((funct7[0] == 1'b0) && (opcode == `INSTR_R)))		// XOR or XORI
								begin
									result = a ^ b;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1)		// DIV
								begin
									isDiv = 1;
								end
								else if(opcode == `INSTR_RW && funct7[0] == 1'b1)		// DIVW
								begin
									isDiv = 1;
									isDivw= 1;
								end
							end
						`OR: //Actually OR or REM
							begin 
								if(opcode == `INSTR_I || ((funct7[0] == 1'b0) && (opcode == `INSTR_R)))		// OR
								begin
									result = a | b;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1)	// REM
								begin
									isRem = 1;
								end
								else if(opcode == `INSTR_RW&& funct7[0] == 1'b1)	// REMW
								begin
									isRem = 1;
									isRemw= 1;
								end
								else if(opcode == `INSTR_CSR)		// CSRRSI
								begin
									csr_wdata = csr_rdata | zimm;
									csr_result = csr_rdata;
								end
							end 
						`AND://Actually AND or REMU
							begin 
								if(opcode == `INSTR_I || ((funct7[0] == 1'b0) && (opcode == `INSTR_R)))		// AND
								begin
									result = a & b;
								end
								else if(opcode == `INSTR_R && funct7[0] == 1'b1)	// REMU
								begin
									isRemu = 1;
								end
								else if(opcode == `INSTR_RW&& funct7[0] == 1'b1)	// REMUW
								begin
									isRemu = 1;
									isRemuw= 1;
								end
								else if(opcode == `INSTR_CSR)		// CSRRCI
								begin
									csr_wdata = csr_rdata & ~zimm;
									csr_result = csr_rdata;
								end
							end 
						`SLL://Actually SLL
							begin
								case(opcode)
									`INSTR_R:							// SLL
									begin 
										result = a << b[5:0];
									end
									`INSTR_I: 							// SLLI
									begin 
										result = a << b[5:0];
									end
									`INSTR_RW: 							// SLLW
									begin
										tem64 = a << b[4:0];
										tem32 = tem64[31:0];
										result = {{32{tem32[31]}}, tem32};
									end
									`INSTR_IW: 							// SLLIW
									begin
										if(b[5] == 0)
										begin
											tem64 = a << b[4:0];
											tem32 = tem64[31:0];
											result = {{32{tem32[31]}}, tem32};
										end
									end
									`INSTR_CSR: 						// CSRRW
									begin
										csr_wdata = a;
										csr_result = csr_rdata;
									end
									default:
										result = 'b0;
								endcase
							end
						`SRL_SRA: //Actually SRA or SRL or DIVU
							begin
								if(opcode == `INSTR_CSR)		// CSRRWI
								begin
									csr_wdata = zimm;
									csr_result = csr_rdata;
								end
								else if(funct7[5] == 1'b1) 			// SRA
								begin
									case(opcode)
										`INSTR_R:
										begin
											if(funct7[0] == 0)		// SRA
											begin
												result = (a >> b[5:0]) | (( {64{a[63]}} << (64 - {1'b0, b[5:0]}) ));
											end
										end
										`INSTR_I: 				// SRAI
										begin
											result = (a >> b[5:0]) | (( {64{a[63]}} << (64 - {1'b0, b[5:0]}) ));
										end
										`INSTR_RW: 				// SRAW
										begin
											tem32 = (a[31:0] >> b[4:0]) | (( {32{a[31]}} << (32 - {1'b0, b[4:0]}) ));
											result= {{32{tem32[31]}}, tem32};
										end 
										`INSTR_IW: 				// SRAIW
										begin
											if(b[5] == 0)
											begin
												tem32 = (a[31:0] >> b[4:0]) | (( {32{a[31]}} << (32 - {1'b0, b[4:0]}) ));
												result= {{32{tem32[31]}}, tem32};
											end
										end
										default:
											result = 'b0;
									endcase 
								end
								else 							// SRL or DIVU
								begin  
									case(opcode)
									`INSTR_R:				// SRL or DIVU
									begin
										if(funct7[0] == 0)
										begin
											result = (a >> b[5:0]) & (~( {64{1'b1}} << (64 - {1'b0, b[5:0]}) ));
										end
										else if(funct7[0] == 1) // DIVU
										begin
											isDivu = 1;
										end
									end
									`INSTR_I: 				// SRLI
									begin
										result = (a >> b[5:0]) & (~( {64{1'b1}} << (64 - {1'b0, b[5:0]}) ));
									end
									`INSTR_RW: 				// SRLW or DIVUW
									begin
										if(funct7[0] == 0) 	// SRLW 
										begin
											tem32 = (a[31:0] >> b[4:0]) & (~( {32{1'b1}} << (32 - {1'b0, b[4:0]}) ));
											result= {{32{tem32[31]}}, tem32};
										end
										else if(funct7[0] == 1) // DIVUW
										begin
											isDivu = 1;
											isDivuw = 1;
										end
									end 
									`INSTR_IW: 				// SRLIW
									begin
										if(b[5] == 0)
										begin
											tem32 = (a[31:0] >> b[4:0]) & (~( {32{1'b1}} << (32 - {1'b0, b[4:0]}) ));
											result= {{32{tem32[31]}}, tem32};
										end
									end
									default:
										result = 'b0;
									endcase
								end
							end
						`SLT:
							begin 
								if(opcode == `INSTR_CSR)		// CSRRS
								begin
									csr_wdata = csr_rdata | a;
									csr_result = csr_rdata;
								end
								else
								begin
									result = ($signed(a) < $signed(b))? `REG_WIDTH'b1 : `REG_WIDTH'b0;
								end
							end 
						`SLTU:
							begin 
								if(opcode == `INSTR_CSR)	// CSRRC
								begin
									csr_wdata = csr_rdata & ~a;
									csr_result = csr_rdata;
								end
								else
								begin
									result = (a < b)? `REG_WIDTH'b1 : `REG_WIDTH'b0;
								end
							end 
						default:
							result = 'b0;
					endcase
				end 
			2'b01: // B型指�????
				begin
					case(funct3)
						3'b000: // beq
						begin
							pc_o = (JudgeE == 1'b1)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeE == 1'b1)? 1'b1 : 1'b0;
						end
						3'b001: // bne
						begin
							pc_o = (JudgeE == 1'b0)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeE == 1'b1)? 1'b0 : 1'b1;
						end
						3'b100: // blt
						begin
							pc_o = (JudgeL == 1'b1)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeL == 1'b1)? 1'b1 : 1'b0;
						end
						3'b101: // bge
						begin
							pc_o = (JudgeL == 1'b0)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeL == 1'b1)? 1'b0 : 1'b1;
						end
						3'b110: // bltu
						begin
							pc_o = (JudgeUL == 1'b1)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeUL == 1'b1)? 1'b1 : 1'b0;
						end
						3'b111: // bgeu
						begin
							pc_o = (JudgeUL == 1'b0)? (pc_i+imm) : pc_i;
							branchFlag = (JudgeUL == 1'b1)? 1'b0 : 1'b1;
						end
						default:
							result = 'b0;
					endcase    
				end
			2'b11: // J、JR、U、UPC型指�????
				begin
					case(opcode)
						`INSTR_J:begin 
							result  = pc_i + `REG_WIDTH'h4;
							pc_o    = pc_i + imm;
							branchFlag = 1'b1;
						end
						`INSTR_JR:begin 
							result  = pc_i + `REG_WIDTH'h4;
							pc_o    = a + imm;
							branchFlag = 1'b1;
						end
						`INSTR_U:begin 
							result  = imm;
						end 
						`INSTR_UPC:begin 
							result  = pc_i + imm;
						end
						default:
							result = 'b0;
					endcase
				end
			2'b00: // IL和S型的指令
				begin
					result = a + imm;
				end
			default:
				result = 'b0;
		endcase
	end

	multiplier_multicycle multiplier_multicycle(
		.clk(clk), .rst(rst), .valid(isMul),.hand_in(hand_in|drespdataok),
		.a(a), .b(b), .c(result_mul),
		.done(muldone)
		);

	divider_multicycle #(64) divider_multicycle1 (
		.clk(clk), .rst(rst), .valid_div(isDiv), .valid_divu(isDivu), .valid_rem(isRem), .valid_remu(isRemu), .valid_w(valid_w),
		.a(a), .b(b), .c(result_div1),
		.done(divdone1),.hand_in(hand_in|drespdataok)
		);
	divider_multicycle #(32) divider_multicycle2 (
		.clk(clk), .rst(rst), .valid_div(isDiv), .valid_divu(isDivu), .valid_rem(isRem), .valid_remu(isRemu), .valid_w(valid_w),
		.a(a[31:0]), .b(b[31:0]), .c(result_div2),
		.done(divdone2),.hand_in(hand_in|drespdataok)
		);

	assign done = ((isMul & muldone) | ((isDiv|isDivu|isRem|isRemu) & divdone1 & (~valid_w)) | (divdone2 & valid_w));
	always_comb
	begin
		result_ff = 0;
		if(isMul == 1 && isMulw == 0) begin
			result_ff = result_mul;
		end
		else if(isMul == 1 && isMulw == 1) begin
			result_ff = {{32{result_mul[31]}}, result_mul[31:0]};
		end
		else if(isRemuw|isRemw|isDivw|isDivuw) begin
			result_ff = {{32{result_div2[31]}}, result_div2[31:0]};
		end
		else if(isDiv|isDivu|isRem|isRemu) begin
			result_ff = result_div1;
		end
	end 
	endmodule 

module ALUreg
    import defines::*;(
    input  logic                        clk,
    input  logic                        rst,
    input  logic                        hand_in,drespdataok,
	input  logic 				   	tembranchFlag_ex_mem_i,
	input  logic [`REG_WIDTH-1:0]  	temaluout_ex_mem_i,
	input  logic [`REG_WIDTH-1:0]  	temaluoutff_ex_mem_i,
	input  logic [`REG_WIDTH-1:0]  	temalupc_ex_mem_i,
	input  logic 				   	temaludone_ex_mem_i,

	output logic 				   	branchFlag_ex_mem_i,
	output logic [`REG_WIDTH-1:0]  	aluout_ex_mem_i,
	output logic [`REG_WIDTH-1:0]  	aluoutff_ex_mem_i,
	output logic [`REG_WIDTH-1:0]  	alupc_ex_mem_i,
	output logic 				   	aludone_ex_mem_i
	);
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			branchFlag_ex_mem_i <= 0;
		else if(temaludone_ex_mem_i)
			branchFlag_ex_mem_i <= tembranchFlag_ex_mem_i;
		else
			branchFlag_ex_mem_i <= branchFlag_ex_mem_i;
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			aluout_ex_mem_i <= 0;
		else if(temaludone_ex_mem_i)
			aluout_ex_mem_i <= temaluout_ex_mem_i;
		else
			aluout_ex_mem_i <= aluout_ex_mem_i;
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			aluoutff_ex_mem_i <= 0;
		else if(temaludone_ex_mem_i)
			aluoutff_ex_mem_i <= temaluoutff_ex_mem_i;
		else
			aluoutff_ex_mem_i <= aluoutff_ex_mem_i;
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			alupc_ex_mem_i <= 0;
		else if(temaludone_ex_mem_i)
			alupc_ex_mem_i <= temalupc_ex_mem_i;
		else
			alupc_ex_mem_i <= alupc_ex_mem_i;
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			aludone_ex_mem_i <= 0;
		else if(hand_in)
			aludone_ex_mem_i <= 0;
		else if(drespdataok)
			aludone_ex_mem_i <= 0;
		else if(temaludone_ex_mem_i)
			aludone_ex_mem_i <= 1;
		else
			aludone_ex_mem_i <= aludone_ex_mem_i;
	end

	endmodule

module ex_mem_regs
	import defines::*;(
	input  clk,
	input  rst,hand_in,drespdataok,
	input  logic 				  branchFlag_ex_mem_i,
	input  logic [`REG_WIDTH-1:0] aluout_ex_mem_i,
	input  logic [`REG_WIDTH-1:0] aluoutff_ex_mem_i,
	input  logic [`REG_WIDTH-1:0] alupc_ex_mem_i,
	input  logic 				  aludone_ex_mem_i,
	input  logic 				  regwirte_ex_mem_i,
	input  logic 				  memtoreg_ex_mem_i,
	input  logic 				  memtowrite_ex_mem_i,
	input  logic [`REG_ADDR_WIDTH-1:0] rdAddr_ex_mem_i,
	input  logic 				  Iswaitedflag_data_ex_mem_i,
	input  logic 				  Iswaitedflag_addr_ex_mem_i,
	input  logic 				  Iswaitedflag_alu_ex_mem_i,
	input  logic [2:0]			  funct3_ex_mem_i,
	input  logic [`REG_WIDTH-1:0] writedata_ex_mem_i,
	input  logic [`REG_WIDTH-1:0] curpc_ex_mem_i,
	input  logic [`REG_ADDR_WIDTH-1:0] rs1addr_ex_mem_i,
	input  logic [`REG_ADDR_WIDTH-1:0] rs2addr_ex_mem_i,
	input  logic 				  valid_ex_mem_i,

	input  logic  						csr_we_ex_mem_i,
	input  logic  						is_mret_ex_mem_i,
	input  logic [11:0]				csr_addr_ex_mem_i,
	input  logic [`REG_WIDTH-1:0]  	zimm_ex_mem_i,
	input  logic  						ecall_ex_mem_i,
	input  logic [`REG_WIDTH-1:0]  	csr_wdata_ex_mem_i,
	input  logic [`REG_WIDTH-1:0]  	csr_result_ex_mem_i,
	input  logic [6:0]  				opcode_ex_mem_i,


	output logic 				  branchFlag_ex_mem_o,
	output logic [`REG_WIDTH-1:0] aluout_ex_mem_o,
	output logic [`REG_WIDTH-1:0] aluoutff_ex_mem_o,
	output logic [`REG_WIDTH-1:0] alupc_ex_mem_o,
	output logic 				  aludone_ex_mem_o,
	output logic 				  regwrite_ex_mem_o,
	output logic 				  memtoreg_ex_mem_o,
	output logic 				  memtowrite_ex_mem_o,
	output logic [`REG_ADDR_WIDTH-1:0] rdAddr_ex_mem_o,
	output logic 				  Iswaitedflag_data_ex_mem_o,
	output logic 				  Iswaitedflag_addr_ex_mem_o,
	output logic 				  Iswaitedflag_alu_ex_mem_o,
	output logic [2:0]			  funct3_ex_mem_o,
	output logic [`REG_WIDTH-1:0] writedata_ex_mem_o,
	output logic [`REG_WIDTH-1:0] curpc_ex_mem_o,
	output logic [`REG_ADDR_WIDTH-1:0] rs1addr_ex_mem_o,
	output logic [`REG_ADDR_WIDTH-1:0] rs2addr_ex_mem_o,
	output logic 				  valid_ex_mem_o,
	output logic 				  Iswaitedflag_data_ex_mem_o_flag,
	output logic 				  Iswaitedflag_addr_ex_mem_o_flag,
	input  logic 				  flag_j,

	output logic  						csr_we_ex_mem_o,
	output logic  						is_mret_ex_mem_o,
	output logic [11:0]				csr_addr_ex_mem_o,
	output logic [`REG_WIDTH-1:0]  	zimm_ex_mem_o,
	output logic  						ecall_ex_mem_o,
	output logic [`REG_WIDTH-1:0]  	csr_wdata_ex_mem_o,
	output logic [`REG_WIDTH-1:0]  	csr_result_ex_mem_o,
	output logic [6:0]  				opcode_ex_mem_o,

	input  logic					is_mret_mem_wb_o
    );

	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				csr_we_ex_mem_o <= 0;
			else if(hand_in)
				csr_we_ex_mem_o <= csr_we_ex_mem_i;
			else
				csr_we_ex_mem_o <= csr_we_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				is_mret_ex_mem_o <= 0;
			else if(hand_in)
				is_mret_ex_mem_o <= is_mret_ex_mem_i;
			else
				is_mret_ex_mem_o <= is_mret_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				csr_addr_ex_mem_o <= 0;
			else if(hand_in)
				csr_addr_ex_mem_o <= csr_addr_ex_mem_i;
			else
				csr_addr_ex_mem_o <= csr_addr_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				zimm_ex_mem_o <= 0;
			else if(hand_in)
				zimm_ex_mem_o <= zimm_ex_mem_i;
			else
				zimm_ex_mem_o <= zimm_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				ecall_ex_mem_o <= 0;
			else if(hand_in)
				ecall_ex_mem_o <= ecall_ex_mem_i;
			else
				ecall_ex_mem_o <= ecall_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				csr_wdata_ex_mem_o <= 0;
			else if(hand_in)
				csr_wdata_ex_mem_o <= csr_wdata_ex_mem_i;
			else
				csr_wdata_ex_mem_o <= csr_wdata_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				csr_result_ex_mem_o <= 0;
			else if(hand_in)
				csr_result_ex_mem_o <= csr_result_ex_mem_i;
			else
				csr_result_ex_mem_o <= csr_result_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				opcode_ex_mem_o <= 0;
			else if(hand_in)
				opcode_ex_mem_o <= opcode_ex_mem_i;
			else
				opcode_ex_mem_o <= opcode_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | flag_j)
				branchFlag_ex_mem_o <= 0;
			else if(hand_in)
				branchFlag_ex_mem_o <= branchFlag_ex_mem_i;
			else
				branchFlag_ex_mem_o <= branchFlag_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | is_mret_mem_wb_o)
				valid_ex_mem_o <= 0;
			else if(hand_in)
				valid_ex_mem_o <= valid_ex_mem_i;
			else
				valid_ex_mem_o <= valid_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				aluout_ex_mem_o <= 0;
			else if(hand_in)
				aluout_ex_mem_o <= aluout_ex_mem_i;
			else
				aluout_ex_mem_o <= aluout_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				aluoutff_ex_mem_o <= 0;
			else if(hand_in)
				aluoutff_ex_mem_o <= aluoutff_ex_mem_i;
			else
				aluoutff_ex_mem_o <= aluoutff_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				alupc_ex_mem_o <= 0;
			else if(hand_in)
				alupc_ex_mem_o <= alupc_ex_mem_i;
			else
				alupc_ex_mem_o <= alupc_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				aludone_ex_mem_o <= 0;
			else if(hand_in)
				aludone_ex_mem_o <= aludone_ex_mem_i;
			else
				aludone_ex_mem_o <= aludone_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				regwrite_ex_mem_o <= 0;
			else if(hand_in)
				regwrite_ex_mem_o <= regwirte_ex_mem_i;
			else
				regwrite_ex_mem_o <= regwrite_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				memtoreg_ex_mem_o <= 0;
			else if(hand_in)
				memtoreg_ex_mem_o <= memtoreg_ex_mem_i;
			else
				memtoreg_ex_mem_o <= memtoreg_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				memtowrite_ex_mem_o <= 0;
			else if(hand_in)
				memtowrite_ex_mem_o <= memtowrite_ex_mem_i;
			else
				memtowrite_ex_mem_o <= memtowrite_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				rdAddr_ex_mem_o <= 0;
			else if(hand_in)
				rdAddr_ex_mem_o <= rdAddr_ex_mem_i;
			else
				rdAddr_ex_mem_o <= rdAddr_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				Iswaitedflag_data_ex_mem_o <= 0;
			else if(hand_in)
				Iswaitedflag_data_ex_mem_o <= Iswaitedflag_data_ex_mem_i;
			else
				Iswaitedflag_data_ex_mem_o <= Iswaitedflag_data_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				Iswaitedflag_addr_ex_mem_o <= 0;
			else if(hand_in)
				Iswaitedflag_addr_ex_mem_o <= Iswaitedflag_addr_ex_mem_i;
			else
				Iswaitedflag_addr_ex_mem_o <= Iswaitedflag_addr_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				Iswaitedflag_alu_ex_mem_o <= 0;
			else if(hand_in)
				Iswaitedflag_alu_ex_mem_o <= Iswaitedflag_alu_ex_mem_i;
			else
				Iswaitedflag_alu_ex_mem_o <= Iswaitedflag_alu_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				funct3_ex_mem_o <= 0;
			else if(hand_in)
				funct3_ex_mem_o <= funct3_ex_mem_i;
			else
				funct3_ex_mem_o <= funct3_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				writedata_ex_mem_o <= 0;
			else if(hand_in)
				writedata_ex_mem_o <= writedata_ex_mem_i;
			else
				writedata_ex_mem_o <= writedata_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				curpc_ex_mem_o <= 0;
			else if(hand_in)
				curpc_ex_mem_o <= curpc_ex_mem_i;
			else
				curpc_ex_mem_o <= curpc_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				rs1addr_ex_mem_o <= 0;
			else if(hand_in)
				rs1addr_ex_mem_o <= rs1addr_ex_mem_i;
			else
				rs1addr_ex_mem_o <= rs1addr_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				rs2addr_ex_mem_o <= 0;
			else if(hand_in)
				rs2addr_ex_mem_o <= rs2addr_ex_mem_i;
			else
				rs2addr_ex_mem_o <= rs2addr_ex_mem_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | drespdataok)
				Iswaitedflag_data_ex_mem_o_flag <= 0;
			else if(hand_in)
				Iswaitedflag_data_ex_mem_o_flag <= Iswaitedflag_data_ex_mem_i;
			else
				Iswaitedflag_data_ex_mem_o_flag <= Iswaitedflag_data_ex_mem_o_flag;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst | drespdataok)
				Iswaitedflag_addr_ex_mem_o_flag <= 0;
			else if(hand_in)
				Iswaitedflag_addr_ex_mem_o_flag <= Iswaitedflag_addr_ex_mem_i;
			else
				Iswaitedflag_addr_ex_mem_o_flag <= Iswaitedflag_addr_ex_mem_o_flag;
		end

	endmodule

module Memreg
    import defines::*;(
    input  logic                   clk,
    input  logic                   rst,
	input  logic 				   dbusdataok_i,
	input  logic [`REG_WIDTH-1:0]  dbusdata_i,
	output logic 				   dbusdataok_o,
	output logic [`REG_WIDTH-1:0]  dbusdata_o
	);
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			dbusdataok_o <= 0;
		else if(dbusdataok_i)
			dbusdataok_o <= dbusdataok_i;
		else
			dbusdataok_o <= dbusdataok_o;
	end
	always@(posedge clk or posedge rst)
	begin
		if(rst)
			dbusdata_o <= 0;
		else if(dbusdataok_i)
			dbusdata_o <= dbusdata_i;
		else
			dbusdata_o <= dbusdata_o;
	end
	endmodule

module mem_wb_regs
	import defines::*;(
	input  clk,
	input  rst,hand_in,
	input  logic [`REG_WIDTH-1:0]   	aluout_mem_wb_i,
	input  logic [`REG_WIDTH-1:0]   	aluoutff_mem_wb_i,
	input  logic [`REG_WIDTH-1:0]   	dreaddata_mem_wb_i,
	input  logic    			regwrite_mem_wb_i,
	input  logic    			memtoreg_mem_wb_i,
	input  logic [`REG_ADDR_WIDTH-1:0] rdAddr_mem_wb_i,
	input  logic 				  Iswaitedflag_data_mem_wb_i,
	input  logic 				  Iswaitedflag_addr_mem_wb_i,
	input  logic 				  Iswaitedflag_alu_mem_wb_i,
	input  logic [2:0]			  funct3_alu_mem_wb_i,
	input  logic [`REG_WIDTH-1:0] curpc_mem_wb_i,
	input  logic [`REG_ADDR_WIDTH-1:0] rs1addr_mem_wb_i,
	input  logic [`REG_ADDR_WIDTH-1:0] rs2addr_mem_wb_i,
	input  logic 				  valid_mem_wb_i,

	input  logic  						csr_we_mem_wb_i,
	input  logic  						is_mret_mem_wb_i,
	input  logic [11:0]				csr_addr_mem_wb_i,
	input  logic [`REG_WIDTH-1:0]  	zimm_mem_wb_i,
	input  logic  						ecall_mem_wb_i,
	input  logic [`REG_WIDTH-1:0]  	csr_wdata_mem_wb_i,
	input  logic [`REG_WIDTH-1:0]  	csr_result_mem_wb_i,
	input  logic [6:0]  				opcode_mem_wb_i,


	output logic [`REG_WIDTH-1:0]   	aluout_mem_wb_o,
	output logic [`REG_WIDTH-1:0]   	aluoutff_mem_wb_o,
	output logic [`REG_WIDTH-1:0]   	dreaddata_mem_wb_o,
	output logic    			regwrite_mem_wb_o,
	output logic    			memtoreg_mem_wb_o,
	output logic [`REG_ADDR_WIDTH-1:0] rdAddr_mem_wb_o,
	output logic 				  Iswaitedflag_data_mem_wb_o,
	output logic 				  Iswaitedflag_addr_mem_wb_o,
	output logic 				  Iswaitedflag_alu_mem_wb_o,
	output logic [2:0]			  funct3_alu_mem_wb_o,
	output logic [`REG_WIDTH-1:0] curpc_mem_wb_o,
	output logic [`REG_ADDR_WIDTH-1:0] rs1addr_mem_wb_o,
	output logic [`REG_ADDR_WIDTH-1:0] rs2addr_mem_wb_o,
	output logic				  valid_mem_wb_o,

	output logic  						csr_we_mem_wb_o,
	output logic  						is_mret_mem_wb_o,
	output logic [11:0]				csr_addr_mem_wb_o,
	output logic [`REG_WIDTH-1:0]  	zimm_mem_wb_o,
	output logic  						ecall_mem_wb_o,
	output logic [`REG_WIDTH-1:0]  	csr_wdata_mem_wb_o,
	output logic [`REG_WIDTH-1:0]  	csr_result_mem_wb_o,
	output logic [6:0]  				opcode_mem_wb_o
    );
	
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				csr_we_mem_wb_o <= 0;
			else if(hand_in & is_mret_mem_wb_o)
				csr_we_mem_wb_o <= 0;
			else if(hand_in & ecall_mem_wb_o)
				csr_we_mem_wb_o <= 0;
			else if(hand_in)
				csr_we_mem_wb_o <= csr_we_mem_wb_i;
			else
				csr_we_mem_wb_o <= csr_we_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				is_mret_mem_wb_o <= 0;
			else if(hand_in)
				is_mret_mem_wb_o <= is_mret_mem_wb_i;
			else
				is_mret_mem_wb_o <= is_mret_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				csr_addr_mem_wb_o <= 0;
			else if(hand_in)
				csr_addr_mem_wb_o <= csr_addr_mem_wb_i;
			else
				csr_addr_mem_wb_o <= csr_addr_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				zimm_mem_wb_o <= 0;
			else if(hand_in)
				zimm_mem_wb_o <= zimm_mem_wb_i;
			else
				zimm_mem_wb_o <= zimm_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				ecall_mem_wb_o <= 0;
			else if(hand_in)
				ecall_mem_wb_o <= ecall_mem_wb_i;
			else
				ecall_mem_wb_o <= ecall_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				csr_wdata_mem_wb_o <= 0;
			else if(hand_in)
				csr_wdata_mem_wb_o <= csr_wdata_mem_wb_i;
			else
				csr_wdata_mem_wb_o <= csr_wdata_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				csr_result_mem_wb_o <= 0;
			else if(hand_in)
				csr_result_mem_wb_o <= csr_result_mem_wb_i;
			else
				csr_result_mem_wb_o <= csr_result_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				opcode_mem_wb_o <= 0;
			else if(hand_in)
				opcode_mem_wb_o <= opcode_mem_wb_i;
			else
				opcode_mem_wb_o <= opcode_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				aluout_mem_wb_o <= 0;
			else if(hand_in)
				aluout_mem_wb_o <= aluout_mem_wb_i;
			else
				aluout_mem_wb_o <= aluout_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				valid_mem_wb_o <= 0;
			// if(hand_in & ecall_mem_wb_o)
			// 	valid_mem_wb_o <= 0;
			else if(hand_in)
				valid_mem_wb_o <= valid_mem_wb_i;
			else
				valid_mem_wb_o <= valid_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				rs1addr_mem_wb_o <= 0;
			else if(hand_in)
				rs1addr_mem_wb_o <= rs1addr_mem_wb_i;
			else
				rs1addr_mem_wb_o <= rs1addr_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				rs2addr_mem_wb_o <= 0;
			else if(hand_in)
				rs2addr_mem_wb_o <= rs2addr_mem_wb_i;
			else
				rs2addr_mem_wb_o <= rs2addr_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				aluoutff_mem_wb_o <= 0;
			else if(hand_in)
				aluoutff_mem_wb_o <= aluoutff_mem_wb_i;
			else
				aluoutff_mem_wb_o <= aluoutff_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				dreaddata_mem_wb_o <= 0;
			else if(hand_in)
				dreaddata_mem_wb_o <= dreaddata_mem_wb_i;
			else
				dreaddata_mem_wb_o <= dreaddata_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				regwrite_mem_wb_o <= 0;
			else if(hand_in & is_mret_mem_wb_o)
				regwrite_mem_wb_o <= 0;
			else if(hand_in & ecall_mem_wb_o)
				regwrite_mem_wb_o <= 0;
			else if(hand_in)
				regwrite_mem_wb_o <= regwrite_mem_wb_i;
			else
				regwrite_mem_wb_o <= regwrite_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				memtoreg_mem_wb_o <= 0;
			else if(hand_in)
				memtoreg_mem_wb_o <= memtoreg_mem_wb_i;
			else
				memtoreg_mem_wb_o <= memtoreg_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				rdAddr_mem_wb_o <= 0;
			else if(hand_in)
				rdAddr_mem_wb_o <= rdAddr_mem_wb_i;
			else
				rdAddr_mem_wb_o <= rdAddr_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				Iswaitedflag_data_mem_wb_o <= 0;
			else if(hand_in)
				Iswaitedflag_data_mem_wb_o <= Iswaitedflag_data_mem_wb_i;
			else
				Iswaitedflag_data_mem_wb_o <= Iswaitedflag_data_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				Iswaitedflag_addr_mem_wb_o <= 0;
			else if(hand_in)
				Iswaitedflag_addr_mem_wb_o <= Iswaitedflag_addr_mem_wb_i;
			else
				Iswaitedflag_addr_mem_wb_o <= Iswaitedflag_addr_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				Iswaitedflag_alu_mem_wb_o <= 0;
			else if(hand_in)
				Iswaitedflag_alu_mem_wb_o <= Iswaitedflag_alu_mem_wb_i;
			else
				Iswaitedflag_alu_mem_wb_o <= Iswaitedflag_alu_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				funct3_alu_mem_wb_o <= 0;
			else if(hand_in)
				funct3_alu_mem_wb_o <= funct3_alu_mem_wb_i;
			else
				funct3_alu_mem_wb_o <= funct3_alu_mem_wb_o;
		end
	always@(posedge clk or posedge rst)
		begin
			if(rst)
				curpc_mem_wb_o <= 0;
			else if(hand_in)
				curpc_mem_wb_o <= curpc_mem_wb_i;
			else
				curpc_mem_wb_o <= curpc_mem_wb_o;
		end
	
	endmodule

module PC_register 
	import common::*; #(parameter WIDTH = 64) (
    input  logic    		 clk, reset,
    input  logic    		 branchFlag,
	input  logic			 hand_in,
    input  logic    		 flag_j,
    input  logic    		 flag_b,
    input  logic [WIDTH-1:0] alu_pc,
    input  logic [WIDTH-1:0] pcnext,
    input  logic [WIDTH-1:0] pc_j,
    input  logic [WIDTH-1:0] pc_b,
    output logic [WIDTH-1:0] q,
    input  logic    		 stallFlag,

	input  logic    		 is_mret,
    input  logic [WIDTH-1:0] mepc,

    // output logic [WIDTH-1:0] pc_reg,

    input  logic			 ecall,
    input  logic [WIDTH-1:0] csr_pc,
	output logic [WIDTH-1:0] pc_ooo
    );

	logic [WIDTH-1:0] q_reg;

    always_ff @(posedge clk, posedge reset)
    if (reset)   
		q_reg <= PCINIT;
	else if(stallFlag & hand_in)
		q_reg <= q_reg;
	else if(flag_j & hand_in)
		q_reg <= pc_j;
	else if(flag_b & hand_in)
		q_reg <= pc_b;  
		
	else if(is_mret & hand_in)
		q_reg <= mepc;
	else if(ecall & hand_in)
		q_reg <= csr_pc;

    else if(hand_in)
	    q_reg <= ((branchFlag == 1)? alu_pc : q + 'h4);
	else
		q_reg <= q_reg;

	logic [WIDTH-1:0] tem_pc;
    always_comb
    if (reset)   
		tem_pc = PCINIT;
	else if(stallFlag)
		tem_pc = q_reg;
	else if(flag_j)
		tem_pc = pc_j;
	else if(flag_b)
		tem_pc = pc_b;  
	else if(is_mret)
		tem_pc = mepc;
	else if(ecall)
		tem_pc = csr_pc;
    else if(hand_in)
	    tem_pc = ((branchFlag == 1)? alu_pc : q + 'h4);
	else
		tem_pc = 0;
	assign pc_ooo = (hand_in == 0) ? q_reg : tem_pc;

	assign q = q_reg;
	endmodule

module registerFile
    import defines::*; (
    input  logic clk,
    input  logic rst,
    input  logic we,
    input  logic flag_IL_S,
    input  logic flag_Alu,
    input  logic memtoreg,
    input  logic [`REG_ADDR_WIDTH-1:0]  writeAddr,
    input  logic [`REG_ADDR_WIDTH-1:0]  readAddr1,
    input  logic [`REG_ADDR_WIDTH-1:0]  readAddr2,

    input  logic [`REG_WIDTH-1:0]       writeData3,
    input  logic [`REG_WIDTH-1:0]       aluout,
    input  logic [`REG_WIDTH-1:0]       aluout_ff,
    output logic [`REG_WIDTH-1:0]       readData1,
    output logic [`REG_WIDTH-1:0]       readData2,
    output reg   [`REG_WIDTH-1:0]       regs_nxt [`REG_COUNT-1:0],

    input  logic [6:0] opcode,
    input  logic [2:0] funct3,
    input  logic [`REG_WIDTH-1:0]       csr_result
    );

    reg   [`REG_WIDTH-1:0] regs [`REG_COUNT-1:0]; // 32�?64位寄存器
	logic flag;
    // �? always_comb 块用于计算下�?个状态的�?
    always_comb begin
		flag = 0;
        for (int i = 0; i < `REG_COUNT; i++) begin
            regs_nxt[i] = regs[i];
        end
        if (we && writeAddr != `REG_ADDR_WIDTH'b0) 
		begin
            if(~(flag_IL_S | flag_Alu)&& opcode != `INSTR_CSR) 
			begin
                regs_nxt[writeAddr] = (memtoreg == 1) ? writeData3 : aluout;
            end 
			else if(flag_IL_S & memtoreg) 
			begin
                case(funct3[2:0])
                    3'b000:  regs_nxt[writeAddr]  = {{56{writeData3[7]}}, writeData3[7:0]};  // lb
                    3'b100:  regs_nxt[writeAddr]  = {56'b0                , writeData3[7:0]};  // lbu
                    3'b001:  regs_nxt[writeAddr]  = {{48{writeData3[15]}}, writeData3[15:0]};// lh
                    3'b101:  regs_nxt[writeAddr]  = {48'b0                , writeData3[15:0]}; // lhu
                    3'b010:  regs_nxt[writeAddr]  = {{32{writeData3[31]}}, writeData3[31:0]};// lw
                    3'b110:  regs_nxt[writeAddr]  = {32'b0                , writeData3[31:0]}; // lwu
                    3'b011:  regs_nxt[writeAddr]  = writeData3[63:0];                         // ld
                    default: regs_nxt[writeAddr]  = writeData3[63:0];
                endcase
            end 
			else if(flag_Alu) 
			begin
                regs_nxt[writeAddr] = aluout_ff;
            end 
			else if(opcode == `INSTR_CSR)
			begin
				flag = 1;
                regs_nxt[writeAddr] = csr_result;
			end
        end
    end

    // 将寄存器的状态更新放在时钟或复位的边沿上
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            for (int i = 0; i < `REG_COUNT; i++) begin
                regs[i] <= `REG_WIDTH'b0;
            end
        end else begin
            for (int i = 0; i < `REG_COUNT; i++) begin
                regs[i] <= regs_nxt[i];
            end
        end
    end

    // 数据读取逻辑
    assign readData1 = (readAddr1 == 0) ? 0 : regs[readAddr1];
    assign readData2 = (readAddr2 == 0) ? 0 : regs[readAddr2];
	endmodule

module core import common::*; import defines::*;(
	input  logic       		 	clk, reset,
	output ibus_req_t  		 	ireq,
	input  ibus_resp_t 		 	iresp,
	output dbus_req_t  		 	dreq,
	input  dbus_resp_t 		 	dresp,
	input  logic       		 	trint, swint, exint
	);	
	logic 					 	hand_in;
	CSR_reg 					csr_reg;
    logic [`REG_WIDTH-1:0]   	csr_pc;

    logic [`REG_WIDTH-1:0]   	readData; 						// 从数据内存中读到的数据，读取出的数据
    logic [`REG_WIDTH-1:0]	 	writeData;   					// �????要写入数据内存的内容
    logic [`REG_WIDTH-1:0]	 	pc;
	logic [`REG_WIDTH-1:0] 		q;
    assign pc = q;
    logic [`REG_WIDTH-1:0]      pcnext;
    logic [`REG_WIDTH-1:0]      pc_b;

	logic [`REG_WIDTH-1:0] 		regs_nxt [`REG_COUNT-1:0]; 		//32�????64位寄存器
	
	// if_id
	logic [`INSTR_WIDTH-1:0] 	instr_if_id_i;
	assign instr_if_id_i 	= 	iresp.data;
	logic [`REG_WIDTH-1:0]   	pc_if_id_o;
	logic [`INSTR_WIDTH-1:0] 	instr_if_id_o;
	logic 						valid_if_id_o;

    logic [`REG_WIDTH-1:0] 		tem_rD1, tem_rD2;
    logic [`REG_WIDTH-1:0] 		tem_csrD;
	logic [`REG_ADDR_WIDTH-1:0] tem_rA1, tem_rA2;

	// id_ex
	logic  						csr_we_id_ex_i;
	logic  						is_mret_id_ex_i;
	logic [11:0]				csr_addr_id_ex_i;
	logic [`REG_WIDTH-1:0]  	zimm_id_ex_i;
	logic  						ecall_id_ex_i;

	logic  						csr_we_id_ex_o;
	logic  						is_mret_id_ex_o;
	logic [11:0]				csr_addr_id_ex_o;
	logic [`REG_WIDTH-1:0]  	zimm_id_ex_o;
	logic  						ecall_id_ex_o;

	logic [`REG_WIDTH-1:0] 		pc_id_ex_i;
	logic [`REG_WIDTH-1:0] 		rs1data_id_ex_i;
	logic [`REG_WIDTH-1:0] 		rs2data_id_ex_i;
	logic [`REG_ADDR_WIDTH-1:0] rdAddr_id_ex_i;
	logic  						memread_id_ex_i;
	logic 						memtoreg_id_ex_i;
	logic  						memtowrite_id_ex_i;
	logic 						isNotImm_id_ex_i;
	logic 						regwrite_id_ex_i;
	logic [`REG_WIDTH-1:0]		imm_id_ex_i;
	logic [`ALUOP_WIDTH-1:0]	aluop_id_ex_i;
	logic [2:0]					funct3_id_ex_i;
	logic [6:0]					funct7_id_ex_i;
	logic [6:0]					opcode_id_ex_i;
	logic 						Iswaitedflag_data_id_ex_i;
	logic 						Iswaitedflag_addr_id_ex_i;
	logic 						Iswaitedflag_alu_id_ex_i;
	logic 						valid_id_ex_i;
	assign						valid_id_ex_i = valid_if_id_o;
	logic [`REG_ADDR_WIDTH-1:0] rs1addr_id_ex_i, rs1addr_id_ex_o;
	logic [`REG_ADDR_WIDTH-1:0] rs2addr_id_ex_i, rs2addr_id_ex_o;

	logic [`REG_WIDTH-1:0] 		pc_id_ex_o;
	logic [`REG_WIDTH-1:0] 		rs1data_id_ex_o;
	logic [`REG_WIDTH-1:0] 		rs2data_id_ex_o;
	logic [`REG_ADDR_WIDTH-1:0] rdAddr_id_ex_o;
	logic  						memread_id_ex_o;
	logic 						memtoreg_id_ex_o;
	logic  						memtowrite_id_ex_o;
	logic 						isNotImm_id_ex_o;
	logic 						regwrite_id_ex_o;
	logic [`REG_WIDTH-1:0]		imm_id_ex_o;
	logic [`ALUOP_WIDTH-1:0]	aluop_id_ex_o;
	logic [2:0]					funct3_id_ex_o;
	logic [6:0]					funct7_id_ex_o;
	logic [6:0]					opcode_id_ex_o;
	logic 						Iswaitedflag_data_id_ex_o;
	logic 						Iswaitedflag_addr_id_ex_o;
	logic 						Iswaitedflag_alu_id_ex_o;
	logic 						valid_id_ex_o;

	logic [2:0]					forwardae_id, forwardbe_id;
	logic [1:0] 				forwardae;
	logic [1:0] 				forwardbe;
	logic 						stallflag;
	logic [1:0] 				forward_csr;

	// ex_mem
	logic  						csr_we_ex_mem_i;
	logic  						is_mret_ex_mem_i;
	logic [11:0]  	csr_addr_ex_mem_i;
	logic [`REG_WIDTH-1:0]  	zimm_ex_mem_i;
	logic  						ecall_ex_mem_i;
	logic [`REG_WIDTH-1:0]  	csr_wdata_ex_mem_i;
	logic [`REG_WIDTH-1:0]  	csr_result_ex_mem_i;
	logic [6:0]  				opcode_ex_mem_i;
	assign  						csr_we_ex_mem_i = csr_we_id_ex_o;
	assign  						is_mret_ex_mem_i = is_mret_id_ex_o;
	assign						csr_addr_ex_mem_i = csr_addr_id_ex_o;
	assign						zimm_ex_mem_i = zimm_id_ex_o;
	assign  						ecall_ex_mem_i = ecall_id_ex_o;
	assign						opcode_ex_mem_i = opcode_id_ex_o;

	logic  						csr_we_ex_mem_o;
	logic  						is_mret_ex_mem_o;
	logic [11:0]  	csr_addr_ex_mem_o;
	logic [`REG_WIDTH-1:0]  	zimm_ex_mem_o;
	logic  						ecall_ex_mem_o;
	logic [`REG_WIDTH-1:0]  	csr_wdata_ex_mem_o;
	logic [`REG_WIDTH-1:0]  	csr_result_ex_mem_o;
	logic [6:0]  				opcode_ex_mem_o;

	logic 				   Iswaitedflag_data_ex_mem_i;
	logic 				   Iswaitedflag_addr_ex_mem_i;
	logic 				   Iswaitedflag_alu_ex_mem_i;

	logic 				   tembranchFlag_ex_mem_i;
	logic [`REG_WIDTH-1:0] temaluout_ex_mem_i;
	logic [`REG_WIDTH-1:0] temaluoutff_ex_mem_i;
	logic [`REG_WIDTH-1:0] temalupc_ex_mem_i;
	logic 				   temaludone_ex_mem_i;

	logic 				   tembranchFlag_ex_mem_i2;
	logic [`REG_WIDTH-1:0] temaluout_ex_mem_i2;
	logic [`REG_WIDTH-1:0] temaluoutff_ex_mem_i2;
	logic [`REG_WIDTH-1:0] temalupc_ex_mem_i2;
	logic 				   temaludone_ex_mem_i2;
	logic [`REG_WIDTH-1:0] writedata_for;


 	logic 				   branchFlag_ex_mem_i;
	logic [`REG_WIDTH-1:0] aluout_ex_mem_i;
	logic [`REG_WIDTH-1:0] aluoutff_ex_mem_i;
	logic [`REG_WIDTH-1:0] alupc_ex_mem_i;
	logic 				   aludone_ex_mem_i;
	assign					branchFlag_ex_mem_i = (Iswaitedflag_alu_id_ex_o == 1)?tembranchFlag_ex_mem_i2:tembranchFlag_ex_mem_i;
	assign					aluout_ex_mem_i = (Iswaitedflag_alu_id_ex_o == 1)?temaluout_ex_mem_i2:temaluout_ex_mem_i;
	assign					aluoutff_ex_mem_i = (Iswaitedflag_alu_id_ex_o == 1)?temaluoutff_ex_mem_i2:temaluoutff_ex_mem_i;
	// assign					alupc_ex_mem_i = (Iswaitedflag_alu_ex_mem_i == 1)?temalupc_ex_mem_i2:temalupc_ex_mem_i;
	assign					alupc_ex_mem_i = 0;
	// assign					aludone_ex_mem_i = (Iswaitedflag_alu_ex_mem_i == 1)?temaludone_ex_mem_i2:temaludone_ex_mem_i;
	assign					aludone_ex_mem_i = 1;

	logic 				   regwrite_ex_mem_i;
	logic 				   memtoreg_ex_mem_i;
	logic 				   memtowrite_ex_mem_i;
	logic [`REG_ADDR_WIDTH-1:0]	rdAddr_ex_mem_i;
	logic [2:0]			   funct3_ex_mem_i;
	logic [`REG_WIDTH-1:0] writedata_ex_mem_i;
	logic [`REG_WIDTH-1:0] curpc_ex_mem_i;
	logic [`REG_ADDR_WIDTH-1:0] rs1addr_ex_mem_i, rs2addr_ex_mem_i;
	logic				   valid_ex_mem_i;
	assign 				   regwrite_ex_mem_i 			= regwrite_id_ex_o;
	assign 				   memtoreg_ex_mem_i 			= memtoreg_id_ex_o;
	assign 				   memtowrite_ex_mem_i 			= memtowrite_id_ex_o;
	assign 				   rdAddr_ex_mem_i				= rdAddr_id_ex_o;
	assign 				   Iswaitedflag_data_ex_mem_i 	= Iswaitedflag_data_id_ex_o;
	assign 				   Iswaitedflag_addr_ex_mem_i 	= Iswaitedflag_addr_id_ex_o;
	assign 				   Iswaitedflag_alu_ex_mem_i 	= Iswaitedflag_alu_id_ex_o;
	assign				   funct3_ex_mem_i 				= funct3_id_ex_o;
	assign 				   writedata_ex_mem_i			= writedata_for;
	// assign 				   writedata_ex_mem_i			= rs2data_id_ex_o;
	assign 				   curpc_ex_mem_i				= pc_id_ex_o;
	assign 				   rs1addr_ex_mem_i				= rs1addr_id_ex_o;
	assign 				   rs2addr_ex_mem_i				= rs2addr_id_ex_o;
	assign				   valid_ex_mem_i				= valid_id_ex_o;

	logic 				   branchFlag_ex_mem_o;
	logic [`REG_WIDTH-1:0] aluout_ex_mem_o;
	logic [`REG_WIDTH-1:0] aluoutff_ex_mem_o;
	logic [`REG_WIDTH-1:0] alupc_ex_mem_o;
	logic 				   aludone_ex_mem_o;

	logic 				   regwrite_ex_mem_o;
	logic 				   memtoreg_ex_mem_o;
	logic 				   memtowrite_ex_mem_o;
	logic [`REG_ADDR_WIDTH-1:0]	rdAddr_ex_mem_o;
	logic 				   Iswaitedflag_data_ex_mem_o;
	logic 				   Iswaitedflag_addr_ex_mem_o;
	logic 				   Iswaitedflag_alu_ex_mem_o;
	logic [2:0]			   funct3_ex_mem_o;
	logic [`REG_WIDTH-1:0] writedata_ex_mem_o;
	logic [`REG_WIDTH-1:0] curpc_ex_mem_o;
	logic [`REG_ADDR_WIDTH-1:0] rs1addr_ex_mem_o, rs2addr_ex_mem_o;
	logic 				   valid_ex_mem_o;
	logic 				   Iswaitedflag_data_ex_mem_o_flag;
	logic 				   Iswaitedflag_addr_ex_mem_o_flag;


	logic 						dbusdataok;
	logic [`REG_WIDTH-1:0]   	dbusdata;
	
	// mem_wb
	logic  						csr_we_mem_wb_i;
	logic  						is_mret_mem_wb_i;
	logic [11:0]				csr_addr_mem_wb_i;
	logic [`REG_WIDTH-1:0]  	zimm_mem_wb_i;
	logic  						ecall_mem_wb_i;
	logic [`REG_WIDTH-1:0]  	csr_wdata_mem_wb_i;
	logic [`REG_WIDTH-1:0]  	csr_result_mem_wb_i;
	logic [6:0]  				opcode_mem_wb_i;
	assign  						csr_we_mem_wb_i = csr_we_ex_mem_o;
	assign  						is_mret_mem_wb_i = is_mret_ex_mem_o;
	assign 	csr_addr_mem_wb_i = csr_addr_ex_mem_o;
	assign	zimm_mem_wb_i = zimm_ex_mem_o;
	assign  						ecall_mem_wb_i = ecall_ex_mem_o;
	assign	csr_wdata_mem_wb_i = csr_wdata_ex_mem_o;
	assign	csr_result_mem_wb_i = csr_result_ex_mem_o;
	assign	opcode_mem_wb_i = opcode_ex_mem_o;

	logic  						csr_we_mem_wb_o;
	logic  						is_mret_mem_wb_o;
	logic [11:0]  				csr_addr_mem_wb_o;
	logic [`REG_WIDTH-1:0]  	zimm_mem_wb_o;
	logic  						ecall_mem_wb_o;
	logic [`REG_WIDTH-1:0]  	csr_wdata_mem_wb_o;
	logic [`REG_WIDTH-1:0]  	csr_result_mem_wb_o;
	logic [6:0]  				opcode_mem_wb_o;

	logic [`REG_WIDTH-1:0]   	aluout_mem_wb_i;
	logic [`REG_WIDTH-1:0]   	aluoutff_mem_wb_i;
	logic [`REG_WIDTH-1:0]   	dreaddata_mem_wb_i; // readData == dresp.data
	logic 					   	regwrite_mem_wb_i;
	logic 					   	memtoreg_mem_wb_i;
	logic [`REG_ADDR_WIDTH-1:0] rdAddr_mem_wb_i;
	logic 				  		Iswaitedflag_data_mem_wb_i;
	logic 				  		Iswaitedflag_addr_mem_wb_i;
	logic 				  		Iswaitedflag_alu_mem_wb_i;
	logic [2:0]				  	funct3_alu_mem_wb_i;
	logic [`REG_WIDTH-1:0]   	curpc_mem_wb_i;
	logic [`REG_ADDR_WIDTH-1:0] rs1addr_mem_wb_i, rs2addr_mem_wb_i;
	logic 				  		valid_mem_wb_i;
	assign						aluout_mem_wb_i 	= aluout_ex_mem_o;
	assign						aluoutff_mem_wb_i 	= aluoutff_ex_mem_o;
	assign						dreaddata_mem_wb_i 	= dbusdata;
	assign 					   	regwrite_mem_wb_i	= regwrite_ex_mem_o;
	assign 					   	memtoreg_mem_wb_i	= memtoreg_ex_mem_o;
	assign 					   	rdAddr_mem_wb_i		= rdAddr_ex_mem_o;
	assign 				  		Iswaitedflag_data_mem_wb_i = Iswaitedflag_data_ex_mem_o;
	assign 				  		Iswaitedflag_addr_mem_wb_i = Iswaitedflag_addr_ex_mem_o;
	assign 				  		Iswaitedflag_alu_mem_wb_i  = Iswaitedflag_alu_ex_mem_o;
	assign 				  		funct3_alu_mem_wb_i = funct3_ex_mem_o;
	assign 				  		curpc_mem_wb_i 		= curpc_ex_mem_o;
	assign 				  		valid_mem_wb_i 		= valid_ex_mem_o;
	assign 				  		rs1addr_mem_wb_i 	= rs1addr_ex_mem_o;
	assign 				  		rs2addr_mem_wb_i 	= rs2addr_ex_mem_o;

	logic [`REG_WIDTH-1:0]   	aluout_mem_wb_o;
	logic [`REG_WIDTH-1:0]   	aluoutff_mem_wb_o;
	logic [`REG_WIDTH-1:0]   	dreaddata_mem_wb_o;
	logic    					regwrite_mem_wb_o;
	logic    					memtoreg_mem_wb_o;
	logic [`REG_ADDR_WIDTH-1:0] rdAddr_mem_wb_o;
	logic 				  		Iswaitedflag_data_mem_wb_o;
	logic 				  		Iswaitedflag_addr_mem_wb_o;
	logic 				  		Iswaitedflag_alu_mem_wb_o;
	logic [2:0]				  	funct3_alu_mem_wb_o;
	logic [`REG_WIDTH-1:0]   	curpc_mem_wb_o;
	logic [`REG_ADDR_WIDTH-1:0] rs1addr_mem_wb_o, rs2addr_mem_wb_o;
	logic 				  		valid_mem_wb_o;

	/* 提交节点hand_in的设�???? */
	// logic 					 	hand_in;
	// assign 					 	hand_in = (Iswaitedflag_alu_ex_mem_i==1)?(iresp.data_ok&temaludone_ex_mem_i2):iresp.data_ok;
	// assign 					 	hand_in = (Iswaitedflag_alu_ex_mem_i==1)?(iresp.data_ok&temaludone_ex_mem_i2):((Iswaitedflag_addr_ex_mem_o|Iswaitedflag_data_ex_mem_o==1)?dresp.data_ok:iresp.data_ok);
	// assign 					 	hand_in = ((~(Iswaitedflag_data|Iswaitedflag_addr|Iswaitedflag_alu)) & iresp.data_ok) | dresp.data_ok | (Iswaitedflag_alu & ALUdone);

	logic stallFlag;
	assign stallFlag = (opcode_id_ex_i == `INSTR_B) & (Iswaitedflag_addr_ex_mem_i|Iswaitedflag_data_ex_mem_i) & ((opcode_id_ex_o == `INSTR_S) | (opcode_id_ex_o == `INSTR_IL));

	logic flag_j, flag_b;
	assign flag_j = (opcode_id_ex_o == `INSTR_J) || (opcode_id_ex_o == `INSTR_JR);
	// assign flag_b = (opcode_id_ex_o == `INSTR_B);


	logic [`REG_WIDTH-1:0]   	pc_ooo;
    PC_register pcReg(
        .clk(clk), 
        .reset(reset), 
		.hand_in(hand_in),
        .branchFlag(branchFlag_ex_mem_o),
        .alu_pc(alupc_ex_mem_o),
        .pcnext(pcnext),
		.flag_j(flag_j),.pc_j(temalupc_ex_mem_i),
		.flag_b(flag_b),.pc_b(pc_b), 
        .q(q),.stallFlag(stallFlag),

		.is_mret(is_mret_mem_wb_o),
		.mepc(csr_reg.mepc),

		// .pc_reg(),
		.ecall(ecall_mem_wb_o),
		.csr_pc(csr_pc),

		.pc_ooo(pc_ooo)
    	);

    adder pcAdd(
        q, 
        `REG_WIDTH'h4, 
        pcnext
    	);

	if_id_regs if_id_regs(
		.clk(clk), .rst(reset),.hand_in(hand_in),.valid_if_id_o(valid_if_id_o),
		.pc_if_id_i(pc),
		.pc_if_id_o(pc_if_id_o),
		.instr_if_id_i(instr_if_id_i),
		.instr_if_id_o(instr_if_id_o),.flag_clear(flag_j|flag_b|is_mret_mem_wb_o|ecall_mem_wb_o),.stallFlag(stallFlag)
		);
	// decoder part's temp
	always_comb
	begin
		// 上条ex
		if(regwrite_ex_mem_i == 1 && rs1addr_id_ex_i == rdAddr_ex_mem_i && memtoreg_ex_mem_i == 0 && rs1addr_id_ex_i != 0)
			forwardae_id = 3'b011;
		//上上条mem
		else if(regwrite_ex_mem_o == 1 && rs1addr_id_ex_i == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 0 && rs1addr_id_ex_i != 0)
			forwardae_id = 3'b010;
		// 上上条mem l�????
		else if(regwrite_ex_mem_o == 1 && rs1addr_id_ex_i == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 && rs1addr_id_ex_i != 0)
			forwardae_id = 3'b100;
		// 上上上条wb
		else if(regwrite_mem_wb_o == 1 && rs1addr_id_ex_i == rdAddr_mem_wb_o && rs1addr_id_ex_i != 0)
			forwardae_id = 3'b001;
		else
			forwardae_id = 3'b000;

		if(regwrite_ex_mem_i == 1 && rs2addr_id_ex_i == rdAddr_ex_mem_i && memtoreg_ex_mem_i == 0 && rs2addr_id_ex_i != 0)
			forwardbe_id = 3'b011;
		else if(regwrite_ex_mem_o == 1 && rs2addr_id_ex_i == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 0 && rs2addr_id_ex_i!=0)
			forwardbe_id = 3'b010;
		else if(regwrite_ex_mem_o == 1 && rs2addr_id_ex_i == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 && rs2addr_id_ex_i != 0)
			forwardbe_id = 3'b100;
		else if(regwrite_mem_wb_o == 1 && rs2addr_id_ex_i == rdAddr_mem_wb_o && rs2addr_id_ex_i!=0)
			forwardbe_id = 3'b001;
		else
			forwardbe_id = 3'b000;
		// L型指令，跟下面的写在�????�????
		// if(regwrite_ex_mem_o == 1 && rs1addr_id_ex_i == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 && rs1addr_id_ex_i != 0 && (Iswaitedflag_addr_ex_mem_o|Iswaitedflag_data_ex_mem_o))
		// 	stallflag = 1;
	end

    id id(
        .rst(reset),
        .input_pc(pc_if_id_o),
        .instr(instr_if_id_o),
        .pc(pc_id_ex_i),
		.pc_b(pc_b),
        .rs1data(tem_rD1),
        .rs2data(tem_rD2),
        .rs1addr(rs1addr_id_ex_i),
        .rs2addr(rs2addr_id_ex_i),
        .rs1_data_o(rs1data_id_ex_i),
        .rs2_data_o(rs2data_id_ex_i),
        .rdAddr(rdAddr_id_ex_i),
        .memread(memread_id_ex_i),.memtoreg(memtoreg_id_ex_i),.memwrite(memtowrite_id_ex_i),
		.isNotImm(isNotImm_id_ex_i),.regwrite(regwrite_id_ex_i),.aluop(aluop_id_ex_i),
		.funct3(funct3_id_ex_i),.funct7(funct7_id_ex_i),.imm(imm_id_ex_i),.opcode(opcode_id_ex_i),
		.Iswaitedflag_data(Iswaitedflag_data_id_ex_i),
		.Iswaitedflag_addr(Iswaitedflag_addr_id_ex_i),
		.Iswaitedflag_alu(Iswaitedflag_alu_id_ex_i),
		.aluout_ex_mem_o((Iswaitedflag_alu_ex_mem_o==1)?aluoutff_ex_mem_o:aluout_ex_mem_o),.wb_data(regs_nxt[rdAddr_mem_wb_o]),
		.forwardae(forwardae_id),.forwardbe(forwardbe_id),.aluout_ex_mem_i((Iswaitedflag_alu_id_ex_o==1)?aluoutff_ex_mem_i:aluout_ex_mem_i),.flag_b(flag_b),
		.dreaddata_mem_wb_i(dreaddata_mem_wb_i >> ((aluout_mem_wb_i[3:0]%8)*8)), .func3_dbusdone(funct3_ex_mem_o),
		
		.csr_we(csr_we_id_ex_i),
		.is_mret(is_mret_id_ex_i),
		.csr_addr(csr_addr_id_ex_i),
		.zimm(zimm_id_ex_i),
		.ecall(ecall_id_ex_i)
    	);


	id_ex_regs id_ex_regs(
		clk,reset,hand_in,
		pc_id_ex_i,rs1data_id_ex_i,rs2data_id_ex_i,rdAddr_id_ex_i,
		memread_id_ex_i,memtoreg_id_ex_i,memtowrite_id_ex_i,isNotImm_id_ex_i,
		regwrite_id_ex_i,imm_id_ex_i,aluop_id_ex_i,funct3_id_ex_i,funct7_id_ex_i,
		opcode_id_ex_i,Iswaitedflag_data_id_ex_i,Iswaitedflag_addr_id_ex_i,Iswaitedflag_alu_id_ex_i,
		rs1addr_id_ex_i,rs2addr_id_ex_i,valid_id_ex_i,

		csr_we_id_ex_i,is_mret_id_ex_i,csr_addr_id_ex_i,zimm_id_ex_i,ecall_id_ex_i,

		pc_id_ex_o,rs1data_id_ex_o,rs2data_id_ex_o,rdAddr_id_ex_o,
		memread_id_ex_o,memtoreg_id_ex_o,memtowrite_id_ex_o,isNotImm_id_ex_o,
		regwrite_id_ex_o,imm_id_ex_o,aluop_id_ex_o,funct3_id_ex_o,funct7_id_ex_o,
		opcode_id_ex_o,Iswaitedflag_data_id_ex_o,Iswaitedflag_addr_id_ex_o,Iswaitedflag_alu_id_ex_o,
		rs1addr_id_ex_o,rs2addr_id_ex_o,valid_id_ex_o,flag_j|is_mret_mem_wb_o|ecall_ex_mem_o,flag_b,stallFlag,

		csr_we_id_ex_o,is_mret_id_ex_o,csr_addr_id_ex_o,zimm_id_ex_o,ecall_id_ex_o
		);




	always_comb
	begin
		// 上条
		if(regwrite_ex_mem_o == 1 && rs1addr_id_ex_o == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 
			&& rs1addr_id_ex_o != 0 && (Iswaitedflag_addr_ex_mem_o|Iswaitedflag_data_ex_mem_o))
			forwardae = 2'b11;
		else if(regwrite_ex_mem_o == 1 && rs1addr_id_ex_o == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 0 && rs1addr_id_ex_o != 0)
			forwardae = 2'b10;
		// 上上�????
		else if(regwrite_mem_wb_o == 1 && rs1addr_id_ex_o == rdAddr_mem_wb_o && rs1addr_id_ex_o != 0)
			forwardae = 2'b01;
		else
			forwardae = 2'b00;

		if(regwrite_ex_mem_o == 1 && rs2addr_id_ex_o == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 && rs2addr_id_ex_o != 0 && (Iswaitedflag_addr_ex_mem_o|Iswaitedflag_data_ex_mem_o))
			forwardbe = 2'b11;
		else if(regwrite_ex_mem_o == 1 && rs2addr_id_ex_o == rdAddr_ex_mem_o && rs2addr_id_ex_o!=0)
			forwardbe = 2'b10;
		else if(regwrite_mem_wb_o == 1 && rs2addr_id_ex_o == rdAddr_mem_wb_o && rs2addr_id_ex_o!=0)
			forwardbe = 2'b01;
		else
			forwardbe = 2'b00;

		if(csr_we_ex_mem_o == 1 && csr_addr_id_ex_o == csr_addr_ex_mem_o && csr_addr_id_ex_o != 0)
			forward_csr = 2'b11;
		else if(csr_we_mem_wb_o == 1 && csr_addr_id_ex_o == csr_addr_mem_wb_o && csr_addr_id_ex_o != 0)
			forward_csr = 2'b01;
		else
			forward_csr = 2'b00;

		if(regwrite_ex_mem_o == 1 && rs1addr_id_ex_o == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 && rs1addr_id_ex_o != 0 && (Iswaitedflag_addr_ex_mem_o|Iswaitedflag_data_ex_mem_o))
			stallflag = 1;
		else if(regwrite_ex_mem_o == 1 && rs2addr_id_ex_o == rdAddr_ex_mem_o && memtoreg_ex_mem_o == 1 && rs2addr_id_ex_o != 0 && (Iswaitedflag_addr_ex_mem_o|Iswaitedflag_data_ex_mem_o))
			stallflag = 1;
		else
			stallflag = 0;
	end
    ALU alu(
		.clk(clk), .rst(reset),.forwardae(forwardae),.forwardbe(forwardbe),.hand_in(hand_in),.drespdataok((dresp.data_ok&stallflag)),
		.aluout_ex_mem_o((Iswaitedflag_alu_ex_mem_o==1)?aluoutff_mem_wb_i:aluout_mem_wb_i),.wb_data(regs_nxt[rdAddr_mem_wb_o]),.dreaddata_mem_wb_i(dreaddata_mem_wb_i >> ((aluout_mem_wb_i[3:0]%8)*8)),
        .pc_i(pc_id_ex_o),
        .rs1_data_i(rs1data_id_ex_o),.rs2_data_i(rs2data_id_ex_o),
        .funct3(funct3_id_ex_o),
        .funct7(funct7_id_ex_o),
        .imm(imm_id_ex_o),
        .opcode(opcode_id_ex_o),
        .isNotImm(isNotImm_id_ex_o),
        .aluop(aluop_id_ex_o),

        .branchFlag(tembranchFlag_ex_mem_i),
        .result (temaluout_ex_mem_i),
		.result_ff(temaluoutff_ex_mem_i),
        .pc_o(temalupc_ex_mem_i),
		.done(temaludone_ex_mem_i),
		.writedata(writedata_for),
		
		.func3_dbusdone(funct3_ex_mem_o),

		.csr_rdata(tem_csrD), .zimm(zimm_id_ex_o), .csr_wdata(csr_wdata_ex_mem_i), .csr_result(csr_result_ex_mem_i),
		.forward_csr(forward_csr),
		.csr_wdata_ex_mem_o(csr_wdata_ex_mem_o),
    	.csr_wdata_mem_wb_o(csr_wdata_mem_wb_o),

		.opcode_ex_mem_o(opcode_ex_mem_o),
		.opcode_mem_wb_o(opcode_mem_wb_o),
		.csr_result_ex_mem_o(csr_result_ex_mem_o),
    	.csr_result_mem_wb_o(csr_result_mem_wb_o)
    );    

	ALUreg ALUreg(
		clk,reset,hand_in,(dresp.data_ok&stallflag),
		// clk,reset,hand_in,((Iswaitedflag_data_ex_mem_o_flag|Iswaitedflag_addr_ex_mem_o_flag)&stallflag),
		tembranchFlag_ex_mem_i,temaluout_ex_mem_i,temaluoutff_ex_mem_i,
		temalupc_ex_mem_i,temaludone_ex_mem_i,
		tembranchFlag_ex_mem_i2,temaluout_ex_mem_i2,temaluoutff_ex_mem_i2,
		temalupc_ex_mem_i2,temaludone_ex_mem_i2
	);


	ex_mem_regs ex_mem_regs(
		clk,reset,hand_in,dresp.data_ok,
		branchFlag_ex_mem_i,
		aluout_ex_mem_i,aluoutff_ex_mem_i,alupc_ex_mem_i,aludone_ex_mem_i,
		regwrite_ex_mem_i,memtoreg_ex_mem_i,memtowrite_ex_mem_i,rdAddr_ex_mem_i,
		Iswaitedflag_data_ex_mem_i,Iswaitedflag_addr_ex_mem_i,Iswaitedflag_alu_ex_mem_i,
		funct3_ex_mem_i,writedata_ex_mem_i,curpc_ex_mem_i,
		rs1addr_ex_mem_i,rs2addr_ex_mem_i,valid_ex_mem_i,

		csr_we_ex_mem_i,is_mret_ex_mem_i,csr_addr_ex_mem_i,zimm_ex_mem_i,ecall_ex_mem_i,csr_wdata_ex_mem_i,csr_result_ex_mem_i,opcode_ex_mem_i,

		branchFlag_ex_mem_o,
		aluout_ex_mem_o,aluoutff_ex_mem_o,alupc_ex_mem_o,aludone_ex_mem_o,
		regwrite_ex_mem_o,memtoreg_ex_mem_o,memtowrite_ex_mem_o,rdAddr_ex_mem_o,
		Iswaitedflag_data_ex_mem_o,Iswaitedflag_addr_ex_mem_o,Iswaitedflag_alu_ex_mem_o,
		funct3_ex_mem_o,writedata_ex_mem_o,curpc_ex_mem_o,
		rs1addr_ex_mem_o,rs2addr_ex_mem_o,valid_ex_mem_o,Iswaitedflag_data_ex_mem_o_flag,Iswaitedflag_addr_ex_mem_o_flag,flag_j|is_mret_mem_wb_o|ecall_mem_wb_o,

		csr_we_ex_mem_o,is_mret_ex_mem_o,csr_addr_ex_mem_o,zimm_ex_mem_o,ecall_ex_mem_o,csr_wdata_ex_mem_o,csr_result_ex_mem_o,opcode_ex_mem_o,is_mret_mem_wb_o|ecall_mem_wb_o
		);
	CSR_reg csr_reg_clk;

	logic [11:0] i_page_table_index1;
	logic [11:0] i_page_table_index2;
	logic [11:0] i_page_table_index3;
	assign i_page_table_index1 = {3'b0, pc_ooo[38:30]};
	assign i_page_table_index2 = {3'b0, pc_ooo[29:21]};
	assign i_page_table_index3 = {3'b0, pc_ooo[20:12]};
	logic [1:0] dreq_access_count;

	logic [`REG_WIDTH-1:0] dresp_data1_reg;
	logic [`REG_WIDTH-1:0] dresp_data2_reg;
	logic [`REG_WIDTH-1:0] dresp_data;
	logic [`REG_WIDTH-1:0] dresp_data_reg;
	always @(posedge clk, posedge reset)
	begin
		if(reset)
		begin
			dreq_access_count <= 0;
			dresp_data1_reg <= 0;
			dresp_data2_reg <= 0;
			dresp_data_reg <= 0;
		end
		else if(hand_in)
		begin
			dreq_access_count <= 0;
			dresp_data1_reg <= 0;
			dresp_data2_reg <= 0;
			dresp_data_reg <= 0;
		end
		else if(dresp.data_ok && dreq_access_count == 0)
		begin
			dreq_access_count <= 1;
			dresp_data1_reg <= dresp.data;
			dresp_data_reg <= dresp.data;
		end
		else if(dresp.data_ok && dreq_access_count == 1)
		begin
			dreq_access_count <= 2;
			dresp_data2_reg <= dresp.data;
		end
		else if(dresp.data_ok && dreq_access_count == 2)
		begin
			dreq_access_count <= 3;
			dresp_data_reg <= dresp.data;
		end
		else
		begin
			dreq_access_count <= dreq_access_count;
			dresp_data1_reg <= dresp_data1_reg;
			dresp_data2_reg <= dresp_data2_reg;
			dresp_data_reg <= dresp_data_reg;
		end
	end
	assign dresp_data  = (dresp.data_ok == 0)? dresp_data_reg : dresp.data;

	logic trans_needed;
	// assign trans_needed = (ecall_mem_wb_o == 0)?(csr_reg_clk.mode != 3) && (csr_reg_clk.satp.mode == 8):(csr_reg.mode != 3) && (csr_reg.satp.mode == 8);
	assign trans_needed = (csr_reg_clk.mode != 3) && (csr_reg_clk.satp.mode == 8);

	// assign ireq.valid 	= 1'b1; 
	// assign ireq.addr 	= (trans_needed == 0)?pc_ooo:{8'b0, dresp_data[53:10], pc_ooo[11:0]};
	always @(posedge clk, posedge reset)
	begin
		if(reset)
			ireq.addr <= 0;
		else if(trans_needed == 0)
			ireq.addr <= pc;
		else 
			ireq.addr <= {8'b0, dresp_data[53:10], pc_ooo[11:0]};
		ireq.valid <= 1;
	end

	assign 					 	dreq.valid	= ((Iswaitedflag_data_ex_mem_o_flag|Iswaitedflag_addr_ex_mem_o_flag))|(trans_needed && dreq_access_count != 2'b11) & !is_mret_mem_wb_o & !ecall_mem_wb_o ;
	// assign 					 	dreq.addr	= aluout_ex_mem_o; 
	// assign 						writeData 	= ((regwrite_mem_wb_o == 1 && writedata_ex_mem_o == rdAddr_mem_wb_o && rs1addr_id_ex_o != 0)==1)?dreaddata_mem_wb_o >> ((aluout_mem_wb_o[3:0]%8)*8):writedata_ex_mem_o;
	assign 						writeData 	= writedata_ex_mem_o;
	always_comb
		begin
			if((trans_needed && dreq_access_count != 2'b11))
			begin
				case(dreq_access_count)
					0: dreq.addr = {8'b0, csr_reg.satp.ppn, 12'b0} + {52'b0, (i_page_table_index1 << 3)};
					1: dreq.addr = {8'b0, dresp_data1_reg[53:10], 12'b0} + {52'b0, (i_page_table_index2 << 3)};
					2: dreq.addr = {8'b0, dresp_data2_reg[53:10], 12'b0} + {52'b0, (i_page_table_index3 << 3)};
					default:dreq.addr = 0;
				endcase
				dreq.size = MSIZE8;
				dreq.strobe = 8'b0;
				dreq.data = 0;
			end
			else if(memtowrite_ex_mem_o == 1)
			begin
				dreq.addr	= aluout_ex_mem_o;
				// dreq.size	= {1'b0, funct3_ex_mem_o[1:0]};
				case(funct3_ex_mem_o[1:0])
					2'b00: dreq.size = MSIZE1;
					2'b01: dreq.size = MSIZE2;
					2'b10: dreq.size = MSIZE4;
					2'b11: dreq.size = MSIZE8;
					default: dreq.size = MSIZE8;
				endcase
				case(funct3_ex_mem_o[1:0])
					2'b00: 	// sb
						begin 
							case(aluout_ex_mem_o[3:0]%8)
								0: begin dreq.strobe = 8'b00000001; dreq.data = writeData; 		end 
								1: begin dreq.strobe = 8'b00000010; dreq.data = writeData << 8; end 
								2: begin dreq.strobe = 8'b00000100; dreq.data = writeData << 16; end 
								3: begin dreq.strobe = 8'b00001000; dreq.data = writeData << 24; end 
								4: begin dreq.strobe = 8'b00010000; dreq.data = writeData << 32; end 
								5: begin dreq.strobe = 8'b00100000; dreq.data = writeData << 40; end 
								6: begin dreq.strobe = 8'b01000000; dreq.data = writeData << 48; end 
								7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
								default : begin dreq.strobe = 8'b00000001; dreq.data = writeData;end 
							endcase
						end
					2'b01:  // sh
						begin 
							case(aluout_ex_mem_o[3:0]%8)
								0: begin dreq.strobe = 8'b00000011; dreq.data = writeData; 		 end 
								1: begin dreq.strobe = 8'b00000110; dreq.data = writeData << 8;  end 
								2: begin dreq.strobe = 8'b00001100; dreq.data = writeData << 16; end 
								3: begin dreq.strobe = 8'b00011000; dreq.data = writeData << 24; end 
								4: begin dreq.strobe = 8'b00110000; dreq.data = writeData << 32; end 
								5: begin dreq.strobe = 8'b01100000; dreq.data = writeData << 40; end 
								6: begin dreq.strobe = 8'b11000000; dreq.data = writeData << 48; end 
								7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
								default : begin dreq.strobe = 8'b00000001; dreq.data = writeData;end 
							endcase
						end
					2'b10:  // sw
						begin 
							case(aluout_ex_mem_o[3:0]%8)
								0: begin dreq.strobe = 8'b00001111; dreq.data = writeData; 		 end 
								1: begin dreq.strobe = 8'b00011110; dreq.data = writeData << 8;  end 
								2: begin dreq.strobe = 8'b00111100; dreq.data = writeData << 16; end 
								3: begin dreq.strobe = 8'b01111000; dreq.data = writeData << 24; end 
								4: begin dreq.strobe = 8'b11110000; dreq.data = writeData << 32; end 
								5: begin dreq.strobe = 8'b11100000; dreq.data = writeData << 40; end 
								6: begin dreq.strobe = 8'b11000000; dreq.data = writeData << 48; end 
								7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
								default : begin dreq.strobe = 8'b00000001; dreq.data = writeData;end 
							endcase
						end
					2'b11:  // sd
						begin 
							case(aluout_ex_mem_o[3:0]%8)
								0: begin dreq.strobe = 8'b11111111; dreq.data = writeData; 		 end 
								1: begin dreq.strobe = 8'b11111110; dreq.data = writeData << 8;  end 
								2: begin dreq.strobe = 8'b11111100; dreq.data = writeData << 16; end 
								3: begin dreq.strobe = 8'b11111000; dreq.data = writeData << 24; end 
								4: begin dreq.strobe = 8'b11110000; dreq.data = writeData << 32; end 
								5: begin dreq.strobe = 8'b11100000; dreq.data = writeData << 40; end 
								6: begin dreq.strobe = 8'b11000000; dreq.data = writeData << 48; end 
								7: begin dreq.strobe = 8'b10000000; dreq.data = writeData << 56; end 
								default : begin dreq.strobe = 8'b00000001; dreq.data = writeData;end 
							endcase
						end
					default:	begin dreq.strobe = 8'b11111111; dreq.data = writeData; end 
				endcase
			end
			else
				begin dreq.addr = aluout_ex_mem_o; dreq.strobe = 8'b00000000; dreq.data = 64'b0; dreq.size	= MSIZE1;end
		end
	
	// assign 					 	hand_in = (Iswaitedflag_alu_ex_mem_i==1)?(iresp.data_ok&temaludone_ex_mem_i2):iresp.data_ok;
	always @(posedge clk, posedge reset)
	begin
	    if(reset)
	       csr_reg_clk <= 0;
		else if(hand_in)
			csr_reg_clk <= csr_reg;
	end
	always_comb
	begin
		hand_in = 0;
		if(trans_needed)
		begin
			if(Iswaitedflag_alu_ex_mem_i)
				hand_in = iresp.data_ok&temaludone_ex_mem_i2;
			else
				hand_in = iresp.data_ok;
		end
		else if(!trans_needed)
				hand_in = iresp.data_ok;
	end

	Memreg Memreg(
		clk, reset,
		dresp.data_ok, dresp.data,
		dbusdataok, dbusdata
		);

	mem_wb_regs mem_wb_regs(
		clk,reset,hand_in,
		aluout_mem_wb_i,aluoutff_mem_wb_i,
		dreaddata_mem_wb_i,
		regwrite_mem_wb_i,memtoreg_mem_wb_i,rdAddr_mem_wb_i,
		Iswaitedflag_data_mem_wb_i,Iswaitedflag_addr_mem_wb_i,Iswaitedflag_alu_mem_wb_i,
		funct3_alu_mem_wb_i,curpc_mem_wb_i,
		rs1addr_mem_wb_i,rs2addr_mem_wb_i,valid_mem_wb_i,

		csr_we_mem_wb_i,is_mret_mem_wb_i,csr_addr_mem_wb_i,zimm_mem_wb_i,ecall_mem_wb_i,csr_wdata_mem_wb_i,csr_result_mem_wb_i,opcode_mem_wb_i,

		aluout_mem_wb_o,aluoutff_mem_wb_o,
		dreaddata_mem_wb_o,
		regwrite_mem_wb_o,memtoreg_mem_wb_o,rdAddr_mem_wb_o,
		Iswaitedflag_data_mem_wb_o,Iswaitedflag_addr_mem_wb_o,Iswaitedflag_alu_mem_wb_o,
		funct3_alu_mem_wb_o,curpc_mem_wb_o,
		rs1addr_mem_wb_o,rs2addr_mem_wb_o,valid_mem_wb_o,

		csr_we_mem_wb_o,is_mret_mem_wb_o,csr_addr_mem_wb_o,zimm_mem_wb_o,ecall_mem_wb_o,csr_wdata_mem_wb_o,csr_result_mem_wb_o,opcode_mem_wb_o
		);
    registerFile rfile(
        .clk(clk), 
        .rst(reset),
        .we(regwrite_mem_wb_o),
		.flag_IL_S(Iswaitedflag_data_mem_wb_o|Iswaitedflag_addr_mem_wb_o),
		// .flag_IL_S_done(dresp.data_ok),
		// .flag_IL_S_done(1),
		.flag_Alu(Iswaitedflag_alu_mem_wb_o),
		// .flag_Alu_done(ALUdone),
		// .flag_Alu_done(1),
        .memtoreg(memtoreg_mem_wb_o),
        .writeAddr(rdAddr_mem_wb_o),
        .aluout(aluout_mem_wb_o),
        .aluout_ff(aluoutff_mem_wb_o),
        .readAddr1(rs1addr_id_ex_i),
        .readAddr2(rs2addr_id_ex_i),
        .writeData3(dreaddata_mem_wb_o >> ((aluout_mem_wb_o[3:0]%8)*8)),
        .readData1(tem_rD1),
        .readData2(tem_rD2),
		.regs_nxt(regs_nxt),

		.opcode(opcode_mem_wb_o),
		.funct3(funct3_alu_mem_wb_o),
		.csr_result(csr_result_mem_wb_o)
    );
	logic ecall, is_mret;
	// assign is_mret = is_mret_mem_wb_o;
	always @(posedge clk, posedge reset)
	begin
		if(reset)
			ecall <= 0;
		else if(hand_in)
			ecall <= ecall_mem_wb_i;
		else 
			ecall <= 0;
	end
	always @(posedge clk, posedge reset)
	begin
		if(reset)
			is_mret <= 0;
		else if(hand_in)
			is_mret <= is_mret_mem_wb_i;
		else 
			is_mret <= 0;
	end

	csr			csr(
		.clk(clk),
		.rst(reset),
		.csr_we(csr_we_mem_wb_o),
		.csr_addr_r(csr_addr_ex_mem_i),
		.csr_addr_w(csr_addr_mem_wb_o),

		.csr_wdata(csr_wdata_mem_wb_o),
		.csr_rdata(tem_csrD),

		.csr_reg(csr_reg),
		.is_mret(is_mret),
		.pc(curpc_mem_wb_o),
		.csr_pc(csr_pc),
		.ecall(ecall),
		.hand_in(hand_in)
	);

`ifdef VERILATOR
	DifftestInstrCommit DifftestInstrCommit(
		.clock              (clk),
		.coreid             (0),			// 无需改动
		.index              (0),			// 多发射时，例化多个该模块。前四个 Lab无需改动�????
		.valid              (hand_in&valid_mem_wb_o),				// 无提交（或提交的指令是flush导致的bubble时，�????0�????
		.pc                 (curpc_mem_wb_o),	// 这条指令�???? pc
		.instr              (0),			// 这条指令的内容，可不改动
		.skip               (hand_in&valid_mem_wb_o&(Iswaitedflag_addr_mem_wb_o|Iswaitedflag_data_mem_wb_o)&(aluout_mem_wb_o[31]==0)),			// 提交的是�????条内存读写指令，且这部分内存属于设备（addr[31] == 0）时，skip �???? 1
		// .skip               (0),			// 提交的是�????条内存读写指令，且这部分内存属于设备（addr[31] == 0）时，skip �???? 1
		.isRVC              (0),			// 前四�???? Lab 无需改动
		.scFailed           (0),			// 前四�???? Lab 无需改动
		.wen                (regwrite_mem_wb_o),		// 这条指令是否写入通用寄存器，1 bit
		.wdest              ({3'b0, rdAddr_mem_wb_o}),	// 写入哪个通用寄存�????
		.wdata              (regs_nxt[rdAddr_mem_wb_o])	// 写入的�??
	);

	DifftestArchIntRegState DifftestArchIntRegState (
		.clock              (clk),
		.coreid             (0),
		.gpr_0              (regs_nxt[0]),
		.gpr_1              (regs_nxt[1]),
		.gpr_2              (regs_nxt[2]),
		.gpr_3              (regs_nxt[3]),
		.gpr_4              (regs_nxt[4]),
		.gpr_5              (regs_nxt[5]),
		.gpr_6              (regs_nxt[6]),
		.gpr_7              (regs_nxt[7]),
		.gpr_8              (regs_nxt[8]),
		.gpr_9              (regs_nxt[9]),
		.gpr_10             (regs_nxt[10]),
		.gpr_11             (regs_nxt[11]),
		.gpr_12             (regs_nxt[12]),
		.gpr_13             (regs_nxt[13]),
		.gpr_14             (regs_nxt[14]),
		.gpr_15             (regs_nxt[15]),
		.gpr_16             (regs_nxt[16]),
		.gpr_17             (regs_nxt[17]),
		.gpr_18             (regs_nxt[18]),
		.gpr_19             (regs_nxt[19]),
		.gpr_20             (regs_nxt[20]),
		.gpr_21             (regs_nxt[21]),
		.gpr_22             (regs_nxt[22]),
		.gpr_23             (regs_nxt[23]),
		.gpr_24             (regs_nxt[24]),
		.gpr_25             (regs_nxt[25]),
		.gpr_26             (regs_nxt[26]),
		.gpr_27             (regs_nxt[27]),
		.gpr_28             (regs_nxt[28]),
		.gpr_29             (regs_nxt[29]),
		.gpr_30             (regs_nxt[30]),
		.gpr_31             (regs_nxt[31])
	);

	// （暂时不用管�????
    DifftestTrapEvent DifftestTrapEvent(
		.clock              (clk),
		.coreid             (0),
		.valid              (0),
		.code               (0),
		.pc                 (0),
		.cycleCnt           (0),
		.instrCnt           (0)
	);

	DifftestCSRState DifftestCSRState(
		.clock              (clk),
		.coreid             (0),
		.priviledgeMode     (csr_reg.mode),
		.mstatus            (csr_reg.mstatus),
		.sstatus            (csr_reg.mstatus & 64'h800000030001e000),
		.mepc               (csr_reg.mepc),
		.sepc               (0),
		.mtval              (csr_reg.mtval),
		.stval              (0),
		.mtvec              (csr_reg.mtvec),
		.stvec              (0),
		.mcause             (csr_reg.mcause),
		.scause             (0),
		.satp               (csr_reg.satp),
		.mip                (csr_reg.mip),
		.mie                (csr_reg.mie),
		.mscratch           (csr_reg.mscratch),
		.sscratch           (0),
		.mideleg            (0),
		.medeleg            (0)
	);
`endif
endmodule
`endif
