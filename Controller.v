`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Class: Fundamentals of Digital Logic and Processor
// Designer: Shulin Zeng
// 
// Create Date: 2021/04/30
// Design Name: MultiCycleCPU
// Module Name: Controller
// Project Name: Multi-cycle-cpu
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Controller(reset, clk, OpCode, Funct, 
                PCWrite, PCWriteCond, IorD, MemWrite, MemRead,
                IRWrite, MemtoReg, RegDst, RegWrite, ExtOp, LuiOp,
                ALUSrcA, ALUSrcB, ALUOp, PCSource);
    //Input Clock Signals
    input reset;
    input clk;
    //Input Signals
    input  [5:0] OpCode;
    input  [5:0] Funct;
    //Output Control Signals
    output PCWrite;
    output PCWriteCond;
    output IorD;
    output MemWrite;
    output MemRead;
    output IRWrite;
    output MemtoReg;
    output [1:0] RegDst;
    output RegWrite;
    output ExtOp;
    output LuiOp;
    output [1:0] ALUSrcA;
    output [1:0] ALUSrcB;
    output reg [3:0] ALUOp;
    output [1:0] PCSource;
      
    //--------------Your code below-----------------------
	assign PCWrite = state == EX_CAL_MEM || state == EX_CAL || state == EX_BRANCH || state == EX_JUMP;
	assign PCWriteCond = inst_beq;
	assign IorD = state == IF;
	assign MemWrite = state == MEM_WRITE;
	assign MemRead = state == MEM_READ;
	assign IRWrite = state == ID;
	assign RegDst = inst_jal ? 2'b00 :
					write_rt ? 2'b01 : 2'b10;
	assign MemtoReg = state == WB_FROM_MEM;
	assign RegWrite = state == WB_FROM_MEM || state == WB_CAL || state == EX_JUMP && (inst_jal || inst_jalr);
	assign ExtOp = ~(inst_and || inst_andi || inst_or || inst_xor || inst_nor);
	assign LuiOp = inst_lui;
	
	assign ALUSrcA = (state == ID || state == EX_JUMP || state == EX_BRANCH) ? 2'b00 : 
	 				 (state == EX_CAL && (inst_sll || inst_srl || inst_sra)) ? 2'b01 :
					 (state == EX_CAL &&  inst_lui                         ) ? 2'b10 : 2'b11;

	assign ALUSrcB = (state == ID)                   ? 2'b11:
					 (state == EX_CAL && inst_rtype) ? 2'b00 :
					 (state == EX_BRANCH)         ? 2'b10 : 2'b11;
	
	assign PCSource = (inst_jr ) ? 2'b10 :
					  (inst_j  ) ? 2'b11 : 
					  (inst_beq) ? 2'b01 : 2'b00;

	assign inst_load	= OpCode == 6'h23;
	assign inst_store	= OpCode == 6'h2b;
	assign inst_lui		= OpCode == 6'h0f;
	assign inst_add		= OpCode == 6'h00 && Funct == 6'h20; 
	assign inst_addu	= OpCode == 6'h00 && Funct == 6'h21; 
	assign inst_sub		= OpCode == 6'h00 && Funct == 6'h22; 
	assign inst_subu	= OpCode == 6'h00 && Funct == 6'h23; 
	assign inst_addi	= OpCode == 6'h08;
	assign inst_addiu	= OpCode == 6'h09;
	assign inst_and		= OpCode == 6'h00 && Funct == 6'h24;
	assign inst_or		= OpCode == 6'h00 && Funct == 6'h25;
	assign inst_xor		= OpCode == 6'h00 && Funct == 6'h26;
	assign inst_nor		= OpCode == 6'h00 && Funct == 6'h27;
	assign inst_andi	= OpCode == 6'h0c;
	assign inst_sll		= OpCode == 6'h00 && Funct == 6'h00;
	assign inst_srl		= OpCode == 6'h00 && Funct == 6'h02;
	assign inst_sra		= OpCode == 6'h00 && Funct == 6'h03;
	assign inst_slt		= OpCode == 6'h00 && Funct == 6'h2a;
	assign inst_sltu	= OpCode == 6'h00 && Funct == 6'h2b;
	assign inst_slti	= OpCode == 6'h0a;
	assign inst_sltiu	= OpCode == 6'h0b;
	assign inst_beq		= OpCode == 6'h04;
	assign inst_j		= OpCode == 6'h02;
	assign inst_jal		= OpCode == 6'h03;
	assign inst_jr		= OpCode == 6'h00 && Funct == 6'h08;
	assign inst_jalr	= OpCode == 6'h00 && Funct == 6'h09;

	assign inst_cal = inst_lui   || inst_add   || inst_addu  || inst_sub   || inst_subu 
	               || inst_addi  || inst_and   || inst_or    || inst_xor   || inst_nor
				   || inst_andi  || inst_sll   || inst_srl   || inst_sra   || inst_slt
				   || inst_sltu  || inst_slti  || inst_sltiu ;
	
	assign write_rt = inst_lw    || inst_sw    || inst_addi  || inst_addiu || inst_andi 
	               || inst_slti  || inst_sltiu ;

	assign inst_rtype = Opcode == 6'b0;

	assign inst_jump = inst_j || inst_jr || inst_jal || inst_jalr;

	localparam IF			= 4'd0,
			   ID			= 4'd1,
			   EX_CAL_MEM	= 4'd2, 
			   MEM_READ		= 4'd3,
			   WB_FROM_MEM  = 4'd4,
			   MEM_WRITE	= 4'd5,
			   EX_CAL		= 4'd6,
			   WB_CAL		= 4'd7,
			   EX_BRANCH	= 4'd8,
			   EX_JUMP		= 4'd9;

	reg [3:0] state;
	reg [3:0] next_state;

	always @(posedge clk) begin
		if (rst)
			state <= IF;
		else
			state <= next_state;
	end

	always @(*) begin
		case (state)
		IF:
			next_state = ID;
		ID:
			if (inst_load || inst_store)
				next_state = EX_CAL_MEM;
			else if (inst_cal)
				next_state = EX_CAL;
			else if (inst_beq)
				next_state = EX_BRANCH;
			else
				next_state = IF;
		EX_CAL_MEM:
			if (inst_load)
				next_state = MEM_READ;
			else
				next_state = MEM_WRITE;
		MEM_READ:
			next_state = WB_FROM_MEM;
		WB_FROM_MEM:
			next_state = IF;
		MEM_WRITE:
			next_state = IF;
		EX_CAL:
			next_state = WB_CAL;
		WB_CAL:
			next_state = IF;
		EX_BRANCH:
			next_state = IF;
		EX_JUMP:
			next_state = IF;
		default:
			next_state = IF;
		endcase
	end

    //--------------Your code above-----------------------


    //ALUOp
    always @(*) begin
        ALUOp[3] = OpCode[0];
        if (state == IF || state == ID) begin
            ALUOp[2:0] = 3'b000;
        end else if (OpCode == 6'h00) begin 
            ALUOp[2:0] = 3'b010;
        end else if (OpCode == 6'h04) begin
            ALUOp[2:0] = 3'b001;
        end else if (OpCode == 6'h0c) begin
            ALUOp[2:0] = 3'b100;
        end else if (OpCode == 6'h0a || OpCode == 6'h0b) begin
            ALUOp[2:0] = 3'b101;
        end else begin
            ALUOp[2:0] = 3'b000;
        end
    end

endmodule
