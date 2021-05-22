`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Class: Fundamentals of Digital Logic and Processor
// Designer: Shulin Zeng
// 
// Create Date: 2021/04/30
// Design Name: MultiCycleCPU
// Module Name: MultiCycleCPU
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

module MultiCycleCPU (reset, clk);
    //Input Clock Signals
    input reset;
    input clk;

    //--------------Your code below-----------------------
    // Wire
    wire [31:0] mem_address;
    wire [31:0] mem_write_data;
    wire [31:0] mem_mem_data;
    wire        mem_memread; 
    wire        mem_memwrite;
    
    wire        reg_regwrite;
    wire [ 4:0] reg_read_addr1;
    wire [ 4:0] reg_read_addr2;
    wire [ 4:0] reg_write_addr;
    wire [31:0] reg_write_data;
    wire [31:0] reg_read_data1;
    wire [31:0] reg_read_data2;

    wire [ 4:0] alu_aluconf;
    wire        alu_sign;
    wire [31:0] alu_in1;
    wire [31:0] alu_in2;
	wire        alu_zero;
	wire [31:0] alu_result;

	wire [ 3:0] aluop;
	wire [ 5:0] funct;

	wire        extop;
	wire        luiop;
	wire [15:0] immediate;
	wire [31:0] immextout;
	wire [31:0] immextshift;

	wire        irwrite;
	wire [31:0] ir_reg;
	wire [ 5:0] ir_opcode;
	wire [ 4:0] ir_rs;
	wire [ 4:0] ir_rt;
	wire [ 4:0] ir_rd;
	wire [ 4:0] ir_sa;
	wire [ 5:0] ir_funct;


	wire        pc_writecond;
	wire        iord;
	wire        irwrite;
	wire        memtoreg;
	wire [ 1:0] regdst;
	wire [ 1:0] alusrca;
	wire [ 1:0] alusrcb;
	wire [ 1:0] pcsrc;
	
	wire [31:0] nextpc;
	wire [31:0] pc;
	wire [31:0] pc_jump;	
	reg  [31:0] aluout;

	always @(posedge clk) begin
		if (rst)
			aluout <= 32'b0;
		else
			aluout <= alu_result;
	end

	// DataPath
	assign mem_address = iord ? pc : aluout;
	assign mem_write_data = reg_read_data2;

	assign reg_write_addr = ({5{regdst == 2'b00}} & 5'd31 ) | 
	                        ({5{regdst == 2'b01}} & ir_rt ) |
							({5{regdst == 2'b10}} & ir_rd ) ;

	assign reg_write_data = memtoreg ? mdr : aluout ;

	assign alu_in1 = ({32{alusrca == 2'b00}} & PC            ) |
	                 ({32{alusrca == 2'b01}} & ir_sa         ) |
					 ({32{alusrca == 2'b10}} & reg_read_data1) |
					 ({32{alusrca == 2'b10}} & 32'b0         ) ;

	assign alu_in2 = ({32{alusrcb == 2'b00}} & reg_read_data2) |
					 ({32{alusrcb == 2'b01}} & immextout     ) |
					 ({32{alusrcb == 2'b10}} & immextshift   ) ;

	assign pc_jump = {aluout[31:28], ir_reg[25:0], 2'b0};

	assign nextpc = ({32{pcsrc == 2'b00}} & aluout        ) | 
	                ({32{pcsrc == 2'b01}} & alu_result    ) |
					({32{pcsrc == 2'b10}} & reg_read_data1) |
					({32{pcsrc == 2'b11}} & pc_jump       ) ;
	// Module instances
	
	// RegTemp
	RegTemp u_rt(
		.reset  (reset       ),
		.clk    (clk         ),
		.Data_i (mem_mem_data),
		.Data_o (mdr         )
	);
	
	// PC
	PC u_pc(
		.reset   (reset   ),
		.clk     (clk     ),
		.PCWrite (pc_write),
		.PC_i    (nextpc  ),
		.PC_o    (pc      )
	);

	// Controller
	Controller u_ctrl(
		.reset       (reset       ),
		.clk         (clk         ),
		.OpCode      (ir_opcode   ),
		.Funct       (ir_funct    ),
		.PCWrite     (pc_write    ),
		.PCWriteCond (pc_writecond),
		.IorD        (iord        ),
		.MemWrite    (mem_memwrite),
		.MemRead     (mem_memread ),
		.IRWrite     (irwrite     ),
		.MemtoReg    (memtoreg    ),
		.RegDst      (regdst      ),
		.RegWrite    (reg_regwrite),
		.ExtOp       (extop       ),
		.LuiOp       (luiop       ),
		.ALUSrcA     (alusrca     ),
		.ALUSrcB     (alusrcb     ),
		.ALUOp       (aluop       ),
		.PCSource    (pcsrc       )
	);

	// InstReg
	InstReg u_ireg(
		.reset       (reset    ),
		.clk         (clk      ),
		.IRWrite     (irwrite  ),
		.Instruction (ir_reg   ),
		.Opcode      (ir_opcode),
		.rs          (ir_rs    ),
		.rt          (ir_rt    ),
		.rd          (ir_rd    ),
		.Shamt       (ir_sa    ),
		.Funct       (ir_funct )
	);

	// ImmProcess
	ImmProcess u_imm(
		.ExtOp       (extop      ),
		.LuiOp       (luiop      ),
		.Immediate   (immediate  ),
		.ImmExtOut   (immextout  ),
		.ImmExtShift (immextshift)
	);

	// ALUControl
	ALUControl u_aluctrl(
		.ALUop   (aluop      ),
		.Funct   (funct      ),
		.ALUConf (alu_aluconf),
		.Sign    (alu_sign   )
	);

    // ALU
    ALU u_alu(
        .ALUConf (alu_aluconf),
        .Sign    (alu_sign   ),
        .In1     (alu_in1    ),
        .In2     (alu_in2    ),
        .Zero    (alu_zero   ),
        .Result  (alu_result )
    );

    // Ideal Memory
    InstAndDataMemory u_mem(
        .reset      (reset          ),
        .clk        (clk            ),
        .Address    (mem_address    ),
        .Write_data (mem_write_data ),
        .MemRead    (mem_memoryread ),
        .MemWrite   (mem_memorywrite),
        .Mem_data   (mem_mem_data   )
    );

    // Register Files
    RegisterFile u_rf(
        .reset          (reset         ),
        .clk            (clk           ),
        .RegWrite       (reg_regwrite  ),
        .Read_register1 (reg_read_addr1),
        .Read_register2 (reg_read_addr2),
		.Write_register (reg_write_addr),
        .Write_data     (reg_write_data),
        .Read_data1     (reg_read_data1),
        .Read_data2     (reg_read_data2)
    );

    //--------------Your code above-----------------------

endmodule
