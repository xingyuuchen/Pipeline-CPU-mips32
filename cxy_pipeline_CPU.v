// 1.设计一个基于MIPS指令集的CPU。
// 2.CPU需要包含寄存器组、RAM模块、ALU模块、指令译码模块。
// 3.该CPU能运行基本的汇编指令。
// 4.实现cache，流水线或其他现代CPU的高级功能（加分项）
// Author: 陈星宇   Time: 2019.12.26 ~ 2019.12.29


// 以下为ALU的输入
`define ADD 	3'b100
`define ADDU	3'b101
`define SUB 	3'b110
`define AND		3'b000
`define OR		3'b001
`define SLT		3'b011
`define MUL		3'b010		// 自己扩展的MUL指令
`define MOV		3'b111		// 自己扩展的MOV指令

// 以下为func字段
`define FUNC_ADD 	6'b100000
`define FUNC_ADDU	6'b100001
`define FUNC_SUB 	6'b100010
`define FUNC_AND	6'b100100
`define FUNC_OR		6'b100101
`define FUNC_SLT	6'b101010
`define FUNC_MUL	6'b101001	// 自己扩展的MUL指令
`define FUNC_MOV	6'b100111	// 自己扩展的MOV指令


// 以下为指令的操作码字段
`define R 	6'b000000	// R型指令 op 6, rs 5, rt 5, rd 5, shamt 5, func 6
`define LW 	6'b100011
`define SW 	6'b101011
`define BEQ 6'b000100
`define J 	6'b000010	// J型指令 op 6, addr 26


// 以下为寄存器名的宏定义
`define EAX	5'b00000
`define EBX	5'b00001
`define ECX	5'b00010
`define EDX	5'b00011



`timescale 1 ns / 1 ps

// 5位2输入选择器
module Mux5_2(in0, in1, sel, out);
    input[4:0] in0, in1; 
	input sel;
    output[4:0] out;
	reg[4:0] out;
	
	always @(*) begin
		if (sel == 1)
			out = in1;
		else
			out = in0;
	end
endmodule


// 32位2输入选择器
module Mux32_2(in0, in1, sel, out);
    input[31:0] in0, in1; 
	input sel;
    output[31:0] out;
	reg[31:0] out;
	
	always @(*) begin
		if (sel == 1)
			out = in1;
		else
			out = in0;
	end
	// assign out = sel == 1 ? in1 : in0;
endmodule

// 32位4输入选择器
module Mux32_4(in0, in1, in2, in3, sel, out);
    input[31:0] in0, in1, in2, in3; 
	input[1:0] sel;
    output[31:0] out;
	reg[31:0] out;
	
	always @(*) begin
		if (sel == 00)
			out = in0;
		else if (sel == 2'b01)
			out = in1;
		else if (sel == 2'b10)
			out = in2;
		else if (sel == 2'b11)
			out = in3;
		else
			out = 32'bz;
	end
endmodule

// 16/32符号扩展
module SigExt16_32(in, out);
    input[15:0] in; 
    output[31:0] out;
	reg[31:0] out;
	
	always @(in) begin
		if (in[15] == 1)
			out = {16'b1111111111111111, in};
		else
			out = {16'b0000000000000000, in};
	end
endmodule


module SHL2_26(in, out);
    input[25:0] in; 
    output[27:0] out;
	reg[27:0] out;
	
	always @(in) begin
		out = in << 2;
	end
endmodule


module SHL2_32(in, out);
    input[31:0] in; 
    output[31:0] out;
	reg[31:0] out;
	
	always @(in) begin
		out = in << 2;
	end
endmodule


module ADDER32(in0, in1, out);
    input[31:0] in0, in1;
    output[31:0] out;
	reg[31:0] out;
	
	always @(in0 or in1) begin
		out = in0 + in1;
	end
endmodule


// PC寄存器模块
module PC_Reg(D, Q, PCWr, clk);
	input clk, PCWr;
    input[31:0] D;
    output[31:0] Q;
	reg[31:0] Q;
	
	initial begin
		Q = -1;
	end
	
	always @(posedge clk) begin
		if (PCWr == 1)
			Q = D;
	end
endmodule


module ALU(in1, in2, mod, out, zf);
	input[2:0] mod;
	output[31:0] out;
	reg[31:0] out;
	output zf;
	reg zf;
	input[31:0] in1, in2;
	
	always @(in1 or in2 or mod) begin
		case(mod)
			`ADD: out = in1 + in2;
			`SUB: out = in1 - in2;
			`AND: out = in1 & in2;
			`OR:  out = in1 | in2;
			`MUL: out = in1 * in2;		// 自己扩展的MUL指令
			`MOV: out = in1;			// 自己扩展的MOV指令
			default: out = 32'bz;
		endcase
		if (out == 0)
			zf = 1;
		else zf = 0;
	end
endmodule


// 将CU的功能分模块实现, 此模块单独控制ALU的运算
module ALU_CU(ALUOP, func, clk, ALUCtrl);
	input clk;
	input[1:0] ALUOP;
	input[5:0] func;		// function字段, R型指令使用
	output[2:0] ALUCtrl;
	reg[2:0] ALUCtrl;
	
	always @(ALUOP or func) begin
		if (ALUOP == 2'b00)
			ALUCtrl = `ADD;		// LW 和 SW
		else if (ALUOP == 2'b01)
			ALUCtrl = `SUB;
		else begin		// 10
			case(func)
				`FUNC_ADD: 	ALUCtrl = `ADD;
				`FUNC_ADDU: ALUCtrl = `ADDU;
				`FUNC_SUB:	ALUCtrl = `SUB;
				`FUNC_AND:	ALUCtrl = `AND;
				`FUNC_OR:	ALUCtrl = `OR;
				`FUNC_SLT:	ALUCtrl = `SLT;
				`FUNC_MUL:	ALUCtrl = `MUL;		// 自己扩展的MUL指令
				`FUNC_MOV:	ALUCtrl = `MOV;		// 自己扩展的MOV指令
				default: 	ALUCtrl = 3'bz;
			endcase		
		end
	end
endmodule


// 主控单元
module MCU(op_code, clk, regDst, jump, regWr, branch,
			memtoReg, ALUOP, memWr, memRd, ALUSrc);
	input clk;
	input[5:0] op_code;		// 操作码
	output[1:0] ALUOP;
	reg[1:0] ALUOP;
	output regDst, jump, regWr, branch,
			memtoReg, memWr, memRd, ALUSrc;
	reg regDst, jump, regWr, branch,
			memtoReg, memWr, memRd, ALUSrc;
	
	always @(posedge clk) begin
		#1
		regDst = ~op_code[0] & ~op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & ~op_code[5];
		ALUSrc = (op_code[0] & op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & op_code[5]) | (op_code[0] &
					op_code[1] & ~op_code[2] & op_code[3] & ~op_code[4] & op_code[5]);
		memtoReg = op_code[0] & op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & op_code[5];
		regWr = (~op_code[0] & ~op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & ~op_code[5]) | (op_code[0] & op_code[1]
					& ~op_code[2] & ~op_code[3] & ~op_code[4] & op_code[5]);
		memWr = op_code[0] & op_code[1] & ~op_code[2] & op_code[3]
					& ~op_code[4] & op_code[5];
		memRd = op_code[0] & op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & op_code[5];
		branch = ~op_code[0] & ~op_code[1] & op_code[2] & ~op_code[3]
					& ~op_code[4] & ~op_code[5];
		jump = ~op_code[0] & op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & ~op_code[5];
		ALUOP[1] = ~op_code[0] & ~op_code[1] & ~op_code[2] & ~op_code[3]
					& ~op_code[4] & ~op_code[5];
		ALUOP[0] = ~op_code[0] & ~op_code[1] & op_code[2] & ~op_code[3]
					& ~op_code[4] & ~op_code[5];
	end
endmodule


// 指令存储器
module Inst_Mem(addr, inst, clk);
	output[31:0] inst;		// 32位指令
	reg[31:0] inst;
	input[31:0] addr;	    // 32位地址线
	input clk;
	
	reg[31:0] instr[4*1024*1024-1:0];		// 存储矩阵(注意此处本来应该还有个1024的！！ 因为说数组过大报warning)
	initial begin
//---------------------------第一个程序(简单算术运算)------------------------------------------------------------
		/*
		instr[0] = {`LW, `EAX, `EBX, 16'h0A};		// 把M[第0个寄存器的内容1+10]=13赋给 第1个寄存器
		instr[1] = {`LW, `ECX, `EDX, 16'h09};		// 把M[第2个寄存器的内容3+9]=11赋给 第3个寄存器
// 13 * 11 = 143 -> Reg[00000] 
		instr[2] = {`R, `EBX, `EDX, `EAX, 5'b00000, `FUNC_MUL};	// 把第1和第3个寄存器的值加/减/乘/除, 赋给第4个寄存器
// 143 - 13 = 130 -> Reg[00100]
		instr[3] = {`R, `EAX, `EBX, `EAX, 5'b00000, `FUNC_SUB};
// 跳转!!!
		instr[4] = {`J, 26'h0A};					// 跳转到0AH * 4 = 40D
		//instr[6] = {`R, 5'b00100, 5'b00001, 5'b00100, 5'b00000, `FUNC_MUL};		// 用于验证流水线预加载的指令不对结果造成影响
		//instr[7] = {`R, 5'b00100, 5'b00001, 5'b00100, 5'b00000, `FUNC_ADD};
// 130 * 13 = 1690
		instr[40] = {`R, `EAX, 5'b00001, `EAX, 5'b00000, `FUNC_MUL};
		*/
//---------------------------------------------------------------------------------------------------------------

//----------------------------斐波那契数列----------------------------------------------------------------------- 
		///*
		instr[0] = {`R, `EAX, 5'b00100, `EAX, 5'bz, `FUNC_SUB};	// DEC EAX	(EAX是斐波那契的项数, 每次减一)
		instr[1] = {`BEQ, `EAX, 5'b00100, 16'h0A};				// 两个寄存器相等时, 离开程序
		instr[2] = {`R, `EBX, 5'bz, `EDX, 5'bz, `FUNC_MOV};		// MOV EDX, EBX
		instr[3] = {`R, `EBX, `ECX, `EBX, 5'bz, `FUNC_ADD};		// ADD EBX, ECX
		instr[4] = {`R, `EDX, 5'bz, `ECX, 5'bz, `FUNC_MOV};		// MOV ECX, EDX
		instr[5] = {`J, 26'h00};								// 若不相等, 跳转到0
		//*/
//---------------------------------------------------------------------------------------------------------------
	end
	
	always @(posedge clk) begin
		#1
		inst = instr[addr];
	end
endmodule


// 寄存器堆RF
module RF(R_Reg1, R_Reg2, R_data1, R_data2, W_Reg, W_data, we, clk);
	input[4:0] R_Reg1, R_Reg2, W_Reg;		// 5位地址线
	input[31:0] W_data;
	input clk, we;
	output[31:0] R_data1, R_data2;
	reg[31:0] R_data1, R_data2;
	
	reg[31:0] Reg[31:0];
	initial begin
//----------------------------------第一个程序----------------------------------
		/*
		Reg[0] = 32'h01;	// EAX
		Reg[1] = 32'h00;	// EBX
		Reg[2] = 32'h03;	// ECX
		Reg[3] = 32'h07;	// EDX
		*/
//----------------------------------------------------------------------------------

//----------------------------------斐波那契数列------------------------------------
		///*
		Reg[0] = 32'h08;	// EAX  斐波那契的项数
		Reg[1] = 32'h01;	// EBX
		Reg[2] = 32'h01;	// ECX
		Reg[3] = 32'h00;	// EDX
		Reg[4] = 32'h01;	// 1
		//*/
//----------------------------------------------------------------------------------
		$monitor($time, "us:  EAX=%d, EBX=%d, ECX=%d, EDX=%d", Reg[0], Reg[1], Reg[2], Reg[3]);
		//$monitor($time, "us:  EBX=%d", Reg[1]);
		
	end
	
	always @(R_Reg1 or R_Reg2) begin
		R_data1 = Reg[R_Reg1];
		R_data2 = Reg[R_Reg2];
	end
	
	always @(posedge clk) begin
		#3
		if (we == 1) 		// 写
			Reg[W_Reg] = W_data;
		R_data1 = Reg[R_Reg1];
		R_data2 = Reg[R_Reg2];
		
	end
endmodule


// 数据存储器 2^32 * 32 = 16GB
module Data_Mem(addr, R_data, W_data, clk, R, W);
	output[31:0] R_data;		// 32位数据线
	reg[31:0] R_data;
	input[31:0] addr, W_data;	    // 32位地址线
	input clk, R, W;
	
	reg[31:0] mem[4*1024*1024-1:0];	// 存储矩阵(注意此处本来应该还有个1024的！！ 因为说数组过大报warning)
	initial begin
		mem[11] = 32'h0D;
		mem[12] = 32'h0B;
	end
	
	always @(addr) begin
		if (R == 1)
			R_data = mem[addr];
	end
	
	always @(posedge clk) begin
		if (R == 1 && W != 1) begin
			R_data = mem[addr];
			end
		else if (W == 1)
			mem[addr] = W_data;
	end
endmodule



// 流水线CPU特有
// 转发控制单元
module Transfer_CU(EX_MA_RegWr, EX_MA_MemtoReg, EX_MA_MemWr, 		// 完成
			ID_EX_IR, EX_MA_IR, MA_WB_IR, ID_EX_MemWr, MA_WB_RegWr,
			ALUSrc_A, ALUSrc_B, MemSrc, ALUSrc);
	input EX_MA_RegWr, EX_MA_MemtoReg, EX_MA_MemWr, 
				ID_EX_MemWr, MA_WB_RegWr, ALUSrc;
	input[31:0] ID_EX_IR, EX_MA_IR, MA_WB_IR;
	output[1:0] ALUSrc_A, ALUSrc_B;
	output MemSrc;
	reg[1:0] ALUSrc_A, ALUSrc_B;
	reg MemSrc;
	
	initial begin
		ALUSrc_A = 2'b00;
		if (ALUSrc == 1)
			ALUSrc_B = 2'b11;
		else
			ALUSrc_B = 2'b00;
		MemSrc = 0;
	end
	
	always @(*) begin
		ALUSrc_A = 2'b00;
		if (ALUSrc == 1)
			ALUSrc_B = 2'b11;
		else
			ALUSrc_B = 2'b00;
		
		if ((EX_MA_RegWr & ~EX_MA_MemtoReg & (EX_MA_IR[15:11] == ID_EX_IR[25:21])) == 1)
			ALUSrc_A = 2'b01;
		if ((EX_MA_RegWr & ~EX_MA_MemtoReg & (EX_MA_IR[15:11] == ID_EX_IR[20:16])) == 1)
			ALUSrc_B = 2'b01;
		if ((MA_WB_RegWr & ~ID_EX_MemWr & 		// 此处对书上的组合逻辑做了修改
					(MA_WB_IR[20:16] == ID_EX_IR[25:21] | MA_WB_IR[15:11] == ID_EX_IR[25:21])) == 1)
			ALUSrc_A = 2'b10;
		if ((MA_WB_RegWr & ~ID_EX_MemWr & 		// 此处对书上的组合逻辑做了修改
					(MA_WB_IR[20:16] == ID_EX_IR[20:16] | MA_WB_IR[15:11] == ID_EX_IR[20:16])) == 1)
			ALUSrc_B = 2'b10;
			
		if (MA_WB_RegWr & EX_MA_MemWr & 
					(MA_WB_IR[20:16] == EX_MA_IR[20:16]) == 1)
			MemSrc = 1;
		else
			MemSrc = 0;
	end
endmodule



// 流水线CPU特有
// 冒险检测单元
module Risk_Detect(FI_ID_IR_Rs, FI_ID_IR_Rt, ID_EX_MemRd, ID_EX_Jump,
					EX_MA_Branch, ID_EX_IR_Rt, EX_MA_ZF, PCWr, FI_ID_RegWr,
					clear0, clear1, clk);
	input ID_EX_MemRd, ID_EX_Jump, EX_MA_Branch, EX_MA_ZF, clk;
	input[4:0] FI_ID_IR_Rs, FI_ID_IR_Rt, ID_EX_IR_Rt;
	output PCWr, FI_ID_RegWr, clear0, clear1;
	reg PCWr, FI_ID_RegWr, clear0, clear1;
	integer i = 0;
	
	initial begin
		FI_ID_RegWr = 1;
		PCWr = 1;
		clear0 = 0;
		clear1 = 0;
	end
	
	always @(*) begin
		clear1 = 0;
		PCWr = 1;
		FI_ID_RegWr = 1;	
		// 检测load-use
		if (ID_EX_MemRd & ((ID_EX_IR_Rt == FI_ID_IR_Rs) | 
				(ID_EX_IR_Rt == FI_ID_IR_Rt)) == 1) begin
			PCWr = 0;
			FI_ID_RegWr = 0;
			clear0 = 1;
		end
		// 检测BEQ指令预测失败
		if ((EX_MA_Branch) & (EX_MA_ZF == 1) == 1) begin
			clear0 = 1;
			clear1 = 1;
		end 
		
		// 检测jump指令
		if (ID_EX_Jump == 1) begin
			clear0 = 1;		
			i = 2;	// 置0三个周期
		end 
	end
	
	always @(posedge clk) begin
		if (i > 0) begin	// i > 0 clear0就=1
			i = i - 1;
			end
		else
			clear0 = 0;
	end
endmodule




module main;
	reg clk;
	wire zf;	// ALU的运算结果为0 flag
	
	wire[31:0] PC;
	wire[31:0] PC_new;		// 自增或跳转后的PC值
	wire[27:0] PC_low28;	// PC的低28位
	wire[31:0] inst;		// 指令
	wire[2:0] ALUCtrl;
	wire[31:0] RF_R_data1, RF_R_data2, RF_W_data, 
					DM_R_data, ALU_inA, ALU_inB, ALU_out;
	wire[31:0] sigExt_out;		// inst[15:0]符号扩展到32位的输出
	wire[31:0] add1_out, add2_in1, add2_out;
	wire[4:0] RF_W_Reg;
	
	// 从MCU出来的, 暂存起来
	wire RegDst_tmp, jump_tmp, RegWr_tmp, branch_tmp,
			MemtoReg_tmp, MemWr_tmp, MemRd_tmp, ALUSrc_tmp;	
	wire[1:0] ALUOP_tmp;
	
	// 真正的控制信号
	wire RegDst, jump, RegWr, branch,
			MemtoReg, MemWr, MemRd, ALUSrc;
	wire[1:0] ALUOP;

// 流水线新增
	wire PCWr, MemSrc, clear0, clear1, FI_ID_RegWr, PCSrc;
	wire[1:0] ALUSrc_A, ALUSrc_B;
	wire[31:0] mux_branch_jump_out;
	wire[31:0] DM_W_data;
	
	
	
	
// 级间寄存器
// FI/ID间
	reg[31:0] FI_ID_NPC1, FI_ID_IR;
// ID/EX间
	reg[31:0] ID_EX_NPC1, ID_EX_Imm32, ID_EX_IR, ID_EX_Rs, ID_EX_Rt;
	reg ID_EX_RegDst, ID_EX_RegWr, ID_EX_MemtoReg, ID_EX_MemRd, ID_EX_MemWr, 
			ID_EX_Branch, ID_EX_Jump, ID_EX_ALUSrc;
	reg[1:0] ID_EX_ALUOP;
// EX/MA间
	reg[31:0] EX_MA_NPC2, EX_MA_NPC3, EX_MA_IR, EX_MA_ALUOut, EX_MA_Rt;
	reg EX_MA_ZF;
	reg EX_MA_RegDst, EX_MA_RegWr, EX_MA_MemtoReg, EX_MA_MemRd, EX_MA_MemWr,
			EX_MA_Branch, EX_MA_Jump;
// MA/WB间
	reg[31:0] MA_WB_ALUOut, MA_WB_MEMOut, MA_WB_IR;
	reg MA_WB_RegDst, MA_WB_RegWr, MA_WB_MemtoReg;
	
	
	assign ALUOP = ID_EX_ALUOP;
	assign ALUSrc = ID_EX_ALUSrc;
	assign MemRd = EX_MA_MemRd;
	assign MemWr = EX_MA_MemWr;
	assign branch = EX_MA_Branch;
	assign jump = EX_MA_Jump;
	assign RegDst = MA_WB_RegDst;
	assign RegWr = MA_WB_RegWr;
	assign MemtoReg = MA_WB_MemtoReg;
	assign PCSrc = (EX_MA_ZF & branch) | jump;
	
			
	PC_Reg pc_reg(PC_new, PC, PCWr, clk);
	MCU mcu(FI_ID_IR[31:26], clk, RegDst_tmp, jump_tmp, RegWr_tmp, branch_tmp,
			MemtoReg_tmp, ALUOP_tmp, MemWr_tmp, MemRd_tmp, ALUSrc_tmp);	
	ALU_CU alu_cu(ALUOP, ID_EX_IR[5:0], clk, ALUCtrl);
	ALU alu(ALU_inA, ALU_inB, ALUCtrl, ALU_out, zf);
	Inst_Mem IM(PC, inst, clk);
	Data_Mem DM(EX_MA_ALUOut, DM_R_data, DM_W_data, clk, MemRd, MemWr);
	Mux32_2 mux_mem_to_reg(MA_WB_ALUOut, MA_WB_MEMOut, MemtoReg, RF_W_data);
	Mux5_2 mux_RegDst(MA_WB_IR[20:16], MA_WB_IR[15:11], RegDst, RF_W_Reg);
	RF rf(FI_ID_IR[25:21], FI_ID_IR[20:16], RF_R_data1, RF_R_data2, RF_W_Reg, RF_W_data, RegWr, clk);
	SigExt16_32 sigext(FI_ID_IR[15:0], sigExt_out);
	Mux32_4 mux_alu_src_A(ID_EX_Rs, EX_MA_ALUOut, RF_W_data, 32'bz, ALUSrc_A, ALU_inA);
	Mux32_4 mux_alu_src_B(ID_EX_Rt, EX_MA_ALUOut, RF_W_data, ID_EX_Imm32, ALUSrc_B, ALU_inB);
	SHL2_26 shl_26(ID_EX_IR[25:0], PC_low28);
	SHL2_32 shl_32(ID_EX_Imm32, add2_in1);
	ADDER32 add1(1, PC, add1_out);			// 我是按字编址, 每次PC加1
	ADDER32 add2(ID_EX_NPC1, add2_in1, add2_out);
	Mux32_4 mux_branch_jump(32'bz, EX_MA_NPC3, EX_MA_NPC2, 32'bz, 
				{EX_MA_ZF&branch, jump}, mux_branch_jump_out);
	Mux32_2 mux_PCSrc(add1_out, mux_branch_jump_out, PCSrc, PC_new);
	Mux32_2 mux_MemSrc(EX_MA_Rt, RF_W_data, MemSrc, DM_W_data);
	
	// 转发控制单元
	Transfer_CU TCU(EX_MA_RegWr, EX_MA_MemtoReg, EX_MA_MemWr, 
			ID_EX_IR, EX_MA_IR, MA_WB_IR, ID_EX_MemWr, MA_WB_RegWr,
			ALUSrc_A, ALUSrc_B, MemSrc, ALUSrc);
	// 冒险检测单元
	Risk_Detect RD(FI_ID_IR[25:21], FI_ID_IR[20:16], ID_EX_MemRd, ID_EX_Jump,
					EX_MA_Branch, ID_EX_IR[20:16], EX_MA_ZF, PCWr, FI_ID_RegWr,
					clear0, clear1, clk);
	
	
	integer i;
    initial begin
		//$monitor($time, "ns:  PC=%d, op_code=%b, RegDst=%b, RegWr=%b, memtoReg=%b, MemRd=%b, MemWr=%b, branch=%b, jump=%b, ALUOP=%b, ALUSrc=%b",
		//						PC, inst[31:26], RegDst, RegWr, MemtoReg, MemRd, MemWr, branch, jump, ALUOP, ALUSrc);
		//$monitor($time, "us:  PC=%b, inst=%b, M[12]=%d", PC, inst, DM_R_data);
		
		//$monitor($time, "us: code=%b, RF_R_data1=%d, IDEX_Rs=%d, ALUinA=%d, ALUinB=%d, out=%d, IMM32=%d",
		//			inst[31:26], RF_R_data1, ID_EX_Rs, ALU_inA, ALU_inB, RF_W_data, ID_EX_Imm32);
		
		$dumpfile("cxy_pipeline_CPU.vcd");
        $dumpvars(0, main);
		
		clk = 0;
		//PC = 0;
		//FI_ID_RegWr = 1;
		
        #10000 $finish;
		
    end
	
	
	always #50 clk = ~clk;
	always @(posedge clk) begin
		// 级间控制信号寄存器
		MA_WB_RegDst = EX_MA_RegDst;
		MA_WB_RegWr = EX_MA_RegWr;
		MA_WB_MemtoReg = EX_MA_MemtoReg;
		if (clear1 == 1) begin
			EX_MA_RegDst = 1'bz;
			EX_MA_RegWr = 1'bz;
			EX_MA_MemtoReg = 1'bz;
			EX_MA_MemRd = 1'bz;
			EX_MA_MemWr = 1'bz;
			EX_MA_Branch = 1'bz;
			EX_MA_Jump = 1'bz;
			end
		else begin
			EX_MA_RegDst = ID_EX_RegDst;
			EX_MA_RegWr = ID_EX_RegWr;
			EX_MA_MemtoReg = ID_EX_MemtoReg;
			EX_MA_MemRd = ID_EX_MemRd;
			EX_MA_MemWr = ID_EX_MemWr;
			EX_MA_Branch = ID_EX_Branch;
			EX_MA_Jump = ID_EX_Jump;
		end
		
		if (clear0 == 1) begin
			ID_EX_RegDst = 1'bz;
			ID_EX_RegWr = 1'bz;
			ID_EX_MemtoReg = 1'bz;
			ID_EX_MemRd = 1'bz;
			ID_EX_MemWr = 1'bz;
			ID_EX_Branch = 1'bz;
			ID_EX_Jump = 1'bz;
			ID_EX_ALUOP = 1'bz;
			ID_EX_ALUSrc = 1'bz;
			end
		else begin
			ID_EX_RegDst = RegDst_tmp;
			ID_EX_RegWr = RegWr_tmp;
			ID_EX_MemtoReg = MemtoReg_tmp;
			ID_EX_MemRd = MemRd_tmp;
			ID_EX_MemWr = MemWr_tmp;
			ID_EX_Branch = branch_tmp;
			ID_EX_Jump = jump_tmp;
			ID_EX_ALUOP = ALUOP_tmp;
			ID_EX_ALUSrc = ALUSrc_tmp;
		end
		// 级间其他寄存器
		MA_WB_ALUOut = EX_MA_ALUOut;
		MA_WB_MEMOut = DM_R_data;
		MA_WB_IR = EX_MA_IR;
		EX_MA_NPC3 = {ID_EX_NPC1[31:28], PC_low28};
		EX_MA_NPC2 = add2_out;
		EX_MA_ZF = zf;
		EX_MA_ALUOut = ALU_out;
		EX_MA_Rt = ID_EX_Rt;
		EX_MA_IR = ID_EX_IR;
		ID_EX_NPC1 = FI_ID_NPC1;
		ID_EX_Rs = RF_R_data1;
		ID_EX_Rt = RF_R_data2;
		ID_EX_Imm32 = sigExt_out;
		ID_EX_IR = FI_ID_IR;
		if (FI_ID_RegWr == 1) begin
			FI_ID_NPC1 = add1_out;
			FI_ID_IR = inst;
		end
	end
endmodule












