module adder(a,b,y);
    input [31:0] a, b;
    output [31:0] y;
    assign y = a + b;
endmodule


module Mux_2x1 (I0,I1,s,out);

    input [31:0]I0,I1;
    input s;
    output [31:0]out;

    assign out= (s) ? I1 : I0;
    
endmodule

module Mux_4x1(I0,I1,I2,s,out);
    input [31:0] I0,I1,I2;
    input [1:0] s;
    output [31:0] out;
    assign out = (s == 2'b00) ? I0:
                    (s == 2'b01) ? I1: I2;
                 
endmodule


module ALU (aluControl,a,b,result,zero);
    input [2:0] aluControl;
    input [31:0] a;
    input [31:0] b;
    output reg [31:0] result;
    output  zero;

    always @(*)
    begin
        case(aluControl)
            3'b000: result = a + b;  
            3'b001: result = a - b;   
            3'b010: result = a & b;   
            3'b011: result = a | b; 
            3'b100:result = a >>> b[5:0];  
            3'b101: result = (a < b) ? 32'b00000000000000000000000000000001 : 32'b00000000000000000000000000000000;
            default: result = 32'h00000000;
        endcase
       
    end
    assign zero = &(~result);
endmodule



module PC (PCNext,clk,rst,PC);
    input [31:0] PCNext;
    input clk,rst;
    output reg [31:0] PC;
    always @(posedge clk)
    begin
        if(~rst)
            PC <= {32{1'b0}};
        else
            PC <= PCNext;
    end
endmodule

module PC_Adder (PC,PCPlus4);

    input [31:0]PC;
    output [31:0]PCPlus4;

    assign PCPlus4 = PC + 32'h4;
    
endmodule


module Instruction_Memory ( rst, A, RD);

    input rst;
    input [31:0] A;
    output  [31:0] RD;
    reg [31:0] mem [1023:0];

    assign RD = (~rst) ? {32{1'b0}} : mem[A[31:2]];
    // result[count[i]] = arr[i]
    initial begin
        mem[0] = 32'h00000413;// addi s0, zero, 0x00000000  // arr dizisinin başlangıç adresi    
        mem[1] = 32'h05000493; // addi s1, zero, 0x00000050  // count dizisinin başlangıç adresi
        mem[2] = 32'h0A000993; // addi s3, zero, 0x000000A0  // result dizisinin başlangıç adresi
        mem[3] = 32'h00000913; // addi s2, zero, 0  // Döngü değişkeni (i) 
        mem[4] = 32'h01400293; //addi t0, zero, 20 // dizi boyutu
        mem[5] = 32'h00048393; //addi t2,s1,0 // t2 = count adresini 
        mem[6] = 32'h00080F93;// addi t6,s0,0 // arr adresi
        mem[7] = 32'h02590663;// for:beq s2, t0, end_loop // Döngü değişkeni (i) dizinin boyutuna eşitse döngüden çık
        mem[8] = 32'h0003AE83; //lw t4, 0(t2)// count dizisinin i. elemanını yükle
        mem[9] = 32'h000FA303;//lw t1,0(t6) // arr dizisinin i.elemanını yukle 
        mem[10] = 32'h004F8F93;//addi t6,t6,4 // arr adresini bir sonrakine tasi
        mem[11] = 32'h00438393; //addi t2, t2, 4// count adresini bir sonraki elemana taşı
        mem[12] = 32'h00190913;//addi s2, s2, 1// i = i + 1
        mem[13] = 32'h01DE8EB3;//add t4,t4,t4 
        mem[14] = 32'h01DE8EB3;//add t4,t4,t4
        mem[15] = 32'h01D98F33;//add t5,s3,t4 // result+ t4 adresi
        mem[16] =32'h006F2023;//sw t1,0(t5) // arr in i.değerini bu adrese at 
        mem[17] =32'hFD9FF0EF;//jal ra for // Döngüye geri dön    -36 olmalı


    end
    endmodule



module register_file(
    input clk, rst, WE3,
    input [4:0] A1, A2, A3,
    input [31:0] WD3,
    output [31:0] RD1, RD2
);

    reg [31:0] Register [31:0]; 

    always @(posedge clk) begin
        if (~rst) begin
            for (integer i = 0; i < 32; i = i + 1) begin
                Register[i] <= 32'h00000000;
            end
        end
        
    end
    
    always @ (posedge clk)
    begin
        if(WE3)
            Register[A3] <= WD3;
    end
    
    
    assign RD1 = (~rst) ? 32'd0 : Register[A1];
    assign RD2 = (~rst) ? 32'd0 : Register[A2];

endmodule



module ALU_Decoder(ALUOp,funct3,funct7,op,ALUControl);
    input [1:0] ALUOp;
    input [2:0] funct3;
    input [6:0] funct7;
    input [6:0] op; 
    output [2:0] ALUControl;

    wire [2:0] ALUControl_Logic;
    assign ALUControl = (ALUOp == 2'b00) ? 3'b000 :
                        (ALUOp == 2'b01) ? 3'b001 :
                        (ALUOp == 2'b10) ? ALUControl_Logic : 3'bxxx;

    

    assign ALUControl_Logic = (funct3 == 3'b000) ? 
                                    ({op[5], funct7[5]} == 2'b11 ? 3'b001 : 3'b000) :
                                    (funct3 == 3'b010) ? 3'b101 :
                                    (funct3 == 3'b110) ? 3'b011 :
                                    (funct3 == 3'b111) ? 3'b010 :
                                    (funct3 == 3'b101) ? 3'b100 : 
                                    3'b000;

endmodule




module Main_Decoder(Op,RegWrite,ImmSrc,ALUSrc,MemWrite,ResultSrc,Branch,Jump,ALUOp,zero,PCSrc);
    input [6:0]Op;
    input zero; 
    output RegWrite,ALUSrc,MemWrite,Branch,Jump,PCSrc;
    output [1:0]ImmSrc,ALUOp,ResultSrc;


    assign RegWrite = (Op == 7'b0000011 | Op == 7'b0110011 | Op == 7'b0010011 | Op == 7'b1101111) ? 1'b1 :1'b0 ;

    assign ImmSrc = (Op == 7'b0000011 | Op == 7'b0010011) ? 2'b00 : 
                    (Op == 7'b0100011) ? 2'b01 :
                    (Op == 7'b1100011) ? 2'b10 : 2'b11  ;                     

    assign ALUSrc = (Op == 7'b0000011 | Op == 7'b0100011 | Op == 7'b0010011) ? 1'b1 : 1'b0 ;

    assign MemWrite = (Op == 7'b0100011) ? 1'b1 :1'b0 ;

    assign ResultSrc = (Op == 7'b0000011) ? 2'b01 :
                        (Op == 7'b0110011 | Op == 7'b0010011) ? 2'b00 : 2'b10 ;

    assign Branch = (Op == 7'b1100011) ? 1'b1 :1'b0 ;

    assign Jump = (Op == 7'b1101111) ? 1'b1:1'b0;

    assign PCSrc = (zero & Branch) | Jump;
    assign ALUOp = (Op == 7'b0000011 | Op == 7'b0100011) ? 2'b00 :
                    (Op == 7'b0110011 | Op == 7'b0010011) ? 2'b10 : 2'b01 ;

endmodule

module Control_Unit(Op,RegWrite,ImmSrc,ALUSrc,MemWrite,ResultSrc,Branch,Jump,funct3,funct7,ALUControl,zero,PCSrc);

    input [6:0]Op,funct7;
    input [2:0]funct3;
    input zero;
    output RegWrite,ALUSrc,MemWrite,Jump,Branch;
    output [1:0]ImmSrc,ResultSrc;
    output [2:0]ALUControl;
    output PCSrc;

    wire [1:0]ALUOp;

    Main_Decoder Main_Decoder(
                .Op(Op),
                .RegWrite(RegWrite),
                .ImmSrc(ImmSrc),
                .MemWrite(MemWrite),
                .ResultSrc(ResultSrc),
                .Branch(Branch),
                .Jump(Jump),
                .ALUSrc(ALUSrc),
                .ALUOp(ALUOp),
                .zero(zero),
                .PCSrc(PCSrc)
    );

    ALU_Decoder ALU_Decoder(
                            .ALUOp(ALUOp),
                            .funct3(funct3),
                            .funct7(funct7),
                            .op(Op),
                            .ALUControl(ALUControl)
    );


endmodule

module Data_Memory(clk,rst,WE,WD,A,RD);

    input clk,rst,WE;
    input [31:0]A,WD;
    output [31:0]RD;

    reg [31:0] mem [1023:0];

    always @ (posedge clk)
    begin
        if(WE)
            mem[A[31:2]] <= WD;
    end

    assign RD = (~rst) ? 32'd0 : mem[A[31:2]];
    
    
    

    initial begin
        // arr
        mem[0] = 32'h00000003;
        mem[1] = 32'h00000007;
        mem[2] = 32'h00000002;
        mem[3] = 32'h00000006;
        mem[4] = 32'h00000005;
        mem[5] = 32'h00000004;
        mem[6] = 32'h00000001;
        mem[7] = 32'h000003E8;
        mem[8] = 32'h000003E7;
        mem[9] = 32'h00000019;
        mem[10] = 32'h0000005A;
        mem[11] = 32'h00000064;
        mem[12] = 32'h0000001E;
        mem[13] = 32'h00000014;
        mem[14] = 32'h0000000A;
        mem[15] = 32'h000000C8;
        mem[16] = 32'h00000CE4;
        mem[17] = 32'h000000FA;
        mem[18] = 32'h0000000C;
        mem[19] = 32'h0000004B;
        //count
        mem[20] = 32'h00000011;
        mem[21] = 32'h0000000D;
        mem[22] = 32'h00000012;
        mem[23] = 32'h0000000E;
        mem[24] = 32'h0000000F;
        mem[25] = 32'h00000010;
        mem[26] = 32'h00000013;
        mem[27] = 32'h00000001;
        mem[28] = 32'h00000002;
        mem[29] = 32'h00000009;
        mem[30] = 32'h00000006;
        mem[31] = 32'h00000005;
        mem[32] = 32'h00000008;
        mem[33] = 32'h0000000A;
        mem[34] = 32'h0000000C;
        mem[35] = 32'h00000004;
        mem[36] = 32'h00000000;
        mem[37] = 32'h00000003;
        mem[38] = 32'h0000000B;
        mem[39] = 32'h00000007;
       


    end
    always @(posedge clk) begin
        if (~rst) begin
            for (integer i = 40; i < 60; i = i + 1) begin
                mem[i] <= 32'h00000000;
            end
        end
        
    end



endmodule


module Sign_Extend (input [31:0] Instr, input [1:0] ImmSrc, output reg [31:0] Imm_Ext);
    always @*
    begin
        case (ImmSrc)
            2'b00: Imm_Ext = {{20{Instr[31]}}, Instr[31:20]};
            2'b01: Imm_Ext = {{20{Instr[31]}}, Instr[31:25], Instr[11:7]};
            2'b10: Imm_Ext = {{20{Instr[31]}}, Instr[7], Instr[30:25], Instr[11:8], 1'b0};
            2'b11: Imm_Ext = {{12{Instr[31]}}, Instr[19:12], Instr[20], Instr[30:21], 1'b0};
            default: Imm_Ext = 32'b0; 
        endcase;
    end
endmodule



module single_cycle(clk, rst);
    input clk, rst;
    wire [31:0] PC, PCNext, RD_Instr, RD1, RD2, ImmExt, ALUResult, ReadData, PCTarget, PCPlus4, SrcB, Result;
    wire RegWrite, MemWrite, ALUSrc, Branch, Jump, PCSrc,zero;
    wire [1:0] ResultSrc, ImmSrc, AluOp;
    wire [2:0] AluControl;

    PC PC_inst(
        .PCNext(PCNext),
        .clk(clk),
        .rst(rst),
        .PC(PC)
    );

    PC_Adder PC_adder(
        .PC(PC),
        .PCPlus4(PCPlus4)
    );
    adder PC_Target_adder(
        .a(PC),
        .b(ImmExt),
        .y(PCTarget)
    );

    Mux_2x1 PCMUX(
        .I0(PCPlus4),
        .I1(PCTarget),
        .s(PCSrc),
        .out(PCNext)
    );

    Instruction_Memory Instruction_Memory(
        .rst(rst),
        .A(PC),
        .RD(RD_Instr)
    );

    register_file Register_File(
        .clk(clk),
        .rst(rst),
        .WE3(RegWrite),
        .WD3(Result),
        .A1(RD_Instr[19:15]),
        .A2(RD_Instr[24:20]),
        .A3(RD_Instr[11:7]),
        .RD1(RD1),
        .RD2(RD2)
    );

    Sign_Extend Sign_Extend(
        .Instr(RD_Instr),
        .ImmSrc(ImmSrc),
        .Imm_Ext(ImmExt)
    );


     Mux_2x1 MuxRegister_ALU(
        .I0(RD2),
        .I1(ImmExt),
        .s(ALUSrc),
        .out(SrcB)
    );

     ALU ALU(
            .aluControl(AluControl),
            .a(RD1),
            .b(SrcB),
            .result(ALUResult),
            .zero(zero)
    );

    Control_Unit ControlUnit(
        .Op(RD_Instr[6:0]),
        .RegWrite(RegWrite),
        .ImmSrc(ImmSrc),
        .ALUSrc(ALUSrc),
        .MemWrite(MemWrite),
        .ResultSrc(ResultSrc),
        .Branch(Branch),
        .Jump(Jump),
        .PCSrc(PCSrc),
        .funct3(RD_Instr[14:12]),
        .funct7(RD_Instr[31:25]),
        .zero(zero),
        .ALUControl(AluControl)
    );

     Data_Memory Data_Memory(
        .clk(clk),
        .rst(rst),
        .WE(MemWrite),
        .WD(RD2),
        .A(ALUResult),
        .RD(ReadData)
    );

    Mux_4x1 MUXDataMemory_Register(
        .I0(ALUResult),
        .I1(ReadData),
        .I2(PCPlus4),
        .s(ResultSrc),
        .out(Result)
    );

endmodule









