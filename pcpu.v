// Project #2: 16-bit pipeline processor
// Verilog source file for "System LSI design"
//                                                  2022.10  T. Ikenaga
// 

// Operation Code (please add operaions!)
`define NOP   5'b00000
`define HALT  5'b00001
`define LOAD  5'b00010
`define STORE 5'b00011
`define LDIH  5'b10000
`define ADD   5'b01000
`define ADDI  5'b01001
`define ADDC  5'b10001
`define SUB   5'b01010
`define SUBI  5'b01011
`define SUBC  5'b10010
`define CMP   5'b01100


`define AND   5'b01101
`define OR    5'b01110
`define XOR   5'b01111
`define SLL   5'b00100
`define SRL   5'b00101
`define SLA   5'b00110
`define SRA   5'b00111

`define JUMP  5'b11000
`define JMPR  5'b11001
`define BZ    5'b11010
`define BNZ   5'b11011
`define BN    5'b11100
`define BNN   5'b11101
`define BC    5'b11110
`define BNC		5'b11111

//added operation
`define MUL   5'b10011

// FSM for CPU controler
`define idle 1'b0
`define exec 1'b1

module pcpu (
  reset,
  clock,
  enable,
  start,
  i_addr,
  i_datain,
  d_addr,
  d_datain,
  d_dataout,
  d_we,
  select_y,
  y
);

input reset, clock, enable, start;
input [15:0] i_datain;  //指令
output [7:0] i_addr;  //指令地址
output [7:0] d_addr;  //store指令储存的地址
input [15:0] d_datain;  //load指令需要储存的数字
output [15:0] d_dataout;  //store指令写入数据寄存器的数字
output d_we;  //数据寄存器的使能

// for Debug
input [3:0] select_y;
output [15:0] y;

// Definition of F/F
reg [7:0] pc;
reg [15:0] id_ir, ex_ir, mem_ir, wb_ir;
reg [15:0] gr[0:7];
reg [15:0] reg_A, reg_B, reg_C, reg_C1;
reg [15:0] smdr, smdr1;
reg zf, nf, cf, dw;
reg state;
reg cf_temp;

// Definition of temporary variables
reg [15:0] ALUo;
reg [15:0] y;
reg next_state;

//parameters for mul operation
reg [15:0] mul_store [0:7];
reg [15:0] add_store [0:7];
reg negative_digital;


wire jump_flag;

assign i_addr    = pc;
assign d_we      = dw;  //store指令要用的
assign d_addr    = reg_C[7:0];  //jmp指令跳转地区
assign d_dataout = smdr1;  //store指令写入数据寄存器的数字
assign jump_flag = (((mem_ir[15:11] == `BZ) && (zf == 1'b1))||((mem_ir[15:11] == `BNZ) && (zf == 1'b0))||
                ((mem_ir[15:11] == `BN) && (nf == 1'b1))||((mem_ir[15:11] == `BNN) && (nf == 1'b0))||
                (mem_ir[15:11] == `JUMP)||(mem_ir[15:11] == `JMPR)||
                ((mem_ir[15:11] == `BC) && (cf == 1'b1))||((mem_ir[15:11] == `BNC) && (cf == 1'b0)));
//write_reg_enble
function reg_enable;
  input [4:0] op;
  begin
    reg_enable = ((op == `LOAD)|| (op == `LDIH)||(op == `MUL)||
                    (op == `ADD)|| (op == `ADDI)|| (op == `ADDC)||
                    (op == `SUB)|| (op == `SUBI)|| (op == `SUBC)||
                    (op == `AND)|| (op == `OR)|| (op == `XOR)||
                    (op == `SLL)|| (op == `SRL)|| (op == `SLA)|| (op == `SRA));
  end
endfunction
//RegA <- r1
function regA_r1;
  input [4:0] op;
  begin
    regA_r1 = ((op == `LDIH)||
              (op == `ADDI)|| (op == `SUBI)||
              (op == `JMPR)||
              (op == `BZ)|| (op == `BNZ)||
              (op == `BN)|| (op == `BNN)||
              (op == `BC)|| (op == `BNC));
  end
endfunction
//RegA <- r2
function regA_r2;
  input [4:0] op;
  begin
    regA_r2 = ((op == `LOAD)|| (op == `STORE)||
              (op == `ADD)|| (op == `ADDC)||
              (op == `SUB)|| (op == `SUBC)||
              (op == `CMP)||(op == `MUL)||
              (op == `AND)|| (op == `OR)|| (op == `XOR)||
              (op == `SLL)|| (op == `SRL)|| (op == `SLA)|| (op == `SRA));
  end
endfunction
//RegB<- r3
function regB_r3;
  input [4:0] op;
  begin
    regB_r3 = ((op == `ADD)||(op == `ADDC)|| (op == `MUL)||
              (op == `SUB)||(op == `SUBC)|| (op == `CMP)||
              (op == `AND)|| (op == `OR)|| (op == `XOR));
  end
endfunction
//RegB <- val3
function RegB_val3;
  input [4:0] op;
  begin
    RegB_val3 = ((op == `LOAD) || (op == `STORE)||
                (op == `SLL) || (op == `SRL) || 
                (op == `SLA) || (op == `SRA));
  end
endfunction
//RegB <- {val2,val3}
function RegB_val2_val3;
  input [4:0] op;
  begin
    RegB_val2_val3 = ((op == `BZ) || (op == `BNZ) || 
                      (op == `ADDI) || (op == `SUBI)||
                      (op == `JUMP)||(op == `JMPR)||
                      (op == `BN)||(op == `BNN)||
                      (op == `BC)||(op == `BNC));
  end
endfunction



// CPU Control (FSM)
always @(posedge clock or negedge reset) begin
  if (!reset) state <= `idle;
  else state <= next_state;
end

always @(state or enable or start or wb_ir[15:11]) begin
  case (state)
    `idle:
    if ((enable == 1'b1) && (start == 1'b1)) next_state <= `exec;
    else next_state <= `idle;
    `exec:
    if ((enable == 1'b0) || (wb_ir[15:11] == `HALT)) next_state <= `idle;
    else next_state <= `exec;
  endcase
end

// IF Block (1st Stage)
always @(posedge clock or negedge reset) begin
  if (!reset) begin
    id_ir <= 16'b0000000000000000;
    pc    <= 8'b00000000;
  end 
  else if (state == `exec) begin
    if (jump_flag)begin
          pc <= reg_C[7:0];
          id_ir <= 16'b0000000000000000;
    end
    //Data forwarding
    else if ((id_ir[15:11] == `LOAD)&&
            (((i_datain[10:8] == id_ir[10:8]) && regA_r1(i_datain[15:11]))||
            ((i_datain[6:4] == id_ir[10:8]) && regA_r2(i_datain[15:11]))||
            ((i_datain[2:0] == id_ir[10:8]) && regB_r3(i_datain[15:11]))))
            begin
              pc <= pc;
              id_ir <= 16'b0000000000000000;
    end
    else begin
      pc <= pc + 1;
      id_ir <= i_datain;
    end
       
  end
end

// ID Block (2nd Stage)
always @(posedge clock or negedge reset) begin
  if (!reset) begin
    ex_ir <= 16'b0000000000000000;
    reg_A <= 16'b0000000000000000;
    reg_B <= 16'b0000000000000000;
    smdr  <= 16'b0000000000000000;
    mul_store[0] <= 16'b0000000000000000;
    mul_store[1] <= 16'b0000000000000000;
    mul_store[2] <= 16'b0000000000000000;
    mul_store[3] <= 16'b0000000000000000;
    mul_store[4] <= 16'b0000000000000000;
    mul_store[5] <= 16'b0000000000000000;
    mul_store[6] <= 16'b0000000000000000;
    mul_store[7] <= 16'b0000000000000000;
    add_store[0] <= 16'b0000000000000000;
    add_store[1] <= 16'b0000000000000000;
    add_store[2] <= 16'b0000000000000000;
    add_store[3] <= 16'b0000000000000000;
    add_store[4] <= 16'b0000000000000000;
    add_store[5] <= 16'b0000000000000000;
    add_store[6] <= 16'b0000000000000000;
    add_store[7] <= 16'b0000000000000000;
    negative_digital <= 1'b0;
  end 
    // if (regA_r1(id_ir[15:11]))  //RegA <- R1
    //   reg_A <= gr[(id_ir[10:8])];
    // else if (regA_r2(id_ir[15:11])) reg_A <= gr[(id_ir[6:4])];
    // else if (id_ir[15:11] == `JUMP) reg_A <= 16'b0000_0000_0000_0000;     
    // else reg_A <= reg_A;
    // else if (regB_r3(id_ir[15:11])) reg_B <= gr[id_ir[2:0]];
    //   else if (RegB_val3(id_ir[15:11])) reg_B <= {12'b000000000000, id_ir[3:0]};
    //   else if (RegB_val2_val3(id_ir[15:11])) reg_B <= {8'b00000000, id_ir[7:0]};
    //   else if (id_ir[15:11]==`LDIH) reg_B <= {id_ir[7:0], 8'b00000000};
    //   else reg_B <= reg_B;
  else if (state == `exec) begin
    if(jump_flag)ex_ir<=16'b0000000000000000;
    else ex_ir <= id_ir;
    
    //Store
    if (id_ir[15:11] == `STORE) begin
      if(id_ir[10:8]==ex_ir[10:8]&&reg_enable(ex_ir[15:11]))begin
        smdr <= ALUo;
      end
      else if(id_ir[10:8]==mem_ir[10:8]&&reg_enable(mem_ir[15:11]))begin
        if(mem_ir[15:11]==`LOAD)smdr <= d_datain;
        else smdr <= reg_C;
      end
      else if((id_ir[10:8]==wb_ir[10:8])&&(reg_enable(wb_ir[15:11])))begin
        smdr <= reg_C1;
      end
      else smdr  <= gr[id_ir[10:8]];
    end 
    //regA_r1
    if(regA_r1(id_ir[15:11]))begin
      if((id_ir[10:8]==ex_ir[10:8])&&reg_enable(ex_ir[15:11]))reg_A<=ALUo;
      else if(id_ir[10:8]==mem_ir[10:8]&&reg_enable(mem_ir[15:11]))begin
        if(mem_ir[15:11]==`LOAD)begin
          reg_A <= d_datain;
        end
        else reg_A <= reg_C;
      end       
      else if((id_ir[10:8]==wb_ir[10:8])&&reg_enable(wb_ir[15:11]))reg_A <= reg_C1;
      else reg_A <= gr[id_ir[10:8]];
    end
    //regA_r2
    else if(regA_r2(id_ir[15:11]))begin
      if((id_ir[6:4]==ex_ir[10:8])&&reg_enable(ex_ir[15:11]))reg_A<=ALUo;
      else if(id_ir[6:4]==mem_ir[10:8]&&reg_enable(mem_ir[15:11]))begin
        if(mem_ir[15:11]==`LOAD)begin
          reg_A <= d_datain;
        end
        else reg_A <= reg_C;
      end
      else if((id_ir[6:4]==wb_ir[10:8])&&reg_enable(wb_ir[15:11]))reg_A <= reg_C1;
      else reg_A <= gr[(id_ir[6:4])];
    end
    //JUMP reg_A <= 16'b0;  
    else if (id_ir[15:11] == `JUMP) reg_A <= 16'b0000000000000000;  
    //regB_r3
    if(regB_r3(id_ir[15:11]))begin
      if((id_ir[2:0]==ex_ir[10:8])&&reg_enable(ex_ir[15:11]))reg_B<=ALUo;
      
      else if(id_ir[2:0]==mem_ir[10:8]&&reg_enable(mem_ir[15:11]))begin
        if(mem_ir[15:11]==`LOAD)begin
          reg_B <= d_datain;
        end
        else reg_B <= reg_C;
      end
      else if((id_ir[2:0])==wb_ir[10:8]&&reg_enable(wb_ir[15:11]))reg_B<=reg_C1;
      else reg_B <= gr[id_ir[2:0]];
    end
    else if (RegB_val3(id_ir[15:11])) reg_B <= {12'b000000000000, id_ir[3:0]};
    else if (RegB_val2_val3(id_ir[15:11])) reg_B <= {8'b00000000, id_ir[7:0]};
    else if (id_ir[15:11]==`LDIH) reg_B <= {id_ir[7:0], 8'b00000000};
  end
end

// EX Block (3rd Stege)
always @(posedge clock or negedge reset) begin
  if (!reset) begin
    mem_ir <= 16'b0000000000000000;
    reg_C <= 16'b0000000000000000;
    smdr1 <= 16'b0000000000000000;
    zf <= 1'b0;
    nf <= 1'b0;
    cf <= 1'b0;
    dw <= 1'b0;
  end 
  else if (state == `exec) begin
    reg_C  <= ALUo;
    if(jump_flag)mem_ir<=16'b000000000000;      
    else mem_ir<=ex_ir;
    // change th flag
    if (!jump_flag&&((ex_ir[15:11] == `CMP) ||(ex_ir[15:11] == `ADD) || (ex_ir[15:11] == `ADDI)||(ex_ir[15:11] == `ADDC) ||
        (ex_ir[15:11] == `SUB)||(ex_ir[15:11] == `SUBI)||(ex_ir[15:11] == `SUBC)||(ex_ir[15:11]==`MUL)||
        (ex_ir[15:11] == `AND)||(ex_ir[15:11] == `OR)||(ex_ir[15:11] == `XOR)||(ex_ir[15:11] == `LDIH)||
        (ex_ir[15:11] == `SLL)||(ex_ir[15:11] == `SRL)||(ex_ir[15:11] == `SLA)||(ex_ir[15:11] == `SRA))) begin
      if (ALUo == 16'b0000000000000000) zf <= 1'b1;
      else zf <= 1'b0;
      if (ALUo[15] == 1'b1) nf <= 1'b1;
      else nf <= 1'b0;

      if((ex_ir[15:11] == `CMP) ||(ex_ir[15:11] == `ADD) || (ex_ir[15:11] == `ADDI)||(ex_ir[15:11] == `ADDC) || 
         (ex_ir[15:11] == `SUB)||(ex_ir[15:11] == `SUBI)||(ex_ir[15:11] == `SUBC)||(ex_ir[15:11]==`MUL)) cf <= cf_temp;
      else if((ex_ir[15:11] == `AND)||(ex_ir[15:11] == `OR)||(ex_ir[15:11] == `XOR))cf <= 0;
      else if((ex_ir[15:11] == `SLL)||(ex_ir[15:11] == `SRL)||(ex_ir[15:11] == `SLA)||(ex_ir[15:11] == `SRA))cf <= cf;
      else cf <= cf;
    end

    if (ex_ir[15:11] == `STORE) begin
      dw <= 1'b1;
      smdr1 <= smdr;
    end 
    else dw <= 1'b0;
  end
end

// MEM Block (4th Stage)
always @(posedge clock or negedge reset) begin
  if (!reset) begin
    wb_ir  <= 16'b0000000000000000;
    reg_C1 <= 16'b0000000000000000;
  end 
  else if (state == `exec) begin
    wb_ir <= mem_ir;
    reg_C1 <= (mem_ir[15:11]==`LOAD)? d_datain : reg_C;
  end
end

// WB Block (5th Stage)
always @(posedge clock or negedge reset) begin
  if (!reset) begin
    gr[0] <= 16'b0000000000000000;
    gr[1] <= 16'b0000000000000000;
    gr[2] <= 16'b0000000000000000;
    gr[3] <= 16'b0000000000000000;
    gr[4] <= 16'b0000000000000000;
    gr[5] <= 16'b0000000000000000;
    gr[6] <= 16'b0000000000000000;
    gr[7] <= 16'b0000000000000000;
  end 
  else if (state == `exec) begin
    if (wb_ir[10:8] != 3'b000) begin
      if (reg_enable(wb_ir[15:11])) 
        gr[wb_ir[10:8]] <= reg_C1;
    end
  end
end

function [15:0]mul_result;
  input [15:0] reg_A;
  input [15:0] reg_B;
  begin
    
  end
endfunction
// ALU module
always @(reg_A or reg_B or ex_ir[15:11])begin
  case (ex_ir[15:11])
    `LOAD:  ALUo = reg_A + reg_B;
    `STORE: ALUo = reg_A + reg_B;
  //todo
    `LDIH: {cf_temp,ALUo[15:0]} = reg_A + reg_B;
    `ADD:  {cf_temp,ALUo[15:0]} = reg_A + reg_B;
    `ADDI: {cf_temp,ALUo[15:0]} = reg_A + reg_B;
    `ADDC: {cf_temp,ALUo[15:0]} = reg_A + reg_B + cf;
    `SUB:  {cf_temp,ALUo[15:0]} = reg_A - reg_B;
    `SUBI: {cf_temp,ALUo[15:0]} = reg_A - reg_B;
    `SUBC: {cf_temp,ALUo[15:0]} = reg_A - reg_B - cf;
    `CMP:  {cf_temp,ALUo[15:0]} = reg_A - reg_B;
    `MUL: begin
      if(reg_A[15]==1)begin 
        reg_A = ~reg_A + 1;
        negative_digital = negative_digital + 1;
      end
      if(reg_B[15]==1)begin
        reg_B = ~reg_B + 1;
        negative_digital = negative_digital + 1;
      end
      mul_store[0] = reg_B[0]? {8'b00000000, reg_A[7:0]     } : 16'b0000000000000000;
      mul_store[1] = reg_B[1]? {7'b0000000, reg_A[7:0], 1'b0} : 16'b0000000000000000;
      mul_store[2] = reg_B[2]? {6'b000000, reg_A[7:0], 2'b00} : 16'b0000000000000000;
      mul_store[3] = reg_B[3]? {5'b00000, reg_A[7:0], 3'b000} : 16'b0000000000000000;
      mul_store[4] = reg_B[4]? {4'b0000, reg_A[7:0], 4'b0000} : 16'b0000000000000000;
      mul_store[5] = reg_B[5]? {3'b000, reg_A[7:0], 5'b00000} : 16'b0000000000000000;
      mul_store[6] = reg_B[6]? {2'b00, reg_A[7:0], 6'b000000} : 16'b0000000000000000;
      mul_store[7] = reg_B[7]? {1'b0, reg_A[7:0], 7'b0000000} : 16'b0000000000000000;
      add_store[0] = mul_store[0] + mul_store[1];
      add_store[1] = mul_store[2] + mul_store[3];
      add_store[2] = mul_store[4] + mul_store[5];
      add_store[3] = mul_store[6] + mul_store[7];

      add_store[4] = add_store[0] + add_store[1];
      add_store[5] = add_store[2] + add_store[3];

      add_store[6] = add_store[4] + add_store[5];
      if(negative_digital == 0 || negative_digital == 2){cf_temp,ALUo[15:0]} = add_store[6];
      else {cf_temp,ALUo[15:0]} = ~add_store[6] + 1;
    end
    `AND:   ALUo = reg_A & reg_B;
    `OR:    ALUo = reg_A | reg_B;
    `XOR:   ALUo = reg_A ^ reg_B;
    `SLL:   ALUo = reg_A << reg_B;
    `SRL:   ALUo = reg_A >> reg_B;
    `SLA: begin
      ALUo[15]   = reg_A[15];
      ALUo[14:0] = reg_A[14:0] << reg_B;
    end
    `SRA:   ALUo = ({{15{reg_A[15]}}, 1'b0} << (~reg_B)) | (reg_A >> reg_B);
    `SRA:   ALUo = 0;
    `JUMP:  ALUo = reg_A + reg_B;
    `JUMP:  ALUo = reg_A + reg_B;
    `BZ:    ALUo = reg_A + reg_B;
    `BNZ:   ALUo = reg_A + reg_B;
    `BN:   ALUo = reg_A + reg_B;
    `BNZ:    ALUo = reg_A + reg_B;
    `BC:    ALUo = reg_A + reg_B;
    `BNC:    ALUo = reg_A + reg_B;
    default: ALUo = reg_A + reg_B;//16'bXXXXXXXXXXXXXXXX;
  endcase
end

// Debugging
always @(select_y or gr[1] or gr[2] or gr[3] or gr[4] or gr[5] or gr[6]
       or gr[7] or reg_A or reg_B or reg_C or reg_C1 or smdr or id_ir
       or dw or zf or nf or cf or pc)
 begin
  case (select_y)
    4'b0000: y = {3'b000, dw, 1'b0, zf, nf, cf, pc};
    4'b0001: y = gr[1];
    4'b0010: y = gr[2];
    4'b0011: y = gr[3];
    4'b0100: y = gr[4];
    4'b0101: y = gr[5];
    4'b0110: y = gr[6];
    4'b0111: y = gr[7];
    4'b1000: y = reg_A;
    4'b1001: y = reg_B;
    4'b1011: y = reg_C;
    4'b1100: y = reg_C1;
    4'b1101: y = smdr;
    4'b1110: y = id_ir;
    default: y = 16'bXXXXXXXXXXXXXXXX;
   endcase
  end
endmodule