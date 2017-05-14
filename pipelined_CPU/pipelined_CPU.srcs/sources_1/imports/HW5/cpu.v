`timescale 1ns/1ns
`define WORD_SIZE 16    // data and address word size

`include "opcodes.v"

module cpu(
        input Clk, 
        input Reset_N, 

	// Instruction memory interface
        output i_readM, 
        output i_writeM, 
        output [`WORD_SIZE-1:0] i_address, 
        inout [`WORD_SIZE-1:0] i_data, 

	// Data memory interface
        output d_readM, 
        output d_writeM, 
        output [`WORD_SIZE-1:0] d_address, 
        inout [`WORD_SIZE-1:0] d_data, 

        output [`WORD_SIZE-1:0] num_inst, 
        output [`WORD_SIZE-1:0] output_port, 
        output is_halted,
        
        //debugging

        output[15:0] pc_out,
        output[15:0] pc_next,
        output[15:0] pc_in,
        
       output[15:0] ID_pc,
       output[15:0] ID_pc_pred,
       output[15:0] EX_pc,
       output[15:0] EX_pc_pred,
       
       output[15:0] jumpAddr,
       output[15:0] branchAddr,
       
       
       output jumpPred,
       output branchPred,
       output branchMissPred,
       
       output EX_Branch,
       output b_cond,
       output[1:0] PCsrc,
       
       output[7:0] BHSR,
       output valid,
       output[7:0] tag,
       output[7:0] pc_tag
        
//        //for BTB test
//        output valid,
//        output[7:0] tag,
//        output[15:0] addr
          
       
);

    reg[`WORD_SIZE-1:0] num_inst;
    wire[`WORD_SIZE-1:0] output_port;
    wire[`WORD_SIZE-1:0] to_output_port;
    initial begin num_inst<=0; end
    
	// TODO : Implement your multi-cycle CPU!

    wire [15:0] inst;
    wire ALUsrc;
    wire[1:0] RegDist;
    wire MemWrite;
    wire MemRead;
    wire MemtoReg;
    wire RegWrite;
    wire[3:0] Alucode;
    wire Jump;
    wire JumpR;
    wire Branch;
    
    
    wire[15:0] out1;
    wire[15:0] out2;
    wire[15:0] out3;
    wire[15:0] out4;   
   
    //num_inst
    wire ID_isFetched;
    wire EX_isFetched;
    wire ID_WWD;
    wire EX_WWD;
    
    //debugging

    wire[15:0] pc_out;
    wire[15:0] pc_next;
    wire[15:0] pc_in;
    
            
    wire[15:0] ID_pc;
    wire[15:0] ID_pc_pred;
    wire[15:0] EX_pc;
    wire[15:0] EX_pc_pred;
   
    wire[15:0] jumpAddr;
    wire[15:0] branchAddr;
    

    wire jumpPred;
    wire branchPred;
    wire branchMissPred;
    
    wire EX_Branch;
    wire b_cond;
    wire[1:0] PCsrc;
    
    wire[7:0] BHSR;
    wire valid;
    wire[7:0] tag;
    wire[7:0] pc_tag;
    
    //WWD, num_inst logic
    
    
    latch1 isFetched(1, 0, Clk, ID_isFetched && !IDEX_Flush, EX_isFetched); 
    latch1 WWD(1, 0, Clk, ID_WWD, EX_WWD); 
    
    assign output_port = EX_WWD ? to_output_port : 15'bz;
    
    always @(negedge Clk) begin
        if(EX_isFetched) begin 
            num_inst<=num_inst+1; 
        end
    end

    //stalling and flushing
    wire isStalled;
    
//    //for BTB testing
//    wire valid;
//    wire[7:0] tag;
//    wire[15:0] addr;
    
    datapath DM(
        .clk(Clk),
        .reset_n(Reset_N),
        
        .i_readM(i_readM),
        .i_writeM(i_writeM),
        .i_address(i_address),
        .i_data(i_data),
        
        .d_readM(d_readM),
        .d_writeM(d_writeM),
        .d_address(d_address),
        .d_data(d_data),
        
        .to_output_port(to_output_port),
        .is_halted(is_halted),
        
        .inst(inst),
        .ALUsrc(ALUsrc),
        .RegDist(RegDist),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .RegWrite(RegWrite),
        .Alucode(Alucode),
        .Jump(Jump),
        .JumpR(JumpR),
        .Branch(Branch),
        .Halt(Halt),
        
        .isStalled(isStalled),
               
        .IDEX_Flush(IDEX_Flush),
        
        //debugging
        
        .pc_out(pc_out),
        .pc_next(pc_next),
        .pc_in(pc_in),
        
        .ID_pc(ID_pc),
        .ID_pc_pred(ID_pc_pred),
        .EX_pc(EX_pc),
        .EX_pc_pred(EX_pc_pred),
        
        .jumpAddr(jumpAddr),
        .branchAddr(branchAddr),
        
        
        .jumpPred(jumpPred),
        .branchPred(branchPred),
        .branchMissPred(branchMissPred),
        .b_cond(b_cond),
        .EX_Branch(EX_Branch),
        .PCsrc(PCsrc)
//        .BHSR(BHSR),
//        .valid(valid),
//        .tag(tag),
//        .pc_tag(pc_tag)
        
//        //for BTB testing
//        .valid(valid),
//        .tag(tag),
//        .addr(addr)
        
    );
    
    control CTRL(
        .clk(Clk),
        .inst(inst),
        .ALUsrc(ALUsrc),
        .RegDist(RegDist),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .RegWrite(RegWrite),
        .Alucode(Alucode),
        .Jump(Jump),
        .JumpR(JumpR),
        .Branch(Branch),
        .Halt(Halt),
        
        .isFetched(ID_isFetched),
        .WWD(ID_WWD),
        
        .isStalled(isStalled)
    );
    
endmodule


module brenchPred(
    //read input
    input clk,
    input[15:0] pc,
    input[15:0] pc_next, //pc+1
    
    //write input
    input update,
    input[15:0] pc_update,
    input[15:0] target_addr,
    
    //BHSR update
    input bhsr_update,
    input bhsr_input,
    
    output[15:0] pc_pred
    
//    //debugging
//    output[7:0] BHSR,
//    output valid,
//    output[7:0] tag,
//    output[7:0] pc_tag
    
);
    
    integer i;
    
    reg[15:0] pc_pred;
    initial pc_pred<=0;
    
    wire valid;
    wire[7:0] tag;
    wire[15:0] addr;
    
//    //bhsr
//    reg[7:0] BHSR;
    
//    initial begin
//        for(i=0; i<8; i=i+1) begin
//            BHSR[i] <= 0;
//        end
//    end
    
//    always @(negedge clk) begin
    
//        if(bhsr_update) begin
//            for(i=8; i>=1; i=i-1) BHSR[i] = BHSR[i-1];
//            BHSR[0] = bhsr_input;
            
//        end
    
//    end
    
    
    btb BTB(
    
        //read input
        .clk(clk),
        .pc(pc),
//        .BHSR(BHSR),
        
        //write input
        .update(update),
        .pc_update(pc_update),
        .target_addr(target_addr),
        
        //read output
        .valid(valid),
        .tag(tag),
        .addr(addr)
    );
    
    wire[7:0] pc_tag;
    assign pc_tag = pc[15:8];
    
    //hit : pc<=addr from BTB, miss : pc<=pc+1(pc_next)
    always@(pc or pc_next or valid or tag or addr) begin
        if(valid && pc_tag==tag) pc_pred <= addr;
        else pc_pred <= pc_next;
    end

endmodule
    

module btb(

    //read input
    input clk,
    input[15:0] pc,
//    input[7:0] BHSR,
    
    //write input
    input update,
    input[15:0] pc_update,
    input[15:0] target_addr,
    
    //read output
    output valid,
    output[7:0] tag,
    output[15:0] addr
);

    
    integer i;
    
//    //debugging
//    reg[15:0] debug[15:0];
//    initial begin
//        for(i=0; i<16; i=i+1) begin
//            debug[i]=i;
        
//        end
//    end
    
    //outputs
    
    reg valid;
    reg[7:0] tag;
    reg[15:0] addr;
    
    initial begin
        valid <= 0;
        tag <= 0;
        addr <= 0;
    end
    
    
    //line initiation
    
    reg updates[255:0];
    initial begin
        for(i=0; i<256; i=i+1) updates[i] <= 0;
    end
    
    wire valids[255:0];
    wire[7:0] tags[255:0];
    wire[15:0] addrs[255:0];
    
    generate
        genvar k;
        for(k=0; k<255; k=k+1) begin : dff
            btbLine lines(clk, update && updates[k], pc_update, target_addr, valids[k], tags[k], addrs[k]);
            
        end
    endgenerate
    
    
    wire[7:0] pc_idx;
    wire[7:0] pc_update_idx;
    
    assign pc_idx = pc[7:0]; // XOR
    assign pc_update_idx = pc_update[7:0];
    
    always @(pc) begin
        valid = valids[pc_idx];
        tag = tags[pc_idx];
        addr = addrs[pc_idx];
        
    end
    
    always @(pc_update_idx) begin
        
        for(i=0; i<256; i=i+1) begin
            updates[i] <= 0;
        end
        
        updates[pc_update_idx] <= 1;
        
    end
    

endmodule



module btbLine(
    input clk,      
    input update,
    input[15:0] pc_update,
    input[15:0] target_addr,

    output valid,
    output[7:0] tag,
    output[15:0] addr
    
);

    reg valid;
    reg[7:0] tag;
    reg[15:0] addr;

    initial begin
    
        valid<=0;
        tag<=0;
        addr<=0;
    
    end
    
//    always@(debug) addr<=debug;
    
    //update at every clk
    
    always @(posedge clk) begin
        
        if(update) begin
            valid <= 1;
            tag <= pc_update[15:8];
            addr <= target_addr;
        end
        
    end

endmodule
    
