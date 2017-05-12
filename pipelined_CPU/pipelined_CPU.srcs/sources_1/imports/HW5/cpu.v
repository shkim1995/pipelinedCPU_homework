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
        
        
        ///for debugiging
        output[15:0] pc_out,
        output[15:0] IFID_inst_out,
        output[15:0] ALU_in1,
        output[15:0] ALU_in2,
        output[15:0] ALU_out,
        output[15:0] dmem_rdata,
        output[15:0] RF_wData,
        output[15:0] MEMWB_rdata_in,
        output[15:0] MEMWB_rdata_out
);

    //for debugging
    wire[15:0] pc_out;
    wire[15:0] IFID_inst_out;
    wire[15:0] ALU_in1;
    wire[15:0] ALU_in2;
    wire[15:0] ALU_out;
    wire[15:0] dmem_rdata;
    wire[15:0] RF_wData;
    wire[15:0] MEMWB_rdata_in;
    wire[15:0] MEMWB_rdata_out;

    reg[`WORD_SIZE-1:0] num_inst;
    wire[`WORD_SIZE-1:0] output_port;
    wire[`WORD_SIZE-1:0] to_output_port;
    initial begin num_inst<=0; end
    
	// TODO : Implement your multi-cycle CPU!
    
    wire [15:0] inst;
    wire ALUsrc;
    wire RegDist;
    wire MemWrite;
    wire MemRead;
    wire MemtoReg;
    wire RegWrite;
    wire[3:0] Alucode;
    wire Jump;
    wire Branch;
    
    //num_inst
    wire ID_isFetched;
    wire EX_isFetched;
    wire ID_WWD;
    wire EX_WWD;
    
    //WWD, num_inst logic
    
    initial num_inst <= 1;
    
    latch1 isFetched(1, 0, Clk, ID_isFetched, EX_isFetched); 
    latch1 WWD(1, 0, Clk, ID_WWD, EX_WWD); 
    
    assign output_port = EX_WWD ? to_output_port : 15'bz;
    
    always @(negedge Clk) begin
        if(EX_isFetched) begin 
            num_inst<=num_inst+1; 
            //$display("NUM : %d, output : %d", num_inst, output_port); 
        end
    end

    //stalling and flushing
    wire isStalled;
    
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
        
        //.num_inst(num_inst),
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
        .Branch(Branch),
        
        .isStalled(isStalled),
        
        //for debugging
        .pc_out(pc_out),
        .IFID_inst_out(IFID_inst_out),
        .ALU_in1(ALU_in1),
        .ALU_in2(ALU_in2),
        .ALU_out(ALU_out),
        .dmem_rdata(dmem_rdata),
        .RF_wData(RF_wData),
        
        .MEMWB_rdata_in(MEMWB_rdata_in),
        .MEMWB_rdata_out(MEMWB_rdata_out)
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
        .Branc(Branch),
        
        .isFetched(ID_isFetched),
        .WWD(ID_WWD),
        
        .isStalled(isStalled)
    );
    
endmodule


////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////CONTROL MODULE/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


module control(
    input clk,

    input[15:0] inst,
    output ALUsrc,
    output RegDist,
    output MemWrite,
    output MemRead,
    output MemtoReg,
    output RegWrite,
    output[3:0] Alucode,
    output Jump,
    output Branch,
        
    //for num_inst
    output isFetched,
    output WWD,
    
    
    input isStalled
);

    reg ALUsrc;
    reg RegDist;
    reg MemWrite;
    reg MemRead;
    reg MemtoReg;
    reg RegWrite;
    reg[3:0] Alucode;
    reg Jump;
    reg Branch;
    
    
    reg isFetched;
    reg WWD;
    
    //always @(WWD) $display("controller : %b", WWD);

    wire[3:0] opcode;
    wire[5:0] ftncode;

    assign opcode = inst[15:12];
    assign ftncode = inst[5:0];

    initial begin
        ALUsrc <= 0;
        RegDist <= 0;
        MemWrite <= 0;
        MemRead <= 0;
        MemtoReg <= 0;
        RegWrite <= 0;
        Alucode <= 0;
        Jump<=0;
        Branch<=0;
        
        WWD <= 0;
        isFetched <= 0;
    end

    always @(isStalled) begin
        if(isStalled) begin
            $display("STALLED!!");
            ALUsrc <= 0;
            RegDist <= 0;
            MemWrite <= 0;
            MemRead <= 0;
            MemtoReg <= 0;
            RegWrite <= 0;
            Alucode <= 0;
            Jump<=0;
            Branch<=0;
            
            WWD <= 0;
            isFetched <= 0;
        end
        
    end
    
    always @(inst or isStalled) begin
        $display("inst at ctrl module : %h", inst);
        
        if(!isStalled) begin
        
        //bubble
        if(inst==0) begin
            $display("BUBBLE INSTUCTION!!");
            ALUsrc <= 0;
            RegDist <= 0;
            MemWrite <= 0;
            MemRead <= 0;
            MemtoReg <= 0;
            RegWrite <= 0;
            Alucode <= 0;
            Jump<=0;
            Branch<=0;
            
            WWD <= 0;
            isFetched <= 0;
        end
      
        
        else if(opcode==4 || opcode==5 || opcode==6) begin
//            $display("ctrl : ALU I-type");
            ALUsrc <= 1;
            RegDist <= 0;
            MemWrite <= 0;
            MemRead <= 0;
            MemtoReg <= 0;
            RegWrite <= 1;
            Jump<=0;
            Branch<=0;
            
            WWD <= 0;
            isFetched <= 1;
            
            //ADI
            if(opcode==4) begin
                $display("ctrl : ADI");
                Alucode <= 4'b0000;
            end
            
            //ORI
            if(opcode==5) begin
                $display("ctrl : ORI");
                Alucode <= 4'b1000;
            end
            
            //LHI
            if(opcode==6) begin
                $display("ctrl : LHI");
                Alucode <= 4'b0010;
            end
            
        end
        
        else if(opcode==15 && ftncode!=28 && ftncode!=25 && ftncode!=26 && ftncode!=29) begin
           // $display("ctrl : ALU R-type");
            ALUsrc <= 0;
            RegDist <= 1;
            MemWrite <= 0;
            MemRead <= 0;
            MemtoReg <= 0;
            RegWrite <= 1;
            Jump<=0;
            Branch<=0;
            
            WWD <= 0;
            isFetched <= 1;
            
            //ADD
            if(ftncode==0) begin
                $display("ctrl : ADD");
                Alucode <= 4'b0000;
            end
            
            //SUB
            if(ftncode==1) begin
                $display("ctrl : SUB");
                Alucode <= 4'b0001;
            end
            
            //AND
            if(ftncode==2) begin
                $display("ctrl : AND");
                Alucode <= 4'b0111;
            end
            
            //ORR
            if(ftncode==3) begin
                $display("ctrl : ORR");
                Alucode <= 4'b1000;
            end
            
            //NOT
            if(ftncode==4) begin
                $display("ctrl : NOT");
                Alucode <= 4'b0110;
            end
            
            //TCP
            if(ftncode==5) begin
                $display("ctrl : TCP");
                Alucode <= 4'b1001;
            end
            
            //SHL
            if(ftncode==6) begin
                $display("ctrl : SHL");
                Alucode <= 4'b1101;
            end
            
            //SHR
            if(ftncode==7) begin
                $display("ctrl : SHR");
                Alucode <= 4'b1010;
            end
            
            
            
        end
        
        //WWD
        else if(opcode==15 && ftncode==28) begin
            $display("ctrl : WWD");
            MemWrite <= 0;
            MemRead <= 0;
            RegWrite <= 0;
            Jump<=0;
            Branch<=0;
            
            WWD <= 1;
            isFetched <= 1;
            
        end
        
        //LWD
        else if(opcode==7) begin
        
            $display("ctrl : LWD");
            ALUsrc <= 1;
            RegDist <= 0;
            MemWrite <= 0;
            MemRead <= 1;
            MemtoReg <= 1;
            RegWrite <= 1;
            Alucode <= 4'b0000;
            Jump<=0;
            Branch<=0;
            
            WWD <= 1;
            isFetched <= 1;
        end
        
        //SWD
        else if(opcode==8) begin
        
            $display("ctrl : SWD");
            ALUsrc <= 1;
            MemWrite <= 1;
            MemRead <= 0;
            RegWrite <= 0;
            Alucode <= 4'b0000;
            Jump<=0;
            Branch<=0;
            
            WWD <= 1;
            isFetched <= 1;
        end
        
        //JMP
        else if(opcode==9) begin
            
            $display("ctrl : JMP");
            MemWrite <= 0;
            MemRead <= 0;
            RegWrite <= 0;
            Jump<=1;
            Branch<=0;
            
            WWD <= 1;
            isFetched <= 1;
        end
           
        else begin
            $display("ctrl : nothing!");
           
            MemWrite <= 0;
            MemRead <= 0;
            RegWrite <= 0;
            Jump<=0;
            Branch<=0;
            
            WWD <= 0;
            isFetched <= 0;
            
        end
        
    end
    
    end

    initial begin
        $display("inst at ctrl module : %h", inst);
    end

    


endmodule



//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////DATAPATH MODULE///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


module datapath(
    input clk,
    input reset_n ,
    
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

    //output [`WORD_SIZE-1:0] num_inst, 
    output [`WORD_SIZE-1:0] to_output_port,
    output is_halted,
    
    //for control unit
    output[15:0] inst,
    input ALUsrc,
    input RegDist,
    input MemWrite,
    input MemRead,
    input MemtoReg,
    input RegWrite,
    input[3:0] Alucode,
    input Jump,
    input Branch,
    
    output isStalled,
    
    
    ///for debugiging
            
    output[15:0] pc_out,
    output[15:0] IFID_inst_out,
    output[15:0] ALU_in1,
    output[15:0] ALU_in2,
    output[15:0] ALU_out,
    output[15:0] dmem_rdata,
    output[15:0] RF_wData,
    output[15:0] MEMWB_rdata_in,
    output[15:0] MEMWB_rdata_out
             
    
    
);

    wire[15:0] inst;
    wire[15:0] to_output_port;
    
    
    //stalling & flushing
    wire isStalled;
    wire IFID_Flush;
    wire IDEX_Flush;
    
    
    
    wire PC_enable;
    wire IFID_enable;
    
    assign PC_enable = !isStalled;
    assign IFID_enable = !isStalled;

////////////////////////////////data wires////////////////////////////////////
    
    
    //IF stage
    wire[15:0] pc_in;
    wire[15:0] pc_out;
    
    wire[15:0] imem_in;
    wire[15:0] imem_out;
    
    wire[15:0] IFID_inst_in;
    
    
    //ID stage
    wire[15:0] IFID_inst_out;
    
    wire[1:0] RF_addr1;
    wire[1:0] RF_addr2;
    wire[1:0] RF_addrw;
    wire[15:0] RF_wData;
    wire[15:0] RF_val1;
    wire[15:0] RF_val2;
    
    wire[7:0] SE_in;
    wire[15:0] SE_out;
    
    wire[15:0] IDEX_val1_in;
    wire[15:0] IDEX_val2_in;
    wire[15:0] IDEX_SE_in;
    wire[1:0] IDEX_rt_in;
    wire[1:0] IDEX_rd_in;
    wire[1:0] IDEX_rs_in;
    
    
    //EX stage
    wire[15:0] IDEX_val1_out;
    wire[15:0] IDEX_val2_out;
    wire[15:0] IDEX_SE_out;
    wire[1:0] IDEX_rt_out;
    wire[1:0] IDEX_rd_out;
    wire[1:0] IDEX_rs_out;
    
    wire[15:0] ALU_in1;
    wire[15:0] ALU_in2;
    wire[15:0] ALU_out;
    wire[15:0] RF_dist;
    
    //target address should be added//
    
    wire[15:0] EXMEM_ALU_in;
    wire[15:0] EXMEM_dmem_in;
    wire[15:0] EXMEM_RF_dist_in;
    
    
    //MEM stage
    wire[15:0] EXMEM_ALU_out;
    wire[15:0] EXMEM_dmem_out;
    wire[15:0] EXMEM_RF_dist_out;
    
    wire[15:0] dmem_addr;
    wire[15:0] dmem_wdata;
    wire[15:0] dmem_rdata;
    
    wire[15:0] MEMWB_rdata_in;
    wire[15:0] MEMWB_ALU_in;
    wire[15:0] MEMWB_RF_dist_in;
    
    //WB stage
    wire[15:0] MEMWB_rdata_out;
    wire[15:0] MEMWB_ALU_out;
    wire[15:0] MEMWB_RF_dist_out;
    
    
    
//////////////////////////////control signals/////////////////////////////
    
    wire[1:0] PCsrc;
    
    wire ID_ALUsrc;
    wire ID_RegDist;
    wire ID_MemWrite;
    wire ID_MemRead;
    wire ID_MemtoReg;
    wire ID_RegWrite;
    wire[3:0] ID_Alucode;
    wire ID_Jump;
    wire ID_Branch;
    
    wire EX_ALUsrc;
    wire EX_RegDist;
    wire EX_MemWrite;
    wire EX_MemRead;
    wire EX_MemtoReg;
    wire EX_RegWrite;
    wire[3:0] EX_Alucode;
    wire EX_Branch;
    
    wire[1:0] ALUsrc1;
    wire[1:0] ALUsrc2;
        
    wire MEM_MemWrite;
    wire MEM_MemRead;
    wire MEM_MemtoReg;
    wire MEM_RegWrite;
        
    wire WB_MemtoReg;
    wire WB_RegWrite;
    
    
    
////////////////////////////wire connection///////////////////////////////

//IF stage

assign imem_in = pc_out;
assign IFID_inst_in = imem_out;

//ID stage

//control signals
assign ID_ALUsrc = ALUsrc;
assign ID_RegDist = RegDist;
assign ID_MemWrite = MemWrite;
assign ID_MemRead = MemRead;
assign ID_MemtoReg = MemtoReg;
assign ID_RegWrite = RegWrite;
assign ID_Alucode = Alucode;
assign ID_Jump = Jump;
assign ID_Branch = Branch;

assign RF_addr1 = IFID_inst_out[11:10];
assign RF_addr2 = IFID_inst_out[9:8];
assign SE_in = IFID_inst_out[7:0];
assign inst = IFID_inst_out;

assign IDEX_val1_in = RF_val1;
assign IDEX_val2_in = RF_val2;
assign IDEX_rd_in = IFID_inst_out[7:6];
assign IDEX_rt_in = IFID_inst_out[9:8];
assign IDEX_rs_in = IFID_inst_out[11:10];
assign IDEX_SE_in = SE_out;

//EX stage

assign ALU_in1 = (ALUsrc1 == 2'b00) ? IDEX_val1_out :
                 (ALUsrc1 == 2'b10) ? EXMEM_ALU_out :
                 RF_wData;
wire[15:0] temp;
assign temp = (ALUsrc2 == 2'b00) ? IDEX_val2_out :
                 (ALUsrc2 == 2'b10) ? EXMEM_ALU_out :
                 RF_wData;
assign ALU_in2 = EX_ALUsrc ? IDEX_SE_out : temp;
assign RF_dist = EX_RegDist ? IDEX_rd_out : IDEX_rt_out;
assign to_output_port = ALU_in1;

assign EXMEM_ALU_in = ALU_out;
assign EXMEM_dmem_in = IDEX_val2_out;
assign EXMEM_RF_dist_in = RF_dist;

//always @(EXMEM_RF_dist_out) $display("EXMEM_RF_dist_out: %d", EXMEM_RF_dist_out);

//MEM stage

assign dmem_addr = EXMEM_ALU_out;
assign dmem_wdata = EXMEM_dmem_out;

assign MEMWB_rdata_in = dmem_rdata;
assign MEMWB_ALU_in = EXMEM_ALU_out;
assign MEMWB_RF_dist_in = EXMEM_RF_dist_out;

//WB stage

assign RF_wData = WB_MemtoReg ? MEMWB_rdata_out : MEMWB_ALU_out;
assign RF_addrw = MEMWB_RF_dist_out;



/////////////////////////////////module init////////////////////////////////////


//ALU
ALU alu( .A(ALU_in1), .B(ALU_in2), .Cin(0), .OP(EX_Alucode), .C(ALU_out));


//IMEM

reg i_readM;
reg i_writeM;
wire[`WORD_SIZE-1:0] i_address;
wire[`WORD_SIZE-1:0] i_data;

assign imem_out = i_readM ? i_data : `WORD_SIZE'bz;
assign i_data = `WORD_SIZE'bz;
assign i_address = pc_out;

initial begin 
    i_readM<=1; 
end

//DMEM

wire d_readM;
wire d_writeM;
wire [`WORD_SIZE-1:0] d_address; 
wire [`WORD_SIZE-1:0] d_data;

assign dmem_rdata = d_readM ? d_data : `WORD_SIZE'bz;
assign d_data = d_writeM ? dmem_wdata : `WORD_SIZE'bz;
assign d_address = dmem_addr;
assign d_readM = MEM_MemRead;
assign d_writeM = MEM_MemWrite;


//Reg
 RegisterFile rf(.addr1(RF_addr1), 
                 .addr2(RF_addr2), 
                 .addr3(RF_addrw), 
                 .data(RF_wData), 
                 .write(WB_RegWrite), 
                 .clock(clk),
                 .reset_n(reset_n), 
                 .data1(RF_val1), 
                 .data2(RF_val2)
);

//Sign Extension

signExtension SE(SE_in, SE_out);

//Forwarding Unit

forward FWD(
    .MEM_RegWrite(MEM_RegWrite),
    .WB_RegWrite(WB_RegWrite),
    .MEM_dest(EXMEM_RF_dist_out),
    .WB_dest(MEMWB_RF_dist_out),
    .EX_rs(IDEX_rs_out),
    .EX_rt(IDEX_rt_out),
    
    .ALUsrc1(ALUsrc1),
    .ALUsrc2(ALUsrc2)
);


//Stalling Unit
stalling STU(

    .EX_MemRead(EX_MemRead),
    .EX_dest(EXMEM_RF_dist_in),
    .inst(IFID_inst_out),
    
    .isStalled(isStalled)
);

//////////////////////////////DEBUGGING////////////////////////////////////////

always @(posedge clk) begin
    $display("");
    $display("-----------------------------------");
    $display("");
end

always @(pc_out) $display("pc_out : %h", pc_out);
//always @(IFID_inst_out) $display("IFID_inst_out : %h", IFID_inst_out);
//always @(ALU_in1)$display ("ALU_in1 : %h", ALU_in1);
//always @(ALU_in2)$display ("ALU_in2 : %h", ALU_in2);
//always @(ALU_out)$display ("ALU_out : %h", ALU_out);
//always @(EX_ALUsrc)$display ("EX_ALUsrc : %h", EX_ALUsrc);
//always @(ALUsrc1)$display ("ALUsrc1 : %h", ALUsrc1);
//always @(ALUsrc2)$display ("ALUsrc2 : %h", ALUsrc2);
//always @(MEM_RegWrite)$display ("MEM_RegWrite : %b", MEM_RegWrite);
//always @(WB_RegWrite)$display ("WB_RegWrite : %b", WB_RegWrite);
//always @(MEMWB_rdata_in)$display ("MEMWB_rdata_in : %h (addr : %h)", MEMWB_rdata_in, dmem_addr);
always @(MEMWB_rdata_out)$display ("MEMWB_rdata_out : %h", MEMWB_rdata_out);
always @(RF_wData)$display ("RF_wData : %h", RF_wData);



///////////////////////////////////////////////////////


/////////////////////////////LATCH INIT/////////////////////////////////////////

latch PC(PC_enable, 0, clk, pc_in, pc_out);

reg[15:0] pc_next;

initial begin
    pc_next<=16'h22;
end

always @(pc_out) pc_next = pc_next + 1;

assign pc_in = pc_next;

IFID ifid(
    .clk(clk),
    
    //data signals
    .inst_in(IFID_inst_in),
    .inst_out(IFID_inst_out),
    .enable(IFID_enable)
);


IDEX idex(
    .clk(clk),
    
    //data signals
    
    .val1_in(IDEX_val1_in),
    .val2_in(IDEX_val2_in),
    .SE_in(IDEX_SE_in),
    .rt_in(IDEX_rt_in),
    .rd_in(IDEX_rd_in),
    .rs_in(IDEX_rs_in),
    
    .val1_out(IDEX_val1_out),
    .val2_out(IDEX_val2_out),
    .SE_out(IDEX_SE_out),
    .rt_out(IDEX_rt_out),
    .rd_out(IDEX_rd_out),
    .rs_out(IDEX_rs_out),
    
    
    //control signals
    
     .ALUsrc_in(ID_ALUsrc),
     .RegDist_in(ID_RegDist),
     .MemWrite_in(ID_MemWrite),
     .MemRead_in(ID_MemRead),
     .MemtoReg_in(ID_MemtoReg),
     .RegWrite_in(ID_RegWrite),
     .Alucode_in(ID_Alucode),
     .Branch_in(ID_Branch),
     
     .ALUsrc_out(EX_ALUsrc),
     .RegDist_out(EX_RegDist),
     .MemWrite_out(EX_MemWrite),
     .MemRead_out(EX_MemRead),
     .MemtoReg_out(EX_MemtoReg),
     .RegWrite_out(EX_RegWrite),
     .Alucode_out(EX_Alucode),
     .Branch_out(EX_Branch)
     
);
    

EXMEM exmem(
    .clk(clk),
    
    //data signals
    
    .ALU_in(EXMEM_ALU_in),
    .dmem_in(EXMEM_dmem_in),
    .dist_in(EXMEM_RF_dist_in),
    
    .ALU_out(EXMEM_ALU_out),
    .dmem_out(EXMEM_dmem_out),
    .dist_out(EXMEM_RF_dist_out),
    
    
    //control signals
    
    .MemWrite_in(EX_MemWrite),
    .MemRead_in(EX_MemRead),
    .MemtoReg_in(EX_MemtoReg),
    .RegWrite_in(EX_RegWrite),
    
    .MemWrite_out(MEM_MemWrite),
    .MemRead_out(MEM_MemRead),
    .MemtoReg_out(MEM_MemtoReg),
    .RegWrite_out(MEM_RegWrite)
    
     
);    
    
MEMWB memwb(
    
    .clk(clk),
    
    //data signals
    
    .rdata_in(MEMWB_rdata_in),
    .ALU_in(MEMWB_ALU_in),
    .dist_in(MEMWB_RF_dist_in),
    
    
    .rdata_out(MEMWB_rdata_out),
    .ALU_out(MEMWB_ALU_out),
    .dist_out(MEMWB_RF_dist_out),
    
    //control signals
    
    .MemtoReg_in(MEM_MemtoReg),
    .RegWrite_in(MEM_RegWrite),
    
    .MemtoReg_out(WB_MemtoReg),
    .RegWrite_out(WB_RegWrite)
     
);


endmodule


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////
///////////////////////////SIGN EXTENSTION MODULE//////////////////////////
///////////////////////////////////////////////////////////////////////////

module signExtension(
    input [7:0] in,
    output [15:0] out
);

    genvar j;
    wire [15:0] out;

    assign out[7:0] = in;
    for(j=8; j<16; j=j+1) begin
        assign out[j] = in[7]==0 ? 0 : 1;
    end

endmodule

///////////////////////////////////////////////////////////////////////////
///////////////////////////LATCH MODULEs///////////////////////////////////
///////////////////////////////////////////////////////////////////////////


// when enabled, change 'out value' to 'in value' in every clock

module latch(
    input enable,
    input flush,
    input clk,
    input[15:0] in,
    output[15:0] out
);

    reg[15:0] out;
    
    initial begin
        out <= 16'bx;
    end
    
    always @(posedge clk) begin
        if(flush) out<=0;
        else if(enable) out<=in;
        else out<=out;
        
    end

endmodule


//for 2-bit
module latch2(
    input enable,
    input flush,
    input clk,
    input[1:0] in,
    output[1:0] out
);

    reg[1:0] out;
    
    initial begin
        out <= 2'bx;
    end
    
    always @(posedge clk) begin
        if(flush) out<=0;
        else if(enable) out<=in;
        else out<=out;
        
    end

endmodule


//for 1-bit
module latch1(
    input enable,
    input flush,
    input clk,
    input in,
    output out
);

    reg out;
    
    initial begin
        out <= 1'bx;
    end
    
    always @(posedge clk) begin
        if(flush) out<=0;
        else if(enable) out<=in;
        else out<=out;
        
    end

endmodule

//for 4-bit
module latch4(
    input enable,
    input flush,
    input clk,
    input[3:0] in,
    output[3:0] out
);

    reg[3:0] out;
    
    initial begin
        out <= 4'bx;
    end
    
    always @(posedge clk) begin
        if(flush) out<=0;
        else if(enable) out<=in;
        else out<=out;
        
    end

endmodule


module IFID(
    input clk,
    
    //data signals
    input[15:0] inst_in,
    output[15:0] inst_out,
    
    input enable
);

    latch inst(enable, 0, clk, inst_in, inst_out); //enable, flush must be updated
    
endmodule

module IDEX(
    input clk,
    
    //data signals
    
    input[15:0] val1_in,
    input[15:0] val2_in,
    input[15:0] SE_in,
    input[1:0] rt_in,
    input[1:0] rd_in,
    input[1:0] rs_in,
    
    output[15:0] val1_out,
    output[15:0] val2_out,
    output[15:0] SE_out,
    output[1:0] rt_out,
    output[1:0] rd_out,
    output [1:0] rs_out,
    
    //control signals
    
     input ALUsrc_in,
     input RegDist_in,
     input MemWrite_in,
     input MemRead_in,
     input MemtoReg_in,
     input RegWrite_in,
     input[3:0] Alucode_in,
     input Branch_in,
     
     output ALUsrc_out,
     output RegDist_out,
     output MemWrite_out,
     output MemRead_out,
     output MemtoReg_out,
     output RegWrite_out,
     output[3:0] Alucode_out,
     output Branch_out
     
);
    
    //data signals
    
    latch val1(1, 0, clk, val1_in, val1_out); 
    latch val2(1, 0, clk, val2_in, val2_out); 
    latch SE(1, 0, clk, SE_in, SE_out); 
    latch2 rt(1, 0, clk, rt_in, rt_out); 
    latch2 rd(1, 0, clk, rd_in, rd_out); 
    latch2 rs(1, 0, clk, rs_in, rs_out); 
    
    //control signals
    
    latch1 ALUsrc(1, 0, clk, ALUsrc_in, ALUsrc_out);
    latch1 RegDist(1, 0, clk, RegDist_in, RegDist_out);
    latch1 MemWrite(1, 0, clk, MemWrite_in, MemWrite_out);
    latch1 MemRead(1, 0, clk, MemRead_in, MemRead_out);
    latch1 MemtoReg(1, 0, clk, MemtoReg_in, MemtoReg_out);
    latch1 RegWrite(1, 0, clk, RegWrite_in, RegWrite_out);
    latch4 Alucode(1, 0, clk, Alucode_in, Alucode_out);
    latch1 Branch(1, 0, clk, Branch_in, Branch_out);
    
    
    
endmodule



module EXMEM(
    input clk,
    
    //data signals
    
    input[15:0] ALU_in,
    input[15:0] dmem_in,
    input[1:0] dist_in,
    
    output[15:0] ALU_out,
    output[15:0] dmem_out,
    output[1:0] dist_out,
    
    //control signals
    
     input MemWrite_in,
     input MemRead_in,
     input MemtoReg_in,
     input RegWrite_in,
     
     output MemWrite_out,
     output MemRead_out,
     output MemtoReg_out,
     output RegWrite_out
     
);

    
    latch ALU(1, 0, clk, ALU_in, ALU_out); 
    latch dmem(1, 0, clk, dmem_in, dmem_out); 
    latch2 dist(1, 0, clk, dist_in, dist_out); 

    latch1 MemWrite(1, 0, clk, MemWrite_in, MemWrite_out);
    latch1 MemRead(1, 0, clk, MemRead_in, MemRead_out);
    latch1 MemtoReg(1, 0, clk, MemtoReg_in, MemtoReg_out);
    latch1 RegWrite(1, 0, clk, RegWrite_in, RegWrite_out);
    
    
endmodule

module MEMWB(
    input clk,
    
    //data signals
    
    input[15:0] rdata_in,
    input[15:0] ALU_in,
    input[1:0] dist_in,
    
    output[15:0] rdata_out,
    output[15:0] ALU_out,
    output[1:0] dist_out,
    
    //control signals
    
     input MemtoReg_in,
     input RegWrite_in,
     
     output MemtoReg_out,
     output RegWrite_out
     
);

    
    latch rdata(1, 0, clk, rdata_in, rdata_out); 
    latch ALU(1, 0, clk, ALU_in, ALU_out); 
    latch2 dist(1, 0, clk, dist_in, dist_out); 

    latch1 MemtoReg(1, 0, clk, MemtoReg_in, MemtoReg_out);
    latch1 RegWrite(1, 0, clk, RegWrite_in, RegWrite_out);
    
    
endmodule


///////////////////////////////////////////////////////////////////////////
/////////////////////////////FORWARDING UNIT///////////////////////////////
///////////////////////////////////////////////////////////////////////////


module forward(

    input MEM_RegWrite,
    input WB_RegWrite,
    input[1:0] MEM_dest,
    input[1:0] WB_dest,
    input[1:0] EX_rs,
    input[1:0] EX_rt,
    
    output[1:0] ALUsrc1,
    output[1:0] ALUsrc2

);

    reg[1:0] ALUsrc1;
    reg[1:0] ALUsrc2;

    initial begin
      
       ALUsrc1<=2'b00;
       
       ALUsrc2<=2'b00;
    end

    always @(*) begin
        //$display("EX_rs : %d, MEM_dest : %d, WB_dest : %d", EX_rs, MEM_dest, WB_dest);
        if(MEM_RegWrite && EX_rs==MEM_dest) begin 
            ALUsrc1<=2'b10;
            $display("Forwarded : MEM -> rs");
        end
        
        else if(WB_RegWrite && EX_rs==WB_dest) begin
            ALUsrc1<=2'b11;
            $display("Forwarded : WB -> rs");
        end
        
        else ALUsrc1<=2'b00;
    
        if(MEM_RegWrite && EX_rt==MEM_dest) begin
            ALUsrc2<=2'b10;
            $display("Forwarded : MEM -> rt");
        end
        
        else if(WB_RegWrite && EX_rt==WB_dest) begin
            ALUsrc2<=2'b11;
            $display("Forwarded : WB -> rs");
        end
        
        else ALUsrc2<=2'b00;
    
    end 


endmodule


///////////////////////////////////////////////////////////////////////////
/////////////////////////////STALLING UNIT/////////////////////////////////
///////////////////////////////////////////////////////////////////////////


module stalling(

    input EX_MemRead,
    input[1:0] EX_dest,
    input[15:0] inst,
    
    output isStalled
);

reg isStalled;
initial isStalled <= 0;

wire[1:0] ID_rt;
wire[1:0] ID_rs;
wire[5:0] ftncode;
wire[3:0] opcode;

assign ID_rt = inst[9:8];
assign ID_rs = inst[11:10];
assign ftncode = inst[5:0];
assign opcode = inst[15:12];

wire use_rs;
wire use_rt;

assign use_rs = !(opcode==9 || opcode==10);
assign use_rt = (opcode==15 && (ftncode==0 || ftncode==1 || ftncode==2 || ftncode==3));


//EX stage MEM_Read, ID stage using same register
always @(*) begin
    
    if(EX_MemRead && ID_rs==EX_dest && use_rs) begin
        $display("stalled instruction : %h", inst);
        isStalled <= 1;
    end
    else if(EX_MemRead && ID_rt==EX_dest && use_rt) begin
        $display("stalled instruction : %h", inst);
        isStalled <= 1;
    end
    else begin
        isStalled <= 0;
    end


end

endmodule



///////////////////////////////////////////////////////////////////////////
/////////////////////////////FLUSHING UNIT/////////////////////////////////
///////////////////////////////////////////////////////////////////////////

module flushing(
    input Jump,
    
    output IFID_Flush,
    output IDEX_Flush,
    output PCsrc

);

reg IFID_Flush;
reg IDEX_Flush;

initial begin
    IFID_Flush <= 0;
    IDEX_Flush <= 0;
end    

always @(*) begin
    
    if(!Jump) begin
        IFID_Flush <= 0;
        IDEX_Flush <= 0;
    end
    
    else if (Jump) begin
        IFID_Flush <= 1;
        IDEX_Flush <= 0;
    end
    
end

endmodule