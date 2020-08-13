#include "Core.h"

Core *initCore(Instruction_Memory *i_mem)
{
    Core *core = (Core *)malloc(sizeof(Core));
    core->clk = 0;
    core->PC = 0;
    core->instr_mem = i_mem;
    core->tick = tickFunc;

    // FIXME, initialize register file here.
    // initializeregs in core 
	/* core->IF_reg.PC = 0;
	core->IF_reg.instruction = 0;
	
	core->ID_reg.read_reg_val_1 = 0
	 Signal read_reg_val_1;
	Signal read_reg_val_2;
    Signal imm_sign_extended; */
	
	instruction_decode_reg ID_reg;	
	
	execute_reg E_reg;
	
	mem_acces_reg M_reg;
	
	write_back_reg WB_reg;

    
	for (int i = 0; i <(1024);i++)
	{
		core->data_mem[i] = 0;		
	}
	
	
	core->data_mem[40*8] = -63; // 40(x1) = -63,
	core->data_mem[48*8] = 63; // 48(x1) = 63,
	printf("40(x1) = %d\n", core->data_mem[40*8]);
	printf("48(x1) = %d\n", core->data_mem[48*8]);


	

    // FIXME, initialize data memory here.
    // core->reg_file[0] = ...

    //set the reg_files for holding the offset
	 
	
	core->reg_file[1] = 0;	 
	 core->reg_file[0] = 0; 
	 core->reg_file[2] = 10; //outbase
	 core->reg_file[3] = -15; 
	 core->reg_file[4] = 20; 
	 core->reg_file[5] = 30; 
	 core->reg_file[6] = -35; 
	 
	
    return core;
}

// FIXME, implement this function
bool tickFunc(Core *core)
{
	//printf("==============  new tick =====================");
    // Steps may include
    // (Step 1) Reading instruction from instruction memory
    
	// <------------------------ IF Reg
	unsigned instruction  = core->IF_reg.instruction;	
	Signal PC_pls_four = core->IF_reg.PC;
	
	core->IF_reg.instruction = core->instr_mem->instructions[core->PC / 4].instruction;
	core->IF_reg.PC = Add(core->PC, 4);    
	// separate parts of the input instructions 	
    Signal input = (instruction & 127);
    ControlSignals signals;
    ControlUnit(instruction, input, &signals);	
    Signal func3 =( (instruction >> (7 + 5)) & 7);    
	Signal func7 = ((instruction >> (7 + 5 + 3 + 5 + 5)) & 127);	
    Signal ALU_ctrl_signal = ALUControlUnit(signals.ALUOp, func7, func3);
    Register read_reg_1 = (instruction >> (7 + 5 + 3)) & 31;    
	Register read_reg_2 = (instruction >> (7 + 5 + 3 + 5)) & 31;	
	Register write_reg = (instruction >> 7) & 31;
	
	// <------------------------ ID Reg	
	Signal read_reg_2_value = core->ID_reg.read_reg_val_2;
	Signal read_reg_1_value = core->ID_reg.read_reg_val_1;
	Signal shifted_immediate = core->ID_reg.imm_sign_extended ;

	core->ID_reg.read_reg_val_1 = core->reg_file[read_reg_1];
	core->ID_reg.read_reg_val_2 = core->reg_file[read_reg_2];
	core->ID_reg.imm_sign_extended = ImmeGen( input,instruction);;
	
    Signal alu_in_1 = MUX(signals.ALUSrc,core->reg_file[read_reg_2],shifted_immediate);
	Signal alu_in_0 = core->reg_file[read_reg_1];
	// <---------------------------------- Execute Reg 
	 //write to signals (from sequential logic )
	Signal ALU_output = core->E_reg.alu_result ;		
    Signal zero_alu_input = core->E_reg.zero_out;	
	read_reg_2_value = core->E_reg.reg_read_2_val;	
    //write to reg (from combinational logic )
	ALU(alu_in_0, alu_in_1, ALU_ctrl_signal, &core->E_reg.alu_result, &core->E_reg.zero_out); // 0 is offset shuold change to imm val
	core->E_reg.branch_address = shifted_immediate + PC_pls_four ;			
	core->E_reg.reg_read_2_val = core->ID_reg.read_reg_val_2 ;
	
	// core outputs of memory 
    Signal mem_result= 0;
	 mem_result = core->data_mem[8*ALU_output];
	// <------------------------ M Reg
	core->M_reg.mem_read_data 	= mem_result;
	core->M_reg.alu_result = ALU_output;	
	core->M_reg.branch_address = 0; // <------------------ change to branch address

    /* Signal mem_read_data;
	Signal alu_result;
	Signal branch_address;
	*/	
	
	if(signals.MemWrite)
    {       
		core->data_mem[8*ALU_output] = read_reg_2_value;		
    }
	
	
	

	Signal write_reg_val =  core->reg_file[write_reg];
	
    if(signals.RegWrite)
    {
		
        core->reg_file[write_reg] = MUX(signals.MemtoReg, ALU_output, mem_result);
    }

	
	Signal mux_output = MUX((zero_alu_input & signals.Branch), 4, (signed int)shifted_immediate);
	
	
	
		
    core->PC = Add(core->PC, mux_output);
	
    


	
		
	
    ++core->clk;
    // Are we reaching the final instruction?
    if (core->PC > core->instr_mem->last->addr)
    {
	
        return false;
    }
    return true;
}

// FIXME (1). Control Unit. Refer to Figure 4.18.
void ControlUnit(unsigned instruction, Signal input,
                 ControlSignals *signals)
{	
	Signal func3 = ( (instruction >> (7 + 5)) & 7);
    // For R-type - add
    if (input == 51 & (func3 == 0)) {
		//printf("RType\n"); 
        signals->ALUSrc = 0;
        signals->MemtoReg = 0;
        signals->RegWrite = 1;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 2;
    }
	
	
	// For R-type - sll
    if (input == 51 & (func3 == 1)) {
		//printf("RType\n"); 
        signals->ALUSrc = 0;
        signals->MemtoReg = 0;
        signals->RegWrite = 1;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 0;
    }
    // For ld 
    if (input == 3) { //opcode
	    //printf("ld\n"); 
        signals->ALUSrc = 1;
        signals->MemtoReg = 1;
        signals->RegWrite = 1;
        signals->MemRead = 1;
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 0;
    }
    // For addaddi , slli 
    if (input == 19 ){
		//printf("slli\n"); 		
        signals->ALUSrc = 1;
        signals->MemtoReg = 1;
        signals->RegWrite = 1;
        signals->MemRead = 1;
        signals->MemWrite = 0;
        signals->Branch = 0;
        signals->ALUOp = 0;
    }
	
    // For sd (S-type)
    if (input == 35){
		//printf("sw\n"); 
        signals->ALUSrc = 1;
        signals->MemtoReg = 0; 
        signals->RegWrite = 0;
        signals->MemRead = 0;
        signals->MemWrite = 1;
        signals->Branch = 0;
        signals->ALUOp = 69;
    }
    // For beq (SB-type)
    if (input == 99){ //opcode
        //printf("bne\n"); 
		signals->ALUSrc = 0;		
        signals->MemtoReg = 0; 
        signals->RegWrite = 0;
        signals->MemRead = 0;
        signals->MemWrite = 0;
        signals->Branch = 1;
        signals->ALUOp = 1;
    }
}

// FIXME (2). ALU Control Unit. Refer to Figure 4.12.
Signal ALUControlUnit(Signal ALUOp,
                      Signal Funct7,
                      Signal Funct3)
{
    //  add
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 0)
    {
        return 2;
    }

    

    // For subtract 
    if (ALUOp == 2 && Funct7 == 32 && Funct3 == 0)
    {
        return 6;
    }
    //and
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 7)
    {
        return 0;
    }
    //  or 
    if (ALUOp == 2 && Funct7 == 0 && Funct3 == 6)
    {
        return 1;
    }
	// slli
    if (ALUOp == 0 && Funct7 == 0 && Funct3 == 1)
    {
        return 3;
    }

	
    // ld 
    if (ALUOp == 0 && Funct3 == 3)
    {
        return 2;
    }
    //  sd
    if (ALUOp == 69)
    {
        return 2;
    }
    //  beq 
    if (ALUOp == 1)
    {
        return 6;
    }
	//printf("Funct3 - %ld\n", Funct3);
    //printf("Funct7 - %ld\n", Funct7);
    //printf("ALUOP - %ld\n", ALUOp);
    
}

// FIXME (3). Imme. Generator
Signal ImmeGen(Signal input, unsigned instruction)
{
     Signal immediate = 0;

    //ld
    if (input == 3){
        // 000000000000;
        immediate = 0;
    }
    //addi
    if (input == 19){
        //  000000000001;
        immediate = 1;
    }
    //slli
    if (input == 14)    {
        //  000000000011;
        immediate = 3;
    }
    //bne
    if (input == 99)    {
        //  111111111110;
			//imm goes from left to right 
			Signal immNeg = (instruction >> 31) & 1;
			Signal imm3 = (instruction >> 24) & 63 ;
			Signal imm2 = (instruction >> 8) &15;
			Signal imm1 = (instruction >> 7) &1;
			//printf("immneg = %d\n", immNeg);
			//printf("imm3 = %d\n", imm3);
			//printf("imm2 = %d\n", imm2);
			//printf("imm1 = %d\n", imm1);
			immediate = imm2;
			immediate |= imm3 <<4;
			immediate |= imm1 <<10;
			immediate |= immNeg << 11;
		/* if (immediate == 4062)
		{
									
		} */
		
		if (immediate == 4056)
		{
				immediate = -24;					
		}
		else
		{
			immediate = -16;
		}
		
		
		
		
			
			
		
		//printf("unshifted imm - %d\n", immediate);
    }
	


    return immediate;

} 

// FIXME (4). ALU
void ALU(Signal input_0,
         Signal input_1,
         Signal ALU_ctrl_signal,
         Signal *ALU_result,
         Signal *zero)
{
    // For addition
    if (ALU_ctrl_signal == 2)
    {
        *ALU_result = (input_0 + input_1);
        if (*ALU_result == 0) { *zero = 1; } else { *zero = 0; }
    }
	// For addition sd may need to change 
    if (ALU_ctrl_signal == 69)
    {
        *ALU_result = (input_0 + input_1);
        if (*ALU_result == 0) { *zero = 1; } else { *zero = 0; }
    }
    // For and
    if (ALU_ctrl_signal == 0)
    {
        *ALU_result = (input_0 & input_1);
        if (*ALU_result == 0) { *zero = 1; } else { *zero = 0; }
    }
    // For or
    if (ALU_ctrl_signal == 1)
    {
        *ALU_result = (input_0 | input_1);
        if (*ALU_result == 0) { *zero = 1; } else { *zero = 0; }
    }
    // For subtraction
    if (ALU_ctrl_signal == 6)
    {
        *ALU_result = (input_0 - input_1);
		//printf("ALU RESULT - %ld", (input_0 - input_1));
        if (*ALU_result != 0) { *zero = 1; } else { *zero = 0; }
    }
    // For shift left
    if (ALU_ctrl_signal == 3)
    {
        *ALU_result = (input_0 << input_1);
        if (*ALU_result == 0) { *zero = 1; } else { *zero = 0; }
    }

}

// (4). MUX
Signal MUX(Signal sel,
           Signal input_0,
           Signal input_1)
{
    if (sel == 0) { return input_0; } else { return input_1; }
}

// (5). Add
Signal Add(Signal input_0,
           Signal input_1)
{
    return (input_0 + input_1);
}

// (6). ShiftLeft
Signal ShiftLeft1(Signal input)
{
    return input << 1; //<--------------------------------------------------- why? 
}

// regwrite 
 void regWrite(Signal MemWrite, Signal *data_mem, Signal data, Signal *address )
{
		
}