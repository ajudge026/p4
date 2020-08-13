#ifndef __CORE_H__
#define __CORE_H__

#include "Instruction_Memory.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define BOOL bool

typedef uint8_t Byte;
typedef int64_t Signal;
typedef int64_t Register;


typedef struct instruction_fetch_reg
{    Signal PC;
    Signal instruction;
}instruction_fetch_reg;


typedef struct instruction_decode_reg
{
    Signal read_reg_val_1;
	Signal read_reg_val_2;
    Signal imm_sign_extended;
}instruction_decode_reg;


typedef struct execute_reg
{
    Signal branch_address;
	Signal zero_out;
    Signal alu_result;
	Signal reg_read_2_val;
}execute_reg;


typedef struct mem_acces_reg
{
    Signal mem_read_data;
	Signal alu_result;
	Signal branch_address;	
	
}mem_acces_reg;


typedef struct write_back_reg
{
    Signal reg_write_mux_val;		
}write_back_reg;

struct Core;
typedef struct Core Core;
typedef struct Core
{
    Tick clk; // Keep track of core clock
    Addr PC; // Keep track of program counter

    // What else you need? Data memory? Register file?
    Instruction_Memory *instr_mem;
   
    Byte data_mem[1024]; // data memory

    Register reg_file[32]; // register file.

    bool (*tick)(Core *core);
	
	instruction_fetch_reg IF_reg;
	
	instruction_decode_reg ID_reg;	
	
	execute_reg E_reg;
	
	mem_acces_reg M_reg;
	
	write_back_reg WB_reg;
	
}Core;





Core *initCore(Instruction_Memory *i_mem);
bool tickFunc(Core *core);

// FIXME. Implement the following functions in Core.c
// FIXME (1). Control Unit.
typedef struct ControlSignals
{
    Signal Branch;
    Signal MemRead;
    Signal MemtoReg;
    Signal ALUOp;
    Signal MemWrite;
    Signal ALUSrc;
    Signal RegWrite;
}ControlSignals;
void ControlUnit(unsigned instruction, 
				Signal input,
                 ControlSignals *signals);

// FIXME (2). ALU Control Unit.
Signal ALUControlUnit(Signal ALUOp,
                      Signal Funct7,
                      Signal Funct3);

// FIXME (3). Imme. Generator
Signal ImmeGen(Signal input, unsigned instruction);

// FIXME (4). ALU
void ALU(Signal input_0,
         Signal input_1,
         Signal ALU_ctrl_signal,
         Signal *ALU_result,
         Signal *zero);

// (4). MUX
Signal MUX(Signal sel,
           Signal input_0,
           Signal input_1);

// (5). Add
Signal Add(Signal input_0,
           Signal input_1);

// (6). ShiftLeft1
Signal ShiftLeft1(Signal input);

#endif

