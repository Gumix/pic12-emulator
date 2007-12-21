#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PROG_MEM_SIZE		0x400
#define DATA_MEM_SIZE		0x0FF
#define UNREAl_OFFSET		0x80

#define STACK_SIZE			8

#define STATUS_C			0
#define STATUS_DC			1
#define STATUS_Z			2
#define STATUS_PD			3
#define STATUS_TO			4
#define STATUS_RP0			5

#define INTCON_GIE			7

#define TMR0				0x01
#define PCL					0x02
#define STATUS				0x03
#define FSR					0x04
#define GPIO				0x05
#define PCLATH				0x0A
#define INTCON				0x0B
#define PIR1				0x0C
#define TMR1L				0x0E
#define TMR1H				0x0F
#define T1CON				0x10
#define CMCON				0x19
#define OPTION_REG			0x81
#define TRISIO				0x85
#define PIE1				0x8C
#define PCON				0x8E
#define OSCCAL				0x90
#define WPU					0x95
#define IOC					0x96
#define VRCON				0x99
#define EEDATA				0x9A
#define EEADR				0x9B
#define EECON1				0x9C

#define BIT_GET(x, y)		(((x) >> (y)) & 1)
#define BIT_SET(x, y)		((x) |= (1 << (y)))
#define BIT_CLEAR(x, y)		((x) &= ~(1 << (y)))

#define GET_LOW_7_BITS(x)	((x) & 0x07F)
#define GET_LOW_8_BITS(x)	((x) & 0x0FF)
#define GET_LOW_11_BITS(x)	((x) & 0x7FF)
#define GET_BITS_9_8_7(x)	(((x) >> 7) & 7)

#define REGISTER(x, y)		(*x[(BIT_GET(*mem[STATUS], STATUS_RP0) ? y+UNREAl_OFFSET : y)])

unsigned char W;						// Accumulator
int PC;									// Program counter
unsigned int WDT;						// Watchdog timer
short prog[PROG_MEM_SIZE];				// Program memory
unsigned char real_mem[DATA_MEM_SIZE];
unsigned char *mem[DATA_MEM_SIZE];		// Data memory
int STACK[STACK_SIZE];					// Call stack
unsigned char stack_ptr;

unsigned char sW;						// Saved accumulator for tests
int sPC;								// Saved program counter for tests
unsigned char sreal_mem[DATA_MEM_SIZE];
unsigned char *smem[DATA_MEM_SIZE];		// Saved data memory for tests

void zero_flag_set(unsigned char result)
{
	if (result == 0)
		BIT_SET(*mem[STATUS], STATUS_Z);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_Z);
}

void stack_push(int newPC)
{
	//assert(stack_ptr>=0 && stack_ptr<STACK_SIZE);
	STACK[stack_ptr]= newPC&0x1FFF;//the stack supports only 13-bit integers
	stack_ptr++;
}

int stack_pop()
{
	int newPC= STACK[stack_ptr];
	assert(stack_ptr>0 && stack_ptr<=STACK_SIZE);
	STACK[stack_ptr]= 0;
	stack_ptr--;
	return newPC;
}

void ADDWF(unsigned char d, unsigned char f)
{
	printf("ADDWF 0x%02X, %d", f, d);

	unsigned char result,f_value=REGISTER(mem,f);

	//assert(f_value>=0 && f_value<=127);
	result = W+f_value;

	if(( (int)(f_value&0xff) + (W&0xff) )	> 0xff)
		BIT_SET(*mem[STATUS], STATUS_C);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_C);

	if(( (int)(f_value&0x0f) + (W&0x0f) )	> 0x0f)
		BIT_SET(*mem[STATUS], STATUS_DC);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_DC);

	if (d&0x1){
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}

		REGISTER(mem,f)= result;
	}else
		W = result;

	zero_flag_set(result);
	PC++;
}

void ANDWF(unsigned char d,unsigned char f)
{
	printf("ANDWF 0x%02X, %d", f, d);

	unsigned char result,f_value=REGISTER(mem,f);
	//assert(f_value>=0 && f_value<=127);
	result = W&f_value;
	if (d&0x1)
	{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	else W = result;
	zero_flag_set(result);
	PC++;
}

void CLRF(unsigned char d, unsigned char f)
{
	printf("CLRF 0x%02X", f);

	REGISTER(mem, f) = 0;
	BIT_SET(*mem[STATUS], STATUS_Z);
	PC++;
}

void CLRW()
{
	printf("CLRW");

	W = 0;
	BIT_SET(*mem[STATUS], STATUS_Z);
	PC++;
}

void COMF(unsigned char d,unsigned char f)
{
	printf("COMF 0x%02X, %d", f, d);

	char result = ~REGISTER(mem, f);

	if (result)
		BIT_CLEAR(*mem[STATUS], STATUS_Z);
	else
		BIT_SET(*mem[STATUS], STATUS_Z);

	if (d)
		REGISTER(mem, f) = result;
	else
		W = result;

	PC++;
}

void DECF(unsigned char d, unsigned char f)
{
	printf("DECF 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);;
	//assert(f>=0 && f<=127);
	assert(d==0 || d==1);
	if (result == 0x00)
	{
		result = 0xff;
	}
	else
		result--;
	if (d == 0)
		W = result;
	else{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	zero_flag_set(result);
	PC++;

}

void DECFSZ(unsigned char d, unsigned char f)
{
	printf("DECFSZ 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);;
	//assert(f>=0 && f<=127);
	assert(d==0 || d==1);
	if (result == 0x00)
		result = 0xff;
	else
		result--;
	if (d == 0)
		W = result;
	else{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	if (result == 0)
		PC=PC+2;
	else
		PC++;

}

void INCF(unsigned char d, unsigned char f)
{
	printf("INCF 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);
	result++;

	if (d == 0)
		W = result;
	else{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	zero_flag_set(result);
	PC++;
}

void INCFSZ(unsigned char d, unsigned char f)
{
	printf("INCFSZ 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);

	result++;

	if (d == 0)
		W = result;
	else{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)		// WTF ???
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}

	if (result == 0)
		PC += 2;
	else
		PC++;
}

void IORWF(unsigned char d, unsigned char f)
{
	printf("IORWF 0x%02X, %d", f, d);

	unsigned char result,f_value=REGISTER(mem,f);
	//assert(f_value>=0 && f_value<=127); 
	assert(d==0 || d==1);
	result= (W | f_value) &0x7f;
	if(d&0x1){
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
		REGISTER(mem,f)= result;
	}
	else W= result;
	zero_flag_set(result);
	PC++;
}

void MOVF(unsigned char d, unsigned char f)
{
	printf("MOVF 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);
	if (d)
	{
		REGISTER(mem, f) = result;
		// если f = TMR0 и OPTION_REG[3]=0, сбрасываем значение предварительного делителя
		//(OPTION_REG[0:2] - устанавливаются 1)
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	else
	{
		W=result;
	}
	zero_flag_set(result);
	PC++;
}

void MOVWF(unsigned char d, unsigned char f)
{
	printf("MOVWF 0x%02X", f);

	REGISTER(mem, f) = W;
	PC++;
}

// No operation.
void NOP()
{
	printf("NOP");

	PC++;
}

void RLF(unsigned char d,unsigned char f)
{
	printf("RLF 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);
	unsigned char C=BIT_GET(*mem[STATUS],STATUS_C);
	unsigned char oldC=C;
	C=(result&128)>>7;
	result=(result<<1)|oldC;
	if (d == 0)
		W = result;
	else{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	if(C)
		BIT_SET(*mem[STATUS], STATUS_C);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_C);
	PC++;
}


void RRF(unsigned char d,unsigned char f)
{
	printf("RRF 0x%02X, %d", f, d);

	unsigned char result = REGISTER(mem, f);
	unsigned char C=BIT_GET(*mem[STATUS],STATUS_C);
	unsigned char oldC=C;
	C=(result&1);
	result=(result>>1)|(oldC<<7);
	if (d == 0)
		W = result;
	else{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	if(C)
		BIT_SET(*mem[STATUS], STATUS_C);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_C);
	PC++;
}


void SUBWF(unsigned char d, unsigned char f)
{
	printf("SUBWF 0x%02X, %d", f, d);

	unsigned char result,f_value=REGISTER(mem,f);
	unsigned char Z,C,DC;
	//assert(f_value>=0 && f_value<=127);
	Z= ((int)f_value-W==0)? 1:0;
	DC= ( (int)(f_value&0x0f) - (W&0x0f) )	< 0	? 0:1;
	C=  ( (int)(f_value&0xff) - (W&0xff) )	< 0	? 0:1;

	result= f_value-W;
	if(d&0x1){
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
		REGISTER(mem,f)= result;
	}else W= result;


	if(Z)
		BIT_SET(*mem[STATUS], STATUS_Z);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_Z);

	if(C)
		BIT_SET(*mem[STATUS], STATUS_C);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_C);

	if(DC)
		BIT_SET(*mem[STATUS], STATUS_DC);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_DC);

	PC++;
}

int Dec2Bin(int AnyValueWithinIntRange)
{
	int ResultOfTheFunction=0;
	int Count_D=AnyValueWithinIntRange;
	int i;
	for (i=0;i<=7;i++)
	{
		ResultOfTheFunction=ResultOfTheFunction+(Count_D%2)*pow(10,i);	
		Count_D=Count_D/2;
	}
	return ResultOfTheFunction;
}

int Join(int a, int b)
{
	int ret=a*10000+b;
	return ret;
}

int Bin2Dec(int a)
{
	int b=a;
	int i;
	int ret=0;
	for (i=0;i<=7;i++)
	{
		ret=ret+(b%10)*pow(2,i);
		b=(b-(b%10))/10;
	}
	return ret;
}

void SWAPF(unsigned char d,unsigned char f)
{
	printf("SWAPF 0x%02X, %d", f, d);

	unsigned char AVariableThatContainsTheOriginalValueOfTheRegisterInQuestion=REGISTER(mem,f);
	int ABinaryNumberThatIsCOUNTedByMyFunction=Dec2Bin(AVariableThatContainsTheOriginalValueOfTheRegisterInQuestion);
	int TheTailOfTheBasilisk=ABinaryNumberThatIsCOUNTedByMyFunction%10000;
	int TheHeadOfIt=ABinaryNumberThatIsCOUNTedByMyFunction/10000;
	int BinaryBasilisk=Join(TheTailOfTheBasilisk,TheHeadOfIt);
	unsigned char Basilisk=(unsigned char)Bin2Dec(BinaryBasilisk);
	if (d==0)
		W=Basilisk;
	else{
		REGISTER(mem,f)= Basilisk;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	PC++;
}

void XORWF(unsigned char d, unsigned char f)
{
	printf("XORWF 0x%02X, %d", f, d);

	unsigned char result = W ^ REGISTER(mem, f);

	if (d == 0)
		W = result;
	else
	{
		REGISTER(mem,f)= result;
		if (f==TMR0 && BIT_GET(*mem[OPTION_REG], 3)==0)		// WTF ???
		{ 
			BIT_SET(*mem[OPTION_REG], 0);
			BIT_SET(*mem[OPTION_REG], 1);
			BIT_SET(*mem[OPTION_REG], 2); 
		}
	}
	zero_flag_set(result);
	PC++;
}

void BCF(unsigned char b, unsigned char f)
{
	printf("BCF 0x%02X, %d", f, b);

	BIT_CLEAR(REGISTER(mem, f), b);
	PC++;
}

void BSF(unsigned char b, unsigned char f)
{
	printf("BSF 0x%02X, %d", f, b);

	BIT_SET(REGISTER(mem, f), b);
	PC++;	
}

void BTFSC(unsigned char b, unsigned char f)
{
	printf("BTFSC 0x%02X, %d", f, b);

	if BIT_GET(*mem[f], b)
	{
		PC++;
	}
	else
	{
		PC++;
		NOP();  //NOP
	}
}

void BTFSS(unsigned char b, unsigned char f)
{
	printf("BTFSS 0x%02X, %d", f, b);

	if BIT_GET(*mem[f], b)
	{
		PC++;
		NOP();  //NOP
	}
	else
	{
		PC++;
	}
}

void ADDLW(unsigned char k)
{
	printf("ADDLW 0x%02X", k);

	if ((255 - W) < k) 
		BIT_SET(*mem[STATUS], STATUS_C); // FLAGS |= 1;
	else
		BIT_CLEAR(*mem[STATUS], STATUS_C); // FLAGS &= ~1;
	if ( (255 - (unsigned char)(W << 4)) < (unsigned char)(k << 4) )
		BIT_SET(*mem[STATUS], STATUS_DC); //  FLAGS |= (1 << 1);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_DC); // FLAGS &= ~(1 << 1);
	W += k;
	if (W == 0)
		BIT_SET(*mem[STATUS], STATUS_Z); // FLAGS |= (1 << 2);	
	else 
		BIT_CLEAR(*mem[STATUS], STATUS_Z); // FLAGS &= ~(1 << 2);	
	PC++;
}

void ANDLW(unsigned char k)
{
	printf("ANDLW 0x%02X", k);

	unsigned char result;
	//assert(k>=0 && k<=255);
	result= W&k;
	W=result;
	zero_flag_set(result);
	PC++;
}

void GOTO(short k)
{
	printf("GOTO 0x%02X", k);

	assert ((k>=0)&&(k<=2047));

	int pclath = REGISTER(mem, PCLATH);

	PC = k | (((pclath >> 3) & 3) << 11); // new PC value
}

void CALL(short k)
{
	printf("CALL 0x%02X", k);

	stack_push(PC+1);
	GOTO(k);
}

void CLRWDT()
{
	printf("CLRWDT");

	WDT=0;
	BIT_SET(*mem[STATUS],STATUS_TO);
	BIT_SET(*mem[STATUS],STATUS_PD);
	//Похоже что prescaler общий и у TMR0 и у WDT!
	//В общем пока так можно оставить
	BIT_SET(*mem[OPTION_REG], 0);
	BIT_SET(*mem[OPTION_REG], 1);
	BIT_SET(*mem[OPTION_REG], 2);
}

void IORLW(unsigned char k)
{
	printf("IORLW 0x%02X", k);

	unsigned char result;
	//assert(k>=0 && k<=255);
	result= W|k;
	W=result;

	if(result==0)
		BIT_SET(*mem[STATUS], STATUS_Z);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_Z);

	PC++;
}

void MOVLW(unsigned char k)
{
	printf("MOVLW 0x%02X", k);

	W = k;
	PC++;
}

void RETFIE()
{
	printf("RETFIE");

	PC=stack_pop();
	BIT_SET(*mem[INTCON],INTCON_GIE);
}

void RETLW(unsigned char k)
{
	printf("RETLW 0x%02X", k);

	//assert(k>=0 && k<=255);
	PC=stack_pop();
	W=k;
}

void RETURN()
{
	printf("RETURN");

	PC=stack_pop();
}

void SLEEP()
{
	printf("SLEEP");

	WDT=0;
	BIT_SET(*mem[STATUS],STATUS_TO);
	BIT_CLEAR(*mem[STATUS],STATUS_PD);
	//Похоже что prescaler общий и у TMR0 и у WDT!
	//В общем пока так можно оставить
	BIT_SET(*mem[OPTION_REG], 0);
	BIT_SET(*mem[OPTION_REG], 1);
	BIT_SET(*mem[OPTION_REG], 2);
	//Ну и sleep включить надо не забыть!
}

void SUBLW(unsigned char k)
{

	printf("SUBLW 0x%02X", k);

	unsigned char Z= ((int)k-W==0)? 1:0;
	unsigned char DC= ( (int)(k&0x0f) - (W&0x0f) )	< 0	? 0:1;
	unsigned char C=  ( (int)(k&0xff) - (W&0xff) )	< 0	? 0:1;

	if(Z)
		BIT_SET(*mem[STATUS], STATUS_Z);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_Z);

	if(C)
		BIT_SET(*mem[STATUS], STATUS_C);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_C);

	if(DC)
		BIT_SET(*mem[STATUS], STATUS_DC);
	else
		BIT_CLEAR(*mem[STATUS], STATUS_DC);

	W=k-W;
	PC++;
}

void XORLW(unsigned char k)
{
	printf("XORLW 0x%02X", k);

	W ^= k;
	if (W) BIT_CLEAR(*mem[STATUS], STATUS_Z);
	else BIT_SET(*mem[STATUS], STATUS_Z);
	PC++;
}

void save_state()
{
	int i;

	sW = W;
	sPC = PC;

	for (i = 0; i < DATA_MEM_SIZE; i++)
		*smem[i] = *mem[i];
}

void test_PC_was_increased()
{
	assert(PC == sPC + 1);
}

void test_accumulator_was_untouched()
{
	assert(W == sW);
}

void test_zero_flag_set()
{
	if (W) assert(! BIT_GET(*mem[STATUS], STATUS_Z));
	else assert(BIT_GET(*mem[STATUS], STATUS_Z));
}

void test_WF_zero_flag_set(unsigned char d, unsigned char f)
{
	unsigned char result = d ? REGISTER(mem, f) : W;
	if (result==0)
		assert(BIT_GET(*mem[STATUS], STATUS_Z)==1);
	else
		assert(BIT_GET(*mem[STATUS], STATUS_Z)==0);
}

void test_ADDWF_not_our_mem_was_untouched(unsigned char d,unsigned char f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			if(d==1){
				if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
					continue;
			}else
				assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_ADDWF_was_added(unsigned char i, unsigned char j,unsigned char k,unsigned char d)
{
	if(d==0)
		assert(W == (unsigned char)(j+k));
	else
		assert(REGISTER(mem,i) == (unsigned char)(j+k));
}

void test_ADDWF_carry_flags_set(int i, unsigned char j,unsigned char k){
	unsigned char DC= ( (int)(k&0x0f) + (sW&0x0f) )	> 0x0f	? 1:0;
	unsigned char C=  ( (int)(k&0xff) + (sW&0xff) )	> 0xff	? 1:0;
	assert(BIT_GET(*mem[STATUS],STATUS_C)==C);
	assert(BIT_GET(*mem[STATUS],STATUS_DC)==DC);
}

void test_ADDWF()
{	
	int i, j;
	unsigned char k;

	for (i = 0x20; i <= 0x5F; i++)
		for (j = 0; j <= 127; j++)
		{
			for(k=0;k<=127;k++)
			{
				//Проверка при d=0
				W = j;
				REGISTER(mem,i)= k;
				save_state();
				ADDWF(0,i);

				test_ADDWF_was_added(i,j, k,0);
				test_WF_zero_flag_set(0,i);
				test_ADDWF_not_our_mem_was_untouched(0, i);
				test_ADDWF_carry_flags_set(i,j,k);
				test_PC_was_increased();

				//Проверка при d=1
				W = j;
				REGISTER(mem,i)= k;
				save_state();

				ADDWF(1,i);
				test_ADDWF_was_added(i,j, k,1);
				test_WF_zero_flag_set(1,i);
				test_ADDWF_not_our_mem_was_untouched(1, i);
				test_ADDWF_carry_flags_set(i,j,k);
				test_accumulator_was_untouched();
				test_PC_was_increased();
			}
		}
}

void test_ANDWF_not_our_mem_was_untouched(unsigned char d,unsigned char f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
			assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			if(d==1){
				if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
					continue;
			}else
				assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_ANDWF_was_anded(unsigned char i, unsigned char j,unsigned char k,unsigned char d)
{
	if(d==0)
		assert(W == (j&k));
	else
		assert(REGISTER(mem,i) == (j&k));
}

void test_ANDWF()
{
	int i, j;
	unsigned char k;

	for (i = 0x20; i <= 0x5F; i++)
		for (j = 0; j <= 127; j++)
		{
			for(k=0;k<=127;k++)
			{
				//Проверка при d=0
				W = j;
				REGISTER(mem,i)= k;
				save_state();
				ANDWF(0,i);

				test_ANDWF_was_anded(i,j, k,0);
				test_WF_zero_flag_set(0,i);
				test_ANDWF_not_our_mem_was_untouched(0, i);
				test_PC_was_increased();

				//Проверка при d=1
				W = j;
				REGISTER(mem,i)= k;
				save_state();

				ANDWF(1,i);
				test_ANDWF_was_anded(i,j, k,1);
				test_WF_zero_flag_set(1,i);
				test_ANDWF_not_our_mem_was_untouched(1, i);
				test_accumulator_was_untouched();
				test_PC_was_increased();
			}
		}
};

void test_CLRF_register_is_cleared(unsigned char f)
{	
	assert(REGISTER(mem, f) == 0);
}

void test_CLRF_not_our_mem_was_untouched(unsigned char f)
{
	int i;
	assert(W == sW);
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
			continue;
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
			assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_CLRF()
{
	int i, j;
	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			CLRF(0, i);
			test_CLRF_register_is_cleared(i);
			test_WF_zero_flag_set(1,i);
			test_CLRF_not_our_mem_was_untouched(i);
			test_PC_was_increased();	
		}	
}

void test_CLRW_accumulator_is_cleared()
{	
	assert(W == 0);
}

void test_CLRW_not_our_mem_was_untouched()
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
			assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}

}
void test_CLRW()
{
	save_state();	
	CLRW();
	test_CLRW_accumulator_is_cleared();
	test_zero_flag_set();
	test_CLRW_not_our_mem_was_untouched();
	test_PC_was_increased();
}

void test_COMF_was_complemented(unsigned char d,unsigned char f)
{
	if (d)
		assert(REGISTER(mem, f) == (unsigned char)(~REGISTER(smem, f)));
	else
		assert(W == (unsigned char)(~REGISTER(smem, f)));
}


void test_COMF_not_our_mem_was_untouched(unsigned char d,unsigned char f)
{
	int i;

	if (d)
	{
		assert(W == sW);

		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
				continue;

			switch(i)
			{
			case STATUS:
			case STATUS+UNREAl_OFFSET:
				assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
				assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
				assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
				assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
				break;
			default:
				assert(*mem[i] == *smem[i]);
				break;
			}
		}
	}
	else
	{
		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			switch(i)
			{
			case STATUS:
			case STATUS+UNREAl_OFFSET:
				assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
				assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
				assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
				assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
				break;
			default:
				assert(*mem[i] == *smem[i]);
				break;
			}
		}
	}
}

void test_COMF()
{
	int i, j;
	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	// Bank 1

	for (i = 0x20; i <= 0x5F; i++)			// General Purpose Registers
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			COMF(0, i);
			test_COMF_was_complemented(0, i);
			test_WF_zero_flag_set(0, i);
			test_COMF_not_our_mem_was_untouched(0, i);
			test_PC_was_increased();
		}

	for (i = 0x20; i <= 0x5F; i++)			// General Purpose Registers
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			COMF(1, i);
			test_COMF_was_complemented(1, i);
			test_WF_zero_flag_set(1, i);
			test_COMF_not_our_mem_was_untouched(1, i);
			test_PC_was_increased();
		}
}

void test_DECF_was_decremented(unsigned char d, unsigned char f)
{
	if (d==1)
		if (REGISTER(smem, f)==0)
			assert(REGISTER(mem, f) == 0xff);
		else
			assert(REGISTER(mem, f) == (REGISTER(smem, f) - 1));
	else
		if (REGISTER(smem, f)==0)
			assert(W == 0xff);
		else
			assert(W == (REGISTER(mem, f)-1));
}

void test_DECF_not_our_mem_was_untouched(unsigned char d, unsigned char f)
{
	int i;
	if (d == 0)
	{
		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if ((i == STATUS+UNREAl_OFFSET) || (i == STATUS))
				if (W == 0)
					continue;
				else
				{
					assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
					assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
					assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
					assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
					assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
					continue;
				}
			assert(*mem[i] == *smem[i]);
		}
	}
	else
	{
		assert(W == sW);
		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
				continue;
			if ((i == STATUS+UNREAl_OFFSET) || (i == STATUS))
				if (W == 0)
					continue;
				else
				{
					assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
					assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
					assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
					assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
					assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
					continue;
				}
			assert(*mem[i] == *smem[i]);
		}
	}

}

void test_DECF()
{
	int i, j;
	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			DECF(0, i);
			test_DECF_was_decremented(0, i);
			test_WF_zero_flag_set(0, i);
			test_DECF_not_our_mem_was_untouched(0, i);
			test_PC_was_increased();
		}

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			DECF(1, i);
			test_DECF_was_decremented(1, i);
			test_WF_zero_flag_set(1, i);
			test_DECF_not_our_mem_was_untouched(1, i);
			test_PC_was_increased();
		}
}

void test_INCFSZ_was_incremented(unsigned char d, unsigned char f)
{
	if (d)
		assert(REGISTER(mem, f) == (REGISTER(smem, f) + 1) % 256);
	else
		assert(W == (REGISTER(smem, f) + 1) % 256);
}

void test_INCFSZ_not_our_mem_was_untouched(unsigned char d, unsigned char f)
{
	int i;

	if (d == 0)
	{
		for (i = 0; i < DATA_MEM_SIZE; i++)
			assert(*mem[i] == *smem[i]);
	}
	else
	{
		assert(W == sW);

		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
				continue;

			assert(*mem[i] == *smem[i]);
		}
	}
}

void test_INCFSZ_PC_was_increased(unsigned char d, unsigned char f)
{
	unsigned char result = d ? REGISTER(mem, f) : W;

	if (result)
		assert(PC == sPC + 1);
	else
		assert(PC == sPC + 2);
}

void test_DECFSZ()
{
	int i, j;
	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			DECFSZ(0, i);
			test_DECF_was_decremented(0, i);
			test_INCFSZ_not_our_mem_was_untouched(0, i);
			test_INCFSZ_PC_was_increased(0, i);
		}

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			DECFSZ(1, i);
			test_DECF_was_decremented(1, i);
			test_INCFSZ_not_our_mem_was_untouched(1, i);
			test_INCFSZ_PC_was_increased(1, i);
		}
}

void test_INCF()
{
	int i, j;
	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			INCF(0, i);
			test_INCFSZ_was_incremented(0, i);
			test_WF_zero_flag_set(0, i);
			test_DECF_not_our_mem_was_untouched(0, i);
			test_PC_was_increased();
		}

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			INCF(1, i);
			test_INCFSZ_was_incremented(1, i);
			test_WF_zero_flag_set(1, i);
			test_DECF_not_our_mem_was_untouched(1, i);
			test_PC_was_increased();
		}
}

void test_INCFSZ()
{
	int i, j;

	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			INCFSZ(0, i);
			test_INCFSZ_was_incremented(0, i);
			test_INCFSZ_not_our_mem_was_untouched(0, i);
			test_INCFSZ_PC_was_increased(0, i);
		}

	for (i = 0x20; i <= 0x5F; i++)
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			INCFSZ(1, i);
			test_INCFSZ_was_incremented(1, i);
			test_INCFSZ_not_our_mem_was_untouched(1, i);
			test_INCFSZ_PC_was_increased(1, i);
		}
}


void test_IORWF_was_ored(unsigned char i,unsigned char j,unsigned char k,unsigned char d)
{
	if(d==0)
		assert(W == (j|k));
	else
		assert(REGISTER(mem,i) == (j|k));
}


void test_IORWF_not_our_mem_was_untouched(unsigned char d,unsigned char f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
			assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			if(d==1)
			{
				if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
					continue;
			}
			else
				assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_IORWF()
{
	int i, j;
	unsigned char k;

	for (i = 0x20; i <= 0x5F; i++)
		for (j = 0; j <= 127; j++)
			for(k=0;k<=127;k++)
			{
				//Проверка при d=0
				W = j;
				REGISTER(mem,i)= k;
				save_state();

				IORWF(0,i);

				test_IORWF_was_ored(i,j, k,0);
				test_WF_zero_flag_set(0,i);
				test_IORWF_not_our_mem_was_untouched(0, i);
				test_PC_was_increased();

				//Проверка при d=1
				W = j;
				REGISTER(mem,i)= k;
				save_state();

				IORWF(1,i);

				test_IORWF_was_ored(i,j, k,1);
				test_WF_zero_flag_set(1,i);
				test_IORWF_not_our_mem_was_untouched(1, i);
				test_accumulator_was_untouched();
				test_PC_was_increased();
			}
}

void test_MOVF_check_flags(int j)
{
	// проверяем флаг Z
	if (j)
	{ // если j не равно 0, то STATUS_Z должен быть равен 0
		assert(BIT_GET(*mem[STATUS], STATUS_Z) == 0);
	}
	else
	{ // если j равно 0, то STATUS_Z должен быть равен 1
		assert(BIT_GET(*mem[STATUS], STATUS_Z) == 1);
	}

	// остальные флаги не должны изменяться
	assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
	assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
	assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
	assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
	assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
}

// проверка равенства j и значения аккумулятора
void test_MOVF_accumulator_was_changed_right(int j)
{
	assert(W==j);
}

// проверяем равно ли j значение регистра i, а так же его неизменность
void test_MOVF_check_f(int i,int j)
{
	assert(REGISTER(mem, i) == REGISTER(smem, i)); // значение регистра i не изменилось
	assert(REGISTER(mem, i) == j); // значение регистра i равно j
}

// проверка правильности обработки регистра TMR0
void test_check_TMR0()
{
	assert(REGISTER(mem, TMR0) == REGISTER(smem, TMR0)); // значение регистра TMR0 не изменилось
	// проверяем неизменность OPTION_REG[3:7]
	assert(BIT_GET(*mem[OPTION_REG], 3) == BIT_GET(*smem[OPTION_REG], 3)); // значение OPTION_REG[3] неизменилось
	assert(BIT_GET(*mem[OPTION_REG], 4) == BIT_GET(*smem[OPTION_REG], 4)); // значение OPTION_REG[4] неизменилось
	assert(BIT_GET(*mem[OPTION_REG], 5) == BIT_GET(*smem[OPTION_REG], 5)); // значение OPTION_REG[5] неизменилось
	assert(BIT_GET(*mem[OPTION_REG], 6) == BIT_GET(*smem[OPTION_REG], 6)); // значение OPTION_REG[6] неизменилось
	assert(BIT_GET(*mem[OPTION_REG], 7) == BIT_GET(*smem[OPTION_REG], 7)); // значение OPTION_REG[7] неизменилось
	// проверяем сброс предварительного делителя OPTION_REG[0:2]
	if (BIT_GET(*mem[OPTION_REG], 3)){ // если предварительный делитель подключен к WDT, то значения OPTION_REG[0:2] не должны изменяться
		assert(BIT_GET(*mem[OPTION_REG], 0) == BIT_GET(*smem[OPTION_REG], 0)); // значение OPTION_REG[0] неизменилось
		assert(BIT_GET(*mem[OPTION_REG], 1) == BIT_GET(*smem[OPTION_REG], 1)); // значение OPTION_REG[1] неизменилось
		assert(BIT_GET(*mem[OPTION_REG], 2) == BIT_GET(*smem[OPTION_REG], 2)); // значение OPTION_REG[2] неизменилось
	}
	else
	{ // если предварительный делитель подключен к TMR0, то значения OPTION_REG[0:2] сбрасываются (устанавливаются в 1)
		assert(BIT_GET(*mem[OPTION_REG], 0) == 1); // значение OPTION_REG[0] сброшенно
		assert(BIT_GET(*mem[OPTION_REG], 1) == 1); // значение OPTION_REG[1] сброшенно
		assert(BIT_GET(*mem[OPTION_REG], 2) == 1); // значение OPTION_REG[2] сброшенно
	}
}

void test_MOVF()
{
	int i, j;

	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			MOVF(0, i); // копирование значение регистра i в аккумулятор
			test_PC_was_increased(); // проверяем увеличился ли PC
			test_MOVF_accumulator_was_changed_right(j); // проверяем равно ли j значение аккумулятора
			test_MOVF_check_flags(j);// проверка значений флагов
		}

	for (i = 0x20; i <= 0x5F; i++)			// General Purpose Registers
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			MOVF(1, i); // перезапись значения регистра i в регистр i
			if (i==TMR0)
			{ // если перезаписываем значение регистра TMR0
				test_check_TMR0(); // проверяем правильность обработки регистра TMR0
			}
			test_PC_was_increased(); // проверяем увеличился ли PC
			test_MOVF_check_f(i, j); // проверяем равно ли j значение регистра i
			test_MOVF_check_flags(j);// проверка значений флагов
			test_accumulator_was_untouched(); // проверяем что значение аккумулятора не изменилось
		}
}

void test_MOVWF_not_our_mem_was_untouched(unsigned char f)
{
	int i;
	assert(W == sW);
	assert(REGISTER(mem, f) == W);
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
			continue;
		assert(*mem[i] == *smem[i]);
	}
};

void test_MOVWF()
{
	int i, j;
	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			MOVWF(0, i);
			test_MOVWF_not_our_mem_was_untouched(i);
			test_PC_was_increased();
		}
}

void test_mem_was_untouched()
{
	int i;

	for (i = 0; i < DATA_MEM_SIZE; i++)
		assert(*mem[i] == *smem[i]);
}

void test_NOP()
{
	save_state();

	NOP();

	test_accumulator_was_untouched();
	test_mem_was_untouched();
	test_PC_was_increased();
}

void test_RLF_not_our_mem_was_untouched(char d, char f)
{
	int i;

	if (d == 0)
	{
		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if((i==STATUS)||(i==STATUS+UNREAl_OFFSET))
			{
				assert(BIT_GET(*mem[STATUS], STATUS_Z) == BIT_GET(*smem[STATUS], STATUS_Z));
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
				assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
				assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
				assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
				continue;
			}
			assert(*mem[i] == *smem[i]);
		}
	}
	else
	{
		assert(W == sW);

		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if((i==STATUS)||(i==STATUS+UNREAl_OFFSET))
			{
				assert(BIT_GET(*mem[STATUS], STATUS_Z) == BIT_GET(*smem[STATUS], STATUS_Z));
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
				assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
				assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
				assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
				continue;
			}
			else if(i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
				continue;
			assert(*mem[i] == *smem[i]);
		}
	}
}

void test_RLF()
{
	int i, j;

	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			RLF(0, i);
			test_RLF_not_our_mem_was_untouched(0, i);
			test_PC_was_increased();
		}

	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			RLF(1, i);
			test_RLF_not_our_mem_was_untouched(1, i);
			test_PC_was_increased();
		}
}

void test_RRF_not_our_mem_was_untouched(char d, char f)
{
	int i;

	if (d == 0)
	{
		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if((i==STATUS)||(i==STATUS+UNREAl_OFFSET))
			{
				assert(BIT_GET(*mem[STATUS], STATUS_Z) == BIT_GET(*smem[STATUS], STATUS_Z));
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
				assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
				assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
				assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
				continue;
			}
			assert(*mem[i] == *smem[i]);
		}
	}
	else
	{
		assert(W == sW);

		for (i = 0; i < DATA_MEM_SIZE; i++)
		{
			if((i==STATUS)||(i==STATUS+UNREAl_OFFSET))
			{
				assert(BIT_GET(*mem[STATUS], STATUS_Z) == BIT_GET(*smem[STATUS], STATUS_Z));
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
				assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
				assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
				assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
				continue;
			}
			else if(i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
				continue;
			assert(*mem[i] == *smem[i]);
		}
	}
}

void test_RRF()
{
	int i, j;

	BIT_CLEAR(*mem[STATUS], STATUS_RP0);	

	for (i = 0x20; i <= 0x5F; i++)		
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			RRF(0, i);
			test_RRF_not_our_mem_was_untouched(0, i);
			test_PC_was_increased();
		}

	for (i = 0x20; i <= 0x5F; i++)		
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			RRF(1, i);
			test_RRF_not_our_mem_was_untouched(1, i);
			test_PC_was_increased();
		}
}

void test_SUBWF_not_our_mem_was_untouched(int d,int f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			if(d==1){
				if (i == (BIT_GET(*mem[STATUS], STATUS_RP0) ? f+UNREAl_OFFSET : f))
					continue;
			}else
				assert(*mem[i] == *smem[i]);
			break;
		}
	}

}
void test_SUBWF_was_substracted(int i, unsigned char j,unsigned char k,int d)
{
	if(d==0)
		assert(W == (unsigned char)(k-j));
	else
		assert(REGISTER(mem,i) == (unsigned char)(k-j));
}

void test_SUBWF_carry_flags_set(int i, unsigned char j,unsigned char k){
	unsigned char DC= ( (int)(k&0x0f) - (sW&0x0f) )	< 0	? 0:1;
	unsigned char C=  ( (int)(k&0xff) - (sW&0xff) )	< 0	? 0:1;
	assert(BIT_GET(*mem[STATUS],STATUS_C)==C);
	assert(BIT_GET(*mem[STATUS],STATUS_DC)==DC);
}

void test_SUBWF()
{	int i, j,k;

	for (i = 0x20; i <= 0x5F; i++)//General purpose registers
		for (j = 0; j <= 127; j++)
			for(k=0;k<=127;k++){
				//Проверка при d=0
				W = j;
				REGISTER(mem,i)= k;
				save_state();

				SUBWF(0,i);

				test_SUBWF_was_substracted(i,j, k,0);
				test_SUBWF_carry_flags_set(i,j,k);
				test_WF_zero_flag_set(0,i);
				test_SUBWF_not_our_mem_was_untouched(0, i);
				test_PC_was_increased();

				//Проверка при d=1
				W = j;
				REGISTER(mem,i)= k;
				save_state();

				SUBWF(1,i);

				test_SUBWF_was_substracted(i,j, k,1);
				test_SUBWF_carry_flags_set(i,j,k);
				test_WF_zero_flag_set(1,i);
				test_SUBWF_not_our_mem_was_untouched(1, i);
				test_accumulator_was_untouched();
				test_PC_was_increased();
			}
}

void test_SWAPF_CheckWhetherOrNotTheRightValueWasWrittenToFOrW(char d, char f)
{
	unsigned char AVariableThatContainsTheOriginalValueOfTheRegisterInQuestion=REGISTER(smem,f);
	int ABinaryNumberThatIsCOUNTedByMyFunction=Dec2Bin(AVariableThatContainsTheOriginalValueOfTheRegisterInQuestion);
	int TheTailOfTheBasilisk=ABinaryNumberThatIsCOUNTedByMyFunction%10000;
	int TheHeadOfIt=ABinaryNumberThatIsCOUNTedByMyFunction/10000;
	int BinaryBasilisk=Join(TheTailOfTheBasilisk,TheHeadOfIt);
	unsigned char Basilisk=(unsigned char)Bin2Dec(BinaryBasilisk);	
	if (d==1)
		assert(REGISTER(mem,f)==Basilisk);
	if (d==0)
	{
		assert(W==Basilisk);
	}
}

void test_SWAPF_CheckWhetherOrNotAllOtherRegistersAreUnchanged(char d, char f)
{
	int i;
	if (d==0)
	{
		for (i=0;i<DATA_MEM_SIZE;i++)
			assert(*mem[i]==*smem[i]);
	}
	else
	{
		assert(W==sW);
		for (i=0;i<DATA_MEM_SIZE;i++)
		{
			if (i==(BIT_GET(*mem[STATUS],STATUS_RP0) ? f+UNREAl_OFFSET : f))
				continue;
			assert(*mem[i]==*smem[i]);
		}
	}
}

void test_SWAPF()
{
	int i,j;
	BIT_CLEAR(*mem[STATUS],STATUS_RP0);	// Bank 1
	for (i=0x20;i<=0x5F;i++)			// General Purpose Registers
		for (j=0;j<=0xFF;j++)
		{
			REGISTER(mem,i)=j;
			save_state();
			SWAPF(0,i);
			test_SWAPF_CheckWhetherOrNotTheRightValueWasWrittenToFOrW(0,i);
			test_SWAPF_CheckWhetherOrNotAllOtherRegistersAreUnchanged(0,i);
			test_PC_was_increased(0,i);
		}
	for (i=0x20;i<=0x5F;i++)			// General Purpose Registers
		for (j=0;j<=0xFF;j++)
		{
			REGISTER(mem,i)=j;
			save_state();
			SWAPF(1,i);
			test_SWAPF_CheckWhetherOrNotTheRightValueWasWrittenToFOrW(1,i);
			test_SWAPF_CheckWhetherOrNotAllOtherRegistersAreUnchanged(1,i);
			test_PC_was_increased(1,i);
		}
	BIT_SET(*mem[STATUS],STATUS_RP0);		// Bank 2
	for (i=0x20;i<=0x5F;i++)			// General Purpose Registers
		for (j=0;j<=0xFF;j++)
		{
			REGISTER(mem,i)=j;
			save_state();
			SWAPF(0,i);
			test_SWAPF_CheckWhetherOrNotTheRightValueWasWrittenToFOrW(0,i);
			test_SWAPF_CheckWhetherOrNotAllOtherRegistersAreUnchanged(0,i);
			test_PC_was_increased(0,i);
		}
	for (i=0x20;i<=0x5F;i++)			// General Purpose Registers
		for (j=0;j<=0xFF;j++)
		{
			REGISTER(mem,i) = j;
			save_state();
			SWAPF(1,i);
			test_SWAPF_CheckWhetherOrNotTheRightValueWasWrittenToFOrW(1,i);
			//test_SWAPF_CheckWhetherOrNotAllOtherRegistersAreUnchanged(1,i);
			//test_SWAPF_CheckWhetherOrNotProgramCOUNTerWasIncreased(1,i);
		}
}


void test_XORWF_was_xored(unsigned char d, unsigned char f)
{
	if (d==0)
		assert(W == (sW ^ REGISTER(smem, f)));
	else
		assert(REGISTER(mem, f) == (sW ^ REGISTER(smem, f)));
}

void test_XORWF()
{
	int i, j;
	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			XORWF(0,i);
			test_XORWF_was_xored(0,i);
			test_WF_zero_flag_set(0,i);
			test_DECF_not_our_mem_was_untouched(0,i);
			test_PC_was_increased();
		}
	for (i = 0x20; i <= 0x5F; i++)			
		for (j = 0; j <= 0xFF; j++)
		{
			REGISTER(mem, i) = j;
			save_state();
			XORWF(1,i);
			test_XORWF_was_xored(1,i);
			test_WF_zero_flag_set(1,i);
			test_DECF_not_our_mem_was_untouched(1,i);
			test_PC_was_increased();
		}
}

void test_BCF()
{
	int i,j;
	for (i = 0x20; i <= 0x5F; i++)
	{
		for (j=0;j<8;j++)
		{
			save_state();	
			BCF(j,i);
			if (BIT_GET(REGISTER(smem,i), j)==1)
				assert(BIT_GET(REGISTER(mem,i), j)==0);
			else
				assert(BIT_GET(REGISTER(mem,i), j)==0);
			test_PC_was_increased();
			test_INCFSZ_not_our_mem_was_untouched(1,i);
		}
	}
}

void test_BSF()
{
	int i,j;
	for (i = 0x20; i <= 0x5F; i++)
	{
		for (j=0;j<8;j++)
		{
			save_state();	
			BSF(j,i);
			if (BIT_GET(REGISTER(smem,i), j)==0)
				assert(BIT_GET(REGISTER(mem,i), j)==1);
			else
				assert(BIT_GET(REGISTER(mem,i), j)==1);
			test_PC_was_increased();
			test_INCFSZ_not_our_mem_was_untouched(1,i);
		}
	}
}

void test_PC_was_increased_twice()
{
	assert(PC == sPC + 2);
}

void test_BTFSC()
{
	int f,b;

	for (f = 0x20; f <= 0x5F; f++)
	{
		for (b=0;b<8;b++)
		{
			save_state();	
			BTFSC(b,f);

			if BIT_GET(*mem[f], b)
 			{
				test_PC_was_increased();
			}
			else
			{
				test_PC_was_increased_twice();

			}

		}
	}


	BIT_SET(*mem[STATUS], STATUS_RP0);

	for (f = 0x20; f <= 0x5F; f++)
	{
		for (b=0;b<8;b++)
		{
			save_state();	
			BTFSC(b,f);

			if BIT_GET(*mem[f], b)
 			{
				test_PC_was_increased();	
			}
			else
			{
				test_PC_was_increased_twice();
			}

		}
	}
}

void test_BTFSS()
{
	int f,b;

	for (f = 0x20; f <= 0x5F; f++)
	{
		for (b=0;b<8;b++)
		{
			save_state();	
			BTFSS(b,f);

			if BIT_GET(*mem[f], b)
 			{
				test_PC_was_increased_twice();
			}
			else
			{
				test_PC_was_increased();
			}

		}
	}


	BIT_SET(*mem[STATUS], STATUS_RP0);

	for (f = 0x20; f <= 0x5F; f++)
	{
		for (b=0;b<8;b++)
		{
			save_state();	
			BTFSS(b,f);

			if BIT_GET(*mem[f], b)
 			{
				test_PC_was_increased_twice();
			}
			else
			{
				test_PC_was_increased();
			}

		}
	}
}

void test_ADDLW_not_our_mem_was_untouched()
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_ADDLW()
{
	unsigned char k;
	int i, j;
	for ( j=0; j<255; ++j)
		for ( i=0; i<=255; ++i )
		{
			W = (unsigned char)j;
			k = (unsigned char)i;
			save_state();	
			ADDLW(k);
			assert( (unsigned char)W == (unsigned char)(sW + k) );
			test_zero_flag_set();
			test_ADDLW_not_our_mem_was_untouched();
			test_PC_was_increased();
			if ( (255 - sW) < k) 
				assert(BIT_GET(*mem[STATUS], STATUS_C) == 1); 
			else
				assert(BIT_GET(*mem[STATUS], STATUS_C) == 0); 
			if ( (255 - (unsigned char)(sW << 4)) < (unsigned char)(k << 4) )
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == 1);
			else
				assert(BIT_GET(*mem[STATUS], STATUS_DC) == 0); // FLAGS &= ~(1 << 1);
		}
}

void test_ANDLW_was_anded(unsigned char k)
{
	assert((unsigned char)W== (unsigned char)(k&(unsigned char)sW));
}

void test_ANDLW_not_our_mem_was_untouched(int f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_ANDLW()
{
	unsigned int W_i,k,i;
	for(i=0x20;i<=0x5F;i++)
	{
		for(W_i=0;W_i<0x100;W_i++)
		{
			for(k=0;k<0x100;k++)
			{
				W= W_i;

				save_state();

				ANDLW(k);

				test_ANDLW_was_anded(k);
				test_zero_flag_set();
				test_ANDLW_not_our_mem_was_untouched(i);
			}
		}
	}
}

void test_CALL()
{
	// insert code here...
}

void test_GOTO()
{
	/*
	int testPC=0;
    int adr=124;
	assert ((adr>=0)&&(adr<=8191));
	testPC=adr;
    testPC++;
	GOTO(adr);
	assert(PC==testPC);
	*/
}

void test_IORLW_was_ored(unsigned char k)
{
	assert((unsigned char)W== (unsigned char)(k|(unsigned char)sW));
}

void test_IORLW_not_our_mem_was_untouched(int f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}

}
void test_IORLW()
{
	unsigned int W_i,k,i;
	for(i=0x20;i<=0x5F;i++)
	{
		for(W_i=0;W_i<0x100;W_i++)
		{
			for(k=0;k<0x100;k++)
			{
				W= W_i;

				save_state();

				IORLW(k);

				test_IORLW_was_ored(k);
				test_zero_flag_set();
				test_IORLW_not_our_mem_was_untouched(i);
			}
		}
	}
}

void test_MOVLW_k_was_moved(unsigned char k)
{
	assert(W == k);
}

void test_MOVLW()
{
	int i, j;

	for (i = 0; i <= 0xFF; i++)
		for (j = 0; j <= 0xFF; j++)
		{
			save_state();
			W = i;
			MOVLW(j);
			test_MOVLW_k_was_moved(j);
			test_mem_was_untouched;
			test_PC_was_increased();
		}
}

void test_RETFIE()
{
	// insert code here...
}

void test_RETLW()
{
	// insert code here...
}

void test_RETURN()
{
	// insert code here...
}

void test_SUBLW_substracted(unsigned char k)
{
	assert((unsigned char)W== (unsigned char)(k-(unsigned char)sW));
}

void test_SUBLW_carry_flags_set(unsigned char k)
{
	unsigned char DC= ( (int)(k&0x0f) - (sW&0x0f) )	< 0	? 0:1;
	unsigned char C=  ( (int)(k&0xff) - (sW&0xff) )	< 0	? 0:1;
	assert(BIT_GET(*mem[STATUS],STATUS_C)==C);
	assert(BIT_GET(*mem[STATUS],STATUS_DC)==DC);
}

void test_SUBLW_not_our_mem_was_untouched(int f)
{
	int i;
	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}

}

void test_SUBLW()
{
	unsigned int W_i,k,i;
	for(i=0x20;i<=0x5F;i++){
		for(W_i=0;W_i<0x100;W_i++)
		{
			for(k=0;k<0x100;k++)
			{
				W= W_i;

				save_state();

				SUBLW(k);

				test_SUBLW_substracted(k);
				test_zero_flag_set();
				test_SUBLW_carry_flags_set(k);
				test_SUBLW_not_our_mem_was_untouched(i);
			}
		}
	}
}

void test_XORLW_was_xored(int i, int j)
{
	assert(W == (i^j));
}

void test_XORLW_not_our_mem_was_untouched()
{
	int i;

	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		switch(i)
		{
		case STATUS:
		case STATUS+UNREAl_OFFSET:
			assert(BIT_GET(*mem[STATUS], STATUS_C) == BIT_GET(*smem[STATUS], STATUS_C));
			assert(BIT_GET(*mem[STATUS], STATUS_DC) == BIT_GET(*smem[STATUS], STATUS_DC));
			assert(BIT_GET(*mem[STATUS], STATUS_PD) == BIT_GET(*smem[STATUS], STATUS_PD));
			assert(BIT_GET(*mem[STATUS], STATUS_TO) == BIT_GET(*smem[STATUS], STATUS_TO));
			assert(BIT_GET(*mem[STATUS], STATUS_RP0) == BIT_GET(*smem[STATUS], STATUS_RP0));
			break;
		default:
			assert(*mem[i] == *smem[i]);
			break;
		}
	}
}

void test_XORLW()
{
	int i, j;

	for (i = 0; i <= 0xFF; i++)
		for (j = 0; j <= 0xFF; j++)
		{
			save_state();

			W = i;
			XORLW(j);

			test_XORLW_was_xored(i, j);
			test_zero_flag_set();
			test_XORLW_not_our_mem_was_untouched();
			test_PC_was_increased();
		}
}

void test_all()
{
	test_ADDWF();
	test_ANDWF();
	test_CLRF();
	test_CLRW();
	test_COMF();
	test_DECF();
	test_DECFSZ();
	test_INCF();
	test_INCFSZ();
	test_IORWF();
	test_MOVF();
	test_MOVWF();
	test_NOP();
	test_RLF();
	test_RRF();
	test_SUBWF();
	test_SWAPF();
	test_XORWF();
	test_BCF();
	test_BSF();
	test_BTFSC();
	test_BTFSS();
	test_ADDLW();
	test_ANDLW();
	test_CALL();
	test_GOTO();
	test_IORLW();
	test_MOVLW();
	test_RETFIE();
	test_RETLW();
	test_RETURN();
	test_SUBLW();
	test_XORLW();
}

void init()
{
	int i;

	for (i = 0; i < DATA_MEM_SIZE; i++)
	{
		mem[i] = &real_mem[i];
		real_mem[i] = 0;
	}

	mem[UNREAl_OFFSET+PCL]		= mem[PCL];
	mem[UNREAl_OFFSET+STATUS]	= mem[STATUS];
	mem[UNREAl_OFFSET+FSR]		= mem[FSR];
	mem[UNREAl_OFFSET+PCLATH]	= mem[PCLATH];
	mem[UNREAl_OFFSET+INTCON]	= mem[INTCON];

	BIT_SET(*mem[STATUS], STATUS_PD);
	BIT_SET(*mem[STATUS], STATUS_TO);

	PC = 0;

	for (i = 0; i < DATA_MEM_SIZE; i++)
		smem[i] = &sreal_mem[i];

	smem[UNREAl_OFFSET+PCL]		= smem[PCL];
	smem[UNREAl_OFFSET+STATUS]	= smem[STATUS];
	smem[UNREAl_OFFSET+FSR]		= smem[FSR];
	smem[UNREAl_OFFSET+PCLATH]	= smem[PCLATH];
	smem[UNREAl_OFFSET+INTCON]	= smem[INTCON];
}

char ascii_to_dec(char x)
{
	if (x >= '0' && x <= '9')
		return x - 48;

	if (x >= 'A' && x <= 'F')
		return x - 55;

	return 0;
}

unsigned char get_byte(FILE *f)
{
	char h = ascii_to_dec(fgetc(f));
	char l = ascii_to_dec(fgetc(f));

	return (h << 4) | l;
}

int load_hex_file(const char *file_name)
{
	int i;
	unsigned offset;
	unsigned char buf[260];
	unsigned char checksum;

	FILE *f = fopen(file_name, "r");

	if (!f)
		return 1;

	while (1)
	{
		int c = fgetc(f);

		if (c == EOF)
		{
			fclose(f);
			return 0;
		}

		if ((c == '\r') || (c == '\n'))
			continue;

		if (c != ':')
		{
			fclose(f);
			return 2;
		}

		buf[0] = get_byte(f);
		buf[1] = get_byte(f);
		buf[2] = get_byte(f);
		buf[3] = get_byte(f);

		for (i = 0; i <= buf[0]; i++)
			buf[4 + i] = get_byte(f);

		for (i = 0; i <= 4 + buf[0]; i++)
			checksum += buf[i];

		if (checksum)
		{
			fclose(f);
			return 3;
		}

		switch (buf[3])
        {
        case 0:								// Data Record
			offset = (buf[1] << 8) | buf[2];

			if (offset >= PROG_MEM_SIZE)
				break;

			for (i = 0; i < buf[0]; i+=2)
				prog[offset++] = (buf[i+5] << 8) | buf[i+4];

			break;
		case 1:								// End of File Record
			return 0;
		case 2:								// Extended Segment Address Record
			break;
		case 3:								// Start Segment Address Record
			break;
		case 4:								// Extended Linear Address Record
			break;
		case 5:								// Start Linear Address Record
			break;
		}
	}

	fclose(f);
	return 0;
}

void byte_oriented_file_register_operation(void (*func)(unsigned char, unsigned char))
{
	unsigned char d;	// d = 0 for destination W
						// d = 1 for destination f
	unsigned char f;	// 7-bit file register address

	d = BIT_GET(prog[PC], 7);
	f = GET_LOW_7_BITS(prog[PC]);

	func(d, f);
}

void bit_oriented_file_register_operation(void (*func)(unsigned char, unsigned char))
{
	unsigned char b;		// 3-bit bit address 
	unsigned char f;		// 7-bit file register address

	b = GET_BITS_9_8_7(prog[PC]);
	f = GET_LOW_7_BITS(prog[PC]);

	func(b, f);
}

void literal_or_control_operation(void (*func)(unsigned char))
{
	unsigned char k;	// 8-bit immediate value

	k = GET_LOW_8_BITS(prog[PC]);

	func(k);
}

void call_or_goto_instruction(void (*func)(short))
{
	short k;	// 11-bit immediate value

	k = GET_LOW_11_BITS(prog[PC]);

	func(k);
}

void run()
{
	switch(BIT_GET(prog[PC], 13))
	{
	case 0:															// 0xxxxxxxxxxxxx
		switch(BIT_GET(prog[PC], 12))
		{
		case 0:														// 00xxxxxxxxxxxx
			switch(BIT_GET(prog[PC], 11))
			{
			case 0:													// 000xxxxxxxxxxx
				switch(BIT_GET(prog[PC], 10))
				{
				case 0:												// 0000xxxxxxxxxx
					switch(BIT_GET(prog[PC], 9))
					{
					case 0:											// 00000xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 000000xxxxxxxx
							switch(BIT_GET(prog[PC], 7))
							{
							case 0:									// 0000000xxxxxxx
								switch(BIT_GET(prog[PC], 6))
								{
								case 0:								// 00000000xxxxxx
									switch(BIT_GET(prog[PC], 5))
									{
									case 0:							// 000000000xxxxx
										switch(BIT_GET(prog[PC], 4))
										{
										case 0:						// 0000000000xxxx
											switch(BIT_GET(prog[PC], 3))
											{
											case 1:					// 00000000001xxx
												switch(BIT_GET(prog[PC], 2))
												{
												case 0:				// 000000000010xx
													switch(BIT_GET(prog[PC], 1))
													{
													case 0:			// 0000000000100x
														switch(BIT_GET(prog[PC], 0))
														{
														case 0:		// 00000000001000
															RETURN();
															break;
														case 1:		// 00000000001001
															RETFIE();
															break;
														}
														break;
													}
													break;
												}
												break;
											}
											break;
										}
										break;
									}
									break;
								case 1:								// 00000001xxxxxx
									switch(BIT_GET(prog[PC], 5))
									{
									case 1:							// 000000011xxxxx
										switch(BIT_GET(prog[PC], 4))
										{
										case 0:						// 0000000110xxxx
											switch(BIT_GET(prog[PC], 3))
											{
											case 0:					// 00000001100xxx
												switch(BIT_GET(prog[PC], 2))
												{
												case 0:				// 000000011000xx
													switch(BIT_GET(prog[PC], 1))
													{
													case 1:			// 0000000110001x
														switch(BIT_GET(prog[PC], 0))
														{
														case 1:		// 00000001100011
															SLEEP();
															break;
														}
														break;
													}
													break;
												case 1:				// 000000011001xx
													switch(BIT_GET(prog[PC], 1))
													{
													case 0:			// 0000000110010x
														switch(BIT_GET(prog[PC], 0))
														{
														case 0:		// 00000001100100
															CLRWDT();
															break;
														}
														break;
													}
													break;
												}
												break;
											}
											break;
										}
										break;
									}
									break;
								}

								switch(BIT_GET(prog[PC], 4))
								{
								case 0:								// 0000000xx0xxxx
									switch(BIT_GET(prog[PC], 3))
									{
									case 0:							// 0000000xx00xxx
										switch(BIT_GET(prog[PC], 2))
										{
										case 0:						// 0000000xx000xx
											switch(BIT_GET(prog[PC], 1))
											{
											case 0:					// 0000000xx0000x
												switch(BIT_GET(prog[PC], 0))
												{
												case 0:				// 0000000xx00000
													NOP();
													break;	
												}
												break;
											}
											break;
										}
										break;
									}
									break;
								}
								break;

							case 1:									// 0000001xxxxxxx
								byte_oriented_file_register_operation(MOVWF);
								break;
							}
							break;
						case 1:										// 000001xxxxxxxx
							switch(BIT_GET(prog[PC], 7))
							{
							case 0:									// 0000010xxxxxxx
								CLRW();
								break;
							case 1:									// 0000011xxxxxxx
								byte_oriented_file_register_operation(CLRF);
								break;
							}
							break;
						}
						break;
					case 1:											// 00001xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 000010xxxxxxxx
							byte_oriented_file_register_operation(SUBWF);
							break;
						case 1:										// 000011xxxxxxxx
							byte_oriented_file_register_operation(DECF);
							break;
						}
						break;
					}
					break;
				case 1:												// 0001xxxxxxxxxx
					switch(BIT_GET(prog[PC], 9))
					{
					case 0:											// 00010xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 000100xxxxxxxx
							byte_oriented_file_register_operation(IORWF);
							break;
						case 1:										// 000101xxxxxxxx
							byte_oriented_file_register_operation(ANDWF);
							break;
						}
						break;
					case 1:											// 00011xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 000110xxxxxxxx
							byte_oriented_file_register_operation(XORWF);
							break;
						case 1:										// 000111xxxxxxxx
							byte_oriented_file_register_operation(ADDWF);
							break;
						}
						break;
					}
					break;
				}
				break;
			case 1:													// 001xxxxxxxxxxx
				switch(BIT_GET(prog[PC], 10))		
				{
				case 0:												// 0010xxxxxxxxxx
					switch(BIT_GET(prog[PC], 9))
					{
					case 0:											// 00100xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 001000xxxxxxxx
							byte_oriented_file_register_operation(MOVF);
							break;
						case 1:										// 001001xxxxxxxx
							byte_oriented_file_register_operation(COMF);
							break;
						}
						break;
					case 1:											// 00101xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 001010xxxxxxxx
							byte_oriented_file_register_operation(INCF);
							break;
						case 1:										// 001011xxxxxxxx
							byte_oriented_file_register_operation(DECFSZ);
							break;
						}
						break;
					}
					break;
				case 1:												// 0011xxxxxxxxxx
					switch(BIT_GET(prog[PC], 9))
					{
					case 0:											// 00110xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 001100xxxxxxxx
							byte_oriented_file_register_operation(RRF);
							break;
						case 1:										// 001101xxxxxxxx
							byte_oriented_file_register_operation(RLF);
							break;
						}
						break;
					case 1:											// 00111xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 001110xxxxxxxx
							byte_oriented_file_register_operation(SWAPF);
							break;
						case 1:										// 001111xxxxxxxx
							byte_oriented_file_register_operation(INCFSZ);
							break;
						}
						break;
					}
					break;
				}
				break;
			}
			break;
		case 1:														// 01xxxxxxxxxxxx
			switch(BIT_GET(prog[PC], 11))
			{
			case 0:													// 010xxxxxxxxxxx
				switch(BIT_GET(prog[PC], 10))		
				{
				case 0:												// 0100xxxxxxxxxx
					bit_oriented_file_register_operation(BCF);
					break;
				case 1:												// 0101xxxxxxxxxx
					bit_oriented_file_register_operation(BSF);
					break;
				}
				break;
			case 1:													// 011xxxxxxxxxxx
				switch(BIT_GET(prog[PC], 10))		
				{
				case 0:												// 0110xxxxxxxxxx
					bit_oriented_file_register_operation(BTFSC);
					break;
				case 1:												// 0111xxxxxxxxxx
					bit_oriented_file_register_operation(BTFSS);
					break;
				}
				break;
			}
			break;
		}
		break;
	case 1:															// 1xxxxxxxxxxxxx
		switch(BIT_GET(prog[PC], 12))
		{
		case 0:														// 10xxxxxxxxxxxx
			switch(BIT_GET(prog[PC], 11))
			{
			case 0:													// 100xxxxxxxxxxx
				call_or_goto_instruction(CALL);
				break;
			case 1:													// 101xxxxxxxxxxx
				call_or_goto_instruction(GOTO);
				break;
			}
			break;
		case 1:														// 11xxxxxxxxxxxx
			switch(BIT_GET(prog[PC], 11))
			{
			case 0:													// 110xxxxxxxxxxx
				switch(BIT_GET(prog[PC], 10))		
				{
				case 0:												// 1100xxxxxxxxxx
					literal_or_control_operation(MOVLW);
					break;
				case 1:												// 1101xxxxxxxxxx
					literal_or_control_operation(RETLW);
					break;
				}
				break;
			case 1:													// 111xxxxxxxxxxx
				switch(BIT_GET(prog[PC], 10))		
				{
				case 0:												// 1110xxxxxxxxxx
					switch(BIT_GET(prog[PC], 9))
					{
					case 0:											// 11100xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 111000xxxxxxxx
							literal_or_control_operation(IORLW);
							break;
						case 1:										// 111001xxxxxxxx
							literal_or_control_operation(ANDLW);
							break;
						}
						break;
					case 1:											// 11101xxxxxxxxx
						switch(BIT_GET(prog[PC], 8))
						{
						case 0:										// 111010xxxxxxxx
							literal_or_control_operation(XORLW);
							break;
						}
						break;
					}
					break;
				case 1:												// 1111xxxxxxxxxx
					switch(BIT_GET(prog[PC], 9))
					{
					case 0:											// 11110xxxxxxxxx
						literal_or_control_operation(SUBLW);
						break;
					case 1:											// 11111xxxxxxxxx
						literal_or_control_operation(ADDLW);
						break;
					}
					break;
				}
				break;
			}
			break;
		}
		break;
	}
}

void analyze_changes()
{
	int i;

	if (W != sW)
		printf("\tW = %d\n", W);

	for (i = 0; i < DATA_MEM_SIZE; i++)
		if (*mem[i] != *smem[i])
		{
			int n;
			char x = *mem[i];

			switch(i)
			{
			case STATUS:
				printf("\tSTATUS = ");
				break;
			case STATUS+UNREAl_OFFSET:
				continue;
			default:
				printf("\t%02Xh = ", i);
			}

			for(n = 0; n < 8; n++)
			{
				if (x & 0x80) printf("1");
				else printf("0");
				x <<= 1;
			}

			printf("\n");
		}
}

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		printf("Usage: %s hex_file_name\n", argv[0]);
		return 1;
	}

	init();

	//test_all();
	//init();

	if (load_hex_file(argv[1]))
		return 2;

	while (1)
	{
		save_state();
		run();
		getchar();
		analyze_changes();
	}

    return 0;
}
