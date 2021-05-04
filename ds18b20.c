/*
 * Термометр  на STM8,
 * DS18B20 
 * і 4-розрядному 7-сегментному індикаторі
 */

//Піни 7-сегментного індиктора
#define Y1_PORT PD_ODR
#define Y1_PIN PIN4
#define Y2_PORT PA_ODR
#define Y2_PIN PIN1
#define Y3_PORT PA_ODR
#define Y3_PIN PIN2
#define Y4_PORT PC_ODR
#define Y4_PIN PIN3
#define A_PORT PD_ODR
#define A_PIN PIN5
#define B_PORT PA_ODR
#define B_PIN PIN3
#define C_PORT PC_ODR
#define C_PIN PIN5
#define D_PORT PC_ODR
#define D_PIN PIN7
#define E_PORT PD_ODR
#define E_PIN PIN1
#define F_PORT PD_ODR
#define F_PIN PIN6
#define G_PORT PC_ODR
#define G_PIN PIN4
#define DP_PORT PC_ODR
#define DP_PIN PIN6

//таблиця символів для 7-сегментного індикатора
#define ONE   0b01100000
#define TWO   0b11011010
#define THREE 0b11110010
#define FOUR  0b01100110
#define FIVE  0b10110110
#define SIX   0b10111110
#define SEVEN 0b11100000
#define EIGHT 0b11111110
#define NINE  0b11100110
#define ZERO  0b11111100
#define DOT   0b00000001
#define DEG   0b11000110
#define DDEG  0b11010110
#define MINUS 0b00000010
#define CHRE  0b10011110
#define CHRR  0b00001010
#define CHRS  0b10110110
#define CHRA  0b11101110
#define CHRV  0b00111000
#define SPACE 0b00000000
#define UP    0b00000100
#define DOWN  0b00001000

char numbers[] = {ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE};

#include <stdint.h>
#include "stm8.h"

/* Пін 1-Wire data (перший датчик DS18B20) */
#define OW1_PORT PB
#define OW1_PIN  PIN4
/* Пін 1-Wire data (другий датчик DS18B20) */
#define OW2_PORT PB
#define OW2_PIN  PIN5 

int8_t step = 0;//номер поточної ітерації (потрібен для збереження історії раз на кілька ітерацій)
int8_t t1_rate = 0;//ознака зміни t1: >0 - зростання, <0 - спадання, 0 - стала температура
int8_t t2_rate = 0;//ознака зміни t2: >0 - зростання, <0 - спадання, 0 - стала температура

#define HISTORY_SIZE 10 //кількість показів у історії
int16_t t1_history [HISTORY_SIZE] = {0,0,0,0,0,0,0,0,0,0};//історія  першого датчика
int16_t t2_history [HISTORY_SIZE] = {0,0,0,0,0,0,0,0,0,0};//історія  другого датчика
uint8_t t1_index = 0;//поточний індекс в історії t1
uint8_t t2_index = 0;//поточний індекс в історії t2

void debug(int code);//виведення числа на 7-сегментний індикатор (для  цілей зневадження)
void printSave();//виведення тексту "SAVE" під час збереження значення до історії

/*
 * Збереження t1 до історії
 * і оцінка тенденції зміни температури
 */
void log_t1(int16_t t)
{
	int16_t sum = 0;
	uint8_t weight = 10;
	int8_t current = t1_index;
	//розрахунок середньозваженого
	for (uint8_t i =0; i<HISTORY_SIZE; i++){
		current--;
		if (current<0)
			current = HISTORY_SIZE-1;
		sum += t1_history[current] * weight / 10;
		weight--; 
	}
	sum = sum * 2 / 11;
	//встановлення індикатора зміни температури: зростання, спадання або незмінна
	if ((t - sum)>10)
		t1_rate = 1;
	else if((t - sum)<-10)
		t1_rate = -1;
	else
		t1_rate = 0;
	//Збереження останнього значення
	t1_history [t1_index++] = t;
	if (t1_index == 10)
		t1_index = 0;
	printSave();
}

/*
 * Збереження t2 до історії
 * і оцінка тенденції зміни температури
 */
void log_t2(int16_t t)
{
	int16_t sum = 0;
	uint8_t weight = 10;
	int8_t current = t2_index;
	//розрахунок середньозваженого
	for (uint8_t i =0; i<HISTORY_SIZE; i++){
		current--;
		if (current<0)
			current = HISTORY_SIZE-1;
		sum += t2_history[current] * weight / 10;
		weight--; 
	}
	sum = sum * 2 / 11;
	//встановлення індикатора зміни температури: зростання, спадання або незмінна
	if ((t - sum)>10)
		t2_rate = 1;
	else if((t - sum)<-10)
		t2_rate = -1;
	else
		t2_rate = 0;
	//Збереження останнього значення
	t2_history [t2_index++] = t;
	if (t2_index == 10)
		t2_index = 0;
	printSave();
}

void delay_us(uint16_t i);

/* Пауза  */
void delay(unsigned long count) {
    while (count--)
        nop();
}

/*
 * Ініціалізація портів 7-сегментного індикатора
 */
void setup_display()
{
	PA_DDR |= PIN1 | PIN2 | PIN3;
    PA_CR1 |= PIN1 | PIN2 | PIN3;
    
    PC_DDR |= PIN3 | PIN4 | PIN5 | PIN6 | PIN7;
    PC_CR1 |= PIN3 | PIN4 | PIN5 | PIN6 | PIN7;
    
    PD_DDR |= PIN1 | PIN4 | PIN5 | PIN6;
    PD_CR1 |= PIN1 | PIN4 | PIN5 | PIN6;
}

/*
 * Виведення одного символа
 */
void printChar(char c)
{
	A_PORT  &= ~A_PIN; // A low
	B_PORT  &= ~B_PIN; // B low
	C_PORT  &= ~C_PIN; // C low
	D_PORT  &= ~D_PIN; // D low
	E_PORT  &= ~E_PIN; // E low
	F_PORT  &= ~F_PIN; // F low
	G_PORT  &= ~G_PIN; // G low
	DP_PORT &= ~DP_PIN;// DP low
	if ( (c>>7) & 1 )
		A_PORT |= A_PIN; // A high
	if ( (c>>6) & 1 )
		B_PORT |= B_PIN; // B high
	if ( (c>>5) & 1 )
		C_PORT |= C_PIN; // C high
	if ( (c>>4) & 1 )
		D_PORT |= D_PIN; // D high
	if ( (c>>3) & 1 )
		E_PORT |= E_PIN; // E high
	if ( (c>>2) & 1 )
		F_PORT |= F_PIN; // F high
	if ( (c>>1) & 1 )
		G_PORT |= G_PIN; // G high
	if ( (c>>0) & 1 )
		DP_PORT |= DP_PIN; // A high
	delay_us(32);
}

/*
 * Встановити поточний розряд
 */
void setDigit(char d)
{
	delay_us(32);
	Y1_PORT |= Y1_PIN; // Y1 high
	Y2_PORT |= Y2_PIN; // Y2 high
	Y3_PORT |= Y3_PIN; // Y3 high
	Y4_PORT |= Y4_PIN; // Y4 high
	printChar(SPACE);
	delay_us(8);
	if (d==0)
		Y1_PORT &= ~Y1_PIN; // Y1 low
	else if (d==1)
		Y2_PORT &= ~Y2_PIN; // Y2 low
	else if (d==2)
		Y3_PORT &= ~Y3_PIN; // Y3 low
	else if (d==3)
		Y4_PORT &= ~Y4_PIN; // Y1 low
}

/*
 * Виведення повідомлення ErrN
 * (наприклад, коли датчик не знайдено)
 */
void printErr(char code)
{
	for (uint16_t i=0; i<1000; i++){
		setDigit(0);
		printChar(CHRE);
		setDigit(1);
		printChar(CHRR);
		setDigit(2);
		printChar(CHRR);
		setDigit(3);
		if(code<10)
			printChar(numbers[code]);
	}
}

/*
 * Виведення тексту "SAVE"
 */
void printSave()
{
	for (uint16_t i=0; i<5000; i++){
		setDigit(0);
		printChar(CHRS);
		setDigit(1);
		printChar(CHRA);
		setDigit(2);
		printChar(CHRV);
		setDigit(3);
		printChar(CHRE);
	}
}

/*
 * Виведення довільного числа (для зневадження)
 */
void debug(int code)
{
	if (code>9999)
		return;
	for (uint16_t i=0; i<1000; i++){
		setDigit(0);
		printChar(numbers[code/1000]);
		setDigit(1);
		printChar(numbers[code/100%10]);
		setDigit(2);
		printChar(numbers[code/10%10]);
		setDigit(3);
		printChar(numbers[code%10]);
	}
}

/*
 * Виведення температури на індикатор
 */
void display_temperature(uint16_t t, uint8_t is_negative, uint16_t times, uint8_t ow_num)
{
	int16_t tmp = (is_negative) ? -t : t;
	if(!step){
		if (ow_num == 1)
			log_t1(tmp);
		else
			log_t2(tmp);
		}
	//Зниження тактової частоти
	CLK_CKDIVR = 0b11001;
	if (t>999)
		t=999;
	char first = 0;
	char rate =0;
	if(is_negative){
			first = MINUS;
		}
		else{
			first = SPACE;
		}
	//обрання індикатора підвищення чи зниження температури
	if ( ((ow_num == 1)&&(t1_rate>0)) || ((ow_num == 2)&&(t2_rate>0)) )
		rate = UP;
	else if ( ((ow_num == 1)&&(t1_rate<0)) || ((ow_num == 2)&&(t2_rate<0)) )
		rate = DOWN;
	else 
		rate = SPACE;
	if ((t%10)>=5)
		t+=10;
	//Виведення температури
	for (uint16_t i=0; i<times; i++){
		setDigit(0);
		if((i%50)<40)
			printChar(first);
		else
			printChar(first|rate);
		setDigit(1);
		printChar(numbers[t/100%10]);
		setDigit(2);
		printChar(numbers[t/10%10]);
		setDigit(3);
		(ow_num==1) ? printChar(DEG) : printChar(DDEG);
	}
	CLK_CKDIVR = 0;//Підвищення тактової частоти
}

/********************** OneWire/DS18B20 routines ***************************/
void delay_us(uint16_t i) {
    if (i < 9) { // FIXME: Really ugly
        nop();
        return;
    }
    TIM2_CNTRH = 0;
    TIM2_CNTRL = 0;
    TIM2_EGR = 0x01; // Update Generation
    while(1) {
        volatile uint16_t counter = (((TIM2_CNTRH) << 8) | TIM2_CNTRL);
        if (i-6 < counter)
            return;
    }
}

#define OW1_INPUT_MODE()     PORT(OW1_PORT,DDR) &= ~OW1_PIN
#define OW1_OUTPUT_MODE()    PORT(OW1_PORT,DDR) |= OW1_PIN
#define OW1_LOW()            PORT(OW1_PORT,ODR) &= ~OW1_PIN
#define OW1_HIGH()           PORT(OW1_PORT,ODR) |= OW1_PIN
#define OW1_READ()           (PORT(OW1_PORT,IDR) & OW1_PIN)

#define OW2_INPUT_MODE()     PORT(OW2_PORT,DDR) &= ~OW2_PIN
#define OW2_OUTPUT_MODE()    PORT(OW2_PORT,DDR) |= OW2_PIN
#define OW2_LOW()            PORT(OW2_PORT,ODR) &= ~OW2_PIN
#define OW2_HIGH()           PORT(OW2_PORT,ODR) |= OW2_PIN
#define OW2_READ()           (PORT(OW2_PORT,IDR) & OW2_PIN)

void ow1_pull_low(unsigned int us) {
    OW1_OUTPUT_MODE();
    OW1_LOW();
    delay_us(us);
    OW1_INPUT_MODE();
}

void ow2_pull_low(unsigned int us) {
    OW2_OUTPUT_MODE();
    OW2_LOW();
    delay_us(us);
    OW2_INPUT_MODE();
}

void ow_write_byte(uint8_t out, uint8_t ow_num) {
    uint8_t i;
    for (i=0; i < 8; i++) {
        if ( out & ((uint8_t)1<<i) ) {
            // write 1
            (ow_num==1) ? ow1_pull_low(1) : ow2_pull_low(1);
            delay_us(60);
        } else {
            // write 0
            (ow_num==1) ? ow1_pull_low(60) : ow2_pull_low(60);
            delay_us(1);
        }
    }
}

uint8_t ow_read_byte(uint8_t ow_num) {
    uint8_t val = 0;
    uint8_t i;
    for (i=0; i < 8; i++) {
        (ow_num==1) ? ow1_pull_low(1) : ow2_pull_low(1);
        delay_us(5);
        if ((ow_num==1) ? OW1_READ() : OW2_READ()) {
            val |= ((uint8_t)1<<i);
        }
        delay_us(55);
    }
    return val;
}

unsigned int ow_init(uint8_t ow_num) {

    uint8_t input;

    (ow_num==1) ? ow1_pull_low(480) : ow2_pull_low(480);
    delay_us(60);

    input = (ow_num==1) ? !OW1_READ() : !OW2_READ();
    delay_us(420);

    return input;
}

unsigned int ow_convert_temperature(uint8_t ow_num) {
    int cycles = 1; // For debugging purposes

    ow_write_byte(0x44, ow_num); // Convert Temperature

    while (1) {
        (ow_num==1) ? ow1_pull_low(1) : ow2_pull_low(1);
        delay_us(5);
        if ((ow_num==1) ? OW1_READ() : OW2_READ()) {
            return cycles;
        }
        delay_us(55);
        cycles++;
    }
}

void display_ds_temperature(uint8_t high, uint8_t low, uint8_t ow_num) {
    uint8_t is_negative = 0;
    uint16_t decimals = 0; // 4 decimals (e.g. decimals 625 means 0.0625)
    uint16_t i;

    uint16_t temp = ((int16_t)high << 8) | low;
    if (temp & 0x8000) {
        is_negative = 1;
        temp = (~temp) + 1;
    }
    low = temp & 0x0f;
    temp = temp >> 4;

    // low[3:0] mean values 0.5,0.25,0.125 and 0.0625
    for (i=625; i <= 5000; i=i*2) {
        if (low & 0x01) {
            decimals += i;
        }
        low = low >> 1;
    }

    // Display temperature rounded to one decimal
    //display_number_dot((temp*1000 + ((decimals+5)/10) + 50)/100, 2, is_negative);
    display_temperature((temp*1000 + ((decimals+5)/10) + 50)/100, is_negative, 500, ow_num);
}

void read_ds18b20(uint8_t ow_num) {
    uint8_t i;
    uint8_t scratchpad[9];

    if (ow_init(ow_num)) {
        ow_write_byte(0xcc, ow_num); // Skip ROM
        ow_convert_temperature(ow_num);

        ow_init(ow_num);
        ow_write_byte(0xcc, ow_num); // Skip ROM
        ow_write_byte(0xbe, ow_num); // Read Scratchpad
        for (i=0; i<9; i++) {
            scratchpad[i] = ow_read_byte(ow_num);
        }

        display_ds_temperature(scratchpad[1], scratchpad[0], ow_num);

    } else {
        /* DS18B20 was not detected */
        //output_max(0x8, 0xa);
        printErr(ow_num);
    }
}

/***************************************************************************/

int main(void)
{
    // Встановлення максимальної частоти (16 Mhz)
    CLK_CKDIVR = 0;

    // Налаштування таймера (для delay_us)
    TIM2_PSCR = 0x4; // Prescaler: to 1MHz
    TIM2_CR1 |= TIM_CR1_CEN; // Start timer
	
	setup_display();//Ініціалізація дисплея
		
	debug(3333);
	debug(2222);
	debug(1111);
	debug(0000);
	debug(0000);
	
	step = 0;
	t1_index = 0;
	t2_index = 0;
	read_ds18b20(1);
    read_ds18b20(2);
    //Ініціалізація історії першими зчитаними значеннями
	for (uint8_t i =1; i<HISTORY_SIZE; i++){
		t1_history[i] = t1_history[0];
		t2_history[i] = t2_history[0];
	}
	//Головний цикл
    while(1) {
        read_ds18b20(1);
        read_ds18b20(2);
        step--;
        if (step<0)
			step = 127;
    }
}
