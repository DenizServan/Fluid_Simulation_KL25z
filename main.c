#include "MKL25Z4.h"
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

// --- Configuration ---
#define NUM_PARTICLES 24899
#define MATRIX_ROWS   8
#define MATRIX_COLS   8

// Capacitive Touch Configuration
#define CAP_TOUCH_PIN         16      // PTB16 (TSI0_CH9)
#define TOUCH_THRESHOLD       150     // Adjust based on sensitivity
#define TOUCH_DEBOUNCE_MS     200     // Debounce period

// LED Matrix Configuration
#define ROW_GPIO_PINS_B 0x0F08U       // PTB3 and PTB8-11
#define ROW_GPIO_PINS_E 0x700000U     // PTE20-22
#define COL_GPIO_PINS   0xFFU         // PTC0-PTC7

// Orientation LEDs on PTD0-PTD7
#define DOWN_LED_PIN       0
#define DOWN_RIGHT_LED_PIN 1
#define RIGHT_LED_PIN      2
#define UP_RIGHT_LED_PIN   3
#define UP_LED_PIN         4
#define UP_LEFT_LED_PIN    5
#define LEFT_LED_PIN       6
#define DOWN_LEFT_LED_PIN  7

// Joystick Configuration
#define JOYSTICK_VRX_PIN 0            // PTB0
#define JOYSTICK_VRY_PIN 1            // PTB1
#define JOYSTICK_SW_PIN  2            // PTB2
#define ADC_CHANNEL_VRX  8            // ADC0_SE8
#define ADC_CHANNEL_VRY  9            // ADC0_SE9

#define JOYSTICK_CENTER        512
#define JOYSTICK_THRESHOLD     200
#define JOYSTICK_DIAG_THRESHOLD 150
#define NEUTRAL_ZONE           100

typedef struct {
    int8_t x, y;
    uint8_t active;
} Particle;

static Particle particles[NUM_PARTICLES];
static uint8_t display_buffer[MATRIX_ROWS];

volatile uint8_t current_orientation = DOWN_LED_PIN;
volatile int8_t gravity_dx = 0, gravity_dy = 1;
volatile uint8_t update_simulation_flag = 0;
volatile uint32_t msTicks = 0;

// Touch Sensing Variables
volatile uint32_t last_touch_time = 0;
volatile uint16_t touch_baseline = 1000;
volatile bool restart_simulation_flag = false;

// Speed Control
typedef enum {
    SPEED_LOW = 0,
    SPEED_MEDIUM,
    SPEED_FAST,
    SPEED_COUNT
} ParticleSpeed;

volatile ParticleSpeed current_speed_mode = SPEED_MEDIUM;
const uint32_t pit_load_values[SPEED_COUNT] = {
    2399999, // 10Hz
    1199999, // 20Hz
    599999   // 40Hz
};
const char* speed_strings[SPEED_COUNT] = {
    "Speed: Low   ",
    "Speed: Medium",
    "Speed: Fast  "
};

// Function Prototypes
void Init_All(void);
void Init_Orientation_LEDs(void);
void Init_GPIO_Matrix(void);
void Init_Joystick(void);
void Init_ADC(void);
void Init_PIT_Timer(void);
void Init_SysTick(void);
void Init_TSI(void);
void Initialize_Particles(void);
void Update_Gravity_Vector(void);
void Update_Orientation_LEDs(void);
void Check_Joystick(void);
void Check_Capacitive_Touch(void);
void Update_Particle_Positions(void);
void Update_Display_Buffer(void);
void Update_LED_Matrix(void);
uint16_t Read_ADC(uint8_t channel);
uint16_t Read_TSI(void);
void DelayMs(uint32_t ms);
void DelayUs(uint32_t us);
void Blink_All_LEDs(void);
void Turn_On_All_LEDs(void);
void Turn_Off_All_LEDs(void);

int main(void) {
    Init_All();
    Initialize_Particles();
    Update_Gravity_Vector();
    Update_Orientation_LEDs();

    __enable_irq();

    while (1) {
        Check_Joystick();
        Check_Capacitive_Touch();
        
        if (restart_simulation_flag) {
            // Turn on all LEDs, wait 1 second, then restart simulation
            Turn_On_All_LEDs();
            DelayMs(1000);
            Turn_Off_All_LEDs();
            Initialize_Particles();
            restart_simulation_flag = false;
        }
        
        if (update_simulation_flag) {
            Update_Particle_Positions();
            Update_Display_Buffer();
            update_simulation_flag = 0;
        }
        Update_LED_Matrix();
    }
}

// System Initialization
void Init_All(void) {
    Init_SysTick();
    Init_Orientation_LEDs();
    Init_GPIO_Matrix();
    Init_Joystick();
    Init_ADC();
    Init_PIT_Timer();
    Init_TSI();
}

void Init_SysTick(void) {
    if (SysTick_Config(48000000 / 1000)) while (1); // 1ms interrupts
    NVIC_SetPriority(SysTick_IRQn, 3);
}

void SysTick_Handler(void) {
    msTicks++;
}

// LED Control Functions
void Turn_On_All_LEDs(void) {
    // Turn on all orientation LEDs
    GPIOD->PSOR = 0xFF;
    
    // Turn on all matrix LEDs (columns are active low)
    GPIOC->PCOR = COL_GPIO_PINS;
    
    // Enable all rows
    GPIOB->PSOR = ROW_GPIO_PINS_B;
    GPIOE->PSOR = ROW_GPIO_PINS_E;
}

void Turn_Off_All_LEDs(void) {
    // Turn off all orientation LEDs
    GPIOD->PCOR = 0xFF;
    
    // Turn on the current orientation LED
    GPIOD->PSOR = (1U << current_orientation);
    
    // Turn off all matrix LEDs
    GPIOC->PSOR = COL_GPIO_PINS;
    
    // Disable all rows
    GPIOB->PCOR = ROW_GPIO_PINS_B;
    GPIOE->PCOR = ROW_GPIO_PINS_E;
}

void Blink_All_LEDs(void) {
    for (int i = 0; i < 3; i++) {
        Turn_On_All_LEDs();
        DelayMs(200);
        Turn_Off_All_LEDs();
        DelayMs(200);
    }
}

// Capacitive Touch Functions
void Init_TSI(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_TSI_MASK;
    
    PORTB->PCR[CAP_TOUCH_PIN] = PORT_PCR_MUX(0); // TSI0_CH9 on PTB16
    
    TSI0->GENCS = TSI_GENCS_TSIEN_MASK |    // Enable TSI
                  TSI_GENCS_MODE(0) |       // Capacitive mode
                  TSI_GENCS_REFCHRG(4) |    // Reference charge 4µA
                  TSI_GENCS_DVOLT(0) |      // Voltage rails
                  TSI_GENCS_EXTCHRG(7) |    // External charge 32µA
                  TSI_GENCS_PS(4) |         // Prescaler 4
                  TSI_GENCS_NSCN(10) |      // 10 scans
                  TSI_GENCS_EOSF_MASK;      // Clear end-of-scan flag
    
    
}

uint16_t Read_TSI(void) {
    TSI0->DATA = TSI_DATA_TSICH(9) | TSI_DATA_SWTS_MASK; // Start scan on CH9
    while (!(TSI0->GENCS & TSI_GENCS_EOSF_MASK));        // Wait for scan
    uint16_t result = TSI0->DATA & TSI_DATA_TSICNT_MASK; // Read count
    TSI0->GENCS |= TSI_GENCS_EOSF_MASK;                  // Clear flag
    return result;
}

void Check_Capacitive_Touch(void) {
    static uint32_t last_check = 0;
    
    if ((msTicks - last_check) > 100) {
        uint16_t touch_value = Read_TSI();
        
        // Update dynamic baseline (slow moving average)
        touch_baseline = (touch_baseline * 99 + touch_value) / 100;
        
        // Check for touch
        if ((touch_value - touch_baseline) > TOUCH_THRESHOLD) {
            if ((msTicks - last_touch_time) > TOUCH_DEBOUNCE_MS) {
                // Touch detected, trigger restart with LED blinking
                restart_simulation_flag = true;
                last_touch_time = msTicks;
            }
        }
        last_check = msTicks;
    }
}

// GPIO Functions
void Init_Orientation_LEDs(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    for (int i = 0; i < 8; i++) {
        PORTD->PCR[i] = PORT_PCR_MUX(1);
    }
    GPIOD->PDDR |= 0xFF;
    GPIOD->PCOR = 0xFF;
}

void Init_GPIO_Matrix(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTE_MASK;
    for (int i = 20; i <= 22; i++) PORTE->PCR[i] = PORT_PCR_MUX(1);
    GPIOE->PDDR |= ROW_GPIO_PINS_E;
    GPIOE->PCOR = ROW_GPIO_PINS_E;
    PORTB->PCR[3] = PORT_PCR_MUX(1);
    for (int i = 8; i <= 11; i++) PORTB->PCR[i] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= ROW_GPIO_PINS_B;
    GPIOB->PCOR = ROW_GPIO_PINS_B;
    for (int i = 0; i < 8; i++) PORTC->PCR[i] = PORT_PCR_MUX(1);
    GPIOC->PDDR |= COL_GPIO_PINS;
    GPIOC->PSOR = COL_GPIO_PINS;
}

void Init_Joystick(void) {
    PORTB->PCR[JOYSTICK_SW_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    GPIOB->PDDR &= ~(1UL << JOYSTICK_SW_PIN);
}

void Init_ADC(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    PORTB->PCR[JOYSTICK_VRX_PIN] = PORT_PCR_MUX(0) | PORT_PCR_DSE_MASK;
    PORTB->PCR[JOYSTICK_VRY_PIN] = PORT_PCR_MUX(0) | PORT_PCR_DSE_MASK;
    ADC0->CFG1 = ADC_CFG1_MODE(2) | ADC_CFG1_ADICLK(0) | ADC_CFG1_ADIV(3);
    ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);
}

void Init_PIT_Timer(void) {
    SIM->SCGC6 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0;
    PIT->CHANNEL[0].LDVAL = pit_load_values[current_speed_mode];
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
    NVIC_SetPriority(PIT_IRQn, 1);
    NVIC_EnableIRQ(PIT_IRQn);
}

uint16_t Read_ADC(uint8_t channel) {
    ADC0->SC1[0] = channel;
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return ADC0->R[0];
}

void Check_Joystick(void) {
    uint16_t x_value = Read_ADC(ADC_CHANNEL_VRX);
    uint16_t y_value = Read_ADC(ADC_CHANNEL_VRY);
    int16_t x_offset = x_value - JOYSTICK_CENTER;
    int16_t y_offset = y_value - JOYSTICK_CENTER;
    uint8_t new_orientation = current_orientation;	//Initial Orientation

    if (abs(x_offset) > NEUTRAL_ZONE || abs(y_offset) > NEUTRAL_ZONE) {
        if (y_offset > JOYSTICK_DIAG_THRESHOLD && x_offset > JOYSTICK_DIAG_THRESHOLD) {
            new_orientation = DOWN_RIGHT_LED_PIN;
        } else if (y_offset < -JOYSTICK_DIAG_THRESHOLD && x_offset > JOYSTICK_DIAG_THRESHOLD) {
            new_orientation = UP_RIGHT_LED_PIN;
        } else if (y_offset < -JOYSTICK_DIAG_THRESHOLD && x_offset < -JOYSTICK_DIAG_THRESHOLD) {
            new_orientation = UP_LEFT_LED_PIN;
        } else if (y_offset > JOYSTICK_DIAG_THRESHOLD && x_offset < -JOYSTICK_DIAG_THRESHOLD) {
            new_orientation = DOWN_LEFT_LED_PIN;
        } else if (y_offset > JOYSTICK_THRESHOLD) {
            new_orientation = DOWN_LED_PIN;
        } else if (y_offset < -JOYSTICK_THRESHOLD) {
            new_orientation = UP_LED_PIN;
        } else if (x_offset > JOYSTICK_THRESHOLD) {
            new_orientation = RIGHT_LED_PIN;
        } else if (x_offset < -JOYSTICK_THRESHOLD) {
            new_orientation = LEFT_LED_PIN;
        }
        if (new_orientation != current_orientation) {
            current_orientation = new_orientation;
            Update_Gravity_Vector();
            Update_Orientation_LEDs();
        }
    }

    static bool last_button_state = true;
    static uint32_t button_debounce_start = 0;
    static bool debouncing = false;
    const uint32_t DEBOUNCE_MS = 50;

    bool current_button_state = (GPIOB->PDIR & (1UL << JOYSTICK_SW_PIN)) != 0;
    if (last_button_state && !current_button_state && !debouncing) {
        debouncing = true;
        button_debounce_start = msTicks;
    }
    if (debouncing && (msTicks - button_debounce_start) >= DEBOUNCE_MS) {
        if (!current_button_state) {
            current_speed_mode = (ParticleSpeed)((current_speed_mode + 1) % SPEED_COUNT);
            PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;
            PIT->CHANNEL[0].LDVAL = pit_load_values[current_speed_mode];
            PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
        }
        debouncing = false;
    }
    last_button_state = current_button_state;
}

void Initialize_Particles(void) {
    srand(msTicks);
    for (int i = 0; i < NUM_PARTICLES; i++) {
        particles[i].x = rand() % MATRIX_COLS;	//Randomized Initial State
        particles[i].y = rand() % MATRIX_ROWS;
        particles[i].active = 1;
    }
    Update_Display_Buffer();
}

void Update_Gravity_Vector(void) {
    switch (current_orientation) {
        case DOWN_LED_PIN:       gravity_dx = 0;  gravity_dy = 1;  break;
        case DOWN_RIGHT_LED_PIN: gravity_dx = 1;  gravity_dy = 1;  break;
        case RIGHT_LED_PIN:      gravity_dx = 1;  gravity_dy = 0;  break;
        case UP_RIGHT_LED_PIN:   gravity_dx = 1;  gravity_dy = -1; break;
        case UP_LED_PIN:         gravity_dx = 0;  gravity_dy = -1; break;
        case UP_LEFT_LED_PIN:    gravity_dx = -1; gravity_dy = -1; break;
        case LEFT_LED_PIN:       gravity_dx = -1; gravity_dy = 0;  break;
        case DOWN_LEFT_LED_PIN:  gravity_dx = -1; gravity_dy = 1;  break;
    }
}

void Update_Orientation_LEDs(void) {
    GPIOD->PCOR = 0xFF;
    GPIOD->PSOR = (1U << current_orientation);
}

void Update_Particle_Positions(void) {
    uint8_t grid[MATRIX_ROWS][MATRIX_COLS] = {0};		
    for (int i = 0; i < NUM_PARTICLES; i++) {		//Get current particle position x and y for every particle
        if (!particles[i].active) continue;
        int y = particles[i].y, x = particles[i].x;
        if (x >= 0 && x < MATRIX_COLS && y >= 0 && y < MATRIX_ROWS) //Additional check for boundries
					{
            grid[y][x] = 1;
        }
    }
    for (int i = 0; i < NUM_PARTICLES; i++) {	//Updating particle positions
        if (!particles[i].active) continue;
        int8_t x = particles[i].x, y = particles[i].y;	//Get current cordinates
        grid[y][x] = 0;		//Clear previous position
        int nx = x + gravity_dx, ny = y + gravity_dy;		//Getting the new position by adding current gravity vectors
			
			
        if (nx < 0 || nx >= MATRIX_COLS || ny < 0 || ny >= MATRIX_ROWS || grid[ny][nx]) {
            if (gravity_dx != 0 && gravity_dy != 0) {
                int nx1 = x + gravity_dx, ny1 = y;
                int nx2 = x, ny2 = y + gravity_dy;
                bool can1 = (nx1 >= 0 && nx1 < MATRIX_COLS && ny1 >= 0 && ny1 < MATRIX_ROWS && !grid[ny1][nx1]);	//Check the x move validness
                bool can2 = (nx2 >= 0 && nx2 < MATRIX_COLS && ny2 >= 0 && ny2 < MATRIX_ROWS && !grid[ny2][nx2]);	//Check the y move validness
                if (can1 || can2) {	//If at least one possible
                    if (can1 && can2) {	//If both possible choose randomly
                        if (rand() & 1) { nx = nx1; ny = ny1; }
                        else { nx = nx2; ny = ny2; }
                    } else if (can1) { nx = nx1; ny = ny1; }
                    else { nx = nx2; ny = ny2; }
                } else {
                    nx = x; ny = y;
                }
            } else {	
                int dx1 = gravity_dy, dy1 = -gravity_dx;
                int dx2 = -gravity_dy, dy2 = gravity_dx;
                int x1 = x + dx1, y1 = y + dy1;
                int x2 = x + dx2, y2 = y + dy2;
                bool can1 = (x1 >= 0 && x1 < MATRIX_COLS && y1 >= 0 && y1 < MATRIX_ROWS && !grid[y1][x1]);
                bool can2 = (x2 >= 0 && x2 < MATRIX_COLS && y2 >= 0 && y2 < MATRIX_ROWS && !grid[y2][x2]);
                if (can1 || can2) {
                    if (can1 && can2) {
                        if (rand() & 1) { nx = x1; ny = y1; }
                        else { nx = x2; ny = y2; }
                    } else if (can1) { nx = x1; ny = y1; }
                    else { nx = x2; ny = y2; }
                } else {//Else state 
                    nx = x; ny = y;
                }
            }
        }
				//Update particle position
        particles[i].x = nx;
        particles[i].y = ny;
        grid[ny][nx] = 1;
    }
}

void Update_Display_Buffer(void) {
    for (int r = 0; r < MATRIX_ROWS; r++) {
        display_buffer[r] = 0;
    }
    for (int i = 0; i < NUM_PARTICLES; i++) {
        if (!particles[i].active) continue;
        int y = particles[i].y, x = particles[i].x;
        if (x >= 0 && x < MATRIX_COLS && y >= 0 && y < MATRIX_ROWS) {
            display_buffer[y] |= (1 << x);
        }
    }
}

void Update_LED_Matrix(void) {
    const uint32_t rowB[5] = {1UL << 3, 1UL << 8, 1UL << 9, 1UL << 10, 1UL << 11};
    const uint32_t rowE[3] = {1UL << 20, 1UL << 21, 1UL << 22};
    for (int r = 0; r < MATRIX_ROWS; r++) {
        GPIOB->PCOR = ROW_GPIO_PINS_B;
        GPIOE->PCOR = ROW_GPIO_PINS_E;
        if (r < 3) GPIOE->PSOR = rowE[r];
        else GPIOB->PSOR = rowB[r - 3];
        GPIOC->PDOR = ~display_buffer[r];
        for (volatile int d = 0; d < 50; d++);
    }
    GPIOB->PCOR = ROW_GPIO_PINS_B;
    GPIOE->PCOR = ROW_GPIO_PINS_E;
}

void PIT_IRQHandler(void) {
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        update_simulation_flag = 1;
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
    }
}

// Utility Functions
void DelayMs(uint32_t ms) {
    uint32_t startTicks = msTicks;
    while ((msTicks - startTicks) < ms);
}

void DelayUs(uint32_t us) {
    uint32_t cycles = (us * 48) / 12;
    if (cycles == 0) cycles = 1;
    for (volatile uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}