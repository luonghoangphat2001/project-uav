#include <reg52.h>
#include <intrins.h>

#define TOC_DO_NHAY_MET 7

// =========================================================
// 1. PHAN CUNG & I2C
// =========================================================
#define ADC_PORT P0

sbit RD_ADC      = P2^2;
sbit WR_ADC      = P2^3;
sbit INTR        = P3^6;

sbit I2C_SCL     = P2^0;
sbit I2C_SDA     = P2^1;
sbit LDR_PIN     = P1^0;

#define PCF8574_LCD_ADDR 0x40
#define PCF8574_U7_ADDR  0x42
#define PCF8574_U6_ADDR  0x44

#define MASK_LED_RED    0x01
#define MASK_LED_YELLOW 0x02
#define MASK_LED_BLUE   0x04
#define MASK_LED_GREEN  0x08
#define MASK_LED_PURPLE 0x10
#define MASK_LED_PINK   0x20
#define MASK_LED_NIGHT  0x40
#define MASK_LED_FLASH  0x80

sbit TR          = P1^5;
sbit ECHO        = P1^6;

sbit GAS_PIN     = P1^2;
sbit PIR_SENSOR  = P1^3;
sbit BTN_TROM    = P1^4;
sbit BUZZER      = P1^7;

sbit CAM_SENSOR  = P2^6;
sbit RAIN_SENSOR = P2^7;

sbit MOT_IN1     = P3^4;
sbit MOT_IN2     = P3^5;

// =========================================================
// 2. BIEN TOAN CUC
// =========================================================
typedef enum {
    NORMAL, MOTION, INTRUDER, OBSTACLE,
    GAS_LEAK, RAIN_WARNING, SMALL_FIRE, BIG_FIRE
} SystemState;

SystemState currentState = NORMAL;
SystemState lastState = NORMAL; // Theo doi de chi gui UART 1 lan khi doi trang thai

unsigned char timer_ticks = 0;
unsigned char current_temp = 0;
unsigned int  distance_cm = 0;
unsigned char distance_m = 0;
unsigned char tick_counter_1m = 0;

bit blink_flag = 0;
bit fast_blink_flag = 0;
unsigned char led_u7_state = 0x00;

bit intruder_active = 0;
bit last_btn_trom = 1;
bit motion_active = 0;
bit rain_active = 0;

bit cam_active = 0;
bit last_cam_state = 1;
bit cam_triggered_flag = 0;

unsigned char gps_timer_ticks = 0;
bit gps_10s_flag = 0;
bit is_daytime = 1;
bit last_daytime = 1;

unsigned char code seg7_CA[10] = {
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 
    0x92, 0x82, 0xF8, 0x80, 0x90
};

// =========================================================
// 3. DELAY & UART
// =========================================================
void Delay_ms(unsigned char ms) {
    unsigned char i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 112; j++);
}

void Delay_Trigger(void) {
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
}

void UART_Init(void) {
    SCON  = 0x50;
    T2CON = 0x34;
    RCAP2H = 0xFF;
    RCAP2L = 0xDC; // 9600 bps voi 11.0592 MHz
    TH2    = 0xFF;
    TL2    = 0xDC;
}

void UART_TxChar(char c) {
    SBUF = c;
    while(!TI);
    TI = 0;
}

void UART_SendString(char code *str) {
    while(*str) UART_TxChar(*str++);
}

// In so ra UART cuc nhe
void UART_PrintNum(unsigned int num) {
    unsigned char d = 0;
    while(num >= 100) { num -= 100; d++; }
    if(d) UART_TxChar(d + '0');
    d = 0;
    while(num >= 10)  { num -= 10; d++; }
    UART_TxChar(d + '0');
    UART_TxChar(num + '0');
}

// =========================================================
// 4. I2C MODULE
// =========================================================
void I2C_Start(void) {
    I2C_SDA = 1; I2C_SCL = 1;
    I2C_SDA = 0;
    I2C_SCL = 0;
}

void I2C_Stop(void) {
    I2C_SDA = 0; I2C_SCL = 1;
    I2C_SDA = 1;
}

void I2C_Write(unsigned char dat) {
    unsigned char i;
    for(i = 0; i < 8; i++) {
        I2C_SDA = (dat & 0x80) ? 1 : 0;
        I2C_SCL = 1;
        I2C_SCL = 0;
        dat <<= 1;
    }
    I2C_SDA = 1; I2C_SCL = 1;
    I2C_SCL = 0;
}

// =========================================================
// 5. LCD MODULE
// =========================================================
void LCD_Send_I2C(unsigned char dat) {
    I2C_Start();
    I2C_Write(PCF8574_LCD_ADDR);
    I2C_Write(dat);
    I2C_Stop();
}

void LCD_Write_Nibble(unsigned char nibble, unsigned char rs) {
    unsigned char d = (nibble & 0xF0) | rs;
    LCD_Send_I2C(d | 0x04);
    LCD_Send_I2C(d & ~0x04);
}

void LCD_Cmd(unsigned char cmd) {
    LCD_Write_Nibble(cmd, 0);
    LCD_Write_Nibble(cmd << 4, 0);
    Delay_ms(1);
}

void LCD_Data(unsigned char dat) {
    LCD_Write_Nibble(dat, 1);
    LCD_Write_Nibble(dat << 4, 1);
}

void LCD_Init(void) {
    Delay_ms(20);
    LCD_Write_Nibble(0x30, 0); Delay_ms(5);
    LCD_Write_Nibble(0x30, 0); Delay_ms(1);
    LCD_Write_Nibble(0x30, 0);
    LCD_Write_Nibble(0x20, 0);
    LCD_Cmd(0x28);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x01);
}

void LCD_PrintStr(char code *str) {
    while(*str) LCD_Data(*str++);
}

void LCD_Print(char code *s1, char code *s2) {
    LCD_Cmd(0x80); LCD_PrintStr(s1);
    LCD_Cmd(0xC0); LCD_PrintStr(s2);
}

void LCD_PrintNum(unsigned int num) {
    unsigned char d = 0;
    while(num >= 100) { num -= 100; d++; }
    if(d) LCD_Data(d + '0');
    d = 0;
    while(num >= 10)  { num -= 10; d++; }
    LCD_Data(d + '0');
    LCD_Data(num + '0');
}

// =========================================================
// 6. MODULE LED
// =========================================================
void Update_7Seg(unsigned char number) {
    I2C_Start();
    I2C_Write(PCF8574_U6_ADDR);
    I2C_Write(seg7_CA[number % 10]);
    I2C_Stop();
}

void Update_U7(void) {
    I2C_Start();
    I2C_Write(PCF8574_U7_ADDR);
    I2C_Write(led_u7_state);
    I2C_Stop();
}

// =========================================================
// 7. DOC CAM BIEN
// =========================================================
unsigned char Read_ADC(void) {
    unsigned char adc_val;
    unsigned int timeout = 0;

    ADC_PORT = 0xFF;
    WR_ADC = 0; _nop_(); WR_ADC = 1;

    while(INTR == 1) {
        if(++timeout > 300) return current_temp;
    }

    RD_ADC = 0; _nop_();
    adc_val = ADC_PORT;
    RD_ADC = 1;
    return adc_val;
}

unsigned int Read_Sonar(void) {
    unsigned int time;
    unsigned int timeout = 0;

    TR = 1;
    Delay_Trigger();
    TR = 0;

    while(!ECHO) {
        if(++timeout > 500) return 999;
    }

    TH0 = 0; TL0 = 0; TF0 = 0;
    TR0 = 1;
    while(ECHO && !TF0);
    TR0 = 0;

    if(TF0) { TF0 = 0; return 999; }
    time = (TH0 << 8) | TL0;
    return (time >> 6); 
}

// =========================================================
// 8. TIMER 1 (50ms)
// =========================================================
void Timer1_ISR(void) interrupt 3 {
    TH1 = 0x3C;
    TL1 = 0xB0;
    timer_ticks++;

    if((timer_ticks & 1) == 0)
        fast_blink_flag = ~fast_blink_flag;

    if(timer_ticks >= 20) {
        timer_ticks = 0;
        blink_flag = ~blink_flag;
    }

    if(++gps_timer_ticks >= 200) {
        gps_timer_ticks = 0;
        gps_10s_flag = 1;
    }

    if(currentState != OBSTACLE && currentState != MOTION) {
        if(++tick_counter_1m >= TOC_DO_NHAY_MET) {
            tick_counter_1m = 0;
            if(++distance_m > 9) distance_m = 0;
        }
    }
}

// =========================================================
// 9. LOGIC CAM BIEN
// =========================================================
void Read_Sensors(void) {
    current_temp = (unsigned char)(((unsigned int)Read_ADC() * 12) / 5);
    distance_cm  = Read_Sonar();

    if(BTN_TROM == 0) {
        if(last_btn_trom == 1) {
            intruder_active = ~intruder_active;
            last_btn_trom = 0;
        }
    } else {
        last_btn_trom = 1;
    }

    motion_active = (PIR_SENSOR == 0);
    rain_active   = (RAIN_SENSOR == 0);

    if(CAM_SENSOR == 0) {
        cam_active = 1;
        if(last_cam_state == 1) {
            cam_triggered_flag = 1;
            last_cam_state = 0;
        }
    } else {
        cam_active = 0;
        last_cam_state = 1;
    }

    is_daytime = (LDR_PIN == 0);
}

void Determine_State(void) {
    if(current_temp >= 80)           currentState = BIG_FIRE;
    else if(current_temp >= 50)      currentState = SMALL_FIRE;
    else if(GAS_PIN == 0)            currentState = GAS_LEAK;
    else if(rain_active)             currentState = RAIN_WARNING;
    else if(distance_cm < 20)        currentState = OBSTACLE;
    else if(intruder_active)         currentState = INTRUDER;
    else if(motion_active)           currentState = MOTION;
    else                             currentState = NORMAL;
}

// =========================================================
// 10. THUC THI HANH DONG & GUI UART KHI DOI TRANG THAI
// =========================================================
void Execute_Action(void) {
    led_u7_state = MASK_LED_BLUE;

    // Chi gui Serial 1 lan khi he thong chuyen sang trang thai moi
    if(currentState != lastState) {
        switch(currentState) {
            case BIG_FIRE:
                UART_SendString("Chay lon roi cuu hoa di!\r\n");
                break;
            case SMALL_FIRE:
                UART_SendString("Chay nho roi!\r\n");
                break;
            case GAS_LEAK:
                UART_SendString("Co gas kia!\r\n");
                break;
            case RAIN_WARNING:
                UART_SendString("Troi mua roi!\r\n");
                break;
            case MOTION:
                UART_SendString("Co xe!\r\n");
                break;
            case INTRUDER:
                UART_SendString("Bo Nguoi Ta An trom!\r\n");
                break;
            case OBSTACLE:
                UART_SendString("Giu Khoang cach");
                UART_PrintNum(distance_cm);
                UART_SendString(" cm\r\n");
                break;
            default:
                break;
        }
        lastState = currentState;
    }
		
		if(is_daytime != last_daytime) {
        if(is_daytime) {
            UART_SendString("NGAY\r\n");
        } else {
            UART_SendString("DEM\r\n");
        }
        last_daytime = is_daytime;
    }

    // Dieu khien LCD va Coi / LED
    switch(currentState) {
        case BIG_FIRE:
            if(fast_blink_flag) led_u7_state |= MASK_LED_RED;
            BUZZER = fast_blink_flag;
            LCD_Print("BIG FIRE", "DANGER!");
            MOT_IN1 = 0; MOT_IN2 = 0;
            break;

        case SMALL_FIRE:
            if(fast_blink_flag) led_u7_state |= MASK_LED_RED;
            BUZZER = blink_flag;
            LCD_Cmd(0x80); LCD_PrintStr("SMALL FIRE");
            LCD_Cmd(0xC0); LCD_PrintStr("Temp: ");
            LCD_PrintNum(current_temp);
            LCD_PrintStr(" C   ");
            MOT_IN1 = 0; MOT_IN2 = 0;
            break;

        case GAS_LEAK:
            led_u7_state |= MASK_LED_YELLOW;
            BUZZER = 1;
            LCD_Print("GAS WARNING", "LEAK DETECTED");
            break;

        case RAIN_WARNING:
            BUZZER = 1;
            LCD_Print("RAIN WARNING", "RAIN DETECTED");
            MOT_IN1 = 0; MOT_IN2 = 0;
            break;

        case OBSTACLE:
            BUZZER = 0;
            LCD_Cmd(0x80); LCD_PrintStr("OBSTACLE");
            LCD_Cmd(0xC0); LCD_PrintStr("Dist: ");
            LCD_PrintNum(distance_cm);
            LCD_PrintStr(" cm  ");
            MOT_IN1 = 0; MOT_IN2 = 0;
            break;

        case INTRUDER:
            led_u7_state |= MASK_LED_PURPLE;
            BUZZER = 1;
            LCD_Print("INTRUDER", "SECURITY ALERT");
            MOT_IN1 = 0; MOT_IN2 = 0;
            break;

        case MOTION:
            led_u7_state |= MASK_LED_GREEN;
            BUZZER = 0;
            LCD_Print("MOTION/VEHICLE", "XE DUNG LAI");
            MOT_IN1 = 0; MOT_IN2 = 0;
            break;

        case NORMAL:
        default:
            BUZZER = 0;
            LCD_Cmd(0x80); LCD_PrintStr("UAV NORMAL");
            LCD_Cmd(0xC0);
            LCD_PrintStr(is_daytime ? "NGAY T:" : "DEM  T:");
            LCD_PrintNum(current_temp);
            LCD_PrintStr("C M:");
            LCD_Data(distance_m + '0');
            LCD_PrintStr(" ");
            MOT_IN1 = 1; MOT_IN2 = 1;
            break;
    }

    if(rain_active)      led_u7_state |= MASK_LED_PINK;
    if(is_daytime == 0)  led_u7_state |= MASK_LED_NIGHT;
		else                  led_u7_state &= ~MASK_LED_NIGHT;
    if(cam_active)       led_u7_state |= MASK_LED_FLASH;
}

// =========================================================
// 11. KHOI TAO HE THONG
// =========================================================
void Init_System(void) {
    BUZZER = 0;
    MOT_IN1 = 1; MOT_IN2 = 1;
    RAIN_SENSOR = 1; PIR_SENSOR = 1; CAM_SENSOR = 1; LDR_PIN = 1;

    TMOD = 0x11;
    TH1  = 0x3C; TL1 = 0xB0;
    TR1  = 1; ET1 = 1; EA = 1;

    UART_Init();
    LCD_Init();

    led_u7_state = 0;
    Update_U7();
    Update_7Seg(0);

    LCD_Print("SYSTEM INIT...", "WAITING...");

    // Doc truoc trang thai ngay/dem ban dau
    is_daytime = (LDR_PIN == 0);
	  last_daytime = is_daytime;

    // Thong bao khoi dong he thong kem ngay/dem va toa do GPS
    UART_SendString("SYSTEM ONLINE: GPS & CAM READY\r\n");
		if(is_daytime) {
			UART_SendString("BAN NGAY\r\n");
		} else {
			UART_SendString("BAN DEM\r\n");
		}
    Delay_ms(200);
}

// =========================================================
// 12. MAIN LOOP
// =========================================================
void main(void) {
    Init_System();

    while(1) {
        Read_Sensors();
        Determine_State();
        Execute_Action();

        // Phat hien vat the qua Camera
        if(cam_triggered_flag) {
            cam_triggered_flag = 0;
        }

        // Dinh ky 10s cap nhat toa do GPS
        if(gps_10s_flag) {
            gps_10s_flag = 0;
        }

        Update_U7();
        Update_7Seg(distance_m);

        Delay_ms(10);
    }
}
