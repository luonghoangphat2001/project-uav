#include <reg52.h>
#include <stdio.h>
#include <intrins.h>

#define TOC_DO_NHAY_MET 7

// =========================================================
// 1. DINH NGHIA CHAN KET NOI
// =========================================================

#define ADC_PORT P0

sbit CS      = P3^0;
sbit RD_ADC  = P3^1;
sbit WR_ADC  = P3^2;
sbit INTR    = P3^3;

// I2C LCD
sbit I2C_SCL = P2^0;
sbit I2C_SDA = P2^1;

// I2C U7 (Dieu khien LED) - Noi vao PCF3 va PCF4
sbit I2C_SCL_U7 = P2^2;
sbit I2C_SDA_U7 = P2^3;

// I2C U6 (LED 7 doan)
sbit I2C_SCL_7SEG = P1^0;
sbit I2C_SDA_7SEG = P1^1;

#define PCF8574_LCD_ADDR 0x40
#define PCF8574_U6_ADDR  0x40
#define PCF8574_U7_ADDR  0x40

// Mask cac LED tren U7 (P0 den P5)
#define MASK_LED_RED    0x01
#define MASK_LED_YELLOW 0x02
#define MASK_LED_BLUE   0x04
#define MASK_LED_GREEN  0x08
#define MASK_LED_PURPLE 0x10
#define MASK_LED_PINK   0x20

sbit TR      = P1^5;
sbit ECHO    = P1^6;

sbit GAS_PIN = P1^2;
sbit PIR_SENSOR = P1^3;
sbit BTN_TROM = P1^4;

sbit BUZZER = P1^7;

// Cam bien mua
sbit RAIN_SENSOR = P2^7;

sbit MOT_IN1 = P3^4;
sbit MOT_IN2 = P3^5;
sbit MOT_EN1 = P3^6;
sbit MOT_EN2 = P3^7;


// =========================================================
// 2. BIEN TOAN CUC VA TRANG THAI
// =========================================================

typedef enum
{
    NORMAL,
    MOTION,
    INTRUDER,
    OBSTACLE,
    GAS_LEAK,
    RAIN_WARNING,
    SMALL_FIRE,
    BIG_FIRE
} SystemState;

SystemState currentState = NORMAL;

unsigned int timer_ticks = 0;
unsigned char current_temp = 0;
unsigned int distance_cm = 0;

bit blink_flag = 0;
bit fast_blink_flag = 0;

// Bien trang thai cua U7 de gui qua I2C
unsigned char led_u7_state = 0x00;

// Bien hien thi quang duong
unsigned char distance_m = 0;
unsigned int tick_counter_1m = 0;

unsigned char last_dist_m = 0xFF;
unsigned char last_seg_data = 0xFF;

// Trang thai nut trom
bit intruder_active = 0;
bit last_btn_trom = 1;

// Trang thai nut xe
bit motion_active = 0;
bit last_btn_xe = 1;

// Trang thai cam bien mua
bit rain_active = 0;

// Ma LED 7 doan common anode
unsigned char code seg7_CA[] =
{
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 
    0x92, 0x82, 0xF8, 0x80, 0x90
};


// =========================================================
// 3. CAC HAM TAO TRE
// =========================================================

void Delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 112; j++);
}

void Delay_Trigger()
{
    volatile unsigned char i;
    for(i = 0; i < 50; i++)
        _nop_();
}


// =========================================================
// 4. GIAO TIEP I2C LCD
// =========================================================

void I2C_Start()
{
    I2C_SDA = 1;
    I2C_SCL = 1;
    _nop_();
    I2C_SDA = 0;
    _nop_();
    I2C_SCL = 0;
}

void I2C_Stop()
{
    I2C_SDA = 0;
    I2C_SCL = 1;
    _nop_();
    I2C_SDA = 1;
    _nop_();
}

void I2C_Write(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        I2C_SDA = (dat & 0x80) ? 1 : 0;
        I2C_SCL = 1;
        _nop_();
        I2C_SCL = 0;
        dat <<= 1;
    }
    I2C_SDA = 1;
    I2C_SCL = 1;
    _nop_();
    I2C_SCL = 0;
}

void LCD_Send_I2C(unsigned char dat)
{
    I2C_Start();
    I2C_Write(PCF8574_LCD_ADDR);
    I2C_Write(dat);
    I2C_Stop();
}

void LCD_Write_Nibble(unsigned char nibble, unsigned char rs)
{
    unsigned char data_i2c;
    data_i2c = (nibble & 0xF0) | rs;
    LCD_Send_I2C(data_i2c | 0x04);
    Delay_ms(1);
    LCD_Send_I2C(data_i2c & ~0x04);
    Delay_ms(1);
}

void LCD_Cmd(unsigned char cmd)
{
    LCD_Write_Nibble(cmd & 0xF0, 0);
    LCD_Write_Nibble((cmd << 4) & 0xF0, 0);
}

void LCD_Data(unsigned char dat)
{
    LCD_Write_Nibble(dat & 0xF0, 1);
    LCD_Write_Nibble((dat << 4) & 0xF0, 1);
}

void LCD_Init()
{
    Delay_ms(20);
    LCD_Write_Nibble(0x30, 0);
    Delay_ms(5);
    LCD_Write_Nibble(0x30, 0);
    Delay_ms(1);
    LCD_Write_Nibble(0x30, 0);
    Delay_ms(1);
    LCD_Write_Nibble(0x20, 0);
    Delay_ms(1);
    LCD_Cmd(0x28);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x01);
    Delay_ms(2);
}

void LCD_Print(char* str_line1, char* str_line2)
{
    LCD_Cmd(0x80);
    while(*str_line1) LCD_Data(*str_line1++);
    LCD_Cmd(0xC0);
    while(*str_line2) LCD_Data(*str_line2++);
}


// =========================================================
// 5. GIAO TIEP I2C U6 (LED 7 DOAN)
// =========================================================

void I2C_Start_7Seg()
{
    I2C_SDA_7SEG = 1;
    I2C_SCL_7SEG = 1;
    _nop_();
    I2C_SDA_7SEG = 0;
    _nop_();
    I2C_SCL_7SEG = 0;
}

void I2C_Stop_7Seg()
{
    I2C_SDA_7SEG = 0;
    I2C_SCL_7SEG = 1;
    _nop_();
    I2C_SDA_7SEG = 1;
    _nop_();
}

void I2C_Write_7Seg(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        I2C_SDA_7SEG = (dat & 0x80) ? 1 : 0;
        I2C_SCL_7SEG = 1;
        _nop_();
        I2C_SCL_7SEG = 0;
        dat <<= 1;
    }
    I2C_SDA_7SEG = 1;
    I2C_SCL_7SEG = 1;
    _nop_();
    I2C_SCL_7SEG = 0;
}

void Update_7Seg(unsigned char number)
{
    unsigned char num_to_display;
    num_to_display = number % 10;
    last_seg_data = seg7_CA[num_to_display];

    if(rain_active == 1)
        last_seg_data |= 0x80;
    else
        last_seg_data &= 0x7F;

    I2C_Start_7Seg();
    I2C_Write_7Seg(PCF8574_U6_ADDR);
    I2C_Write_7Seg(last_seg_data);
    I2C_Stop_7Seg();
}

// =========================================================
// 5B. GIAO TIEP I2C U7 (CAC LED TRANG THAI)
// =========================================================

void I2C_Start_U7()
{
    I2C_SDA_U7 = 1;
    I2C_SCL_U7 = 1;
    _nop_();
    I2C_SDA_U7 = 0;
    _nop_();
    I2C_SCL_U7 = 0;
}

void I2C_Stop_U7()
{
    I2C_SDA_U7 = 0;
    I2C_SCL_U7 = 1;
    _nop_();
    I2C_SDA_U7 = 1;
    _nop_();
}

void I2C_Write_U7(unsigned char dat)
{
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        I2C_SDA_U7 = (dat & 0x80) ? 1 : 0;
        I2C_SCL_U7 = 1;
        _nop_();
        I2C_SCL_U7 = 0;
        dat <<= 1;
    }
    I2C_SDA_U7 = 1;
    I2C_SCL_U7 = 1;
    _nop_();
    I2C_SCL_U7 = 0;
}

void Update_U7()
{
    I2C_Start_U7();
    I2C_Write_U7(PCF8574_U7_ADDR);
    I2C_Write_U7(led_u7_state);
    I2C_Stop_U7();
}


// =========================================================
// 6. DOC CAM BIEN
// =========================================================

unsigned char Read_ADC()
{
    unsigned char adc_val;
    unsigned int timeout = 0;

    ADC_PORT = 0xFF;
    CS = 0;
    WR_ADC = 0;
    _nop_();
    WR_ADC = 1;
    CS = 1;

    while(INTR == 1)
    {
        timeout++;
        if(timeout > 5000) return 0;
    }

    CS = 0;
    RD_ADC = 0;
    _nop_();
    adc_val = ADC_PORT;
    RD_ADC = 1;
    CS = 1;

    return adc_val;
}

unsigned int Read_Sonar()
{
    unsigned int time;
    unsigned int timeout = 0;

    TR = 1;
    Delay_Trigger();
    TR = 0;

    while(!ECHO)
    {
        timeout++;
        if(timeout > 10000) return 999;
    }

    TH0 = 0;
    TL0 = 0;
    TF0 = 0;
    TR0 = 1;

    while(ECHO && !TF0);
    TR0 = 0;

    if(TF0)
    {
        TF0 = 0;
        return 999;
    }

    time = (TH0 << 8) | TL0;
    return (time / 58);
}


// =========================================================
// 7. NGAT TIMER 1
// =========================================================

void Timer1_ISR() interrupt 3
{
    TH1 = 0x3C;
    TL1 = 0xB0;

    timer_ticks++;

    if(timer_ticks % 2 == 0)
    {
        fast_blink_flag = ~fast_blink_flag;
    }

    if(timer_ticks >= 20)
    {
        timer_ticks = 0;
        blink_flag = ~blink_flag;
    }

    if(currentState != OBSTACLE && currentState != MOTION)
    {
        tick_counter_1m++;
        if(tick_counter_1m >= TOC_DO_NHAY_MET)
        {
            tick_counter_1m = 0;
            distance_m++;
            if(distance_m > 9)
                distance_m = 0;
        }
    }
}


// =========================================================
// 8. KHOI TAO HE THONG
// =========================================================

void Init_System()
{
    BUZZER = 0;
    MOT_EN1 = 1;
    MOT_EN2 = 1;

    RAIN_SENSOR = 1;
    PIR_SENSOR = 1;

    TMOD = 0x11;
    TH1 = 0x3C;
    TL1 = 0xB0;

    TR1 = 1;
    ET1 = 1;
    EA = 1;

    LCD_Init();
    rain_active = 0;

    led_u7_state = 0;
    Update_U7();
    Update_7Seg(0);

    LCD_Print("SYSTEM INIT...", "WAITING...");
    Delay_ms(1000);
}


// =========================================================
// 9. DOC CAC CAM BIEN
// =========================================================

void Read_Sensors()
{
    unsigned int adc_val;
    adc_val = Read_ADC();
    current_temp = (unsigned char)(((unsigned long)adc_val * 240) / 100);
    distance_cm = Read_Sonar();

    // Nut trom
    if(BTN_TROM == 0)
    {
        if(last_btn_trom == 1)
        {
            intruder_active = ~intruder_active;
            last_btn_trom = 0;
        }
    }
    else
    {
        last_btn_trom = 1;
    }

    // Nut xe
    if(PIR_SENSOR == 0)
        motion_active = 1;
    else
        motion_active = 0;
    
    // Cam bien mua
    if(RAIN_SENSOR == 0)
        rain_active = 1;
    else
        rain_active = 0;
}


// =========================================================
// 10. XAC DINH TRANG THAI
// =========================================================

void Determine_State()
{
    if(current_temp >= 80)
        currentState = BIG_FIRE;
    else if(current_temp >= 50 && current_temp < 80)
        currentState = SMALL_FIRE;
    else if(GAS_PIN == 0)
        currentState = GAS_LEAK;
    else if(rain_active == 1)
        currentState = RAIN_WARNING;
    else if(distance_cm < 20)
        currentState = OBSTACLE;
    else if(intruder_active == 1)
        currentState = INTRUDER;
    else if(motion_active == 1)
        currentState = MOTION;
    else
        currentState = NORMAL;
}


// =========================================================
// 11. RESET LED U7
// =========================================================

void Reset_LEDs()
{
    led_u7_state = 0x00; // Xoa trang thai tat ca cac LED tren U7 truoc moi chu ky
}


// =========================================================
// 12. THUC HIEN HANH DONG
// =========================================================

void Execute_Action()
{
    char buff_str[16];

    switch(currentState)
    {
        case BIG_FIRE:
            if(fast_blink_flag) led_u7_state |= MASK_LED_RED;
            BUZZER = fast_blink_flag;
            LCD_Print("BIG FIRE", "DANGER!");
            MOT_IN1 = 0; MOT_IN2 = 0; MOT_EN1 = 0; MOT_EN2 = 0;
            break;

        case SMALL_FIRE:
            if(fast_blink_flag) led_u7_state |= MASK_LED_RED;
            BUZZER = blink_flag;
            sprintf(buff_str, "Temp: %d C    ", current_temp);
            LCD_Print("SMALL FIRE", buff_str);
            MOT_IN1 = 0; MOT_IN2 = 0; MOT_EN1 = 0; MOT_EN2 = 0;
            break;

        case GAS_LEAK:
            led_u7_state |= MASK_LED_YELLOW;
            BUZZER = 1;
            LCD_Print("GAS WARNING", "LEAK DETECTED");
            break;

        case RAIN_WARNING:
            BUZZER = 1;
            LCD_Print("RAIN WARNING", "RAIN DETECTED");
            MOT_IN1 = 0; MOT_IN2 = 0; MOT_EN1 = 0; MOT_EN2 = 0;
            break;

        case OBSTACLE:
            BUZZER = 0;
            sprintf(buff_str, "Dist: %d cm    ", distance_cm);
            LCD_Print("OBSTACLE", buff_str);
            MOT_IN1 = 0; MOT_IN2 = 0; MOT_EN1 = 0; MOT_EN2 = 0;
            break;

        case INTRUDER:
            led_u7_state |= MASK_LED_PURPLE;
            BUZZER = 1;
            LCD_Print("INTRUDER", "SECURITY ALERT");
            MOT_IN1 = 0; MOT_IN2 = 0; MOT_EN1 = 0; MOT_EN2 = 0;
            break;

        case MOTION:
            led_u7_state |= MASK_LED_GREEN;
            BUZZER = 0;
            LCD_Print("MOTION/VEHICLE", "XE DUNG LAI");
            MOT_IN1 = 0; MOT_IN2 = 0; MOT_EN1 = 0; MOT_EN2 = 0;
            break;

        case NORMAL:
        default:
            led_u7_state |= MASK_LED_BLUE;
            BUZZER = 0;
            sprintf(buff_str, "T:%dC Met:%d  ", (int)current_temp, (int)distance_m);
            LCD_Print("UAV NORMAL", buff_str);
            MOT_IN1 = 1; MOT_IN2 = 1; MOT_EN1 = 1; MOT_EN2 = 1;
            break;
    }
}


// =========================================================
// 13. CHUONG TRINH CHINH
// =========================================================

void main()
{
    Init_System();

    while(1)
    {
        Read_Sensors();
        Determine_State();
        Reset_LEDs();
        Execute_Action();

        // Cap nhat cac LED tren U7 (PCF8574 moi)
        Update_U7();

        // Cap nhat 7 doan U6 P7
        Update_7Seg(distance_m);

        Delay_ms(50);
    }
}