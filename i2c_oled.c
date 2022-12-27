/* i2c_oled --- I2C OLED SSD1306 on Blue Pill STM32 board   2022-12-17 */

#include <stm32f1xx.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Size of 128x32 OLED screen
#define MAXX 128
#define MAXY 32
#define MAXROWS 4

#include "image.h"

// Co-ord of centre of screen
#define CENX (MAXX / 2)
#define CENY (MAXY / 2)

// SSD1306 command bytes
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
struct UART_BUFFER U1Buf;

// The frame buffer, 512 bytes
uint8_t Frame[MAXROWS][MAXX];

uint32_t SavedRccCsr = 0u;
volatile uint32_t Milliseconds = 0;
volatile uint8_t Tick = 0;


/* USART1_IRQHandler --- ISR for USART1, used for Rx and Tx */

void USART1_IRQHandler(void)
{
   if (USART1->SR & USART_SR_RXNE) {
      const uint8_t tmphead = (U1Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
      const uint8_t ch = USART1->DR;  // Read received byte from UART
      
      if (tmphead == U1Buf.rx.tail)   // Is receive buffer full?
      {
          // Buffer is full; discard new byte
      }
      else
      {
         U1Buf.rx.head = tmphead;
         U1Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
      }
   }
   
   if (USART1->SR & USART_SR_TXE) {
      if (U1Buf.tx.head != U1Buf.tx.tail) // Is there anything to send?
      {
         const uint8_t tmptail = (U1Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
         
         U1Buf.tx.tail = tmptail;

         USART1->DR = U1Buf.tx.buf[tmptail];    // Transmit one byte
      }
      else
      {
         USART1->CR1 &= ~USART_CR1_TXEIE; // Nothing left to send; disable Tx Empty interrupt
      }
   }
}


/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
   return (Milliseconds);
}


/* SysTick_Handler --- ISR for System Timer overflow, used for 1ms ticker */

void SysTick_Handler(void)
{
   static uint8_t flag = 0;
   
   Milliseconds++;
   Tick = 1;
   
   // DEBUG: 500Hz on PC14 pin
   if (flag)
      GPIOC->BSRR = GPIO_BSRR_BR14; // GPIO pin PC14 LOW
   else
      GPIOC->BSRR = GPIO_BSRR_BS14; // GPIO pin PC14 HIGH
      
   flag = !flag;
}


/* UART1RxByte --- read one character from UART1 via the circular buffer */

uint8_t UART1RxByte(void)
{
   const uint8_t tmptail = (U1Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U1Buf.rx.head == U1Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U1Buf.rx.tail = tmptail;
   
   return (U1Buf.rx.buf[tmptail]);
}


/* UART1RxAvailable --- return true if a byte is available in UART1 circular buffer */

int UART1RxAvailable(void)
{
   return (U1Buf.rx.head != U1Buf.rx.tail);
}


/* UART1TxByte --- send one character to UART1 via the circular buffer */

void UART1TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U1Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U1Buf.tx.tail)   // Wait, if buffer is full
       ;

   U1Buf.tx.buf[tmphead] = data;
   U1Buf.tx.head = tmphead;

   USART1->CR1 |= USART_CR1_TXEIE;   // Enable UART1 Tx Empty interrupt
}


void i2c1_start(void)
{
   I2C1->CR1 |= I2C_CR1_START;         // Trigger I2C Start condition
   
   while (I2C1->CR1 & I2C_CR1_START)   // Wait for completion
      ;
}


void i2c1_stop(void)
{
   I2C1->CR1 |= I2C_CR1_STOP;          // Trigger I2C Start condition
   
   while (I2C1->CR1 & I2C_CR1_STOP)    // Wait for completion
      ;
}


void i2c1_txa(const uint8_t addr)
{
   volatile uint8_t junk;
   
   I2C1->DR = addr;
   
   while ((I2C1->SR1 & I2C_SR1_ADDR) == 0)
      ;
   
   junk = I2C1->SR1;
   junk = I2C1->SR2;
   
   I2C1->SR1 &= ~I2C_SR1_ADDR;
}


void i2c1_txd(const uint8_t data)
{
   I2C1->DR = data;
   
   while ((I2C1->SR1 & I2C_SR1_TXE) == 0)
      ;
}


/* oledData --- send a data byte to the OLED by I2C */

static void oledData(const uint8_t d)
{
    i2c1_txd(d);
}


/* oledCmd --- send a command byte to the OLED by I2C */

static void oledCmd(const uint8_t c)
{
   i2c1_start();
   i2c1_txa(0x78);
   i2c1_txd(0x00);
   i2c1_txd(c);
   i2c1_stop();
}


/* oledCmd1b --- send two command bytes to the OLED by I2C */

static void oledCmd1b(const uint8_t c, const uint8_t b)
{
   i2c1_start();
   i2c1_txa(0x78);
   i2c1_txd(0x00);
   i2c1_txd(c);
   i2c1_txd(b);
   i2c1_stop();
}


/* updscreen --- update the physical screen from the buffer */

static void updscreen(void)
{
   static uint8_t addrCmd[] = {SSD1306_COLUMNADDR, 0, MAXX - 1, SSD1306_PAGEADDR, 4, 7};
   int r, c;
   
   i2c1_start();
   i2c1_txa(0x78);
   i2c1_txd(0x00);
   i2c1_txd(SSD1306_COLUMNADDR);
   i2c1_txd(0);
   i2c1_txd(MAXX - 1);
   i2c1_stop();
   
   i2c1_start();
   i2c1_txa(0x78);
   i2c1_txd(0x00);
   i2c1_txd(SSD1306_PAGEADDR);
   i2c1_txd(4);
   i2c1_txd(7);
   i2c1_stop();
   
   i2c1_start();
   i2c1_txa(0x78);
   i2c1_txd(0x40);
   
   for (r = 0; r < MAXROWS; r++)
      for (c = 0; c < MAXX; c++)
         i2c1_txd(Frame[r][c]);
   
   i2c1_stop();
}


/* OLED_begin --- initialise the SSD1306 OLED */

void OLED_begin(const int wd, const int ht)
{
    // Init sequence for SSD1306 128x64 or 128x32 OLED module
    oledCmd(SSD1306_DISPLAYOFF);                    // 0xAE
    oledCmd1b(SSD1306_SETDISPLAYCLOCKDIV, 0x80);    // 0xD5, the suggested ratio 0x80
    oledCmd1b(SSD1306_SETMULTIPLEX, 0x3F);          // 0xA8
    oledCmd1b(SSD1306_SETDISPLAYOFFSET, 0x00);      // 0xD3, no offset
    oledCmd(SSD1306_SETSTARTLINE | 0x0);            // line #0
    oledCmd1b(SSD1306_CHARGEPUMP, 0x14);            // 0x8D
    oledCmd1b(SSD1306_MEMORYMODE, 0x00);            // 0x20, 0x00 act like ks0108
    oledCmd(SSD1306_SEGREMAP | 0x1);
    oledCmd(SSD1306_COMSCANDEC);
    
    if (ht < 64)
    {
        oledCmd1b(SSD1306_SETCOMPINS, 0x02);        // 0xDA
    }
    else
    {
        oledCmd1b(SSD1306_SETCOMPINS, 0x12);        // 0xDA
    }
    
    oledCmd1b(SSD1306_SETCONTRAST, 0xCF);           // 0x81
    oledCmd1b(SSD1306_SETPRECHARGE, 0xF1);          // 0xd9
    oledCmd1b(SSD1306_SETVCOMDETECT, 0x40);         // 0xDB
    oledCmd(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    oledCmd(SSD1306_NORMALDISPLAY);                 // 0xA6

    oledCmd(SSD1306_DISPLAYON); // Turn on OLED panel
}


/* greyFrame --- clear entire frame to checkerboard pattern */

void greyFrame(void)
{
    int r, c;

    for (r = 0; r < MAXROWS; r++)
    {
        for (c = 0; c < MAXX; c += 2)
        {
            Frame[r][c] = 0xaa;
            Frame[r][c + 1] = 0x55;
        }
    }
}


/* setPixel --- set a single pixel */

void setPixel(const unsigned int x, const unsigned int y)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y / 8][x] |= 1 << (y & 7);
    else
    {
//      Serial.print("setPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* clrPixel --- clear a single pixel */

void clrPixel(const unsigned int x, const unsigned int y)
{
    if ((x < MAXX) && (y < MAXY))
        Frame[y / 8][x] &= ~(1 << (y & 7));
    else
    {
//      Serial.print("clrPixel(");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
//      Serial.println(")");
    }
}


/* setVline --- draw vertical line */

void setVline(const unsigned int x, const unsigned int y1, const unsigned int y2)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        setPixel(x, y);
}


/* clrVline --- draw vertical line */

void clrVline(const unsigned int x, const unsigned int y1, const unsigned int y2)
{
    unsigned int y;

    for (y = y1; y <= y2; y++)
        clrPixel(x, y);
}


/* setHline --- set pixels in a horizontal line */

void setHline(const unsigned int x1, const unsigned int x2, const unsigned int y)
{
    unsigned int x;
    const unsigned int row = y / 8;
    const uint8_t b = 1 << (y  & 7);

    for (x = x1; x <= x2; x++)
        Frame[row][x] |= b;
}


/* clrHline --- clear pixels in a horizontal line */

void clrHline(const unsigned int x1, const unsigned int x2, const unsigned int y)
{
    unsigned int x;
    const unsigned int row = y / 8;
    const uint8_t b = ~(1 << (y  & 7));

    for (x = x1; x <= x2; x++)
      Frame[row][x] &= b;
}


/* setRect --- set pixels in a (non-filled) rectangle */

void setRect(const int x1, const int y1, const int x2, const int y2)
{
    setHline(x1, x2, y1);
    setVline(x2, y1, y2);
    setHline(x1, x2, y2);
    setVline(x1, y1, y2);
}


/* fillRect --- set pixels in a filled rectangle */

void fillRect(const int x1, const int y1, const int x2, const int y2, const int ec, const int fc)
{
    int y;

    for (y = y1; y <= y2; y++)
        if (fc == 0)
            clrHline(x1, x2, y);
        else if (fc == 1)
            setHline(x1, x2, y);

    if (ec == 1)
    {
        setHline(x1, x2, y1);
        setVline(x2, y1, y2);
        setHline(x1, x2, y2);
        setVline(x1, y1, y2);
    }
    else if (ec == 0)
    {
        clrHline(x1, x2, y1);
        clrVline(x2, y1, y2);
        clrHline(x1, x2, y2);
        clrVline(x1, y1, y2);
    }
}


void drawSegA(const int x)
{
   setHline(x + 1, x + 17, 0);
   setHline(x + 2, x + 16, 1);
   setHline(x + 3, x + 15, 2);
}


void drawSegB(const int x)
{
   setVline(x + 18, 1, 13);
   setVline(x + 17, 2, 14);
   setVline(x + 16, 3, 13);
}


void drawSegC(const int x)
{
   setVline(x + 18, 19, 30);
   setVline(x + 17, 18, 29);
   setVline(x + 16, 19, 28);
}


void drawSegD(const int x)
{
   setHline(x + 1, x + 17, 31);
   setHline(x + 2, x + 16, 30);
   setHline(x + 3, x + 15, 29);
}


void drawSegE(const int x)
{
   setVline(x + 0, 17, 30);
   setVline(x + 1, 18, 29);
   setVline(x + 2, 19, 28);
}


void drawSegF(const int x)
{
   setVline(x + 0, 1, 15);
   setVline(x + 1, 2, 14);
   setVline(x + 2, 3, 13);
}


void drawSegG(const int x)
{
   setHline(x + 2, x + 16, 15);
   setHline(x + 1, x + 17, 16);
   setHline(x + 2, x + 16, 17);
}


void drawSegH(const int x)
{
   setHline(x + 18, x + 21, 15);
   setHline(x + 17, x + 21, 16);
   setHline(x + 18, x + 21, 17);
}


void drawSegDP(const int x)
{
   setHline(x + 20, x + 22, 29);
   setHline(x + 20, x + 22, 30);
   setHline(x + 20, x + 22, 31);
}


/* _write --- connect stdio functions to UART1 */

int _write(const int fd, const char *ptr, const int len)
{
   int i;

   for (i = 0; i < len; i++) {
      if (*ptr == '\n')
         UART1TxByte('\r');
      
      UART1TxByte(*ptr++);
   }
  
   return (len);
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   FLASH->ACR |= FLASH_ACR_LATENCY_2;  // Set Flash latency
   
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // Set APB1 bus to not exceed 36MHz
   
   RCC->CR |= RCC_CR_HSEON;   // Switch on High Speed External clock (8MHz on the Blue Pill)
   
   // Wait for HSE to start up
   while ((RCC->CR & RCC_CR_HSERDY) == 0)
      ;
   
   RCC->CFGR |= RCC_CFGR_PLLSRC;       // Select HSE as input to PLL
   RCC->CFGR |= RCC_CFGR_PLLMULL9;     // Select multiply-by-9 to go from 8MHz to 72MHz
   
   RCC->CR |= RCC_CR_PLLON;            // Switch on PLL
   
   // Wait for PLL to start up
   while ((RCC->CR & RCC_CR_PLLRDY) == 0)
      ;
   
   RCC->CFGR |= RCC_CFGR_SW_PLL;       // Select PLL as system clock (72MHz)
   
   // Wait for PLL to select
   while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
      ;
   
   SavedRccCsr = RCC->CSR;
   RCC->CSR |= RCC_CSR_RMVF;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Configure Reset and Clock Control
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                    // Enable clock to GPIO C peripherals on APB2 bus
   
   // Configure PB12, the GPIO pin with the red LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);     // Set PB12 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1); // Configure PB12 as output, 50MHz
   
   // Configure PB13, the GPIO pin with the green LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // Set PB13 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1); // Configure PB13 as output, 50MHz
   
   // Configure PB14, the GPIO pin with the blue LED
   GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);     // Set PB14 to push-pull output mode
   GPIOB->CRH |= (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1); // Configure PB14 as output, 50MHz
   
   // Configure PC13, the GPIO pin with the LED
   GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // Set PC13 to push-pull output mode
   GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1); // Configure PC13 as output, 50MHz
   
   // Configure PC14, the GPIO pin with 500Hz square wave
   GPIOC->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);     // Set PC14 to push-pull output mode
   GPIOC->CRH |= (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1); // Configure PC14 as output, 50MHz
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                    // Enable clock to GPIO A peripherals on APB2 bus
   RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                  // Enable USART1 clock
   
   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;
   
   // Configure PA9, the GPIO pin with alternative function TxD1
   GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);       // Clear configuration bits for PA9
   GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;     // Configure PA9 as alternate function, push-pull
   
   // Configure PA10, the GPIO pin with alternative function RxD1
   GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // Clear configuration bits for PA10
   GPIOA->CRH |= GPIO_CRH_CNF10_1;                        // Configure PA10 as alternate function, floating input
   
   // Configure UART1 - defaults are 1 start bit, 8 data bits, 1 stop bit, no parity
   USART1->CR1 |= USART_CR1_UE;           // Switch on the UART
   USART1->BRR |= (467<<4) | 12;          // Set for 9600 baud (reference manual page 799) 72000000 / (16 * 9600)
   USART1->CR1 |= USART_CR1_RXNEIE;       // Enable Rx Not Empty interrupt
   USART1->CR1 |= USART_CR1_TE;           // Enable transmitter (sends a junk character)
   USART1->CR1 |= USART_CR1_RE;           // Enable receiver
   
   NVIC_EnableIRQ(USART1_IRQn);
}


/* initI2C --- set up the I2C interface */

static void initI2C(void)
{
   // Configure Reset and Clock Control
   RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;                    // Enable clock to I2C1 peripheral on APB1 bus
   RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                    // Enable clock to GPIO B peripherals on APB2 bus
   
   // Configure PB6, the GPIO pin with alternative function SCL1
   GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);       // Clear configuration bits for PB6
   GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF6_1  | GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1;     // Configure PB6 as alternate function, open-drain
   
   // Configure PB7, the GPIO pin with alternative function SDA1
   GPIOB->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);       // Clear configuration bits for PB7
   GPIOB->CRL |= GPIO_CRL_CNF7_1 | GPIO_CRL_CNF7_1  | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1;     // Configure PB7 as alternate function, open-drain
   
   I2C1->CR1 &= ~I2C_CR1_PE;     // Disable I2C peripheral
   
   I2C1->CR2 = 36;      // FREQ field = 36MHz, all interrupts disabled
   I2C1->TRISE = 0x09;  // Set rise time
   I2C1->CCR = 180;     // 100kHz, Standard Mode (Sm)
   
   I2C1->CR1 |= I2C_CR1_PE;      // Enable I2C
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up timer for regular 1ms interrupt
   if (SysTick_Config(72000)) { // 72MHz divided by 1000  (SystemCoreClock / 1000)
      while (1)
         ;
   }
}


int main(void)
{
   uint32_t end;
   uint8_t flag = 0;
   int digit = 0;
   int x = digit * 24;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initI2C();
   initMillisecondTimer();
   
   __enable_irq();   // Enable all interrupts
   
   OLED_begin(MAXX, MAXY);
    
   greyFrame();
    
   updscreen();
   
   printf("\nHello from the STM%dF%d\n", 32, 103);
   
   end = millis() + 500u;
   
   while (1) {
      if (Tick) {
         if (millis() >= end) {
            end = millis() + 500u;
            
            if (flag) {
               GPIOC->BSRR = GPIO_BSRR_BR13; // GPIO pin PC13 LOW, LED on
            }
            else {
               GPIOC->BSRR = GPIO_BSRR_BS13; // GPIO pin PC13 HIGH, LED off
            }
            
            flag = !flag;
            
            printf("millis() = %ld\n", millis());
         }
         
         Tick = 0;
      }
      
      if (UART1RxAvailable()) {
         const uint8_t ch = UART1RxByte();
         
         printf("UART1: %02x\n", ch);
         switch (ch) {
         case 'r':
         case 'R':
            setRect(0, 0, MAXX - 1, MAXY - 1);
            updscreen();
            break;
         case 'q':
         case 'Q':
            setVline(MAXX / 4,       0, MAXY - 1);
            setVline(MAXX / 2,       0, MAXY - 1);
            setVline((MAXX * 3) / 4, 0, MAXY - 1);
            updscreen();
            break;
         case 'i':
         case 'I':
            memcpy(Frame, OLEDImage, sizeof (Frame));
            updscreen();
            break;
         case '0':
            memset(Frame, 0, sizeof (Frame));
            updscreen();
            break;
         case '1':
            digit = 0;
            x = digit * 24;
            break;
         case '2':
            digit = 1;
            x = digit * 24;
            break;
         case '3':
            digit = 2;
            x = digit * 24;
            break;
         case '4':
            digit = 3;
            x = digit * 24;
            break;
         case '5':
            digit = 4;
            x = digit * 24;
            break;
         case 'a':
         case 'A':
            drawSegA(x);
            updscreen();
            break;
         case 'b':
         case 'B':
            drawSegB(x);
            updscreen();
            break;
         case 'c':
         case 'C':
            drawSegC(x);
            updscreen();
            break;
         case 'd':
         case 'D':
            drawSegD(x);
            updscreen();
            break;
         case 'e':
         case 'E':
            drawSegE(x);
            updscreen();
            break;
         case 'f':
         case 'F':
            drawSegF(x);
            updscreen();
            break;
         case 'g':
         case 'G':
            drawSegG(x);
            updscreen();
            break;
         case 'h':
         case 'H':
            drawSegH(x);
            updscreen();
            break;
         case 'p':
         case 'P':
            drawSegDP(x);
            updscreen();
            break;
         }
      }
   }
}

