// *****************************************************************
// RTTY-маяк
// Передача текста RTTY на скорости 45.45 бод (бит=22 мс)
// + данные напряжения питания и температуры
// частоты марк/спейс 3590.830/3591.000
// RU0AOG ver. 1.02 2025-09-17
// *****************************************************************

#define INVERT_OFF                   // INVERT_ON - инверсия, INVERT_OFF - без инверсии
#define VOLT_CAL         148         // калибровка вольтметра (100...200)
#define SI5351_I2C_ADDR 0x60         // адрес si5351
#define PTT_PIN            5         // пин активности передачи
#define SI_PWR_PIN         6         // пин питания si5351
#define DS_GND_PIN         7         // пин общего DS18B20
#define DS_SIG_PIN         8         // сигнальный пин DS18B20
#define DS_PWR_PIN         9         // пин питания DS18B20
#define RTTY_PIN           15        // пин сигнала RTTY
#define PWR_PIN            A7        // пин контроля питания
#define DS18B20_CONVERT_T  0x44
#define DS18B20_READ       0xBE


// Подключаем библиотеки
#include "i2c.h"
#include <util/delay.h>

volatile uint8_t  snd = 0;           // флаг разрешения передачи
volatile uint8_t  ti = 0;            // счётчик тиков таймера
volatile uint8_t  baudotTX;      // регистр передаваемого символа
volatile uint8_t  baudotTX1;     // подготовка кода Бодо передаваемого символа
volatile uint8_t  fig_TX;        // флаг передаваемого символа (0 - цифра, 1 - ЛАТ, 2 - РУС)
volatile uint8_t  fig_TX_prv;    // флаг предыдущего символа   (0 - цифра, 1 - ЛАТ, 2 - РУС)
boolean           crLf;
uint8_t           mybatt = 0;
int16_t           mytemp = 0;


void si5351_write_reg(uint8_t reg, uint8_t data) {
  i2c_begin_write(SI5351_I2C_ADDR);
  i2c_write(reg);
  i2c_write(data);
  i2c_end();
}

#define ONEWIRE_PIN_INPUT()   (DDRB &= ~(1 << 0)) // (DDRB &= ~pin)  // направление передачи, 0-вход
#define ONEWIRE_PIN_OUTPUT()  (DDRB |=  (1 << 0))  // (DDRB |= pin)  // направление передачи, 1-выход
#define ONEWIRE_PIN_LOW()     (PORTB &= ~(1 << 0)) //(PORTB &= ~pin) // состояние выхода 0
#define ONEWIRE_PIN_HIGH()    (PORTB |=  (1 << 0))  //(PORTB |= pin) // состояние выхода 1
#define ONEWIRE_PIN_READ()    (PINB & (1 << 0))    //(PINB & pin) // чтение
#define ONEWIRE_RESET_RETRIES_MAX (128)
#define ONEWIRE_SEARCH_ROM    0xF0
#define ONEWIRE_READ_ROM      0x33
#define ONEWIRE_MATCH_ROM     0x55
#define ONEWIRE_SKIP_ROM      0xCC
#define ONEWIRE_ALARM_SEARCH  0xEC


volatile uint8_t REGS0[26] = {0x03, 0x10, 0x11, 0x12, 0xB7, 0x95, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0xB1, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x10, 0x03};
volatile uint8_t DATA0[26] = {0xFF, 0x80, 0x80, 0x80, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA8, 0xE8, 0x00, 0x7B, 0xAB, 0xD4, 0x44, 0x08, 0x0F, 0x00};

volatile uint8_t REGS1[12] = {0x10, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x10, 0x03};  // 3 590.830 kHz - SPACE
volatile uint8_t DATA1[12] = {0x80, 0xB2, 0xAC, 0x00, 0x7B, 0x55, 0xD5, 0x11, 0xE4, 0x0F, 0x00};

volatile uint8_t REGS2[12] = {0x10, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x10, 0x03};  // 3 591.000 kHz - MARK
volatile uint8_t DATA2[12] = {0x80, 0xB2, 0xD6, 0x00, 0x7B, 0x53, 0xDB, 0xA9, 0x1E, 0x0F, 0x00};

volatile uint8_t REGS3[2] = {0x03, 0x10};
volatile uint8_t DATA3[2] = {0xFF, 0x80};


void SI_init() {
  for (int i=0; i<26; i++){
  si5351_write_reg(REGS0[i],DATA0[i]);
  }
  for (int i=0; i<5; i++){         // si5351 - off
  si5351_write_reg(REGS3[i],DATA3[i]);
  }
}

void write_SPACE() {
  for (int i=1; i<=10; i++){        // SPACE
  #ifdef INVERT_ON
    si5351_write_reg(REGS1[i],DATA1[i]);
  #else
    si5351_write_reg(REGS2[i],DATA2[i]);
  #endif
  }
  #ifdef INVERT_ON
    digitalWrite(RTTY_PIN, LOW);
  #else
    digitalWrite(RTTY_PIN, HIGH);
  #endif
}

void write_MARK() {
  for (int i=1; i<=10; i++){        // MARK
  #ifdef INVERT_ON
    si5351_write_reg(REGS2[i],DATA2[i]);
  #else
    si5351_write_reg(REGS1[i],DATA1[i]);
  #endif
  }
  #ifdef INVERT_ON
    digitalWrite(RTTY_PIN, HIGH);
  #else
    digitalWrite(RTTY_PIN, LOW);
  #endif
}

void write_STOP() {
  for (int i=0; i<=1; i++){        // si5351 - off
  si5351_write_reg(REGS3[i],DATA3[i]);
  }
}

void ISRTimer1 (){      // генератор тиков 45.45 Бод (1 тик = 1000 мкс = 1 мс)
   cli();
   TCCR1A = 0;
   TCCR1B = 0;
   OCR1A  = 0;                // сброс регистров
   TCCR1B |= (1<<WGM12);      // режим СТС
   TCCR1B |= (1<<CS10);       // установить коэффициент деления 1
   OCR1A = 15999;             // 16.000.000 / 1.000 - 1 = 15.999
   TIMSK1 |= (1 << OCIE1A);   // включить прерывание по совпадению таймера
   sei();
}


void setup() {
  i2c_init(400000);
  ISRTimer1();
  pinMode(PTT_PIN,      OUTPUT);          // пин активности передачи
  pinMode(SI_PWR_PIN,   OUTPUT);          // пин питания si5351
  pinMode(DS_GND_PIN,   OUTPUT);          // пин общего DS18B20
  pinMode(DS_PWR_PIN,   OUTPUT);          // пин питания DS18B20
  pinMode(RTTY_PIN,      OUTPUT);         // пин RTTY_PIN
  digitalWrite(DS_GND_PIN, LOW);
  digitalWrite(DS_PWR_PIN, HIGH);
  digitalWrite(PTT_PIN, HIGH);
  digitalWrite(SI_PWR_PIN, LOW);
  digitalWrite(RTTY_PIN, LOW);
}

ISR(TIMER1_COMPA_vect) {
  // генерация тиков скорости бод
  //cli();

   if(snd == 1) {                             // выполняется передача символа
     switch(ti) {                         // в зависимости от текущего тика:
        case 0:   // стартовый бит
        static uint8_t bit0;              // создать блок локальных статических пременных
        static uint8_t bit1;
        static uint8_t bit2;
        static uint8_t bit3;
        static uint8_t bit4;
        static uint8_t bd1;
          bd1 = baudotTX;                 // разобрать слово на биты
          bit0 = bd1 & B00001;
          bit1 = (bd1 >> 1);
          bit1 = bit1 & B00001;
          bit2 = (bd1 >> 2);
          bit2 = bit2 & B00001;
          bit3 = (bd1 >> 3);
          bit3 = bit3 & B00001;
          bit4 = (bd1 >> 4);
          bit4 = bit4 & B00001;
           write_SPACE();
           break;
        case 22:   // бит0
           if  (bit0 == 1) {write_MARK();}
           else            {write_SPACE();}
           break;
        case 44:   // бит1
           if  (bit1 == 1) {write_MARK();}
           else            {write_SPACE();}
           break;
        case 66:   // бит2
           if  (bit2 == 1) {write_MARK();}
           else            {write_SPACE();}
           break;
        case 88:   // бит3
           if  (bit3 == 1) {write_MARK();}
           else            {write_SPACE();}
           break;
        case 110:  // бит4
           if  (bit4 == 1) {write_MARK();}
           else            {write_SPACE();}
           break;
        case 132:                                  // с 132 тика - начало стоп-бита х1,5
           write_MARK();
           break;
        case 165:                                  // с 165 тика - конец передачи
           snd = 0;
           ti = 0;
           //write_STOP();
           break;
        }
   ti++;
   }
  //sei();
}

void Power_down() {
  delay(100);
  TWCR &= ~(1 << TWEN);
  TWCR &= ~(1 << TWINT);
  delay(100);
  digitalWrite(SI_PWR_PIN, LOW);
  delay(100);
}

void Power_up() {
  digitalWrite(SI_PWR_PIN, HIGH);
  delay(100);
  TWCR = (1 << TWINT); // сброс TWI модуля
  delay(100);
  TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEA) | (1<<TWEN);
  delay(100);
  i2c_init(400000);
  delay(100);
  SI_init();
  delay(100);
}

void battmeter() { // Измеритель напряжения питания
    mybatt = map(analogRead(PWR_PIN), 0, 1023, 0, VOLT_CAL);
}

uint8_t onewire_bit(uint8_t value) {
  uint8_t sreg;
  sreg = SREG;
  cli();
    ONEWIRE_PIN_OUTPUT();
    _delay_us(1);
    if (value) {ONEWIRE_PIN_INPUT();}
  _delay_us(14);
  value = !(ONEWIRE_PIN_READ() == 0);
    _delay_us(45);
    ONEWIRE_PIN_INPUT();
    SREG = sreg;
  return value;
}

uint8_t onewire_write(uint8_t value) {
  uint8_t i, r;
  for (i = 0; i < 8; ++i) {
        r = onewire_bit(value & 0x01);
    value >>= 1;
        if (r) {value |= 0x80;}
    }
    return value;
}

uint8_t onewire_read() {
  return onewire_write(0xff);
}

void tempmeter() { // Измеритель напряжения питания
  uint8_t msb, lsb;
  int8_t sign = 1;
  uint16_t t;
  onewire_reset();
  onewire_write(ONEWIRE_SKIP_ROM);
  onewire_write(DS18B20_CONVERT_T);
  onewire_reset();
  onewire_write(ONEWIRE_SKIP_ROM);
  onewire_write(DS18B20_READ);
  lsb = onewire_read();
  msb = onewire_read();
  t = ((uint16_t)msb << 8) | lsb;
  if ((msb & 0xf8) == 0xf8) {   
    t = (65536 - t);
    sign = -1;
  }
  mytemp =  sign * (((uint16_t)t * 10U) / 16U);
}

uint8_t onewire_reset() {
  uint8_t retval, retries;

  ONEWIRE_PIN_LOW();
  ONEWIRE_PIN_INPUT();

  retries = ONEWIRE_RESET_RETRIES_MAX;
  while (!ONEWIRE_PIN_READ()) {
    if (--retries == 0) {
      return (2);
    }
    _delay_us(1);
  }

  ONEWIRE_PIN_OUTPUT();
  _delay_us(480);
  ONEWIRE_PIN_INPUT();
  _delay_us(66);
  retval = ONEWIRE_PIN_READ();
  _delay_us(414);

  return (retval);
}

void loop() {
  Power_up();
    battmeter();
    if (mybatt > 89) {   // включить передачу, если напряжение питания более 9 В
      // передать сообщение
      rttyTxStart();
      rttyTx("VVV ");
      rttyTxDouble("DE RTTY BEACON, ");
      rttyTxDouble("BAND - 80M, ");
      rttyTxDouble("POWER - 3 WATTS, ");
      rttyTxVoltage();
      rttyTxDouble("ANTENNA - 22M LONG WIRE, ");
      rttyTxDouble("WW LOCATOR QTH - NO65JX, ");
      rttyTxTemperature();
      rttyTxDouble("NOCALL, ");
      rttyTxDouble("TESTING RTTY BEACON, ");
      rttyTx("OVER.");
      rttyTxStop();
    }
    Power_down();
    for(byte m = 1; m <= 5; m++) {  // пауза
      delay(60000);
    }
}

void rttyTxTemperature() {
  // процедура повторной передачи сообщения
  for (int k=0; k<=1; k++){
      tempmeter();
      rttyTx("TEMPERATURE IS ");
      tempmeter();
      int tmp1 = mytemp/10;
      int tmp2 = mytemp - tmp1*10;
      char chartmp1[3];
      char chartmp2[3];
      itoa(tmp1, chartmp1, 10);
      itoa(tmp2, chartmp2, 10);
      rttyTx(chartmp1);
      rttyTx(".");
      rttyTx(chartmp2);
      rttyTx(" C, ");
     }
}

void rttyTx(char message[]) {
  // процедура передачи сообщения
  // цикл по символам сообщения
   for(byte j = 0; j < strlen(message); j++) {
     codeBodo(message[j]);                    // закодировать символ, код -> baudotTX1, регистр -> fig_TX, передать символ
     }
}

void rttyTxDouble(char message[]) {
  // процедура повторной передачи сообщения
  // цикл по символам сообщения
  for (int k=0; k<=1; k++){
   for(byte j = 0; j < strlen(message); j++) {
     codeBodo(message[j]);}                    // закодировать символ, код -> baudotTX1, регистр -> fig_TX, передать символ
     }
}

void rttyTxVoltage() {
  // процедура повторной передачи сообщения
  for (int k=0; k<=1; k++){
      rttyTx("VOLTAGE IS ");
      battmeter();
      int bat1 = mybatt/10;
      int bat2 = mybatt - bat1*10;
      char charbat1[3];
      char charbat2[3];
      itoa(bat1, charbat1, 10);
      itoa(bat2, charbat2, 10);
      rttyTx(charbat1);
      rttyTx(".");
      rttyTx(charbat2);
      rttyTx(" VOLTS, ");
     }
}



void rttyTxStart(){
    digitalWrite(PTT_PIN, LOW);              // активировать PTT_PIN - индикация процесса передачи
    delay(100);
    write_MARK();                             // передавать MARK 0.5 с
    delay(500);
  // начало сообщения
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF
    rttyTxDirect(B11011);                     // переключиться на регистр символов
    rttyTxDirect(B11011);                     // переключиться на регистр символов
    rttyTx("------");
    rttyTxDirect(B11111);                     // переключиться на регистр ЛАТ
    rttyTxDirect(B11111);                     // переключиться на регистр ЛАТ
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF    
}

void rttyTxStop(){
  // конец сообщения
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF
    rttyTx("------");
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF
    write_MARK();                             // передавать MARK 0.5 с
    delay(500);
    digitalWrite(PTT_PIN, HIGH);               // выключить PTT_PIN - индикация конца передачи
    write_STOP();
}

void rttyTxDirect(uint8_t command) {
 // прямая передача кода
 while (snd == 1) {}         // ждать, пока не окончится передача предыдущего символа
   baudotTX = command;
   ti = 0;                    // сбросить счётчик тиков таймера1
   snd = 1;                   // запустить передачу символа (сбросится в 0 по завершении передачи)
   while (snd == 1) {}          // ждать, пока не окончится передача
}

void rttyTxSymbol() {
 // передача символа
 while (snd == 1) {}          // ждать, пока не окончится передача предыдущего символа
  if (fig_TX != -1){ 
  if (fig_TX != fig_TX_prv) { // если регистр изменился, то сначала передать код регистра
    switch(fig_TX) {          //   0 - ЛАТ, 1 - цифра, 2 - РУС
      case 0: baudotTX = B11111; break;  // 0 - ЛАТ
      case 1: baudotTX = B11011; break;  // 1 - цифры/символы
      case 2: baudotTX = B00000; break;} // 2 - РУС
    ti = 0;                    // сбросить счётчик тиков таймера1
    snd = 1;                   // запустить передачу символа (сбросится в 0 по завершении передачи)
    while (snd == 1) {}        // ждать, пока не окончится передача
      fig_TX_prv = fig_TX;     // обновить состояние последнего регистра
    }
   baudotTX = baudotTX1;      // вписать в регистр передачи передаваемый символ
   ti = 0;                    // сбросить счётчик тиков таймера1
   snd = 1;                   // запустить передачу символа (сбросится в 0 по завершении передачи)
   while (snd == 1) {}          // ждать, пока не окончится передача
  }
}


void codeBodo(uint8_t chTX) {
  fig_TX = -1;
  baudotTX1 = 0;
  switch(chTX) {       
      case 'A': baudotTX1 = B00011; fig_TX = 0; break;
      case 'B': baudotTX1 = B11001; fig_TX = 0; break;
      case 'C': baudotTX1 = B01110; fig_TX = 0; break;
      case 'D': baudotTX1 = B01001; fig_TX = 0; break;
      case 'E': baudotTX1 = B00001; fig_TX = 0; break;
      case 'F': baudotTX1 = B01101; fig_TX = 0; break;
      case 'G': baudotTX1 = B11010; fig_TX = 0; break;
      case 'H': baudotTX1 = B10100; fig_TX = 0; break;
      case 'I': baudotTX1 = B00110; fig_TX = 0; break;
      case 'J': baudotTX1 = B01011; fig_TX = 0; break;
      case 'K': baudotTX1 = B01111; fig_TX = 0; break;
      case 'L': baudotTX1 = B10010; fig_TX = 0; break;
      case 'M': baudotTX1 = B11100; fig_TX = 0; break;
      case 'N': baudotTX1 = B01100; fig_TX = 0; break;
      case 'O': baudotTX1 = B11000; fig_TX = 0; break;
      case 'P': baudotTX1 = B10110; fig_TX = 0; break;
      case 'Q': baudotTX1 = B10111; fig_TX = 0; break;
      case 'R': baudotTX1 = B01010; fig_TX = 0; break;
      case 'S': baudotTX1 = B00101; fig_TX = 0; break;
      case 'T': baudotTX1 = B10000; fig_TX = 0; break;
      case 'U': baudotTX1 = B00111; fig_TX = 0; break;
      case 'V': baudotTX1 = B11110; fig_TX = 0; break;
      case 'W': baudotTX1 = B10011; fig_TX = 0; break;
      case 'X': baudotTX1 = B11101; fig_TX = 0; break;
      case 'Y': baudotTX1 = B10101; fig_TX = 0; break;
      case 'Z': baudotTX1 = B10001; fig_TX = 0; break;
      case '0': baudotTX1 = B10110; fig_TX = 1; break;
      case '1': baudotTX1 = B10111; fig_TX = 1; break;
      case '2': baudotTX1 = B10011; fig_TX = 1; break;
      case '3': baudotTX1 = B00001; fig_TX = 1; break;
      case '4': baudotTX1 = B01010; fig_TX = 1; break;
      case '5': baudotTX1 = B10000; fig_TX = 1; break;
      case '6': baudotTX1 = B10101; fig_TX = 1; break;
      case '7': baudotTX1 = B00111; fig_TX = 1; break;
      case '8': baudotTX1 = B00110; fig_TX = 1; break;
      case '9': baudotTX1 = B11000; fig_TX = 1; break;
      case '-': baudotTX1 = B00011; fig_TX = 1; break;
      case '?': baudotTX1 = B11001; fig_TX = 1; break;
      case ':': baudotTX1 = B01110; fig_TX = 1; break;
      case '(': baudotTX1 = B01111; fig_TX = 1; break;
      case ')': baudotTX1 = B10010; fig_TX = 1; break;
      case '.': baudotTX1 = B11100; fig_TX = 1; break;
      case ',': baudotTX1 = B01100; fig_TX = 1; break;
      case '/': baudotTX1 = B11101; fig_TX = 1; break;
      case '\r':baudotTX1 = B01000; fig_TX = 0; //CR
              crLf = 1; break;
      case '\n':baudotTX1 = B01000; fig_TX = 0; //LF
              crLf = 1; break;
      case ' ': baudotTX1 = B00100; fig_TX = 0; break; //SPACE
      default:  break;
    }
    rttyTxSymbol();                          // передать символ
}
