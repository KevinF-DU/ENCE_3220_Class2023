
#define BUTTON_1 2
#define BUTTON_2 3
#define GREEN_LED 4
#define RED_LED 5
#define BUZZER 6

#define DATA 9
#define LATCH 8
#define CLOCK 7

#define DIGIT_4 10
#define DIGIT_3 11
#define DIGIT_2 12
#define DIGIT_1 13

volatile unsigned char gISRFlag2 = 0;
unsigned int gReloadTimer2 = 100;  // corresponds to 0.4ms, reads incomming bits

#define BUFF_SIZE 20
char ginputChar;
char gCommsMsgBuff[BUFF_SIZE];
int buffIndex = 0;
byte gPackageFlag = 0;
byte gProcessDataFlag = 0;

unsigned char table[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00 };

void pinSetup() {
  // LEDs Pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Button Pins
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  // Buzer Pins
  pinMode(BUZZER, OUTPUT);

  // 7-Seg Display
  pinMode(DIGIT_1, OUTPUT);
  pinMode(DIGIT_2, OUTPUT);
  pinMode(DIGIT_3, OUTPUT);
  pinMode(DIGIT_4, OUTPUT);

  // Shift Register Pins
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, OUTPUT);
}

char compareArray(char a[], char b[], int size) {
  int i;
  char result = 1;  // default: the arrays are equal

  for (i = 0; i < size; i++) { //used to identify input msg 
    if (a[i] != b[i]) {
      result = 0;
      break;
    }
  }
  return result;
}

void Button_1_ISR(void);
void Button_2_ISR(void);

void intSetup() {
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_1), Button_1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2), Button_2_ISR, RISING);

  // initialize timer1
  noInterrupts();  // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 62500;            // compare match register 16MHz/256
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

  // Initialize Timer2 (16bit) -> Used for Serial Comms
  // Speed of Timer2 = 16MHz/64 = 250 KHz
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A = gReloadTimer2;  // max value 2^16 - 1 = 65535
  TCCR2A |= (1 << WGM11);
  TCCR2B = (1 << CS11) | (1 << CS10);  // 64 prescaler
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

void Active_Buzzer() {
  unsigned char i;
  unsigned char sleep_time = 1;  // ms

  for (i = 0; i < 100; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(sleep_time);  //wait for 1ms
    digitalWrite(BUZZER, LOW);
    delay(sleep_time);  //wait for 1ms
  }
}

void Display(unsigned char num, unsigned char dp) {

  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, table[num] | (dp << 7));
  digitalWrite(LATCH, HIGH);
}

void displayDriver(unsigned int num) {
  digitalWrite(DIGIT_1, LOW);
  Display(num % 10, 0);  //right most digit

  digitalWrite(DIGIT_1, HIGH);

  digitalWrite(DIGIT_4, LOW);
  Display((num / 10) % 6, 0);  //second digit from right

  digitalWrite(DIGIT_4, HIGH);

  digitalWrite(DIGIT_3, LOW);
  Display((int(num / 60) % 60), 1);  //third digit from right

  digitalWrite(DIGIT_3, HIGH);

  digitalWrite(DIGIT_2, LOW);
  Display((int(num / 60) / 10) % 6, 0);  //left most digit
  digitalWrite(DIGIT_2, HIGH);
}

void disp_on() {
  digitalWrite(DIGIT_1, LOW);
  digitalWrite(DIGIT_2, LOW);
  digitalWrite(DIGIT_3, LOW);
  digitalWrite(DIGIT_4, LOW);
}

#define debounce 10000
#define mul 10
#define max 9999

volatile unsigned int counter = 0;
volatile unsigned long clearLED = false;
volatile boolean timer = false, buzzer = false;

void setup() {

 Serial.begin(9600);

  pinSetup();
  intSetup();
  disp_on();

  //Serial.begin(9600);
  // Set PWM frequency for pin 6 to 31.25 kHz //Reduces annoying buzz when timer not active
  TCCR0B = TCCR0B & 0b11111000 | 0x01;
}

void Button_1_ISR()  //increment count button
{
  for (int i = 0; i < debounce * mul; i++) {}
  counter++;
  counter = counter == max ? 0 : counter;
  clearLED = debounce * .005;
}

void Button_2_ISR()  //togle timmer button
{
  for (int i = 0; i < debounce * mul; i++) {}
  timer = !timer;
  if (counter == 0) {
    timer = false;
    buzzer = false;
  }
}

ISR(TIMER1_COMPA_vect)  // timer compare interrupt service routine
{
  if (timer) {
    //counter++;
    //counter = counter == max ? 0 : counter;
    incTimer();
  }
}

void incTimer() {
  if (counter == 0) {
    timer = false;
    buzzer = true;
    return;
  }
  counter--;
}

void loop() {
  displayDriver(counter);
  digitalWrite(GREEN_LED, timer ? HIGH : LOW);
  digitalWrite(RED_LED, clearLED > 0 ? HIGH : LOW);
  clearLED -= clearLED > 0 ? 1 : 0;
  if (counter == 0 && buzzer) {
    Active_Buzzer();
  }


   char auxMsgBuff[BUFF_SIZE];
  int auxCount = 0;
  unsigned char auxDigit = '0';

  // Attend Timer1 flag - receive commands through serial
  if (gISRFlag2 == 1) {
    // Reset ISR Flag
    gISRFlag2 = 0;

    // Read serial
    ginputChar = Serial.read();

    // If normal character from package
    if (gPackageFlag == 1) {
      gCommsMsgBuff[buffIndex] = ginputChar;
      buffIndex++;

      // Safety mechanism in case "\n" is never sent
      if (buffIndex == BUFF_SIZE) {
        gPackageFlag = 0;
        gProcessDataFlag = 1;
      }
    }

    // If start of the package
    if (ginputChar == '$') {
      gPackageFlag = 1;  // Signal start of package

      // Clear Buffer
      for (int i = 0; i < BUFF_SIZE; i++) {
        gCommsMsgBuff[i] = 0;
      }

      // set gCommsMsgBuff Index to zero
      buffIndex = 0;
    }

    // If end of package
    if ((ginputChar == '\n') && (gPackageFlag == 1)) {
      // Signal end of package
      gPackageFlag = 0;
      gProcessDataFlag = 1;
    }
  }

  // Process serial commands
  if (gProcessDataFlag == 1) {
    gProcessDataFlag = 0;

    if (compareArray(gCommsMsgBuff, "STR", 3) == 1) {
      // Start timer function
      timer == true;
    }

    if (compareArray(gCommsMsgBuff, "STP", 3) == 1) {
      // Stop timer function
      timer == false;
    }

    if (compareArray(gCommsMsgBuff, "GET", 3) == 1) {
      // Send clock status
      Serial.print("$" + (String)DIGIT_2 + "" + (String)DIGIT_3 +":" + (String)DIGIT_4 + "" + (String)DIGIT_1 + "\n");
    }
    // ------
  }
}

/**
 * @brief Timer 2 ISR
 * @param TIMER2_COMPA_vect
 * @return
 */
ISR(TIMER2_COMPA_vect)  // Timer1 interrupt service routine (ISR)
{
  if (Serial.available() > 0) {
    gISRFlag2 = 1;
  }
}

