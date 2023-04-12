#define BUTTON_1  2
#define BUTTON_2  3
#define GREEN_LED 4
#define RED_LED   5
#define BUZZER    6

#define DATA      9   
#define LATCH     8  
#define CLOCK     7  

#define DIGIT_4   10
#define DIGIT_3   11
#define DIGIT_2   12
#define DIGIT_1   13

unsigned char table[]=
{0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c
,0x39,0x5e,0x79,0x71,0x00};

void pinSetup()
{
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

void Button_1_ISR(void);
void Button_2_ISR(void);

void intSetup()
{
    
    attachInterrupt(digitalPinToInterrupt(BUTTON_1), Button_1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_2), Button_2_ISR, RISING);

    // initialize timer1 
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 62500;            // compare match register 16MHz/256
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();             // enable all interrupts
}

void Active_Buzzer()
{
  unsigned char i;
  unsigned char sleep_time = 1; // ms
  
  for(i=0;i<100;i++)
   {
    digitalWrite(BUZZER,HIGH);
    delay(sleep_time);//wait for 1ms
    digitalWrite(BUZZER,LOW);
    delay(sleep_time);//wait for 1ms
    }
}

void Display(unsigned char num, unsigned char dp)
{

  digitalWrite(LATCH, LOW);
  shiftOut(DATA, CLOCK, MSBFIRST, table[num] | (dp<<7));
  digitalWrite(LATCH, HIGH);
}

void displayDriver(unsigned int num)
{
  digitalWrite(DIGIT_1, LOW);
  Display(num % 10,0); //right most digit

  digitalWrite(DIGIT_1, HIGH); 

  digitalWrite(DIGIT_4, LOW); 
  Display((num /10) % 6,0); //second digit from right 
  
  digitalWrite(DIGIT_4, HIGH);

  digitalWrite(DIGIT_3, LOW);
  Display(( int(num / 60) % 60),1); //third digit from right
  
  digitalWrite(DIGIT_3, HIGH);

  digitalWrite(DIGIT_2, LOW);
  Display(( int (num / 60) / 10) % 6, 0); //left most digit
  digitalWrite(DIGIT_2, HIGH);
}

void disp_on()
{
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
    pinSetup();
    intSetup();
    disp_on();

    //Serial.begin(9600);
    // Set PWM frequency for pin 6 to 31.25 kHz //Reduces annoying buzz when timer not active
    TCCR0B = TCCR0B & 0b11111000 | 0x01;
}

void Button_1_ISR() //increment count button
{   
    for(int i = 0; i < debounce*mul; i++){}
    counter++;
    counter = counter == max ? 0 : counter;
    clearLED = debounce*.005;
}

void Button_2_ISR() //togle timmer button
{   
    for(int i = 0; i < debounce*mul; i++){}
    timer = !timer;
    if(counter == 0){
      timer = false; 
      buzzer = false;
    }  
}

ISR(TIMER1_COMPA_vect) // timer compare interrupt service routine
{
    if(timer){
        //counter++;
        //counter = counter == max ? 0 : counter;
        incTimer();
    }
}

void incTimer() {
    if(counter == 0){
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
    if(counter == 0 && buzzer){
        Active_Buzzer();
    }
}