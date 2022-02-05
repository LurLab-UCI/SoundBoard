#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#include <DueFlashStorage.h>
DueFlashStorage dueFlashStorage;


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();


#define MAX9744_I2CADDR 0x4B


//#define Wire Wire
//#define PIN_WIRE_SDA         (20u)
//#define PIN_WIRE_SCL         (21u)

#define WHITE 0x7

#define  SAMPLES   32
#define  NSAMPLES   8192
#define  CEILING   4096
uint16_t wave0[SAMPLES];
uint16_t wave1[SAMPLES];
uint16_t waven0[NSAMPLES];
uint16_t waven1[NSAMPLES];
uint16_t temp = 0;
float   tfl = 0;  
String volume = "VOLUME";
String str = "12";

uint8_t noise_flag = 0;
uint8_t signal_off = 0;
uint8_t fpointer = 0;

int8_t up_down_freq = 0;
int8_t left_right = 0;

int8_t column = 0;


uint16_t timer = 0;
uint32_t timer_start = 0;
uint8_t  signal_or_delay = 0; 
uint8_t  active = 1; 

// frequency pars
#define  NCHAR  16
#define  NITEMS  9
char* freqstr[][NCHAR]={" Noise           "," 2 kHz          ", " 4 kHz          ", " 8 kHz          ",
"12 kHz          ", "16 kHz          ","24 kHz          ","32 kHz          ","Random          "};
float freq[8] = {1000,2000,4000,8000,12000,16000,24000,32000};

#define TONE_ON_MIN 500
#define TONE_ON_MAX 2000
#define TONE_OFF_MIN 1000
#define TONE_OFF_MAX 5000
#define INITAL_WAIT_TIME_MS 5000

// volume pars
#define  N_VOL_ITEMS  44
int8_t up_down_vol = 0;
uint8_t rbyte = 0;



void setup() {
 Serial.begin(115200);
 Serial.println("------------------------------------");
  Wire.begin();

 Serial.println(PIN_WIRE_SCL);


  if (! setvolume(up_down_vol)) {
    Serial.println("Failed to set volume, MAX9744 not found!");
    while (1);
  }

  rbyte = dueFlashStorage.read(0); 
  if (rbyte == 254)
  {
    Serial.println("---------------==------");
    noise_flag = dueFlashStorage.read(1); 
    up_down_freq = dueFlashStorage.read(2); 
    up_down_vol = dueFlashStorage.read(3); 
    Serial.print(noise_flag);
    Serial.print(up_down_freq);
    Serial.println(up_down_vol);
    Serial.println("---------------==------");
  }

  analogWriteResolution(12);
  CreateSineWaveTable0();
  CreateSineWaveTable1();
  CreateNoise();
  
  setupDAC();
  float freq_hz = 32000; // Target: 200kHz
  setupTC(freq_hz);
  NVIC_EnableIRQ(DACC_IRQn);

  lcd.begin(16, 2);

  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  int time = millis();
  //lcd.print("Initializing ");
  time = millis() - time;
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");
  lcd.setBacklight(WHITE);


//  signal on at first
  timer = random(1000,5000);
  timer_start = millis();
  signal_or_delay = 1; 
  setvolume(up_down_vol);


  if (column == 0)   // freq mode
      update_frequency();
  if (column == 1)   // volume mode
    {
      Serial.print("VOLUME ");
      Serial.println(up_down_vol);
    }
      
     setvolume(up_down_vol);
      
     dacc_enable_channel(DACC, 0);
     dacc_disable_channel(DACC, 0);
     timer_start = millis();
     signal_or_delay = 0;
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print("Initializing ");
     delay(INITAL_WAIT_TIME_MS);

     
      //start signal
      signal_or_delay = 1;
      Serial.println(fpointer);
      
      //see if random sequence
      if (up_down_freq == NITEMS-1)
      {
          fpointer = random(0,NITEMS-1);
      }
      else
      {
        fpointer = up_down_freq;
      }
      timer = random(500,2000);
      update_frequency();            
      dacc_enable_channel(DACC, 0);
      Serial.print("Start ");
      Serial.println(fpointer);
      setvolume(up_down_vol);
      
      timer_start = millis();
     
 
}

void loop() {

if (active == 1)
{
// check timer
    if ((millis()-timer_start) > timer)

      {
        if (signal_or_delay == 1)
        {
            //stop signal
            signal_or_delay = 0;
             dacc_disable_channel(DACC, 0);
            update_frequency();            
             Serial.println("Stop");
             timer = random(TONE_OFF_MIN,TONE_OFF_MAX);
        }
        else
        {
            //start signal
            signal_or_delay = 1;
            Serial.println(fpointer);
            
            //see if random sequence
            if (up_down_freq == NITEMS-1)
            {
                fpointer = random(1,NITEMS-1);
            }
            else
            {
              fpointer = up_down_freq;
            }
            update_frequency();            
            dacc_enable_channel(DACC, 0);
            Serial.print("Start ");
            Serial.println(fpointer);
            timer = random(TONE_ON_MIN,TONE_ON_MAX);
        }
        timer_start = millis();
      }
}

  
  // print the number of seconds since reset:
//  lcd.print(millis()/1000);

  uint8_t buttons = lcd.readButtons();

  if (buttons)
  {
     decode_button_push(buttons);
  }



  
    delay(100);
  }
  



//--------------------------------------------------------------------------------------------

void decode_button_push(uint8_t buttons)
{
    lcd.clear();
    lcd.setCursor(0,0);

   Serial.println("Button");

//    left or right
    if (buttons & BUTTON_LEFT)
    {
      column = (column - 1);
      if (column <0)
      {
          column = 0;
          Serial.print("column set to  ");
          Serial.println(column);
      }
    }
    
    if (buttons & BUTTON_RIGHT)
    {
      column = (column + 1);
      if (column  > 1)
      {
          column  = 1;
          Serial.print("column set to  ");
          Serial.println(column);
      }
    }

   if (column == 0)   // freq mode
   {
          
        if (buttons & BUTTON_UP) 
        {
          up_down_freq = (up_down_freq + 1)%NITEMS;
          Serial.println("up");
        }
          
        if (buttons & BUTTON_DOWN) {
          up_down_freq = up_down_freq - 1;
          Serial.println("down");
          if (up_down_freq <0)
                up_down_freq = NITEMS-1;
      
          }
          fpointer = up_down_freq;
          update_frequency();            
   }
  else   // volume mode
   {
    
        if (buttons & BUTTON_UP)
        {
          up_down_vol = (up_down_vol + 1)%N_VOL_ITEMS;
    
          Serial.print("VOLUME ");
          Serial.println(up_down_vol);
        }
        if (buttons & BUTTON_DOWN)
        {
          up_down_vol = up_down_vol - 1;
          if (up_down_vol <0)
                up_down_vol = 0;
          Serial.print("VOLUME ");
          Serial.println(up_down_vol);
        }
          lcd.print(String("VOLUME " + String(up_down_vol,DEC)));
          setvolume(up_down_vol);
        lcd.setCursor(0,1);
          if (active == 1)
              lcd.print("LOOP RUNNING");
          else
              lcd.print("LOOP STOPPED");
   }

    
  if (buttons & BUTTON_SELECT) 
     {

      if (signal_off == 0)
      {
        lcd.setCursor(0,1);
        lcd.print("LOOP STOPPED");
        active = 0;
        dacc_disable_channel(DACC, 0);
//        dacc_disable_channel(DACC, 1);
        signal_off = 1;
      }
      else
      {
        lcd.setCursor(0,1);
        lcd.print("LOOP RUNNING");
        active = 1;
        dacc_enable_channel(DACC, 0);
//        dacc_enable_channel(DACC, 1);
        signal_off = 0;
      }
     }

//   save settings after every button push
     dueFlashStorage.write(0,254);
     dueFlashStorage.write(1,noise_flag);
     dueFlashStorage.write(2,up_down_freq);
     dueFlashStorage.write(3,up_down_vol);

     

}
//--------------------------------------------------------------------------------------------



void CreateSineWaveTable0() {
 for(int i = 0; i < SAMPLES; i++) 
  {
       tfl =  CEILING/2 + (CEILING/2)*sin(float(2*3.1416)*float(i+1)/SAMPLES);
       //Serial.println(tfl);
       temp = round(tfl);
//       Serial.println(temp);
       if (temp>(CEILING-1))
          temp = CEILING-1;

      wave0[i] = temp;
  }
}


void CreateSineWaveTable1() {
 for(int i = 0; i < SAMPLES; i++) 
  {
       tfl =  CEILING/2 + (CEILING/2)*sin(float(2*3.1416)*float(i+1)/SAMPLES);
       //Serial.println(tfl);
       temp = round(tfl);
//       Serial.println(temp);
       if (temp>(CEILING-1))
          temp = CEILING-1;

      wave1[i] = temp;
  }
}




void CreateNoise() {
 for(int i = 0; i < NSAMPLES; i++) 
  {
//       tfl =  CEILING/2 + (CEILING/2)*sin(float(2*3.1416)*float(i+1)/SAMPLES);
//       tfl =  CEILING/2 + (CEILING/2)*sin(float(2*3.1416)*float(i+1)/SAMPLES);
        tfl = random(0, 4095);
       //Serial.println(tfl);
       temp = round(tfl);
//       Serial.println(temp);
       if (temp>(CEILING-1))
          temp = CEILING-1;

      waven0[i] = temp;
      waven1[i] = temp;
  }
}

void CreateTriangleWaveTable1() {
  for(int i = 0; i < SAMPLES; i++) { int16_t v = (((1.0 / (SAMPLES - 1)) * (SAMPLES - 1 - i)) * CEILING); if (i > round(SAMPLES/2)) v*=-1;
    wave1[i] = v;
  }
}

// ------------------------------------------------------

// Setting the volume is very simple! Just write the 6-bit
// volume to the i2c bus. That's it!
boolean setvolume(int8_t v) {
  // cant be higher than 63 or lower than 0
  if (v > 63) v = 63;
  if (v < 0) v = 0;
//  tcaselect(2);
//  Serial.print("Setting volume to ");
//  Serial.println(v);
  Wire.beginTransmission(MAX9744_I2CADDR);
  Wire.write(v);
  if (Wire.endTransmission() == 0) 
  {
    Serial.println("true");
    return true;
  }
  else
  {
    Serial.println("false");
    return false;
  }
}



void       update_frequency()            
{
      //Serial.println(fpointer);
      if (column == 0)
      {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(freqstr[column][fpointer]);

      lcd.setCursor(0,1);
      if (active == 1)
        lcd.print("LOOP RUNNING");
      else
        lcd.print("LOOP STOPPED");
      }

      if (fpointer > 0)
      {

//        if (fpointer == NITEMS-1)
//          noise_flag = 0;
//        else
        {
          noise_flag = 0;
          setupTC(freq[fpointer]);
        }
      }
      else
      {
        noise_flag = 1;
        float freq_hz = 16000;
        setupTC(freq_hz);
      }
      NVIC_EnableIRQ(DACC_IRQn);
      dacc_enable_interrupt(DACC, DACC_IER_ENDTX);
}


// Incantations for DAC set-up for analogue wave using DMA and timer interrupt.
// http://asf.atmel.com/docs/latest/sam3a/html/group__sam__drivers__dacc__group.html
void setupDAC() {
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ;   // Start clocking DAC.
  dacc_reset(DACC);
  dacc_set_transfer_mode(DACC, 0);
  dacc_set_power_save(DACC, 0, 1);              // sleep = 0, fast wakeup = 1
  dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_set_trigger(DACC, 1);
//  dacc_set_channel_selection(DACC, 1);
//  dacc_enable_channel(DACC, 1);
  dacc_set_channel_selection(DACC, 0);
  //dacc_enable_channel(DACC, 0);
   dacc_disable_channel(DACC, 1);
  NVIC_DisableIRQ(DACC_IRQn);
  NVIC_ClearPendingIRQ(DACC_IRQn);
  NVIC_EnableIRQ(DACC_IRQn);
  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);
  DACC->DACC_PTCR = 0x00000100;
}

void DACC_Handler(void) {
  if (noise_flag == 1)
  {
    DACC->DACC_TNPR = (uint32_t) waven0;
    DACC->DACC_TNCR = NSAMPLES;                // Number of counts until Handler re-triggered
  }
  else
  {
    DACC->DACC_TNPR = (uint32_t) wave0;
    DACC->DACC_TNCR = SAMPLES;                // Number of counts until Handler re-triggered
  }
}

// System timer clock set-up for DAC wave.
void setupTC (float freq_hz) {  
  int steps = (420000000UL / freq_hz) / (10*SAMPLES);
  pmc_enable_periph_clk(TC_INTERFACE_ID);
  TcChannel * t = &(TC0->TC_CHANNEL)[0];
  t->TC_CCR = TC_CCR_CLKDIS;                // Disable TC clock.
  t->TC_IDR = 0xFFFFFFFF;
  t->TC_SR;                                 // Clear status register.
  t->TC_CMR =                               // Capture mode.
              TC_CMR_TCCLKS_TIMER_CLOCK1 |  // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 48MHz).
              TC_CMR_WAVE |                 // Waveform mode.
              TC_CMR_WAVSEL_UP_RC;          // Count up with automatic trigger on RC compare.
  t->TC_RC = steps;                         // Frequency.
  t->TC_RA = steps /2;                      // Duty cycle (btwn 1 and RC).
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | 
              TC_CMR_ACPA_CLEAR |           // Clear TIOA on counter match with RA0.
              TC_CMR_ACPC_SET;              // Set TIOA on counter match with RC0.
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;  // Enables the clock if CLKDIS is not 1.
}
