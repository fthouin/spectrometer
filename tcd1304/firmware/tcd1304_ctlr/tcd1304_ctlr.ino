  
  
  #include "ADC.h"
  #include <ADC_util.h>
  char dataIn[10];
  char operation[5];
  char quant[4];
  boolean dataRdy=false;
  int analogPin = A0;
  ADC *adc = new ADC(); // adc object
  uint16_t ccd_data[4000]; 
  int intTime=0;
  int number=0;
  void setup() {
    
   setReadTime(2000);
  
    adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 
    adc->adc0->setAveraging(1);                 // set number of averages
    adc->adc0->setResolution(12);               // set bits of resolution
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); 
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
    adc->adc0->singleMode();              // is needed for analogReadFastADC0()
    Serial.begin(9600);
    
  }
   
  void loop() {
  
    if (Serial.available()){
      dataRdy=processIncomingByte(dataIn);
    }
    if(dataRdy){
      //getSubset(0,4,operation,dataIn);
      if(dataIn[0]=='T'){
       //Serial.println("I should goto...");
        getSubset(1,4,quant,dataIn);
        intTime=atoi(quant);
        if (intTime<20){
          intTime=20;
           }
           else if(intTime>200){
            intTime=200;
           }
           else{
            number=ms2Number(intTime);
           }
           setReadTime(number);
        //Serial.println(atoi(quant));
      }
      else if(dataIn[0]=='R'){
         while(PIND & !(1<<4)){}//Wait for the next pulse
         while(PIND & (1<<4)){}//Wait for the ICG pulse to end
         ccd_data[1] = analogReadFastADC0(analogPin); 
  
         
         for (int i=0; i<3694; i++){ 
             ccd_data[i] = analogReadFastADC0(analogPin);
             while(digitalReadFast(4)){}//Wait for the ICG pulse to end 
             while(digitalReadFast(4)){}//Wait for the next pulse
  //           while(digitalReadFast(4)){}//Wait for the ICG pulse to end
         }
         for (int i=0;i<3694;i++){
          Serial.print(i);
          Serial.print('\t');
          Serial.println(ccd_data[i]);
  
         }
      }
      else if(dataIn[0]=='W'){
         //Serial.println("I should tell what I feel...");
         getSubset(1,3,quant,dataIn);
        
         //Serial.println(quant);
      }
      
      else{
         Serial.println("wtf...");
      }
      //Serial.println(operation);
      dataRdy=false;
      dataIn[0]='\0';
    }
  }
  
  boolean processIncomingByte(char dataIn[]){
      char endMarker='\n';
      char rc;
      int iter=0;
      while(dataIn[iter]!='\0'){
        iter++;//Finds the end of the string to append it the read character
    }
    rc=Serial.read();
    dataIn[iter]=rc;
    dataIn[iter+1]='\0';
    if(rc==endMarker){
      return true;
    }
    else{
      return false;
    }
    
  }
  void getSubset(int debut, int fin, char tabOut[], char tabIn[]){
    int k=0;
    for(int i=debut;i<=fin;i++){
      tabOut[k]=tabIn[i];
      k++;
    }
    tabOut[k]='\0';
  }
  
  void setReadTime(unsigned int number0){
     
    // The order of setting the TPMx_SC, TPMx_CNT, and TPMx_MOD
    // seems to matter. You must clear _SC, set _CNT to 0, set _MOD
    // to the desired value, then you can set the bit fields in _SC.
    SIM_SCGC6|=0x03000000; //enable FTM0 and FTM0 module clock
    SIM_SCGC5=SIM_SCGC5|0x3E00; //enable port A/B/C/D/E clock
    // Clear TPM0_SC register (p. 572)
    FTM0_SC = 0;
    FTM1_SC = 0;
    
    // Reset the TPM0_CNT counter (p. 574)
    FTM0_CNT = 0;
    FTM1_CNT = 0;
    
    // Set overflow value (modulo) (p.574)
    //FTM0_MOD = 0xFFFF;
    //double number0=65;
    double number1=40;
  
  
  
  FTM0_FMS=0x00; //clear the WPEN so that WPDIS is set in FTM0_MODE reg
  FTM0_MODE|=0x05; //enable write the FTM CnV register
  //FTM0_MOD=1000;
    
    // Set TPM0_C4SC register (Teensy LC - pin 6) (p. 575)
    // As per the note on p. 575, we must disable the channel
    // first before switching channel modes. We also introduce
    // a magical 1 us delay to allow the new value to take.
    // Bits | Va1ue | Description
    //  7   |    1  | CHF: Clear Channel Flag
    //  6   |    1  | CHIE: Enable Channel Interrupt
    // 5-4  |   10  | MS: Edge-aligned PWM
    // 3-2  |   10  | ELS: Set on reload, clear on match
    //  1   |    0  | Reserved
    //  0   |    0  | DMA: Disable DMA
    FTM0_C4SC = 0;
    FTM0_C3SC = 0;
    FTM0_C2SC = 0;
    delayMicroseconds(1);
    FTM1_C1SC = 0;
    delayMicroseconds(1);
    FTM0_C4SC = 0b101000;
  //  FTM0_C2SC = 0b101000;
  //  FTM0_C3SC = 0b101000;
  //  FTM0_COMBINE=0b1100000000;
  
  
    FTM0_C6SC = 0b101000;
    FTM0_C7SC = 0b101000;
    FTM0_COMBINE=0b11000000000000000000000000;
    FTM0_CNTIN=0x00;
    
    FTM1_C1SC = 0b11101000;
    //FTM0_COMBINE=0x0303;
    //FTM0_MOD=0b1000;
  //Set Duty cycle to 0.5
    FTM0_C4V = 4;
    //FTM0_C6V = 2;
    FTM0_C7V = 2;
    
    FTM1_C1V = number1/2;
    
    FTM0_MOD=number0;
    FTM1_MOD=number1;
  
  //Setting SC registers
      // Bits | Va1ue | Description
    //  8   |    0  | DMA: Disable DMA
    //  7   |    1  | TOF: Clear Timer Overflow Flag
    //  6   |    1  | TOIE: Enable Timer Overflow Interrupt
    //  5   |    0  | CPWMS: TPM in up counting mode
    // 4-3  |   01  | CMOD: Counter incrememnts every TPM clock
    // 2-0  |  000  | PS: Prescale = 1
    FTM0_SC = 0b011001111;
    FTM1_SC = 0b011001000;
  
  
    // Set PORTD_PCR4 register
    // Bits | Value | Description
    // 10-8 |  100  | MUX: Alt 4 attach to FTM0_CH4
    //  7   |    0  | Reserved
    //  6   |    0  | DSE: Low drive strength
    //  5   |    0  | Reserved
    //  4   |    0  | PFE: Disable input filter
    //  3   |    0  | Reserved
    //  2   |    1  | SRE: Slow output slew rate
    //  1   |    0  | PE: Disable pull-up/down
    //  0   |    0  | PS: Internal pull-down
  
    PORTD_PCR4 = 0b10000000100;//PIN 6 
  //  PORTC_PCR4 = 0b10000000100;// PIN 10
  //PORTD_PCR6 = 0b10000000100;//PIN 21 
  PORTD_PCR7 = 0b10000000100;//PIN 5 
      // Set PORTA_PCR13 register
    // Bits | Value | Description
    // 10-8 |  100  | MUX: Alt 3 attach to FTM1_CH1
    //  7   |    0  | Reserved
    //  6   |    0  | DSE: Low drive strength
    //  5   |    0  | Reserved
    //  4   |    0  | PFE: Disable input filter
    //  3   |    0  | Reserved
    //  2   |    1  | SRE: Slow output slew rate
    //  1   |    0  | PE: Disable pull-up/down
    //  0   |    0  | PS: Internal pull-down
    PORTA_PCR13 = 0b01100000100; //PIN 4
  }
  unsigned int ms2Number(unsigned int intTime){
    int number=0;
    number=36000*intTime/128;
    return number;
  }
  
  
  uint16_t analogReadFastADC0(uint8_t pin){
    uint16_t result;
    
    adc->adc0->startReadFast(pin);      // start single read (with adc->adc0->singleMode(); in setup() )
    while(adc->adc0->isConverting()) {} // wait for the ADC to finish
                                        //  __disable_irq();
    result = (uint16_t)ADC0_RA;
                                        //   __enable_irq();
    return result;
  }
