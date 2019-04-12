#include <Arduino.h>
#include <WiFi.h>
#include "time.h"
#include "EEPROM.h"
#include "constants.h"
#include "utils.h"


RTC_DATA_ATTR uint32_t lastIndexEEPROM;
RTC_DATA_ATTR volatile uint32_t pulseCounter = 0;
RTC_DATA_ATTR uint32_t timeStamp = 0;
RTC_DATA_ATTR uint8_t state;
RTC_DATA_ATTR bool isCW;
uint8_t firstInit;


RFM69X radio(
            RFM_SS, 
            RFM_INT, 
            IS_RFM69HW, 
            digitalPinToInterrupt(RFM_INT));

struct __attribute__((__packed__)) Payload{
    int             nodeID;
    uint32_t        pulseCounter;
    uint32_t        timeStamp;
    uint32_t        errors;
};
Payload theData;

EEPROMClass dataLogger("eeprom", 0x1000);

void setup() 
{
  Serial.begin(SERIAL_BAUD);
  
  //Inicia y lee memoria EEPROM
  if (!dataLogger.begin(dataLogger.length())) {
    Serial.println("Failed to initialise dataLogger");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }
  
  Serial.print("Primera vez? ");
  Serial.println(dataLogger.readByte(0));

  //Configura si es el primer encendido, lee variables si no es el primer encendido
  if(dataLogger.readByte(0))
  {
    Serial.println("Configurando la hora");
    setenv("TZ", TZ_INFO, 1);
    tzset(); // Assign the local timezone from setenv
    
    if (timeInt32 == 0 || timeInt32 < 1480000000) 
    {
      Serial.println("Please enter a Unix epoch time after 01/01/2017");
      delay(1000);
      ESP.restart();
    } else 
    {
      timeval epoch = {timeInt32, 0};
      settimeofday((const timeval*)&epoch, 0);
      if(printLocalTime())
      {
        firstInit = 0x01;
      }
      else
      {
        ESP.restart();
      }
      
    }
    
    Serial.println("Borrando memoria...");
    for(int i = 0; i < EEPROM_SIZE; i++)
    {
      dataLogger.put(i, 0);
    }
    
    Serial.println("Inicialización correcta!!!");
    dataLogger.put(0, 0x00); //primer inicio correcto
    dataLogger.put(1, 5);     //indice en primera posicion
    Serial.print("firstInit = "); Serial.println(dataLogger.readByte(0));
    Serial.print("Indice = "); Serial.println(dataLogger.readInt(1));
    dataLogger.end();

    /*esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
    " Seconds");
    Serial.println("Going to sleep now");
    Serial.flush(); 
    esp_light_sleep_start();*/
  }else
  {
    Serial.println("Leyendo memoria...");
    dataLogger.get(1, lastIndexEEPROM);
    Serial.print("lastIndex = "); Serial.println(lastIndexEEPROM);
    
    dataLogger.get(lastIndexEEPROM, timeStamp);
    dataLogger.get(lastIndexEEPROM + sizeof(timeStamp), pulseCounter);

    Serial.println("Ultima lectura en memoria: ");
    Serial.print("Fecha: ");Serial.println(timeStamp);
    Serial.print("Medida: ");Serial.println(pulseCounter);
    //dataLogger.put(0, 0x01);
    //Serial.println(dataLogger.readByte(0));
    dataLogger.end();
  }

  //Inicializa el encoder para contar las vueltas
  pinMode(PINA, INPUT_PULLUP);
  pinMode(PINB, INPUT_PULLUP);
  noInterrupts();
  attachInterrupt(PINA, isrEncoder, FALLING);
  attachInterrupt(PINB, isrEncoder, FALLING);
  bool stateA = digitalRead(PINA);
  bool stateB = digitalRead(PINB);
  if(!stateA && !stateB)
  {
    state = 0;
  }else if(stateA && !stateB)
  {
    state = 1;
  }else if(stateA && stateB)
  {
    state = 2;  
  }else if(!stateA && stateB)
  {
    state = 3;
  }
  interrupts();  

  if(!radio.initialize(FREQUENCY, NODEID, NETWORKID))
  {
    Serial.println("\n************************************************************");
    Serial.println("WARNING: RFM Transceiver initialization failure: Set-up Halted");
    Serial.println("**************************************************************");
    while(1); //Halt the process  
  }

  radio.setHighPower(true);
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(SEND_PROMISCUOUS);

  pinMode(LED, OUTPUT);

  Serial.println("ESP32");
  Serial.println("Using RFM69 Extended mode");
  Serial.println("\nTransmitting at 915 Mhz...");

  /*Inicializacion de timer para enviar datos periodicamente*/
}

long lastPeriod = -1;
void loop() 
{
  if( radio.receiveDone() )
  {
    //Checa si el concentrador se está comunicando
    if(radio.SENDERID == GATEWAYID)
    {
      Serial.print("[");
      Serial.print(radio.SENDERID, DEC);
      Serial.print("] ");
      
      for(byte i = 0; i < radio.DATALEN; i++)//Selecciona el comando
      {
        Serial.print( (char)radio.DATA[i] );
      }

      if ( radio.ACKRequested() ) 
      {
        ackSentCount++;
        radio.sendACK();
        Serial.print(" - ACK sent= ");
        Serial.println( ackSentCount );
        delay(10);
      }
    }
    
    Serial.print("\t[RX_RSSI:");
    Serial.print(radio.RSSI);
    Serial.print("]");

    blinkLED(LED, 5);
  }

  int currentPeriod = millis() / TRANSMITPERIOD;
  if(currentPeriod != lastPeriod)
  {
    theData.nodeID = NODEID;
    theData.pulseCounter = pulseCounter;
    theData.timeStamp = timeStamp;
    theData.errors = 0xFF;

    Serial.print("Sending struct (");
    Serial.print(sizeof(theData));
    Serial.print(" bytes)");
    Serial.print(" frame count = ");
    Serial.print(theData.pulseCounter);
    Serial.print(" uptime= ");
    Serial.print(theData.timeStamp);
    Serial.print(" .... ");

    if (radio.sendWithRetry(
                            GATEWAYID, 
                            (const void*)(&theData),
                            sizeof(theData),
                            SEND_RETRY,
                            SEND_WAIT_WDG)) 
    {
      Serial.print("ok!");
    }
    else
    {
      Serial.println(" nothing...");
    }
    blinkLED(LED, 3);
    lastPeriod = currentPeriod;
  }
}

void isrEncoder()
{
  noInterrupts();
  switch(state)
  {
    case 0:
      if(digitalRead(PINA))
      {
        state++;
      }else if(digitalRead(PINB))
      {
        state = 3;
      }
      break;

    case 1:
      if(!digitalRead(PINA))
      {
        state--;
        pulseCounter--;
        isCW = false;
      }else if(digitalRead(PINB))
      {
        state++;
      }
      break;

    case 2:
      if(!digitalRead(PINA))
      {
        state++;
      }else if(!digitalRead(PINB))
      {
        state--;
      }
      break;

    case 3:
      if(digitalRead(PINA))
      {
        state--;
      }else if(!digitalRead(PINB))
      {
        state = 0;
        pulseCounter++;
        isCW = true;
      }
      break;
  }
}
