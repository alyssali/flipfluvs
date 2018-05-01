/*********************************************************************
Code adapted from bleuart sketch.
This code is for the peripheral bluefruit board that turns on the vibration motor and Pellier
(loaded onto FlipFlop)
This was last updated February 15, 2018.
*********************************************************************/

#include <bluefruit.h>
#include <math.h>

//sensor
const int uvPin = A5;
const int fsrPin = A0;

//actuators
const int peltierPin = 15;
const int motorPin = 16;

//variables
int SPF = 30;
int minuteCounter = 0; //round(time/60000);
int timeSinceLastApplied = 0;
bool firstTime = true;
int uvValue;
int fsrValue;
bool peltierOn = false;

// constants
const int uvThreshold = 40;
const int fsrThreshold = 20;
const int loopTime = 10; // 10 milliseconds
const int peltierOnTime = 10000; // 10 seconds
const int motorOnTime = 1000; // 1 second

BLEClientDis  clientDis;
BLEClientUart clientUart;

// Peripheral uart service
BLEUart bleuart;


void setup()
{
  pinMode(motorPin, OUTPUT); 
  pinMode(peltierPin, OUTPUT);
  digitalWrite(motorPin, LOW);
  digitalWrite(peltierPin, LOW);

  Serial.begin(115200);
  
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 1);
  
  Bluefruit.setName("FlipFlop");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Configure and Start BLE Uart Service
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

    // Callbacks for Peripheral
  Bluefruit.setConnectCallback(prph_connect_callback);
  Bluefruit.setDisconnectCallback(prph_disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
    Serial.print("BLE UART service detected. Connecting ... ");

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");

  Serial.print("Dicovering DIS ... ");
  if ( clientDis.discover(conn_handle) )
  {
    Serial.println("Found it");
    char buffer[32+1];
    
    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      Serial.print("Model: ");
      Serial.println(buffer);
    }

    Serial.println();
  }  

  Serial.print("Discovering BLE Uart Service ... ");

  if ( clientUart.discover(conn_handle) )
  {
    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();

    Serial.println("Ready to receive from Bracelet");
  }else
  {
    Serial.println("Found NONE");
    
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  
  Serial.println("Disconnected");
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& uart_svc)
  {
    Serial.print("[RX]: ");
  
    String message = "";
    
    while ( uart_svc.available() )
      {
        char incomingChar = (char) uart_svc.read();
        Serial.print( incomingChar );
        message = message + incomingChar;
      }
    
    if(message == "20")
      {
        SPF = 20;
        minuteCounter = 0;
        timeSinceLastApplied = round(13 * 0.25 * SPF * 500);
        firstTime = false;
        peltierOn = true;
      }

    else if(message == "30")
    {
        SPF = 30;
        minuteCounter = 0;
        timeSinceLastApplied = round(13 * 0.25 * SPF * 500);
        firstTime = false; 
        peltierOn = true;
    }

    else if(message == "40")
    {
        SPF = 40;
        minuteCounter = 0;
        timeSinceLastApplied = round(13 * 0.25 * SPF * 500);
        firstTime = false;
        peltierOn = true; 
    }

    else if(message == "50")
    {
        SPF = 50;
        minuteCounter = 0;
        timeSinceLastApplied = round(13 * 0.25 * SPF * 500);
        firstTime = false; 
        peltierOn = true;
    }

    Serial.println("SPF: ");
    Serial.print(SPF);
    Serial.println();
  }

void loop()
  {
    fsrValue = analogRead(fsrPin);
    uvValue = analogRead(uvPin);
    Serial.println("uvValue: ");
    Serial.print(uvValue);
    Serial.println();
    
    if (peltierOn) 
    {
        digitalWrite(peltierPin, HIGH);
        Serial.println("turning on peltier");
        delay(peltierOnTime);
        digitalWrite(peltierPin, LOW);
        peltierOn = false;
    }
    if (uvValue > uvThreshold) //  (fsrValue > fsrThreshold)
      {
          Serial.println("checking for uvValue");
          if(minuteCounter > timeSinceLastApplied || firstTime == true)
          {
            digitalWrite(motorPin, HIGH); //turn on motor
            Serial.println("turning on motor");
            //digitalWrite(peltierPin, HIGH);
            
            bleuart.write("SOS",3); //send SOS to bracelet
            Serial.println("sending SOS to bracelet");
            
            delay(motorOnTime);
            digitalWrite(motorPin, LOW);
 
          }
          else 
          {
              digitalWrite(peltierPin, LOW);
              digitalWrite(motorPin, LOW);
          }        
       }
    else 
    {
        digitalWrite(peltierPin, LOW);
        digitalWrite(motorPin, LOW);
    }
    minuteCounter += loopTime;
   }

   /*------------------------------------------------------------------*/
/* Peripheral
 *------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle)
{
  char peer_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, peer_name, sizeof(peer_name));

  Serial.print("[Prph] Connected to ");
  Serial.println(peer_name);
}

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.println("[Prph] Disconnected");
}

void prph_bleuart_rx_callback(void)
{
  // Forward data from Mobile to our peripheral
  char str[20+1] = { 0 };
  bleuart.read(str, 20);

  Serial.print("[Prph] RX: ");
  Serial.println(str);  

  if ( clientUart.discovered() )
  {
    clientUart.print("SOS");
    bleuart.write("SOS",3); //send SOS to bracelet
  }else
  {
    bleuart.println("[Prph] Central role not connected");
  }
}
