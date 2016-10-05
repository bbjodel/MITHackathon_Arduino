
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <SPI.h>

//AHRS commands
#define READ_UNTARED_QUATERION      0x06
#define READ_LINEAR_ACCELERATION    0x29
#define START_BYTE                  0xF6
#define DUMMY_READ_BYTE             0xFF
#define RESET_INTERTNAL_DATA_BUFFER 0x01

//AHHS SPI states
#define AHRD_SPI_IDLE               0x00
#define AHRS_SPI_READY              0x01
#define AHRS_SPI_BUSY               0x02
#define AHRS_SPI_ACCUMULATION       0x04

//ESP8266 Pin defines 
#define SPI_CS                      0x03
#define AHRS_INT                    0x04
#define TRIGGER_PIN                 0x05      

//TCP payload indexies and sizes
#define TIMESTAMP_INDEX             4
#define TIMESTAMP_SIZE              4
#define QUATERIONS_INDEX            (TIMESTAMP_INDEX + TIMESTAMP_SIZE)
#define QUATERIONS_SIZE             16
#define LINEAR_ACCELERATION_INDEX   (QUATERIONS_INDEX + QUATERIONS_SIZE)
#define LINEAR_ACCELERATION_SIZE    12
#define TRIGGER_INDEX               (LINEAR_ACCELERATION_INDEX + LINEAR_ACCELERATION_SIZE)
#define TCP_PAYLOAD_SIZE            37

#define BUFFERED_SAMPLES            20

#define ACCESS_POINT
WiFiClient client;
// Start a TCP Server on port 12345
WiFiServer server(12345);
// Hardcode WiFi parameters.
const char* ssid = "esp_ap"; 
const char* password = "password";

static uint8_t enable_ahrs_int[] =   { 0x1D, 0x01, 0x00 };
static uint8_t raw_trigger_state = HIGH;
static uint8_t trigger_output = LOW;
static uint8_t transmit_buffer[4000];

union float_type{
  uint8_t raw_bytes[4];
  float value;
  uint32_t uint32;
} quaterions[4], linear_acceleration[3];
float_type time_stamp;

//function prototypes
uint8_t spi_read_write(byte data);
void read_ahrs_data(uint8_t* outdata, uint8_t outdata_size, uint8_t* command, uint8_t command_size);
void store_swapped4(void * dest, uint32_t value);
void WriteText();
void prep_raw_data_to_send();
void handle_trigger_state();
void get_arhs_reading();
bool build_tcp_payload(uint8_t *payload);

/*
 * read/witre on spi bus. ahrs requires cs line to be held down for 20us before sending
 * data otherwise doesnt send valid results for some reason ? 
 */
uint8_t spi_read_write(byte data) 
{
  uint8_t ret = 0u;
  digitalWrite(SPI_CS,LOW);
  delayMicroseconds(20);
  ret = SPI.transfer(data);
  delayMicroseconds(20);
  digitalWrite(SPI_CS,HIGH);
 return ret; 
}

/*
 * SPI read / write sequence for the ahrs
 * clears ahrs transmit buffer, sends start byte and command then waits for device to prepare 
 * result. Then stores data recieved.
 * @param outdata - pointer to where data returned will be stored. 
 * @param outdata_size - amount of bytes to read from ahrs. 
 * @param command - pointer to command to send to ahrs.
 * @param command_size - number of bytes the command is.
 */
void read_ahrs_data(uint8_t* outdata, uint8_t outdata_size, uint8_t* command, uint8_t command_size)
{
  uint8_t i;
  (void)spi_read_write(RESET_INTERTNAL_DATA_BUFFER);
  (void)spi_read_write(START_BYTE);
  //send command to ahrs
  for (i = 0; i < command_size; i++)
  {
    (void)spi_read_write(*command++);
  }
  
  //wait for ahrs to process command
  while(spi_read_write(DUMMY_READ_BYTE) != AHRS_SPI_READY);
  for (i = 0; i < outdata_size; i++)
  {
    *outdata++ = spi_read_write(DUMMY_READ_BYTE);
  }
}

/*
 * Swaps endian order and store value to destination
 */
void store_swapped4(void * dest, uint32_t value)
{
    uint8_t * dest_ptr = (uint8_t *)dest;
    const uint8_t * value_ptr = (const uint8_t *) &value;
    dest_ptr[0] = value_ptr[3];
    dest_ptr[1] = value_ptr[2];
    dest_ptr[2] = value_ptr[1];
    dest_ptr[3] = value_ptr[0];
}

/* 
 *  Swaps endianness of ahrs float value readings and inverses trigger input
 *  such that trigger pressed gives 1. 
 */
void prep_raw_data_to_send()
{
  uint8_t i;
  for (i = 0; i < 4; i++)
  {
    store_swapped4(&quaterions[i], *(uint32_t*)quaterions[i].raw_bytes);
  }
  for (i = 0; i < 3; i++)
  {
    store_swapped4(&linear_acceleration[i], *(uint32_t*)linear_acceleration[i].raw_bytes);
  }
  //trigger input inverted to true trigger state.
  trigger_output = HIGH;
  if(raw_trigger_state == HIGH)
  {
    trigger_output = LOW;
  }
}

/* 
 *  Debounces trigger input pin to get clean trigger state
 */
void handle_trigger_state()
{
  static uint8_t last_input_state = HIGH;
  static uint32_t lastDebounceTime = 0;
  static const uint32_t debounceDelay = 10u;
  
  uint8_t reading = digitalRead(TRIGGER_PIN);
  if (reading != last_input_state) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    if (reading != raw_trigger_state) 
    {
      raw_trigger_state = reading;
    }
  }
  last_input_state = reading;
}

/*  
 *  blocks until reading from ahrs is ready then reads quaternions and linear accelerations.
 *  processed data recivied to swap endians
 */
void get_arhs_reading()
{
  while(digitalRead(AHRS_INT) == HIGH);

  time_stamp.uint32 = micros();
  uint8_t command = READ_UNTARED_QUATERION;
  read_ahrs_data((uint8_t*) quaterions, QUATERIONS_SIZE, &command, 1);
  command = READ_LINEAR_ACCELERATION;
  read_ahrs_data((uint8_t*)linear_acceleration, LINEAR_ACCELERATION_SIZE, &command, 1);

  prep_raw_data_to_send();
}

/*! adds samples to transmit buffer and retuns true when transmit 
 * buffer is ready to be sent.
 * @param payload - ahrs reading sample
 * return true if transmit buffer has accumilated enough sample to send
 */
bool build_tcp_payload(uint8_t *payload)
{
  static uint8_t buffered_results_count = 0; 
  memcpy(&payload[TIMESTAMP_INDEX], time_stamp.raw_bytes, TIMESTAMP_SIZE);
  memcpy(&payload[QUATERIONS_INDEX], (uint8_t*) quaterions, QUATERIONS_SIZE);
  memcpy(&payload[LINEAR_ACCELERATION_INDEX], (uint8_t*) linear_acceleration, LINEAR_ACCELERATION_SIZE);
  payload[TRIGGER_INDEX] = trigger_output;

  //copy packet to transmit buffer
  memcpy(&transmit_buffer[buffered_results_count * TCP_PAYLOAD_SIZE], payload, TCP_PAYLOAD_SIZE);
  buffered_results_count++;
  if ( buffered_results_count == BUFFERED_SAMPLES)
  {
    buffered_results_count = 0;
    return true;
  }
  return false;
  
}

void setup() {
  Serial.begin(230400);

 #ifdef ACCESS_POINT
  //ESP8266 will act as an access point 
  WiFi.softAP(ssid,password);
  IPAddress myIP = WiFi.softAPIP();
 #else
  /* 
   *  ESP8266 will connect to wifi network, make sure to change ssid and passwaord
   *  to match the one you are connecting to.
   */    
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  IPAddress myIP =WiFi.localIP();
 #endif
 
  Serial.println("");

  
  Serial.print("My IP address: ");
  Serial.println(myIP);
  // Start the TCP server
  server.begin();
  Serial.println("HTTP server started");

  SPI.begin();
  pinMode(SPI_CS, OUTPUT);
  pinMode(AHRS_INT, INPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  digitalWrite(SPI_CS,HIGH);
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(6000000);

  uint8_t dummy_return;
  read_ahrs_data(&dummy_return, 0 , enable_ahrs_int, 3);
}

// Main 
void loop() 
{
  //setup payload header, 
  static uint8_t payload[TCP_PAYLOAD_SIZE];
  memset(payload, 0xFF, 4);
  
  client = server.available();
  if (client)
  {
    Serial.println("Client connected");
    while (client.connected())
    { 
        handle_trigger_state();
        get_arhs_reading();
        if(build_tcp_payload(payload))
        {
          client.write((uint8_t*)transmit_buffer, BUFFERED_SAMPLES * TCP_PAYLOAD_SIZE);
        }
    }
  }
}
