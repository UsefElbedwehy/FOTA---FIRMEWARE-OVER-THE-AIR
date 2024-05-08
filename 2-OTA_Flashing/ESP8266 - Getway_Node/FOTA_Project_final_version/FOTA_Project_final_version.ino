
#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#elif __has_include(<WiFiNINA.h>)
#include <WiFiNINA.h>
#elif __has_include(<WiFi101.h>)
#include <WiFi101.h>
#elif __has_include(<WiFiS3.h>)
#include <WiFiS3.h>
#endif

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Firebase_ESP_Client.h>

// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the SD card interfaces setting and mounting
#include <addons/SDHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Arsenal"
#define WIFI_PASSWORD "H0CO1380h0co"

#define WRITE_TO_MEM  '1'
#define ERASE_MEM     '2'


/* 2. Define the API Key */
#define API_KEY "AIzaSyC7Px3OFwNP2IDIp17jutQffs4fk_hAQrc"

/* 3. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "elbedwehyy@gmail.com"
#define USER_PASSWORD "123456"

const char* mqtt_server = "test.mosquitto.org";

/* 4. Define the Firebase storage bucket ID e.g bucket-name.appspot.com */
#define STORAGE_BUCKET_ID "useffota.appspot.com"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

bool taskCompleted = false;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
WiFiMulti multi;
#endif

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;



// The Firebase Storage download callback function
void fcsDownloadCallback(FCS_DownloadStatusInfo info)
{
    if (info.status == firebase_fcs_download_status_init)
    {
        Serial.printf("Downloading file %s (%d) to %s\n", info.remoteFileName.c_str(), info.fileSize, info.localFileName.c_str());
    }
    else if (info.status == firebase_fcs_download_status_download)
    {
        Serial.printf("Downloaded %d%s, Elapsed time %d ms\n", (int)info.progress, "%", info.elapsedTime);
    }
    else if (info.status == firebase_fcs_download_status_complete)
    {
        Serial.println("Download completed\n");
    }
    else if (info.status == firebase_fcs_download_status_error)
    {
        Serial.printf("Download failed, %s\n", info.errorMsg.c_str());
    }
}

/*
@fn     : readFile
@brief  : Read the UserApp file from Firebase database.
@param  : path (path of the file)
@retval : void
*/
void readFile(const char *path)
{
  //Open UserApp file
  File file = LittleFS.open(path, "r");

  if(!file)
  {
  Serial.println("Failed to open file for reading");
  return;
  }
  Serial.print("Read from file: ");
  while(file.available())
  { //Reding file
    Serial.write(file.read());
  }
  file.close();
}

/*
@fn     : callback
@brief  : manage the client subscribtion or publishing or both
@param  : topic (the topic tht the node cn subscribe or publish on it)
@param  : payload
@param  : length
@retval : void
*/
void callback(char* topic, byte* payload, unsigned int length) {
  char Data;
   
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Data=((char)payload[i]);
    //Serial.print((char)payload[i]);
  } 
    Serial.println((char)payload[0]);
    
  if (Data == WRITE_TO_MEM)
    {
      Serial.print("Bootloading...");
      digitalWrite(LED_BUILTIN, HIGH);
      BootloaderSendData();
    }
    else if (Data == ERASE_MEM)
    {
      Serial.print("Erasing...");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    delay(15);
}


void reconnect() {
  client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("FotaProj", "hello world");
      // ... and resubscribe
      client.subscribe("FotaProj");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    Serial.begin(115200);
    Serial1.begin(115200);
    digitalWrite(BUILTIN_LED, HIGH);
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    multi.addAP(WIFI_SSID, WIFI_PASSWORD);
    multi.run();
#else
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

    Serial.print("Connecting to Wi-Fi");
    unsigned long ms = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
        if (millis() - ms > 10000)
            break;
#endif
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    /* Assign the user sign in credentials */
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // The WiFi credentials are required for Pico W
    // due to it does not have reconnect feature.
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    config.wifi.clearAP();
    config.wifi.addAP(WIFI_SSID, WIFI_PASSWORD);
#endif

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

    // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
    Firebase.reconnectNetwork(true);

    // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
    // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
    fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

    /* Assign download buffer size in byte */
    // Data to be downloaded will read as multiple chunks with this size, to compromise between speed and memory used for buffering.
    // The memory from external SRAM/PSRAM will not use in the TCP client internal rx buffer.
    config.fcs.download_buffer_size = 2048;

    Firebase.begin(&config, &auth);

    // if use SD card, mount it.
    SD_Card_Mounting(); // See src/addons/SDHelper.h

    
}

void loop()
{

    // Firebase.ready() should be called repeatedly to handle authentication tasks.

    if (Firebase.ready() && !taskCompleted)
    {
        taskCompleted = true;

        Serial.println("\nDownload file...\n");

        // The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
        //if (!Firebase.Storage.download(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, "MQTT.txt" /* path of remote file stored in the bucket */, "/update.txt" /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, fcsDownloadCallback /* callback function */))
        if (!Firebase.Storage.download(&fbdo, STORAGE_BUCKET_ID /* Firebase Storage bucket id */, "UserApp.txt" /* path of remote file stored in the bucket */, "/update.txt" /* path to local file */, mem_storage_type_flash /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, fcsDownloadCallback /* callback function */))
            Serial.println(fbdo.errorReason());
    }
    if (!client.connected()) {
            reconnect();
            }
            client.loop();
            delay(100);
}
char buf[2]={0};
char packet[200]={0};
void BootloaderSendData()
{
  
  File file = LittleFS.open("/update.txt","r");
  while(file.available())
  {
    /*Packet*/
    char CounterByte=0;
    packet[0]=70;       //Length to follow
    packet[1]=0x57;     //cmd ID
    //packet[2]=0x08;
    //packet[3]=0x00;
    //packet[4]=0x80;
   // packet[5]=0x00;
    *(int*)(&packet[2])=0x08008000;   //Memory Address
    packet[6]=64;                     //Data Length
    //Reading data(64 byte) from file 
    //converting from string into hex
    char HexData;
    while(CounterByte < 64)
    {
      file.readBytes(buf,2);
      //Serial.printf("%s",buf);
      HexData = strtol(buf,NULL,16);
      packet[7+CounterByte] = HexData;
      Serial.printf("packet[%i] = %x \n",(7+CounterByte),packet[7+CounterByte]);
      CounterByte++;
    }
    //sending first 7 bytes of the packet
    int i;
    for(i=0;i<7;i++)
    {
    Serial1.write(packet[i]);
    
    }
    //sending data bytes 
    CounterByte=0;
    while(CounterByte < 64)
    {
      Serial1.write(packet[CounterByte+7]);
      CounterByte++;
      
    }
  while (!Serial.available()) {
  }
  //Receiving STM32 response to continue transmiting data
  Serial.read();

  }
}