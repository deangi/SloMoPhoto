#include <M5Family.h>

//----------------------------------------------------------------------
// Slow Motion Camera App for ESP32
//
// This application will take periodic photos and store them
// can either connect to WiFi and use FTP to upload
// the photos to a remote server, or the photos can be
// stored on a mini-SD card.
//
// This is intended to be a battery operated application
// (although powered operation also works)
//
// Based on the Aideepen ESP32 CAM board with SD card drive and
// with an OV2640 camera (2 mega pixels) (1600x1200)
//
// Search for Aideepen ESP32-CAM on Amazon for details
//    Available 2 for $20 as of Jan 2024
//
// -- GPIO allocation from the schematic I saw:
//
// Camera: 34,35,32,25,26,27,23,22,21,19,18,5,0
// Led: 33
// SD: 14,12,4,2,15,13
// GPIO 16 - spare
//
// Configuration file note:
//
//SSID=WiFiSSID
//PASSWORD=WiFiPassword
//SERVER=192.168.95.45
//USESD=false
//#true, yes, false, no
//FTPUSER=username
//FTPPASSWORD=password
//FTPFOLDER=/home/pi/FTP
//PHOTOSPERDAY=24
//
//
// Arduino IDE configuration support:
//
// Board selection: M5Stack-Timer-Cam from M5Stack-Arduino
// Partition Scheme: Default (3Mb no OTA, 1Mb SPIFFS)
//
// Deangi - December 2023
//
//----------------------------------------------------------------------
// --------- H I S T O R Y ---------
// V1.0 Dec 6, 2023
// V1.1 Jan 5, 2024 - add SD MMC support
//

//#define CAMERA_MODEL_M5STACK_PSRAM 1 /* For M5Stack-Timer-Cam, Select board M5Stack-Timer-Cam */
#define CAMERA_MODEL_AI_THINKER 1 /* For Aideepen ESP32-CAM module, Select board AI-Thinker-ESP32-CAM */

#include "Arduino.h"
#include "esp_camera.h"         // Camera interface
#include "soc/soc.h"            // Disable brownout problems
#include "soc/rtc_cntl_reg.h"   // Disable brownout problems
#include "driver/rtc_io.h"      // RTC interface
#include <ESP32Time.h>          // RTC time functions
#include <SPI.h>                // SPI driver code
#include <SPIFFS.h>             // SPI Flash File System for logging data
#include "SD_MMC.h"             // SD_MMC card library
#include <WiFi.h>               // WiFi driver code
#include <WiFiUdp.h>            // UDP code for NTPClient support
#include <NTPClient.h>          // Network Time Protocol (NTP) client library
#include <ESP32_FTPClient.h>    // FTP client to upload photos
#include "camera_pins.h"        // Timer-CAM - M5Stack - camera - pin definitions

//-------------------------------------------------------------
// RTC DATA:
// This camera spends most of it's time in ESP32 deep sleep 
// mode to conserve battery power.   Periodically it wakes
// up to take a photo and save it in flash memory.
// Once in a while during the wake up cycle it will also
// connect to WiFi and attempt to upload the photos to
// a remote FTP server.
//
// Because of deep-sleep - whenever the ESP32 wakes up,
// all of it's normal memory is erased from the last wake
// up cycle.   However this is a small amount of memory
// associated with the real-time-clock (RTC) that doesn't
// get erased - it's only 8kb - but we can use this to
// save some volatile information across deep-sleep
// cycles.  We can also use the flash to save photos
// across deep-sleep cycles.
//
// The items below are allocated to this RTC memory
// so they survive from wake-up to wake-up cycle.
//
RTC_DATA_ATTR int bootCount = 0;  /* A counter of how many sleep cycles we've gone through */
RTC_DATA_ATTR int needNtp = true; /* a flag to know if we need to query the time next upload cycle */
RTC_DATA_ATTR int lastHr=-1;      /* used for hourly tasks such as the upload cycle */

// Information from configuration file
RTC_DATA_ATTR char ssid[128];
RTC_DATA_ATTR char password[128];
RTC_DATA_ATTR int wantSdMmc = false; // store files on SD rather than FTP upload
#define FSTYPE_NONE   (0)
#define FSTYPE_SDMMC  (1)
#define FSTYPE_FFSPI  (2)
RTC_DATA_ATTR int fsType = FSTYPE_NONE;
RTC_DATA_ATTR char uploadHost[128];
RTC_DATA_ATTR char ftpUser[128];
RTC_DATA_ATTR char ftpPswd[128];
RTC_DATA_ATTR char ftpFolder[128];
RTC_DATA_ATTR long delayBetweenPhotosMs;
RTC_DATA_ATTR int photoCount = 0;


// ESP32 resource usage:
// 1) ESP32 CPU - we will run at 80 MHz to save power
//    Had no luck running slower than this with WiFi enabled...
// 2) ESP32 RAM - just used for transient items needed when 
//    we're running the IoT or Upload cycles
// 3) ESP32 FLASH - stores the application and a flash file system
//    In the flash file system, we have a config.ini file and
//    a log file as well as temporarily storing photos until
//    they are uploaded.
//
// 4) ESP32 I/O - camera connection, SD card connection
// 5) ESP32 LED - to provide "flash" photography option
// 6) SD Card notes - Aideepen documentation says up to 4GB cards
//    are supported, but I have seen some 32GB (Samsung Ultra A1 HC) work.
//

#define ms_TO_uS_FACTOR 1000  /* Conversion factor for milli seconds to micro seconds */

// comment this one out if you don't want diagnostic serial output
#define WANTSERIAL (1)

#define SIGNON "\nIOT Slow Motion Camera V1.1, 5-Jan-2024"

char wifiIsConnected=0; // 1 means connected

fs::FS & fileSystem = SD_MMC; // file system

//-----------------------------------------------------------------
// --- Some utility routines for printing to the serial port if
//     you want.

void print(String msg)
{
#ifdef WANTSERIAL
  Serial.print(msg);
#endif
}

void print(char* msg)
{
#ifdef WANTSERIAL
  Serial.print(msg);
#endif
}

void println(String msg)
{
#ifdef WANTSERIAL
  Serial.println(msg);
#endif
}

void println(char* msg)
{
#ifdef WANTSERIAL
  Serial.println(msg);
#endif
}

//-----------------------------------------------------------------
// real time clock (software based, not backed up for power failures
ESP32Time rtc(-8*3600);  // -8 from GMT by default

//-----------------------------------------------------------------
// Camera config
camera_config_t config;

//-------------------------------------------
// File names 
#define CONFIGFN "/config.ini"
#define LOGFN "/iotdevice.log"

//-------------------------------------------------------------
// Append a line to the log file
void appendToLogFile(String msg)
{
  println(msg);

  // append to log file
  File fout = fileSystem.open(LOGFN, FILE_APPEND);
  if (!fout)
  {
    println("Unable to append to log file");
  }
  else
  {
    fout.println(msg);
    fout.close();
  }
}

//----------------------------------------------------------------------
// Compose a log line with a leading time-tag
void addLogLine(char* msg)
{
  char buf[128];
  String ttag = rtc.getTime("%Y/%m/%d,%H:%M:%S"); // "2022/11/16,18:22:01"
  sprintf(buf,"%s,%s",ttag.c_str(), msg);
  appendToLogFile(buf); 
}

//----------------------------------------------------------
// read line from input text file
int readln(File finp, uint8_t* buf, int maxlen)
{
  // return true on successful read, false on EOF
  // 10 or 13 (LF, CR) or both are EOL indicators
  int len=0;
  int eof=false;

  buf[0]=0;
  while (len<(maxlen-1))
  {
    if (!finp.available())
    {
      eof=true;
      break;
    }
    char c = finp.read();
    if (c < 0) 
    {
      eof=true;
      break; // EOF
    }
    if (c==13) continue; // ignore CR
    if (c==10) break; // end-of-line
    buf[len++]=c;
  }
  buf[len]=0; // null terminate
  return !eof;
}

//----------------------------------------------------------
// retrieve a value for a key in the config file
void readKey(char* configFn, char* key, char* outbuf, int maxlen)
{
  outbuf[0] = 0; // returning null string on error 
  //
  // Config file is key=value format
  // SSID=mywifi
  // PASSWORD=mypassword
  // TIMEZONE=-8
  // OFFSET=123590 
  //
  // pass in key with trailing = sign!!! 
  // readKey("/test.cfg","MYKEY=", outbuf, 127);

  File finp = fileSystem.open(CONFIGFN, FILE_READ);
  if (!finp)
  {
    println("Unable to read config file");
    return;
  }
  // scan file and look for key
  char buf[128];
  int n = strlen(key);
  while (readln(finp, (uint8_t*) buf, 127))
  {
    if (strncmp(buf,key,n) == 0) // found
    { 
      println(buf);
      strncpy(outbuf,&buf[n],maxlen);
      break;
    }
  }
  finp.close();
}


int wifiWaitCounter=0;
String ipAddress = String("unknown");

//------------------------------------------------------------------------
// convert IP address to string
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

//----------------------------------------------------------------------
// Connect to some WiFi access point as a station
int connectToWiFi()
{
 // Connect to Wi-Fi, 1=connected, 0=not connected
  int maxWaitTimeToConnect = 50; // seconds
  WiFi.begin(ssid, password);
  wifiWaitCounter=0;
  wifiIsConnected=0;
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    println("Connecting to WiFi..");
    if (++wifiWaitCounter >= maxWaitTimeToConnect)
      break;
  }

  // Print ESP32 Local IP Address
  if (wifiWaitCounter >= maxWaitTimeToConnect)
  {
    addLogLine("WiFi connect failed");  
  }
  else
  {
    ipAddress = "WIFICONNECT,"+IpAddress2String(WiFi.localIP());
    addLogLine((char*)ipAddress.c_str());
    wifiIsConnected=1;
  }
  return wifiIsConnected;
}

//----------------------------------------------------------------------
void turnOffWiFi()
{
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF); 
    wifiIsConnected = 0;
}

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

//----------------------------------------------------------------------
void getNtpTime()
{
  if (wifiIsConnected==0) return; // can't do it if no wifi connected.
  // NTP Client
  timeClient.begin();
  timeClient.setTimeOffset(0);
  int waitingForNtp = true;
  while (waitingForNtp)
  {
    if (!timeClient.update())
      timeClient.forceUpdate();
    else
      waitingForNtp = false;
  }

  unsigned long epochTime = timeClient.getEpochTime();
  println("NTP Time: ");
  println(timeClient.getFormattedDate());
  rtc.setTime(epochTime);
  addLogLine("NTP");
}

//---------------------------------------------------------------
// Setup the camera and connect to it
void setupMeasurement()
{
  //println("Initializing camera");
  // OV2640 camera, max 1600x1200 pixels, color
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.grab_mode=CAMERA_GRAB_LATEST;
  
  if(psramFound()){
    //println("PSRAM found");
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    //println("PSRAM -not- found");
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) 
  {
    char buf[32];
    sprintf(buf,"%ld",err);
    print("Camera init failed code "); println(buf);    
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s,1);
  s->set_saturation(s,-2);
  /* else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val); */
}

#define TOUPPER(c)   (((c>='a') && (c<='z')) ? c-32 : c)


//----------------------------------------------------------------------
// compare strings ignoring case
int stricmp(const char *str1, const char *str2) 
{
    for (; TOUPPER(*str1) == TOUPPER(*str2); ++str1, ++str2) 
        if (*str1 == '\0') return 0; // Strings are equal

    // Strings differ, return difference based on lexical order
    return (TOUPPER(*str1) - TOUPPER(*str2)) > 0 ? 1 : -1;
}

//----------------------------------------------------------------------
// take a photo and either save it on sd or upload it with FTP
void makeMeasurement()
{
  char fname[128];
  camera_fb_t * fb = NULL;

  photoCount++;
  print("Capturing photo :");
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) 
  {
    println("Camera capture failed");
    return;
  }

  if (wantSdMmc)
  {
    // ---- store photo on SD card
    String ttag = rtc.getTime("%Y%m%d-%H%M%S"); // "20231216-182201"
    sprintf(fname, "/DCM%d-%s.jpg", photoCount,ttag.c_str());
    println(fname);
  
    // store to Sd card
    File fout = fileSystem.open(fname,FILE_WRITE);
    if (!fout)
    {
      println("Failed to create photo file");
    }
    else
    {
      fout.write(fb->buf, fb->len);
      fout.close();
      addLogLine(fname);
      //println("File saved");
    }
  }
  else
  {
    // ---- upload photo TO FTP server
    String ttag = rtc.getTime("%Y%m%d-%H%M%S"); // "20231216-182201"
    sprintf(fname, "DCM%d-%s.jpg", bootCount,ttag.c_str());
    println(fname);
    connectToWiFi(); // first connect this ESP32 to the WIFI network
    getNtpTime(); // resync time
    ESP32_FTPClient ftp (uploadHost,ftpUser,ftpPswd, 5000, 2);
    ftp.OpenConnection();
    ftp.InitFile("Type I");
    ftp.ChangeWorkDir(ftpFolder);
    ftp.NewFile(fname);
    ftp.WriteData( fb->buf, fb->len );
    ftp.CloseFile();
    ftp.CloseConnection();
    turnOffWiFi(); // turn off when done
    addLogLine(fname);
    println("Upload complete");
  }
  esp_camera_fb_return(fb); // return frame buffer memory
}


//----------------------------------------------------------------------
// The default setup method called when the ESP32 boots or wakes up from
// deep sleep.
//
// The entire logic of the app happens here, and then we put the ESP32
// back into deep sleep.   This means that the loop() function is NEVER
// CALLED.
//
void setup() 
{
  char buf[128]; // temporary buffer
  char firstTimeDelay = false;

  ++bootCount; // count how many times we wake up/boot up
  
  // Brown-out detector on ESP32 can mis-fire and cause reboots
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  //----------------------------------------------------------------------
  // *** Start out with things we want to do every time we wake up or boot up
  //----------------------------------------------------------------------

  // first, we find out what the reason is that the ESP32 is booting
  // either the reason is an initial boot or a deep sleep wakeup from a timer
  esp_sleep_wakeup_cause_t wakeup_reason; // to see why we booted (either power on, or wake up)
  // let's see the reason that the ESP32 woke up 
  wakeup_reason = esp_sleep_get_wakeup_cause();

#ifdef WANTSERIAL
  Serial.begin(115200);
  println(SIGNON);
#endif

  // ok, we need to access the configuration file.
  // It's either on the SD MMC card, or on the local flash file system.
  // We'll try the SD card first, if that doesn't work, then try the
  // on chip local flash

  int gotSdCardMounted = false;
  // -- Try SD card setup
  fsType = FSTYPE_NONE;
  if(!SD_MMC.begin()) 
  {
    println("SD_MMC Card Mount Failed");
  }
  else
  {
    uint8_t cardType = SD_MMC.cardType();
  
    if(cardType == CARD_NONE) {
      println("No SD card attached");
    }
    else
    {
      gotSdCardMounted = true;
      fileSystem = SD_MMC;
      fsType = FSTYPE_SDMMC;
      print("Mounted SD_MMC card successfully, type is ");

      if(cardType == CARD_MMC){
        println("MMC");
      } else if(cardType == CARD_SD){
        println("SDSC");
      } else if(cardType == CARD_SDHC){
        println("SDHC");
      } else {
        println("UNKNOWN CARD TYPE");
      }
      /*
      uint64_t cardSize = SD_MMC.cardSize();
      int cardSizeInMB = cardSize/(1024 * 1024);
       
      sprintf(buf, "Card size: %d", cardSizeInMB);
      println(buf);
    
    
      uint64_t bytesAvailable = SD_MMC.totalBytes(); 
      int bytesAvailableInMB = bytesAvailable/(1024 * 1024);
    
      sprintf(buf, "MB available: %d", bytesAvailableInMB);
      println(buf);
    
    
      uint64_t bytesUsed = SD_MMC.usedBytes();
      int bytesUsedInMB = bytesUsed/(1024 * 1024);
    
      sprintf(buf, "MB used: %d", bytesUsedInMB);
      println(buf);
      */
    }
  }
  
  // Initialize SPIFFS (SPI flash file system) if SD failed
  if(!gotSdCardMounted)
  {
    if (!SPIFFS.begin(true))
    {
      println("SPIFFS File System Mount Failed, unable to continue");
      delay(1000);
      for (;;) ; // no point in going forward here - can't possibly read config.ini
    }
    else
    {
      fileSystem = SPIFFS; // trying to read configuration file from internal local flash
      fsType = FSTYPE_FFSPI;
      println("Mounted Local flash file system successfully");
    }
  }

  //-------------------------------------------------------------
  // Boot up tasks - Stuff to do on an initial boot from power 
  // on or hard reset
  //-------------------------------------------------------------
  if ((bootCount == 0) || (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER))
  {
    firstTimeDelay = true;
    // read needed data from the config file - 
    // This data is stored in RTC memory so it survives deep sleep
    // So we only read this on first power up
    readKey(CONFIGFN,"SSID=",ssid,127); 
    readKey(CONFIGFN,"PASSWORD=",password,127);
    wantSdMmc = false;
    readKey(CONFIGFN,"USESD=",buf,31);
    if (stricmp(buf,"yes")==0) wantSdMmc=true;
    if (stricmp(buf,"true")==0) wantSdMmc=true;
    if (!wantSdMmc)
    {
      // We don't need these for SD storage mode
      readKey(CONFIGFN,"SERVER=",uploadHost,127);
      readKey(CONFIGFN,"FTPUSER=",ftpUser,127);
      readKey(CONFIGFN,"FTPPASSWORD=",ftpPswd,127);
      readKey(CONFIGFN,"FTPFOLDER=",ftpFolder,127);
    }
    // number of photos to take per day ( 1440 max, one photo per minute)
    // 24  = once per hour
    // 48  = once every 30 minutes
    // 96  = once every 15 minutes
    // 144 = once every 10 minutes
    //
    // If you piece these together into a video at 30 FPS
    // then with 144 photos per day you'll get about 5 seconds 
    // of video per day
    //
    readKey(CONFIGFN,"PHOTOSPERDAY=",buf,30);
    delayBetweenPhotosMs=24L*60L*60L*1000L/atol(buf);
    if (delayBetweenPhotosMs < 60000L)
       delayBetweenPhotosMs = 60000L; // once per minute if fastest
    if (delayBetweenPhotosMs > 24L*60L*60L*1000L)
      delayBetweenPhotosMs = 24L*60L*60L*1000L; // once per day is slowest    

    // initialize the RTC and read network time if possible
    rtc.setTime(0,0,0,1,1,2024); // default time 00:00:00 1/1/2023
    // on first boot, try to connect to WiFi and get time/date from NTP
    connectToWiFi();
    if (connectToWiFi() == 1)
    {
      needNtp = true;
      getNtpTime(); // get NTP time if possible
    }
    turnOffWiFi(); // now we can turn off the WiFi modem to save power    
  }
  

  //-------------------------------------------------------------
  // *** Tasks to do only if we wake from a deep sleep
  //-------------------------------------------------------------
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
  {
    // take a photo and store or upload it
    setupMeasurement();
    makeMeasurement();
  }

  //-----------------------------------
  // *** And ... go back to sleep now
  //-----------------------------------

  // kindly shut down the file system
  if (fsType == FSTYPE_FFSPI) SPIFFS.end();
  if (fsType == FSTYPE_SDMMC) SD_MMC.end();
  
  long milliseconds = millis(); // how long have we been awake?
  sprintf(buf,"Awake for %d ms",milliseconds);
  println(buf);

  // try to wake up exactly at the right time based on how
  // often we are supposed to take a photo and how much
  // time we've been awake.
  //
  // Note: This depends on the accuracy of the clock of the ESP32
  // which may be off by +/- a bit and will vary depending on
  // temperature.   So things may drift a bit over time.
  
  long tts = (delayBetweenPhotosMs - millis()) * ms_TO_uS_FACTOR;
  if (firstTimeDelay)
  {
    // first time, we wait until time is an integer amount of the delayBetweenPhotos
    // so that if we have photos every hour, then they will be every hour on the hour.
    unsigned long nowms = rtc.getHour()*3600L*1000L + rtc.getMinute()*60L*1000L + rtc.getSecond()*1000L;
    unsigned long nperiods = nowms / delayBetweenPhotosMs;
    unsigned long waketimems = (nperiods+1)*delayBetweenPhotosMs;
    tts = (waketimems-nowms)*ms_TO_uS_FACTOR;
  }
  sprintf(buf,"Sleeping for %ld us",tts);
  println(buf);
  
  esp_sleep_enable_timer_wakeup(tts); // how long to sleep
  esp_deep_sleep_start(); // good night!  Have a nice nap.
  // and, for deep sleep, the ESP32 never executes any code past the deep_sleep_start!
  // on next wakeup or reset, the startup() function will be called again.
}

//----------------------------------------------------------------------
void loop() 
{
  // nothing goes here since this app is using the deep-sleep mode
  // and the ESP32 will never really execute any code here.
}
