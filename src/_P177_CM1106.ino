#include "_Plugin_Helper.h"
#ifdef USES_P177

/*

   This plug in is written by Mariete (mario ___ emariete.com) more info at https://emariete.com
   Implemented as described here: https://en.gassensor.com.cn/Product_files/Specifications/CM1106-C%20Single%20Beam%20NDIR%20CO2%20Sensor%20Module%20Specification.pdf
   Plugin is based upon CM1106 plugin by  Dmitry (rel22 ___ inbox.ru)
   CM1106 plugin is based upon SenseAir plugin by Daniel Tedenljung info__AT__tedenljungconsulting.com
   Additional features based on https://geektimes.ru/post/285572/ by Gerben (infernix__AT__gmail.com)

   This plugin reads the CO2 value from Cubic's CM1106 Sensor

   Pin-out:
   CM1106:  Connection:
   VCC     5 V
   GND     GND
   Tx      ESP8266 1st GPIO specified in Device-settings
   Rx      ESP8266 2nd GPIO specified in Device-settings
 */

#define PLUGIN_177
#define PLUGIN_ID_177         177
#define PLUGIN_NAME_177       "Gases - CO2 CM1106"
#define PLUGIN_VALUENAME1_177 "PPM"
#define PLUGIN_READ_TIMEOUT   300

#define PLUGIN_177_FILTER_OFF            1
#define PLUGIN_177_FILTER_OFF_ALLSAMPLES 2
#define PLUGIN_177_FILTER_FAST           3
#define PLUGIN_177_FILTER_MEDIUM         4
#define PLUGIN_177_FILTER_SLOW           5

#define PLUGIN_177_BASE_PPM            415 // based on co2.earth value.

#include <ESPeasySerial.h>


enum cm1106Types {
  CM1106_notDetected,
  CM1106
};


enum cm1106Commands : byte { cm1106CmdReadPPM,
                          cm1106CmdABCEnable,
                          cm1106CmdABCDisable,
                          cm1106CmdCalibrate,
                          cm1106CmdGetSerialNumber,
                          cm1106CmdReadSWVersion,
                          cm1106CmdCalibrateZero,
                          cm1106CmdReset
};

// CM1106 commands: {HEAD LEN CMD DATA1 … DATAn CS}
// Check Sum Cumulative sum of data = 256-(HEAD+LEN+CMD+DATA)%256
// 
// cm1106CmdReadPPM[]              = { 0x11,0x01,0x01,0xED };
// cm1106CmdABCEnable[]            = { 0x11,0x07,0x10,0x64,0x00,0x07,0x01,0x90,0x64,0x78 };
// cm1106CmdABCDisable[]           = { 0x11,0x07,0x10,0x64,0x02,0x07,0x01,0x90,0x64,0x76 };
// cm1106CmdCalibration400ppm[]    = { 0x11,0x03,0x3,0x1,0x90,0xff }; // Calculate Checksum and replace 0xff
// cm1106CmdCalibration600ppm[]    = { 0x11,0x03,0x3,0x2,0x58,0x8F };
// cm1106CmdSWVer[]                = { 0x11,0x01,0x1E,0xD0 };
// cm1106CmdSerNum[]               = { 0x11,0x01,0x1F,0xCF };

byte cm1106CmdData[][10] = {
  { 0x11,0x01,0x01,0xED },
  { 0x11,0x07,0x10,0x64,0x00,0x07,0x01,0x90,0x64,0x78 },
  { 0x11,0x07,0x10,0x64,0x02,0x07,0x01,0x90,0x64,0x76 },
  { 0x11,0x03,0x03,0x02,0x58,0x8F}, // calibrate to 600ppm - D1, D2, CS must be changed before sending cmd
  { 0x11,0x01,0x1F,0xCF},
  { 0x11,0x01,0x1E,0xD0},

  { 0x87, 0x00, 0x00 },
  { 0x8d, 0x00, 0x00 }
};

enum
{
  P177_ABC_enabled  = 0x00,
  P177_ABC_disabled = 0x02
};


struct P177_data_struct : public PluginTaskData_base {
  P177_data_struct() {
    reset();
    sensorResets = 0;
  }

  ~P177_data_struct() {
    reset();
  }

  void reset() {
    if (easySerial != nullptr) {
      delete easySerial;
      easySerial = nullptr;
    }
    linesHandled       = 0;
    checksumFailed     = 0;
    nrUnknownResponses = 0;
    ++sensorResets;

    // Default of the sensor is to run ABC
    ABC_Disable     = false;
    ABC_MustApply   = false;
  }

  bool init(ESPEasySerialPort port, const int16_t serial_rx, const int16_t serial_tx, bool setABCdisabled) {
    if ((serial_rx < 0) || (serial_tx < 0)) {
      return false;
    }
    reset();
    easySerial = new (std::nothrow) ESPeasySerial(port, serial_rx, serial_tx);

    if (easySerial == nullptr) {
      return false;
    }
    easySerial->begin(9600);
    ABC_Disable = setABCdisabled;

    if (ABC_Disable) {
      // No guarantee the correct state is active on the sensor after reboot.
      ABC_MustApply = true;
    }
    lastInitTimestamp = millis();
    initTimePassed    = false;
    return isInitialized();
  }

  bool isInitialized() const {
    return easySerial != nullptr;
  }

  void setABCmode(int abcDisableSetting) {
    boolean new_ABC_disable = (abcDisableSetting == P177_ABC_disabled);

    if (ABC_Disable != new_ABC_disable) {
      // Setting changed in the webform.
      ABC_MustApply = true;
      ABC_Disable   = new_ABC_disable;
    }
  }


  String getBufferHexDump(byte *buffer) {
    String result;

    result.reserve(27);

    for (int i = 0; i < buffer[1]+3; ++i) {
      result += ' ';
      result += String(buffer[i], HEX);
    }
    return result;
  }

  //  Cumulative sum of data = 256-(HEAD+LEN+CMD+DATA)%256
  byte calculateChecksum(byte *buffer) {
    byte checksum = 0;
    uint len = buffer[1] + 2; // 2 bytes for head + len

    for (byte i = 0; i < len; i++) {
      checksum += buffer[i];
    }

    checksum = checksum % 256;
    checksum = 256 - checksum;
    return checksum;
  }

  size_t send_cm1106Cmd(byte CommandId)
  {
    if (!isInitialized()) { return 0; }

    if (!initTimePassed) {
      // Allow for 1 minute of init time.
      initTimePassed = timePassedSince(lastInitTimestamp) > 60000;
    }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log = F("CM1106: send_cm1106Cmd:");
          log += getBufferHexDump(cm1106CmdData[CommandId]);
          addLog(LOG_LEVEL_INFO, log);
        }

    return easySerial->write(cm1106CmdData[CommandId], cm1106CmdData[CommandId][1] + 3);
  }

  size_t getResponse() 
  {
    // get response
    memset(cm1106Resp, 0, sizeof(cm1106Resp));

    long timer   = millis() + PLUGIN_READ_TIMEOUT;
    int  counter = 0;
    int responseLen = 16; // max response length

    while (!timeOutReached(timer) && (counter < responseLen)) {
      if (easySerial->available() > 0) {
        byte value = easySerial->read();

        if (((counter == 0) && (value == 0x16)) || (counter > 0)) {
          cm1106Resp[counter++] = value;
        }

        if (counter == 2) { // after reading 2 bytes, we've got the length
          responseLen = cm1106Resp[1] + 3;
        }
      } else {
        delay(10);
      }
    }

    if (loglevelActiveFor(LOG_LEVEL_INFO)) {
      String log = F("CM1106 Received:");
      log += getBufferHexDump(cm1106Resp);
      // log += stringHexDump(cm1106Resp);
      // log += String(cm1106Resp,HEX);
      addLog(LOG_LEVEL_INFO, log);
    }

    int checksum = calculateChecksum(cm1106Resp);
    if (!(cm1106Resp[responseLen-1] == checksum)) {
      String log = F("CM1106 ChecksumFailed ");
      log += String(checksum, HEX);
      addLog(LOG_LEVEL_INFO, log);
      ++checksumFailed;
      return 0;
    }

    return responseLen;
  }

  bool calibrate_ppm(unsigned int ppm) {
    if (!isInitialized()) { return false; }
    // send read PPM command
    byte D1, D2;
    D1 = (int) ppm / 256;
    D2 = (int) ppm % 256;
    cm1106CmdData[cm1106CmdCalibrate][3] = D1;
    cm1106CmdData[cm1106CmdCalibrate][4] = D2;
    cm1106CmdData[cm1106CmdCalibrate][5] = calculateChecksum(cm1106CmdData[cm1106CmdCalibrate]);    

    bytesRead = send_cm1106Cmd(cm1106CmdCalibrate);
    if (bytesRead != 4) return false;

    getResponse();
    ++linesHandled;

    return (cm1106Resp[2] == 0x03);
  }

  bool read_ppm(unsigned int& ppm) {
    if (!isInitialized()) { return false; }

    // send read PPM command
    bytesRead = send_cm1106Cmd(cm1106CmdReadPPM);

    if (bytesRead != 4) {
      String log = F("Bytes sent not = 4: Bytes sent ");
          log += String(bytesRead);
          addLog(LOG_LEVEL_INFO, log);
      return false;
    }

    if (getResponse() < 8)
    {
      // Timeout
      String log = F("CM1106 Timeout");
      addLog(LOG_LEVEL_INFO, log);
      return false;
    }
    ++linesHandled;

    if ((cm1106Resp[0] == 0x16) && (cm1106Resp[1] == 0x05)) {
      // calculate CO2 PPM
      // ppm = (static_cast<unsigned int>(response[2]) << 8) + response[3];
      ppm = (cm1106Resp[3]*256+cm1106Resp[4]);
    }

    return true;
  }

  bool read_swVersion() {
    if (!isInitialized()) { return false; }

    // send read PPM command
    bytesRead = send_cm1106Cmd(cm1106CmdReadSWVersion);

    if (bytesRead != 4) {
      String log = F("Bytes sent not = 4 in read_swVersion: Bytes sent ");
          log += String(bytesRead);
          addLog(LOG_LEVEL_INFO, log);
      return false;
    }

    if (getResponse() < 15)
    {
      // Timeout
      String log = F("CM1106 Timeout");
      addLog(LOG_LEVEL_INFO, log);
      return false;
    }

    ++linesHandled;

    if ((cm1106Resp[0] == 0x16) && (cm1106Resp[2] == 0x1E)) {
      // calculate CM1106 SW version
      // DF1-DF10:stand for ASCII code of software version, DF11 is reserved
      // Example:
      // When the sensor version is CM V0.0.20, response data as follows:
      // Hexadecimal converted to ASCII code:
      // Note: when 20 converted to ASCII code, it equals to blank space.
      // 16 0C 1E 43 4D 20 56 30 2E 30 2E 32 30 00 97
      // CM V0.0.20
      String swVersion = F("CM1106 SW Version: ");
      for(int k=3; k<13; k++){
        swVersion += (char) cm1106Resp[k];
      }
      addLog(LOG_LEVEL_INFO, swVersion);
    }

    return true;
  }

  cm1106Types getDetectedDevice() {
    if (linesHandled > checksumFailed) {
      return CM1106;
    }
    return CM1106_notDetected;
  }

  uint32_t      linesHandled       = 0;
  uint32_t      checksumFailed     = 0;
  uint32_t      sensorResets       = 0;
  uint32_t      nrUnknownResponses = 0;
  unsigned long lastInitTimestamp  = 0;

  ESPeasySerial *easySerial = nullptr;
  byte           cm1106Resp[15]; // 15 byte response buffer
  int            bytesRead; // Number of bytes readed from serial
  // Default of the sensor is to run ABC
  bool ABC_Disable     = false;
  bool ABC_MustApply   = false;
  bool initTimePassed  = false;
};

boolean Plugin_177(byte function, struct EventStruct *event, String& string)
{
  bool success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
    {
      Device[++deviceCount].Number           = PLUGIN_ID_177;
      Device[deviceCount].Type               = DEVICE_TYPE_SERIAL;
      Device[deviceCount].VType              = Sensor_VType::SENSOR_TYPE_SINGLE;
      Device[deviceCount].Ports              = 0;
      Device[deviceCount].PullUpOption       = false;
      Device[deviceCount].InverseLogicOption = false;
      Device[deviceCount].FormulaOption      = true;
      Device[deviceCount].ValueCount         = 1;
      Device[deviceCount].SendDataOption     = true;
      Device[deviceCount].TimerOption        = true;
      Device[deviceCount].GlobalSyncOption   = true;
      break;
    }

    case PLUGIN_GET_DEVICENAME:
    {
      string = F(PLUGIN_NAME_177);
      break;
    }

    case PLUGIN_GET_DEVICEVALUENAMES:
    {
      strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_177));
      break;
    }

    case PLUGIN_GET_DEVICEGPIONAMES:
    {
      serialHelper_getGpioNames(event);
      break;
    }

    case PLUGIN_WEBFORM_SHOW_CONFIG:
    {
      string += serialHelper_getSerialTypeLabel(event);
      success = true;
      break;
    }


    case PLUGIN_WEBFORM_LOAD:
    {
      {
        byte choice         = PCONFIG(0);
        String options[2]   = { F("Normal"), F("ABC disabled") };
        int optionValues[2] = { P177_ABC_enabled, P177_ABC_disabled };
        addFormSelector(F("Auto Base Calibration"), F("p177_abcdisable"), 2, options, optionValues, choice);
      }
      {
        byte   choiceFilter     = PCONFIG(1);
        String filteroptions[5] =
        { F("Skip Unstable"), F("Use Unstable"), F("Fast Response"), F("Medium Response"), F("Slow Response") };
        int filteroptionValues[5] = {
          PLUGIN_177_FILTER_OFF,
          PLUGIN_177_FILTER_OFF_ALLSAMPLES,
          PLUGIN_177_FILTER_FAST,
          PLUGIN_177_FILTER_MEDIUM,
          PLUGIN_177_FILTER_SLOW };
        addFormSelector(F("Filter"), F("p177_filter"), 5, filteroptions, filteroptionValues, choiceFilter);
      }
      P177_html_show_stats(event);

      success = true;
      break;
    }

    case PLUGIN_WEBFORM_SAVE:
    {
      const int formValue = getFormItemInt(F("p177_abcdisable"));

      P177_data_struct *P177_data =
        static_cast<P177_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr != P177_data) {
        P177_data->setABCmode(formValue);
      }
      PCONFIG(0) = formValue;
      PCONFIG(1) = getFormItemInt(F("p177_filter"));
      success    = true;
      break;
    }

    case PLUGIN_INIT:
    {
      initPluginTaskData(event->TaskIndex, new (std::nothrow) P177_data_struct());
      success = P177_performInit(event);
      break;
    }

    case PLUGIN_EXIT: {
      success = true;
      break;
    }

    case PLUGIN_WRITE:
    {
      P177_data_struct *P177_data =
        static_cast<P177_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr == P177_data) {
        return success;
      }

      String command = parseString(string, 1);

      if (command == F("cm1106calibrate"))
      {
        float current_ppm;
        
        if ( ! string2float(parseString(string,2), current_ppm)) 
          current_ppm = PLUGIN_177_BASE_PPM ;

        P177_data->calibrate_ppm(current_ppm);        
        String log = String(F("CM1106: Calibrated to "));
        log += current_ppm;
        addLog(LOG_LEVEL_INFO, log);
        success = true;
      }
      else if (command == F("cm1106reset"))
      {
        P177_data->send_cm1106Cmd(cm1106CmdReset);
        P177_data->getResponse();
        addLog(LOG_LEVEL_INFO, F("CM1106: Sent sensor reset!"));
        success = true;
      }
      else if (command == F("cm1106abcenable"))
      {
        P177_data->send_cm1106Cmd(cm1106CmdABCEnable);
        P177_data->getResponse();
        addLog(LOG_LEVEL_INFO, F("CM1106: Sent sensor ABC Enable!"));
        success = true;
      }
      else if (command == F("cm1106abcdisable"))
      {
        P177_data->send_cm1106Cmd(cm1106CmdABCDisable);
        P177_data->getResponse();
        addLog(LOG_LEVEL_INFO, F("CM1106: Sent sensor ABC Disable!"));
        success = true;
      }
      else if (command == F("cm1106readppm"))
      {
        unsigned int ppm;
        P177_data->read_ppm(ppm);
        String log = F("CM1106: PPM read:");
        log += ppm;
        addLog(LOG_LEVEL_INFO, log);
        success = true;
      }
      else if (command == F("cm1106readsoftwareversion"))
      {
        P177_data->read_swVersion();
        success = true;
      }
      break;
    }

    case PLUGIN_READ:
    {
      P177_data_struct *P177_data =
        static_cast<P177_data_struct *>(getPluginTaskData(event->TaskIndex));

      if (nullptr == P177_data) {
        return success;
      }
      bool expectReset  = false;
      unsigned int ppm  = 0;
      
      if (P177_data->read_ppm(ppm)) {
        String log = F("CM1106: ");

        // During (and only ever at) sensor boot, 'ppm' is reported as 550
        // We log but don't process readings during that time
        if (ppm == 550) {
          log += F("Bootup detected! ");

          if (P177_data->ABC_Disable) {
            // After bootup of the sensor the ABC will be enabled.
            // Thus only actively disable after bootup.
            P177_data->ABC_MustApply = true;
            log                     += F("Will disable ABC when bootup complete. ");
          }
          success = false;

          // Finally, stable readings are used for variables
        } else {
          const int filterValue = PCONFIG(1);

          UserVar[event->BaseVarIndex]     = (float)ppm;
          success                          = true;          
        }

        if (ppm!=550) {
          // Reading is stable.
          if (P177_data->ABC_MustApply) {
            // Send ABC enable/disable command based on the desired state.
            if (P177_data->ABC_Disable) {
              P177_data->send_cm1106Cmd(cm1106CmdABCDisable);
              addLog(LOG_LEVEL_INFO, F("MHZ19: Sent sensor ABC Disable!"));
            } else {
              P177_data->send_cm1106Cmd(cm1106CmdABCEnable);
              addLog(LOG_LEVEL_INFO, F("MHZ19: Sent sensor ABC Enable!"));
            }
            P177_data->ABC_MustApply = false;
          }
        }

        // Log values in all cases
        log += F("PPM value: ");
        log += ppm;
        addLog(LOG_LEVEL_INFO, log);
        break;

        // log verbosely anything else that the sensor reports
      } else {
        if (loglevelActiveFor(LOG_LEVEL_INFO)) {
          String log = F("CM1106: Unknown response:");
          log += P177_data->getBufferHexDump(P177_data->cm1106Resp);
          addLog(LOG_LEVEL_INFO, log);
        }

        // Check for stable reads and allow unstable reads the first 3 minutes after reset.
        if ((P177_data->nrUnknownResponses > 10) && P177_data->initTimePassed) {
          P177_performInit(event);
        }
        success = false;
        break;
      }
      break;
    }
  }
  return success;
}

bool P177_performInit(struct EventStruct *event) {
  bool success                 = false;
  const int16_t serial_rx      = CONFIG_PIN1;
  const int16_t serial_tx      = CONFIG_PIN2;
  const ESPEasySerialPort port = static_cast<ESPEasySerialPort>(CONFIG_PORT);
  P177_data_struct *P177_data  =
    static_cast<P177_data_struct *>(getPluginTaskData(event->TaskIndex));

  if (nullptr == P177_data) {
    return success;
  }

  if (P177_data->init(port, serial_rx, serial_tx, (PCONFIG(0) == P177_ABC_disabled))) {
    success = true;
    addLog(LOG_LEVEL_INFO, F("CM1106: Init OK "));

    // delay first read, because hardware needs to initialize on cold boot
    // otherwise we get a weird value or read error
    Scheduler.schedule_task_device_timer(event->TaskIndex, millis() + 15000);
  }
  return success;
}

void P177_html_show_stats(struct EventStruct *event) {
  P177_data_struct *P177_data =
    static_cast<P177_data_struct *>(getPluginTaskData(event->TaskIndex));

  if (nullptr == P177_data) {
    return;
  }

  addRowLabel(F("Checksum (pass/fail/reset)"));
  String chksumStats;

  chksumStats  = P177_data->linesHandled;
  chksumStats += '/';
  chksumStats += P177_data->checksumFailed;
  chksumStats += '/';
  chksumStats += P177_data->sensorResets;
  addHtml(chksumStats);
  addRowLabel(F("Detected"));

  switch (P177_data->getDetectedDevice()) {
    case CM1106: addHtml(F("CM1106")); break;
    default: addHtml("---"); break;
  }
}
#endif // USES_P177
