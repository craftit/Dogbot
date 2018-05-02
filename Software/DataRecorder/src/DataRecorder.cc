#include "dogbot/DataRecorder.hh"
#include <math.h>

namespace DogBotN {

  PGDataRecorderC::PGLogEntryC::PGLogEntryC(bool isCommand,const uint8_t *data,int len)
   : m_isCommand(isCommand),
     m_len(len)
  {
    gettimeofday(&m_when,0);
    if(len > sizeof(m_data))
      len = sizeof(m_data);
    memcpy(m_data,data,len);
  }

  // -----------------------------------------------------------------------------

  PGDataRecorderC::PGDataRecorderC(const std::shared_ptr<DogBotAPIC> &api,const std::string &databaseConnectionStr)
    : m_api(api),
      m_coms(api->Connection()),
      m_databaseConnectionString(databaseConnectionStr),
      m_connection(databaseConnectionStr)
  {
    m_recordQueue.reserve(256);
    m_recordQueueWrite.reserve(256);
  }

  //! Destructor, this will wait for the write thread to complete
  //! its current write and exit.
  PGDataRecorderC::~PGDataRecorderC()
  {
    m_terminated = true;
    if(m_writeThread.joinable())
      m_writeThread.join();
  }

  //! Create schema in database.
  void PGDataRecorderC::CreateSchema()
  {
    //pqxx::connection dbcon(m_databaseConnectionString);
    //pqxx::work txn(m_connection,"Create schema");


  }

  //! Find the name for a device
  std::string PGDataRecorderC::DeviceName(int deviceId)
  {
    assert(deviceId < 256);
    if(deviceId < 0 || deviceId > 255)
      return "Invalid";
    std::lock_guard<std::mutex> lock(m_accessDeviceData);

    while(deviceId >= m_deviceNames.size()) {
      m_deviceNames.push_back(std::string());
    }
    if(m_deviceNames[deviceId].empty()) {
      auto servoPtr = m_api->GetServoById(deviceId);
      if(!servoPtr)
        return std::to_string(deviceId);
      m_deviceNames[deviceId] = servoPtr->Name();
    }
    return m_deviceNames[deviceId];
  }


  static std::string AsISO8601(const struct timeval &when) {
    struct tm b;

    //static double offset = (double) (std::chrono::system_clock::now() - StdTimeT::clock::now()).count();

    double seconds =0;
    double fract = (double) when.tv_usec / 1000000.0;

    time_t s =  when.tv_sec;

    gmtime_r(&s,&b);
    const int buffSize = 64;
    std::string ret(buffSize,0);

    //std::cerr << "Seconds:" << seconds << " Fract:" << fract << " Year:" << b.tm_year << " "  << std::endl;

    double sec = (double) b.tm_sec + fract;
    int len = 0;
    if(b.tm_sec < 10) {
      len = snprintf(&ret[0],buffSize,"%04u-%02u-%02uT%02u:%02u:0%1fZ",b.tm_year + 1900,b.tm_mon+1,b.tm_mday,b.tm_hour,b.tm_min,sec);
    } else {
      len = snprintf(&ret[0],buffSize,"%04u-%02u-%02uT%02u:%02u:%2fZ",b.tm_year + 1900,b.tm_mon+1,b.tm_mday,b.tm_hour,b.tm_min,sec);
    }
    ret.resize(len);
    //std::cerr << "Time : " << ret << std::endl;
    return ret;
  }

  //! Log a servo report message.
  bool PGDataRecorderC::LogServoReport(pqxx::work &txn,std::string &query,PGLogEntryC &a)
  {
    if(a.m_len != sizeof(struct PacketServoReportC)) {
      m_log->error("Unexpected 'ServoReport' packet length {} ",a.m_len);
      return false;
    }
    const PacketServoReportC *pkt = (const PacketServoReportC *) a.m_data;

    if(query.empty()) {
      query.reserve(4096);
      query += "INSERT INTO joint_report (sourceid,logtime,synctime,position,velocity,effort,reference,position_limit,torque_limit,velocity_limit,index_sensor) VALUES ";
    } else
      query += ",";

    query += "(";
    query += txn.quote(DeviceName(pkt->m_deviceId));
    query += ",";
    query += txn.quote(AsISO8601(a.m_when));
    query += ",";
    query += txn.quote((unsigned) pkt->m_timestamp);
    query += ",";
    query += txn.quote(ComsC::PositionReport2Angle(pkt->m_position));
    query += ",";
    query += txn.quote(ComsC::VelocityReport2Angle(pkt->m_velocity));
    query += ",";
    query += txn.quote(ComsC::TorqueReport2Fraction(pkt->m_torque));
    query += ",";
    query += txn.quote(ComsPositionRefrenceToString((enum PositionReferenceT)(pkt->m_mode & DOGBOT_SERVOREPORTMODE_POSITIONREF)));
    query += ",";
    query += txn.quote((pkt->m_mode & DOGBOT_SERVOREPORTMODE_LIMITPOSITION) != 0);
    query += ",";
    query += txn.quote((pkt->m_mode & DOGBOT_SERVOREPORTMODE_LIMITTORQUE) != 0);
    query += ",";
    query += txn.quote((pkt->m_mode & DOGBOT_SERVOREPORTMODE_LIMITVELOCITY) != 0);
    query += ",";
    query += txn.quote((pkt->m_mode & DOGBOT_SERVOREPORTMODE_INDEXSENSOR) != 0);
    query += ")";

    return true;
  }


  //! Log a servo demand message.
  bool PGDataRecorderC::LogServo(pqxx::work &txn,std::string &query,PGLogEntryC &a)
  {
    if(a.m_len != sizeof(struct PacketServoC)) {
      m_log->error("Unexpected 'Servo' packet length {} ",a.m_len);
      return false;
    }
    const PacketServoC *pkt = (const PacketServoC *) a.m_data;
    if(query.empty()) {
      query.reserve(4096);
      query += "INSERT INTO joint_demand (sourceid,logtime,position,effort_limit) VALUES ";
    } else
      query += ",";

    query += "(";
    query += txn.quote(DeviceName(pkt->m_deviceId));
    query += ",";
    query += txn.quote(AsISO8601(a.m_when));
    query += ",";
    query += txn.quote(ComsC::PositionReport2Angle(pkt->m_position));
    query += ",";
    query += txn.quote((double) pkt->m_torqueLimit / 65535.0);
    query += ")";

    return true;
  }

  //! Log a parameter report message
  bool PGDataRecorderC::LogParamReport(pqxx::work &txn,std::string &query,PGLogEntryC &a)
  {
    const PacketParam8ByteC *pkt = (const PacketParam8ByteC *) a.m_data;

    if(query.empty()) {
      query.reserve(4096);
      query += "INSERT INTO parameter_report (sourceid,logtime,parameter,value) VALUES ";
    } else
      query += ",";

    enum ComsParameterIndexT paramIndex =  (enum ComsParameterIndexT) pkt->m_header.m_index;
    query += "(";
    query += txn.quote(DeviceName(pkt->m_header.m_deviceId));
    query += ",";
    query += txn.quote(AsISO8601(a.m_when));
    query += ",";
    query += txn.quote(DogBotN::ComsParameterIndexToString(paramIndex));
    query += ",";
    switch(DogBotN::ComsParameterIndexToType(paramIndex))
    {
      case CPIT_Unknown:
      case CPIT_Invalid:
      case CPIT_Custom:
        m_log->warn("Don't know how to handle field {} with parameter type {} ",DogBotN::ComsParameterIndexToString(paramIndex),(int) DogBotN::ComsParameterIndexToType(paramIndex));
        query += txn.quote("?");
        break;
      case CPIT_enum8:
        switch(paramIndex)
        {
          case CPI_PWMMode:
            query += txn.quote(ControlDynamicToString((enum PWMControlDynamicT) pkt->m_data.uint8[0]));
            break;
          case CPI_ControlState:
            query += txn.quote(ControlStateToString((enum ControlStateT) pkt->m_data.uint8[0]));
            break;
          case CPI_HomedState:
            query += txn.quote(HomedStateToString((enum MotionHomedStateT) pkt->m_data.uint8[0]));
            break;
          case CPI_FaultCode:
            query += txn.quote(FaultCodeToString((enum FaultCodeT) pkt->m_data.uint8[0]));
            break;
          case CPI_SafetyMode:
            query += txn.quote(SafetyModeToString((enum SafetyModeT) pkt->m_data.uint8[0]));
            break;
          case CPI_PWMState:
          case CPI_PositionRef:
          case CPI_DeviceType:
          case CPI_JointRelative:
          case CPI_FanMode:
          case CPI_FanState:
          case CPI_JointRole:
          default:
            query += txn.quote(std::to_string((int) pkt->m_data.uint8[0]).c_str());
            break;
        }
        break;
      case CPIT_bool:
        if(pkt->m_data.uint8[0] > 0)
          query += txn.quote("1");
        else
          query += txn.quote("0");
        break;
      case CPIT_uint8:
        query += txn.quote(std::to_string((int) pkt->m_data.uint8[0]).c_str());
        break;
      case CPIT_int8:
        query += txn.quote(std::to_string((int) pkt->m_data.int8[0]).c_str());
        break;
      case CPIT_uint16:
        query += txn.quote(std::to_string(pkt->m_data.uint16[0]).c_str());
        break;
      case CPIT_int16:
        query += txn.quote(std::to_string(pkt->m_data.int16[0]).c_str());
        break;
      case CPIT_uint16_3:
        query += txn.quote((
            std::to_string(pkt->m_data.uint16[0]) + " " +
            std::to_string(pkt->m_data.uint16[1]) + " " +
            std::to_string(pkt->m_data.uint16[2])).c_str());
        break;
      case CPIT_int32:
        query += txn.quote(std::to_string(pkt->m_data.int32[0]).c_str());
        break;
      case CPIT_uint32:
        query += txn.quote(std::to_string(pkt->m_data.uint32[0]).c_str());
        break;
      case CPIT_uint32_2:
        query += txn.quote(std::to_string(pkt->m_data.int32[0])  + " " + std::to_string(pkt->m_data.int32[1]));
        break;
      case CPIT_float32:
        query += txn.quote(std::to_string(pkt->m_data.float32[0]).c_str());
        break;
      case CPIT_float32_2:
        query += txn.quote(std::to_string(pkt->m_data.float32[0])  + " " + std::to_string(pkt->m_data.float32[1]));
        break;
    }

    query += ")";


    return true;
  }

  //! Log an emergency stop
  bool PGDataRecorderC::LogEmergencyStop(pqxx::work &txn,std::string &query,PGLogEntryC &entry)
  {
    const PacketEmergencyStopC *pkt = (const PacketEmergencyStopC *) entry.m_data;

    if(query.empty()) {
      query.reserve(4096);
      query += "INSERT INTO emergency_stop (sourceid,logtime,cause) VALUES ";
    } else
      query += ",";

    query += "(";
    query += txn.quote(DeviceName(pkt->m_deviceId));
    query += ",";
    query += txn.quote(AsISO8601(entry.m_when));
    query += ",";
    query += txn.quote(ComsStateChangeSource((enum StateChangeSourceT) pkt->m_cause));
    query += ")";

    return true;
  }

  //! Log an error
  bool PGDataRecorderC::LogError(pqxx::work &txn,std::string &query,PGLogEntryC &entry)
  {
    const PacketErrorC *pkt = (const PacketErrorC *) entry.m_data;

    if(query.empty()) {
      query.reserve(4096);
      query += "INSERT INTO error (sourceid,logtime,name,cause,data) VALUES ";
    } else
      query += ",";

    query += "(";
    query += txn.quote(DeviceName(pkt->m_deviceId));
    query += ",";
    query += txn.quote(AsISO8601(entry.m_when));
    query += ",";
    query += txn.quote(ComsErrorTypeToString((enum ComsErrorTypeT) pkt->m_errorCode));
    query += ",";
    std::string causeType = std::to_string((int) pkt->m_causeType);
    query += txn.quote(causeType);
    query += ",";
    std::string errorData = std::to_string((int) pkt->m_errorData);
    query += txn.quote(errorData);
    query += ")";

    return true;
  }

  //! Issue a query
  bool PGDataRecorderC::IssueQuery(pqxx::work &txn,std::string &query)
  {
    if(query.empty())
      return true;
    query += ';';
    try {
      txn.exec(query,"Data logging");
    } catch(pqxx::syntax_error &exErr) {
      m_log->error("Caught exception '{}' executing: '{}' ",exErr.what(),query);
      return false;
    }
    return true;
  }


  //! Write worker thread
  void PGDataRecorderC::WriteThread()
  {
    std::chrono::steady_clock::time_point nextUpdate = std::chrono::steady_clock::now();

    while(!m_terminated) {

      std::this_thread::sleep_until(nextUpdate);
      nextUpdate += std::chrono::milliseconds(50);

      {
        m_recordQueueWrite.clear();
        std::lock_guard<std::mutex> lock(m_accessRecordQueue);
        std::swap(m_recordQueue,m_recordQueueWrite);
      }

      pqxx::work txn(m_connection,"Insert Log Data");

      std::vector<std::string> queries((unsigned) CPT_Final);

      int msgCount = 0;

      for(auto &a : m_recordQueueWrite) {

        enum ComsPacketTypeT packetType = (enum ComsPacketTypeT) a.m_data[0];
        int deviceId = a.m_data[1];

        switch(packetType)
        {
          case CPT_NoOp: break;
          case CPT_EmergencyStop: {
            if(LogEmergencyStop(txn,queries[packetType],a))
              msgCount++;
          } break;
          case CPT_ServoReport: {
            if(LogServoReport(txn,queries[packetType],a))
              msgCount++;
          } break;
          case CPT_Servo: {
            if(LogServo(txn,queries[packetType],a))
              msgCount++;
          } break;
          case CPT_ReportParam: {
            if(LogParamReport(txn,queries[packetType],a))
              msgCount++;
          } break;
          case CPT_Error: {
            if(LogError(txn,queries[packetType],a))
              msgCount++;
          } break;
          case CPT_ReadParam: {
            // Just drop these.
          } break;
          default:
            m_log->warn("Message type {} not handled. ",(int) packetType);
            break;
#if 0
          CPT_SyncTime      =  2, // Sync time across controllers.
          CPT_SetParam      =  4, // Set parameter
          CPT_ReadParam     =  8, // Read parameter
          CPT_Pong          =  9, // Ping reply.
          CPT_Ping          = 10, // Ping request
          CPT_AnnounceId    = 11, // Query connected devices
          CPT_QueryDevices  = 12, // Query connected devices
          CPT_SetDeviceId   = 13, // Set device id
          CPT_SaveSetup     = 14, // Save setup to eeprom
          CPT_LoadSetup     = 15, // Load setup from eeprom
          CPT_CalZero       = 16, // Set current position as calibrated zero.
          CPT_Sync          = 17, // Sync data stream
          CPT_PWMState      = 18, // PWM State. Packet holding internal controller data.
          CPT_BridgeMode    = 19, // Enable bridge mode
          CPT_FlashCmdReset   = 20, // Status from a flash command
          CPT_FlashCmdResult  = 21, // Status from a flash command
          CPT_FlashChecksumResult = 22, // Generate a checksum
          CPT_FlashEraseSector = 23, // Erase a flash sector
          CPT_FlashChecksum    = 24, // Generate a checksum
          CPT_FlashData        = 25, // Data packet
          CPT_FlashWrite       = 26, // Write buffer
          CPT_FlashRead        = 27, // Read buffer and send it back
          CPT_Final                  // Use to get count of known packet types.
#endif
        }

      }

      for(auto &query : queries)
        IssueQuery(txn,query);

      txn.commit();
      if(m_recordQueueWrite.size() > 0) {
        m_log->info(" {} of {} messages written to log. ",msgCount,m_recordQueueWrite.size());
        msgCount++;
      }


    }
  }


  //! Start recording
  void PGDataRecorderC::Start()
  {
    if(m_genericHandlerId.IsActive())
      return ; // Already started

    // Initialise array of servo names
    auto servoList = m_api->ListServos();

    {
      std::string query = "INSERT INTO source (sourceid,devicetype,serialnumber,notes) VALUES ";
      pqxx::work txn(m_connection,"Insert Devices");

      query.reserve(4096);
      bool isFirst = true;
      for(auto &a : servoList) {
        if(!a)
          continue;
        assert(a->Id() < 256);
        while(m_deviceNames.size() <= a->Id())
          m_deviceNames.push_back(std::string());
        m_deviceNames[a->Id()] = a->Name();
        m_log->info("Found joint {} ",a->Name());
        if(isFirst) {
          isFirst =  false;
        } else {
          query += ',';
        }
        query += '(';
        query += txn.quote(a->Name());
        query += ',';
        query += txn.quote(a->JointType());
        query += ',';
        query += txn.quote(a->SerialNumber());
        query += ',';
        query += txn.quote(a->Notes());
        query += ')';
      }
      query += " ON CONFLICT DO NOTHING; ";

      txn.exec(query,"Setup sources");

      txn.commit();
    }

    m_writeThread = std::thread([this](){ WriteThread(); });

    m_genericHandlerId = m_coms->SetGenericHandler([this](const uint8_t *data,int len) mutable
                            {
                              RecordPacket(false,data,len);
                            }
    );


    m_sendHandler = m_coms->SetCommandHandler([this](const uint8_t *data,int len) mutable
                            {
                              RecordPacket(true,data,len);
                            }
    );

  }


  //! Stop recording.
  void PGDataRecorderC::Stop()
  {
    m_genericHandlerId.Remove();
    m_sendHandler.Remove();
  }


  void PGDataRecorderC::RecordPacket(bool isCommand,const uint8_t *data,int len)
  {
    std::lock_guard<std::mutex> lock(m_accessRecordQueue);
    m_recordQueue.push_back(PGLogEntryC(isCommand,data,len));
  }

}
