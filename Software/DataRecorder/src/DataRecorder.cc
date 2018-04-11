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
    pqxx::connection dbcon(m_databaseConnectionString);


    pqxx::work txn(m_connection,"Create schema");


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

    if(query.empty())
      query += "INSERT INTO dogbot1.joint_report (sourceid,logtime,synctime,position,effort) VALUES ";
    else
      query += ",";

    query += "(";
    query += txn.quote(m_deviceNames[pkt->m_deviceId]);
    query += ",";
    query += txn.quote(AsISO8601(a.m_when));
    query += ",";
    query += txn.quote((unsigned) pkt->m_timestamp);
    query += ",";
    query += txn.quote(ComsC::PositionReport2Angle(pkt->m_position));
    query += ",";
    query += txn.quote(ComsC::TorqueReport2Fraction(pkt->m_torque));
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
    if(query.empty())
      query += "INSERT INTO dogbot1.joint_demand (sourceid,logtime,position,effort_limit) VALUES ";
    else
      query += ",";

    query += "(";
    query += txn.quote(m_deviceNames[pkt->m_deviceId]);
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

    if(query.empty())
      query += "INSERT INTO dogbot1.parameter_report (sourceid,logtime,parameter,value) VALUES ";
    else
      query += ",";

    enum ComsParameterIndexT paramIndex =  (enum ComsParameterIndexT) pkt->m_header.m_index;
    query += "(";
    query += txn.quote(m_deviceNames[pkt->m_header.m_deviceId]);
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
        break;
      case CPIT_float32:
        query += txn.quote(std::to_string(pkt->m_data.float32[0]).c_str());
        break;
    }

    query += ")";


    return true;
  }

  //! Write worker thread
  void PGDataRecorderC::WriteThread()
  {
    while(!m_terminated) {
      std::vector<PGLogEntryC> batch;

      {
        m_recordQueueWrite.clear();
        std::lock_guard<std::mutex> lock(m_accessRecordQueue);
        std::swap(m_recordQueue,m_recordQueueWrite);
      }

      pqxx::work txn(m_connection,"Insert Log Data");

      std::string query;
      std::string reportQuery;
      std::string demandQuery;
      reportQuery.reserve(2048);

      int msgCount = 0;

      for(auto &a : m_recordQueueWrite) {

        enum ComsPacketTypeT packetType = (enum ComsPacketTypeT) a.m_data[0];
        int deviceId = a.m_data[1];

        //pqxx::result result = txn.exec(query);


        switch(packetType)
        {
          case CPT_NoOp: break;
          case CPT_EmergencyStop: {
          } break;
          case CPT_ServoReport: {
            if(LogServoReport(txn,reportQuery,a))
              msgCount++;
          } break;
          case CPT_Servo: {
            if(LogServo(txn,demandQuery,a))
              msgCount++;
          } break;
          case CPT_ReportParam: {
            if(LogParamReport(txn,demandQuery,a))
              msgCount++;
          } break;

#if 0
          CPT_NoOp    =  0, // No op.
          CPT_EmergencyStop =  1, // Set parameter (Used to for emergency stop)
          CPT_SyncTime      =  2, // Sync time across controllers.
          CPT_Error         =  3, // Error report
          CPT_SetParam      =  4, // Set parameter
          CPT_Servo         =  5, // Servo control position
          CPT_ServoReport   =  6, // Report servo position
          CPT_ReportParam   =  7, // Report parameter
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

      if(!reportQuery.empty()) {
        reportQuery += ';';
        txn.exec(reportQuery,"Report data");
      }
      if(!demandQuery.empty()) {
        reportQuery += ';';
        txn.exec(demandQuery,"Demand data");
      }

      txn.commit();
      if(m_recordQueueWrite.size() > 0) {
        m_log->info(" {} of {} messages written to log. ",msgCount,m_recordQueueWrite.size());
        msgCount++;
      }


    }
  }

  //! Add name of servo with given id to cache.
  void PGDataRecorderC::AddServoName(int id)
  {
    assert(id < 256);
    auto servoPtr = m_api->GetServoById(id);
    if(!servoPtr)
      return ;
    while(m_deviceNames.size() <= id)
      m_deviceNames.push_back("");
    m_deviceNames[id] = servoPtr->Name();
  }


  //! Start recording
  void PGDataRecorderC::Start()
  {
    if(m_genericHandlerId.IsActive())
      return ; // Already started

    // Initialise array of servo names
    auto servoList = m_api->ListServos();
    for(auto &a : servoList) {
      if(!a)
        continue;
      assert(a->Id() < 256);
      while(m_deviceNames.size() <= a->Id())
        m_deviceNames.push_back("");
      m_deviceNames[a->Id()] = a->Name();
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
