#ifndef DOGBOG_DATARECORDER_HEADER
#define DOGBOG_DATARECORDER_HEADER 1

#include "dogbot/DogBotAPI.hh"
#include <pqxx/pqxx>
#include <chrono>
#include <sys/time.h>

namespace DogBotN {



  //! Manage logging parameter data to postgres so it can be viewed by analytic packages such as grafana

  class PGDataRecorderC
  {
  public:
    PGDataRecorderC(const std::shared_ptr<DogBotAPIC> &api,const std::string &databaseConnectionString);

    //! Destructor, this will wait for the write thread to complete
    //! its current write and exit.
    ~PGDataRecorderC();

    //! Create schema in database.
    void CreateSchema();

    //! Start recording
    void Start();

    //! Stop recording
    void Stop();

  protected:

    //! Find the name for a device
    std::string DeviceName(int deviceId);

    class PGLogEntryC
    {
    public:
      PGLogEntryC(bool isCommand,const uint8_t *data,int len);

      struct timeval m_when;
      bool m_isCommand = false;
      int m_len = 0;
      uint8_t m_data[16];
    };

    void RecordPacket(bool isCommand,const uint8_t *data,int len);

    //! Write worker thread
    void WriteThread();

    //! Log a servo report message.
    bool LogServoReport(pqxx::work &txn,std::string &query,PGLogEntryC &entry);

    //! Log a servo demand message.
    bool LogServo(pqxx::work &txn,std::string &query,PGLogEntryC &entry);

    //! Log an emergency stop
    bool LogEmergencyStop(pqxx::work &txn,std::string &query,PGLogEntryC &entry);

    //! Log a parameter report message
    bool LogParamReport(pqxx::work &txn,std::string &query,PGLogEntryC &entry);

    //! Log an error
    bool LogError(pqxx::work &txn,std::string &query,PGLogEntryC &entry);

    //! Issue a query
    bool IssueQuery(pqxx::work &txn,std::string &query);

    std::mutex m_accessDeviceData;
    std::vector<std::string> m_deviceNames;

    std::shared_ptr<spdlog::logger> m_log = spdlog::get("console");

    std::shared_ptr<DogBotAPIC> m_api;
    std::shared_ptr<ComsC> m_coms;

    std::string m_databaseConnectionString;
    pqxx::connection m_connection;

    CallbackHandleC m_genericHandlerId;
    CallbackHandleC m_sendHandler;

    std::mutex m_accessRecordQueue;
    std::vector<PGLogEntryC> m_recordQueue;
    std::vector<PGLogEntryC> m_recordQueueWrite;

    std::thread m_writeThread;
    bool m_terminated = false;
  };

}


#endif
