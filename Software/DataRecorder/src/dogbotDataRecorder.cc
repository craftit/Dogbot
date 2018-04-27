

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>

#include "dogbot/DogBotAPI.hh"
#include "dogbot/Util.hh"
#include "dogbot/DataRecorder.hh"
#include "cxxopts.hpp"
#include <signal.h>

bool g_terminate = false;

void SignalExit(int sig) {
  g_terminate = true;
}

// This provides a network interface for controlling the servos via ZMQ.

int main(int argc,char **argv)
{
  std::string devFilename = "local";
  std::string configFile = DogBotN::DogBotAPIC::DefaultConfigFile();
  std::string jointName = "front_right_knee";
  std::string firmwareFile;
  std::string loadPoseFile;

  auto logger = spdlog::stdout_logger_mt("console");
  bool dumpPose = false;
  bool cycle = false;
  bool serverMode = true;
  float torque = 3.0;
  float range = 45.0;
  float angle = 0;
  float velocityLimit = 300.0;
  try
  {
    cxxopts::Options options(argv[0], "DogBot hardware manager");
    options
      .positional_help("[optional args]")
      .show_positional_help();

    options.add_options()
      ("c,config", "Configuration file", cxxopts::value<std::string>(configFile))
      ("d,device", "Device to use from communication. Typically 'local' for local server or 'usb' for direct connection ", cxxopts::value<std::string>(devFilename))
      ("s,signal", "Catch signals, disable for debug ", cxxopts::value<bool>(serverMode))
      ("h,help", "Print help")
    ;

    auto result = options.parse(argc, argv);

    if (result.count("help"))
    {
      std::cout << options.help({""}) << std::endl;
      exit(0);
    }

  } catch (const cxxopts::OptionException& e)
  {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }

  if(serverMode) {
    signal(SIGINT,&SignalExit);
    signal(SIGTERM,&SignalExit);
    signal(SIGHUP,&SignalExit);
  }


  logger->info("Logging messages. ");
  logger->info("Using config file: '{}'",configFile);
  logger->info("Using communication type: '{}'",devFilename);

  std::shared_ptr<DogBotN::DogBotAPIC> dogbotAPI = std::make_shared<DogBotN::DogBotAPIC>(
      devFilename,
      configFile,
      logger
      );


  DogBotN::PGDataRecorderC recorder(dogbotAPI,"dbname=dogbot user=reactai password=letmein");

  sleep(1); // Leg initial coms take place.

  recorder.Start();

  while(!g_terminate) {
    sleep(1);
  }

  recorder.Stop();


  return 0;
}
