

#include "dogbot/SplineGaitController.hh"

int testSplineGate() {
  DogBotN::SplineGaitControllerC gait;
  gait.PlotGait();
  return 0;
}


int main(int nargs,char **argv)
{
  testSplineGate();
  return 0;
}
