/*

 * DroneScenario5g.cc
 *
 *  Created on: April 22, 2023
 *      Author: iotlab_aime
*/
#include "DroneScenario5g.h"
#include <string>
#include "DroneNetMob.h"
#include "stationaryNodeMob.h"

#define DynamicModuleDebug

namespace inet{
std::vector<Coord> srcPosition;
Define_Module(DroneScenario5g);

DroneScenario5g::DroneScenario5g(){
    std::cout <<"Drone scenario running!" <<std::endl;
};
DroneScenario5g::~DroneScenario5g(){
    cancelAndDelete(executeOneTimestepTrigger);

};
void accessdestinations(){

}
void DroneScenario5g::initialize(int stage){
    cSimpleModule::initialize(stage);
//    std::cout <<"DroneScenario5g initialize called!  stage = " <<stage << std::endl;
    firstStepAt = par("firstStepAt");
    penetrationRate = par("penetrationrate").doubleValue();
    std::cout<<"=====================================" <<std::endl;
    std::cout <<"penetrationRate = " <<penetrationRate<<std::endl;
    numdrones = par("dronesnumber").intValue();
    penetrationTime = 1/penetrationRate;
    updateInterval = par("updateInterval");
    if (firstStepAt == -1) firstStepAt = updateInterval;
    std::string host;
    vehicleNameCounter = 0;
    nextNodeVectorIndex = 0;
    activeDroneCount = 0;
    std::cout <<"~~~~> updateInterval = " <<updateInterval <<"; number of drones = " <<numdrones
            <<"penetrationRate = " <<penetrationRate <<"; penetrationTime = " <<penetrationTime <<std::endl;

    hosts.clear();
    executeOneTimestepTrigger = new cMessage("step");
    scheduleAt(penetrationTime, executeOneTimestepTrigger);
}

void DroneScenario5g::handleMessage(cMessage *msg) {
    std::cout <<"void DroneScenario5g::handleMessage(cMessage *msg) " <<std::endl;
    if (msg->isSelfMessage()) {
        handleSelfMsg(msg);
        return;
    }
    error("DroneScenario5g doesn't handle messages from other modules");
}
void DroneScenario5g::handleSelfMsg(cMessage *msg) {
    std::cout <<"----->  void DroneScenario5g::handleSelfMsg(cMessage *msg) " << msg->getName() << std::endl;

    if (msg == executeOneTimestepTrigger) {
        simpleModuleAdd();
        if (activeDroneCount <= numdrones){
            scheduleAt(simTime() + penetrationTime, executeOneTimestepTrigger);
        }
        return;
    }
    error("DroneScenario5g received unknown self-message");
}

void DroneScenario5g::finish(){

}

void DroneScenario5g::simpleModuleAdd(){
    std::string moduleType = par("moduleType").stdstringValue();
    std::string moduleName = par("moduleName").stdstringValue();
    std::string moduleDisplayStrings = par("moduleDisplayString").stdstringValue();
    cModuleType* nodeType = cModuleType::get(moduleType.c_str());
    int32_t nodeVectorIndex = nextNodeVectorIndex++;
    cModule* parentmod = getParentModule();
  //        std::cout <<"Parent ------>>>>  --->>> " <<parentmod->getFullName()<<std::endl;
    cModule *module = nodeType->create(moduleName.c_str(), parentmod, nodeVectorIndex);

    module->finalizeParameters();
    module->buildInside();
    module->getDisplayString().parse(moduleDisplayStrings.c_str());


    // create activation message
    module->scheduleStart(simTime());
    // initialize mobility for positioning
    cModule *mobiSubmodule = module->getSubmodule("mobility");
    mobiSubmodule->par("initFromDisplayString") = false;
    std::cout<<"-->DroneScenario5g: "<< mobiSubmodule->getFullName() <<"typeName = " << mobiSubmodule->getNedTypeName()<<std::endl;

    int lc = nodeVectorIndex % 10; // Access the last digit of the node index
    double xo = 0, yo = 0, zo = 0;
    Coord InP;
    if (lc < 3){
        xo = 300, yo = 20, zo = 0;
    }
    else if (lc < 6){
        xo = 20, yo = 200, zo = 0;
    }
    else if (lc < 8){
        xo = 580, yo = 190, zo = 0;
    }
    else{
        xo = 300, yo = 380, zo = 0;
        InP = srcPosition[3];
    }
    mobiSubmodule->par("initialX") = xo;
    mobiSubmodule->par("initialY") = yo;
    mobiSubmodule->par("initialZ") = zo;
    mobiSubmodule->par("speed") = 12.0;
    mobiSubmodule->par("changeInterval") = 0.5;
    mobiSubmodule->par("angleDelta") = 10.0;
    mobiSubmodule->par("ndst")  = 6;
    module->callInitialize();
    activeDroneCount++;

#ifdef  DynamicModuleDebug
    std::cout<< mobiSubmodule->getFullName() <<"typeName = " << mobiSubmodule->getNedTypeName()<<std::endl;
    std::cout <<"Initial Position: (" <<xo <<"; " << yo <<"; " <<zo <<")" <<std::endl;
#endif
}

}

