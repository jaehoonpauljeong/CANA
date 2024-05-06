/*
 * DroneNetMob.cc
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */

#include "inet/common/INETMath.h"
#include "DroneNetMob.h"
#include "stationaryNodeMob.h"
#include <fstream>
#include "tools.h"
#include <random>

//#define DEBUG_DRONEMOBILITY

namespace inet {

bool flag_original = false;
Coord originPos;
std::vector<Coord> dst; //Destination Positions
int gen = 0;
int nparcels = 5000;
bool flagArrangedDepot = false;
bool OngoingMission = false;
std::map<std::string, Coord> allnodes;
std::vector<parcel> parcel_depot;
std::vector<double> Lz = {50.0, 100.0, 150.0};
std::map <std::string, std::vector<std::string>> AvDrones; //Drones available for further mission assignment. <Mission Base, Drones>
std::map <std::string, Coord> homePos; //Base stations and their positions
std::map <std::string, droneInf> alldronesInfo;
bool flagDestAssigned = false;

/*std::vector<double> Probs;
std::vector<lanequality> laneq;
bool obset = false;
double obsp = 0;
Coord obp;*/




Define_Module(DroneNetMob);
bool sortDepotByDeadline (parcel i, parcel j) {
    return (i.exp_time < j.exp_time);
}
bool sortDepotByDestination (parcel i, parcel j) {
    return ((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
                 +pow(i.parceldest.z - originPos.z, 2))) < (sqrt(pow(j.parceldest.x - originPos.x, 2)
                    + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2))));
}
bool greedySortDepot (parcel i, parcel j) {
    return (((sqrt(pow(i.parceldest.x - originPos.x, 2) + pow(i.parceldest.y - originPos.y, 2)
            +pow(i.parceldest.z - originPos.z, 2)))/i.weight) < ((sqrt(pow(j.parceldest.x - originPos.x, 2)
               + pow(j.parceldest.y - originPos.y, 2) + pow(j.parceldest.z - originPos.z, 2)))/j.weight));
}
bool SortDepotByWeight (parcel i, parcel j) {
    return (i.weight > j.weight);
}
bool sortDesByShP (Coord i, Coord j) {
    return ((sqrt(pow(i.x - originPos.x, 2) + pow(i.y - originPos.y, 2) + pow(i.z - originPos.z, 2))) <
            (sqrt(pow(j.x - originPos.x, 2) + pow(j.y - originPos.y, 2) + pow(j.z - originPos.z, 2))));
}
DroneNetMob::DroneNetMob()
{
}

void DroneNetMob::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);
//    std::cout << "initializing DroneMobility stage " << stage << endl;

//    EV_TRACE << "initializing DroneMobility stage " << stage << endl;
    if (stage == INITSTAGE_LOCAL) {
        rad heading = deg(par("initialMovementHeading"));
        rad elevation = deg(par("initialMovementElevation"));
        changeIntervalParameter = &par("changeInterval");
        angleDeltaParameter = &par("angleDelta");
        rotationAxisAngleParameter = &par("rotationAxisAngle");
        speedParameter = &par("speed");
        spDevParameter = &par ("spDeviation");
        posError = &par ("positionError");
        quaternion = Quaternion(EulerAngles(heading, -elevation, rad(0)));
        WATCH(quaternion);
        numdst = &par("ndst");
        ox =&par("initialX");
        oy = &par("initialY");
        oz = &par("initialZ");
        mBasePos.NodePos.x = ox->doubleValue();
        mBasePos.NodePos.y = oy->doubleValue();
        mBasePos.NodePos.z = oz->doubleValue();
        originPos = mBasePos.NodePos;
        L = rand() % 3; //Considering 3 layers;

        double sig = posError->doubleValue();
        unsigned seed =100000000;

        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(0.0,sig);
        perr =distribution(generator);
//        std::cout<<"Sig = " <<sig << "---------------->  Err = " << perr <<std::endl;


        droneweightcapacity =  (&par("weightCapacity"))->doubleValue();
        droneremainingbattery =  (&par("remainingBattery"))->doubleValue();
        selectionMethod = (&par("parcelSelecitionMethod"))->intValue();
/*        std::cout <<" droneweightcapacity  = " <<droneweightcapacity <<"; droneremainingbattery = " << droneremainingbattery << ", and selectionMethod = " <<selectionMethod <<std::endl;*/
        std::string bs = (&par("mbase"))->stdstringValue();
        std::map <std::string, std::vector<std::string>>::iterator it;
        it = AvDrones.find(bs);
        maxT2CThresh = par("maxt2cThresh").doubleValue();
        minT2CThresh = par("mint2cThresh").doubleValue();

        int num = par("nd").intValue();
        sp = speedParameter->doubleValue() - spDevParameter->doubleValue();
        n = std::to_string(num);
//        TrigAccident = new cMessage("Trigger Accident");

/*        Defines the base station to which each drone belongs. */
        if (it != AvDrones.end()){
            it->second.push_back(getParentModule()->getFullName());
        }
        else{
            std::vector<std::string> tmp;
            tmp.push_back(getParentModule()->getFullName());
            AvDrones.insert(std::make_pair(bs,tmp));
            homePos.insert(std::make_pair(bs, originPos));
        }
        std::string dn = getParentModule()->getName();
        if (dn.find( "drone") != std::string::npos){
            std::map<double, Coord> pos;
            pos.insert(std::make_pair(simTime().dbl(),originPos));
            follTraj.push_back(pos);
            droneInf d;
            d.dn = getParentModule()->getFullName();
            d.dpos = originPos;
            d.orp = originPos;
            d.fLane = L;
            d.sp = (speedParameter->doubleValue() - spDevParameter->doubleValue());
            d.vel = getCurrentVelocity();
            alldronesInfo.insert(std::make_pair(getParentModule()->getFullName(), d));
        }
        if (std::strcmp(getParentModule()->getFullName(), "drone1[0]") == 0){
            if (numdst->intValue() > 0){
                ndst = numdst->intValue();
                destGen();
            }
            stationaryNodeMob* pt;
            std::map<std::string, Coord> src= pt->returnBStations();
            for (auto i:src){
                allnodes.insert(std::make_pair(i.first, i.second));
            }
            buildgraphnodes(allnodes);

            parcelsDefinition(nparcels);
        }
        targetPosition = missionPathNextDest_n(mBasePos.NodePos);
        myinfo.dn =  getParentModule()->getFullName(), myinfo.orp = originPos, myinfo.dpos = targetPosition;
        myinfo.sp = speedParameter->doubleValue() - spDevParameter->doubleValue();

#ifdef DEBUG_DRONEMOBILITY
        std::cout << " Name -----> " << getParentModule()->getFullName() <<
                " And Src = (" << mBasePos.NodePos.x <<"; "<<mBasePos.NodePos.y <<"; " <<mBasePos.NodePos.z <<")" << std::endl;
        destination = missionPathNextDest(originPos);
        std::cout << " Name -----> " << getParentModule()->getFullName() <<
                " And Speed = " << speedParameter->doubleValue() << std::endl;
        std::cout << " Ndrones -----> " <<(&par("nd"))->intValue() << std::endl;

        stationaryNodeMob* pt;
        std::map<std::string, Coord> dts;
        dts = pt->returnDestinations();
        std::cout << "DST: " <<std::endl;
        for (auto i:dts){
            std::cout <<i.first<<"(" <<i.second.x <<"; " <<i.second.y <<"; " <<i.second.z <<") ";
        }
#endif

    }
    else if (stage == INITSTAGE_LAST){
        std::string name = getParentModule()->getName();
        if (name.find("drone") != std::string::npos){
            myCurrentMission = MissionPathDefinitionFromSource();
        }
    }
}
void DroneNetMob::finish(){
/*
    if(std::strcmp(getParentModule()->getFullName(), "drone1[0]") == 0){
//        std::cout <<" Mob finish !!!!   " ;
//        std::cout << Probs.size() <<" && " <<laneq.size() <<std::endl;

        ofstream res2;
        std::string FPath2 = "/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/cp.csv";

        res2.open(FPath2);
        for (int j = 0; j <Probs.size(); j++){
            res2<<Probs[j] <<std::endl;
        }
        res2.close();

//        std::cout <<" Lane Q: ";
        ofstream res3;
        std::string FPath3 = "/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/lq.csv";
        res3.open(FPath3);
        for (int i = 0; i <laneq.size(); i++){
            res3 <<laneq[i].rlq <<", "
                    <<laneq[i].elq <<", "
                    <<laneq[i].llq <<", "
                    <<laneq[i].dlq <<", "
                    <<laneq[i].ulq <<std::endl;

        }
        res3.close();
//        std::cout <<" -----  Completed! " <<std::endl;
    }*/
}

void DroneNetMob::orient()
{
    if (faceForward)
        lastOrientation = quaternion;
}

void DroneNetMob::setTargetPosition()
{
//    std::cout <<"void DroneNetMob::setTargetPosition()" << getParentModule()->getFullName() << std::endl;
    if (flag_original){  /*Original setTargetPosition definition from INET mass mobility*/
        rad angleDelta = deg(angleDeltaParameter->doubleValue());
        rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
        Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
        quaternion = quaternion * dQ;
        quaternion.normalize();
        Coord direction = quaternion.rotate(Coord::X_AXIS);

        simtime_t nextChangeInterval = *changeIntervalParameter;
        EV_DEBUG << "interval: " << nextChangeInterval << endl;
        sourcePosition = lastPosition;
        targetPosition = lastPosition + direction * (*speedParameter) * nextChangeInterval.dbl()- direction * (*spDevParameter) * nextChangeInterval.dbl()
                + direction * perr;
        previousChange = simTime();
        nextChange = previousChange + nextChangeInterval;
    }
    else{
        rad angleDelta = deg(angleDeltaParameter->doubleValue());
        rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
        Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
        quaternion = quaternion * dQ;
        quaternion.normalize();
        Coord direction = quaternion.rotate(Coord::X_AXIS);
        if (myWayToDest.size() != 0){
            targetPosition = myWayToDest.front();
            myWayToDest.erase (myWayToDest.begin());
            nextChange = simTime() + changeIntervalParameter->doubleValue();
     /*       std::cout <<" Tag  ======================== 1  ==== (" <<targetPosition.x<<"; " <<targetPosition.y <<
                    "; " <<targetPosition.z <<")"<< std::endl;*/
        }
        else if (myCurrentMission.size() != 0){
            myWayToDest = intervalTimesToDestination(lastPosition,myCurrentMission.front());
            myCurrentMission.erase (myCurrentMission.begin());
            targetPosition = myWayToDest.front();
            targetPosition = targetPosition + direction * perr;
            myWayToDest.erase (myWayToDest.begin());
            nextChange = simTime() + changeIntervalParameter->doubleValue();
     /*       std::cout <<" Tag  ======================== 2  ==== (" <<targetPosition.x<<"; " <<targetPosition.y <<
                                   "; " <<targetPosition.z <<")"<< std::endl;*/

         /*   Ref:
            m distance = m(sqrt(dx * dx + dy * dy + dz * dz));
            auto direction = Quaternion::rotationFromTo(Coord::X_AXIS, Coord(dx.get(), dy.get(), dz.get()));
            auto antennaLocalDirection = startOrientation.inverse() * direction;*/
        }
        else{
            myCurrentMission = MissionPathDefinitionFromSource();
            myWayToDest = intervalTimesToDestination(lastPosition,myCurrentMission.front());
            myCurrentMission.erase (myCurrentMission.begin());
            targetPosition = myWayToDest.front();
            targetPosition = targetPosition + direction * perr;
            myWayToDest.erase (myWayToDest.begin());
            nextChange = simTime() + changeIntervalParameter->doubleValue();
        }
        mycarriedmission.push_back(targetPosition);
        mt.push_back(simTime().dbl());
        std::map <std::string, droneInf>::iterator it;
        it = alldronesInfo.find(getParentModule()->getFullName());
        if (it != alldronesInfo.end()){
            it->second.dpos = targetPosition;
            it->second.vel = getCurrentVelocity();
        }
        myinfo.dpos = targetPosition;
    }
}

void DroneNetMob::move()
{
//    std::cout<<"======= DroneNetMob::move() ========== SIMTIME = "<<simTime() << ":: nextChange = " <<nextChange<<std::endl;
    if (flag_original){ /*Original move definition from INET mass mobility*/
        simtime_t now = simTime();
        rad dummyAngle;
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
        }
    }
    else{ /*Modified move function for drone mobility*/
        simtime_t now = simTime();
        rad dummyAngle;
   /*     if (accidentNode == true && simTime() >30){
            accidentGoingOn = true;
            nextChange = 190;
        }*/
        if (now == nextChange) {
            lastPosition = targetPosition;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            EV_INFO << "reached current target position = " << lastPosition << endl;
            setTargetPosition();
            EV_INFO << "new target position = " << targetPosition << ", next change = " << nextChange << endl;
            lastVelocity = (targetPosition - lastPosition) / (nextChange - simTime()).dbl();
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
//            std::cout <<"Vel = " << lastVelocity.x <<"  ; " << lastVelocity.x << "  ; " << lastVelocity.z << std::endl;
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            double alpha = (now - previousChange) / (nextChange - previousChange);
            lastPosition = sourcePosition * (1 - alpha) + targetPosition * alpha;
            handleIfOutside(REFLECT, targetPosition, lastVelocity, dummyAngle, dummyAngle, quaternion);
            /*           std::cout << " Name -----> " << getParentModule()->getFullName() << " , Velocity = ("
                                        << getCurrentVelocity().x << "; " << getCurrentVelocity().y << "; "<< getCurrentVelocity().z <<"), Position = ("
                                        << getCurrentPosition().x<< "; " << getCurrentPosition().y <<"; "<< getCurrentPosition().z <<")"<< std::endl;*/
            /*
             *  Collect the position Information
             */
            std::map<double, Coord> pos;
            pos.insert(std::make_pair(simTime().dbl(),lastPosition));
            follTraj.push_back(pos);
        }
    }
}
std::vector <Coord> DroneNetMob::intervalTimesToDestination (Coord CurPos, Coord TargPos){
    std::vector <Coord> stepinfo;
    double T2Dest = (sqrt(pow(TargPos.x - CurPos.x, 2) + pow(TargPos.y - CurPos.y, 2)
            +pow(TargPos.z - CurPos.z, 2)))/(speedParameter->doubleValue() -spDevParameter->doubleValue()) ;

    if (T2Dest > 0){
        int numST = std::ceil (T2Dest/changeIntervalParameter->doubleValue());
        for (int i=0; i<numST; i++){
            if ((i+1) == numST){
                stepinfo.push_back(TargPos);
            }
            else{
                rad angleDelta = deg(angleDeltaParameter->doubleValue());
                rad rotationAxisAngle = deg(rotationAxisAngleParameter->doubleValue());
                Quaternion dQ = Quaternion(Coord::X_AXIS, rotationAxisAngle.get()) * Quaternion(Coord::Z_AXIS, angleDelta.get());
                quaternion = quaternion * dQ;
                quaternion.normalize();
                Coord direction = quaternion.rotate(Coord::X_AXIS);

                CurPos = CurPos + direction * (*speedParameter) * changeIntervalParameter->doubleValue() -  direction * (*spDevParameter) * changeIntervalParameter->doubleValue();
                if (i > 0){
                    CurPos.z =   Lz[L];
                }
                stepinfo.push_back(CurPos);
            }
        }
    }
    else{
        stepinfo.push_back(TargPos);
    }
#ifdef DEBUG_DRONEMOBILITY
    std::cout <<" Trajectory : ";
    for (auto i: stepinfo){
        std::cout << "(" <<i.x <<"; " <<i.y <<"; " <<i.z <<")";
    }
    std::cout <<std::endl;
#endif
    return stepinfo;
}
/*std::vector <Coord> DroneNetMob::interDestSet (Coord CurPos, Coord TargPos){

}*/
double DroneNetMob::getMaxSpeed() const
{
    return speedParameter->isExpression() ? NaN : speedParameter->doubleValue();
}
/*This function enqueues the destinations in a way that drones can access while defining their path
 * */
void DroneNetMob::destGen(){
    bool flagprev = false;
    /*Randomly define the destinations
     * */
    if (flagprev){
        for (unsigned int i = 0; i < numdst->intValue(); i++){
            Coord nextdst;
            nextdst.x = rand() % 600;
            nextdst.y = rand() % 400;
            nextdst.z = 0;
            dst.push_back(nextdst);
        }
    }
    /*Through a pointer access the destinations
        * Enqueue  their positions for drone trajectories */
    else{
//        std::cout <<"Real destinations ~~~~~~~~~~" <<std::endl;
        stationaryNodeMob* pt;
        std::map<std::string, Coord> dts;
        dts = pt->returnDestinations();
        for (auto i:dts){
            dst.push_back(i.second);
            allnodes.insert(std::make_pair(i.first, i.second));
        }
        //        std::cout << "DST: " <<std::endl;
        //       std::cout <<" Assigned destinations:" <<std::endl;
        for (auto i:dst){
            std::cout <<"(" <<i.x <<"; " <<i.y <<"; " <<i.z <<") :: ";
        }
        std::cout<<std::endl;
    }
}
void DroneNetMob::parcelsDefinition (int nparcels){
    for (unsigned int i = 0; i < nparcels; i++){
        parcel tmpparcel;
        parcel *p;
        tmpparcel.parcelID = i;
        tmpparcel.weight =  rand() % 10 + 1;
        tmpparcel.priority = 1;
        tmpparcel.exp_time = rand() % 300;
        int n = dst.size();
        int dindex = rand() % n;
        tmpparcel.parceldest = dst[dindex];
        parcel_depot.push_back(tmpparcel);
    }
}

std::vector<parcel> DroneNetMob::droneParcelsSelectionFromSource(int parcelSel){
    std::vector<parcel> selectedParcels;
    double packedweight = 0;

    switch(parcelSel){
        /* Closest-Deadline-Parcel-First
         * Depot sorted by Deadline
         * First deliver the ones with small deadline*/
        case CDPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDeadline);

                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                flagArrangedDepot = true;
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Closest-Neighbor-Parcel-First
         * Depot sorted by Positions
         * First deliver the ones closer to source*/
        case CNPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),sortDepotByDestination);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                flagArrangedDepot = true;
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Efficient Parcel Delivery Service
         * Depot sorted in a greedy way ()
         * First deliver the ones with small ratio distance/weight*/
        case EPDS:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),greedySortDepot);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
                flagArrangedDepot = true;
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        /* Randomly-Selected-Parcel-First
         * Randomly select Parcels to be delivered first*/
        case RSPF:{
            if (!flagArrangedDepot){

            }
            else{

            }
            break;
        }
        /* Heaviest Parcel First
         * Depot is sorted based on parcel weight
         * First deliver the heaviest ones to the lightest*/
        case HPF:{
            if (!flagArrangedDepot){
                std::sort (parcel_depot.begin(), parcel_depot.end(),SortDepotByWeight);
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            else{
                int k=0;
                for (unsigned int i = 0; i < parcel_depot.size(); i++){
                    packedweight+=parcel_depot[i].weight;
                    if (packedweight < droneweightcapacity){
                        selectedParcels.push_back(parcel_depot[i]);
                        k++;
                    }
                    else{
                        break;
                    }
                }
                parcel_depot.erase(parcel_depot.begin(), parcel_depot.begin()+k);
            }
            break;
        }
        default:{
/*            std::cout <<" Undefined Selection Method." <<std::endl;*/
        }
    }
    return selectedParcels;
}
Coord DroneNetMob::missionPathNextDest(Coord cpos){
    Coord nextdest, dest;
    double Xerr = -5 + rand() % 10 + 1;
    double Yerr = -5 + rand() % 10 + 1;
    if (!OngoingMission){
        MissionParcels  = droneParcelsSelectionFromSource(selectionMethod);

        double nearestDist = 0;
        int k = 0; //Next Parcel Index
        for (unsigned int i = 0; i < MissionParcels.size(); i++){
            if (i == 0){
                double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                    + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                    +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                nextdest = MissionParcels[i].parceldest;
                nearestDist = tmpd;
                k = i;
            }
            else{
                double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                    + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                    +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                if (tmpd < nearestDist){
                    nextdest = MissionParcels[i].parceldest;
                    nearestDist = tmpd;
                    k = i;
                }
            }
        }
        MissionParcels.erase(MissionParcels.begin()+k);


        OngoingMission = true;
    }
    else{
        if (MissionParcels.size() == 0){
//            nextdest = originPos;
//            std::cout << "::  originPos  ::  ("<<originPos.x <<"; " <<originPos.y <<"; " <<originPos.z <<")" <<std::endl;
            nextdest.x = 0, nextdest.y = 0, nextdest.z = 0;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index
            for (unsigned int i = 0; i < MissionParcels.size(); i++){
                if (i == 0){
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    nextdest = MissionParcels[i].parceldest;
                    nearestDist = tmpd;
                    k = i;
                }
                else{
                    double tmpd = sqrt(pow(MissionParcels[i].parceldest.x - cpos.x, 2)
                                        + pow(MissionParcels[i].parceldest.y - cpos.y, 2)
                                        +pow(MissionParcels[i].parceldest.z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionParcels[i].parceldest;
                        nearestDist = tmpd;
                        k = i;
                    }
                }
            }
            MissionParcels.erase(MissionParcels.begin()+k);
        }
    }
    /*
     * std::cout <<" Destination --- > (" <<nextdest.x <<"; " <<nextdest.y <<"; " <<nextdest.z << ")"
             <<" Origin ---> (" <<originPos.x <<"; "<<originPos.y << "; " << originPos.z<< ")" << std::endl;
     */
    dest.x = nextdest.x + Xerr;
    dest.y = nextdest.y + Yerr;
    dest.z = nextdest.z;
    return dest;
}

Coord DroneNetMob::missionPathNextDest_n(Coord cpos){
    Coord nextdest, dest;
    double Xerr = -5 + rand() % 10 + 1;
    double Yerr = -5 + rand() % 10 + 1;
    if (!OngoingMission){
        MissionPath  = droneMissionDestinationSelection();

        double nearestDist = 0;
        int k = 0; //Next Parcel Index
        int n =  MissionPath.size() -1;
        for (unsigned int i = 0; i < MissionPath.size(); i++){
            if (i == 0){
                double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                    + pow(MissionPath[i].y - cpos.y, 2)
                                    +pow(MissionPath[i].z - cpos.z, 2));
                nextdest = MissionPath[i];
                nearestDist = tmpd;
                k = i;
            }
            else if(MissionPath.size() == 2){
                double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                    + pow(MissionPath[i].y - cpos.y, 2)
                                    +pow(MissionPath[i].z - cpos.z, 2));
                if (tmpd < nearestDist){
                    nextdest = MissionPath[i];
                    nearestDist = tmpd;
                    k = i;
                }
            }
            else{
                double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                    + pow(MissionPath[i].y - cpos.y, 2)
                                    +pow(MissionPath[i].z - cpos.z, 2));
                if (tmpd < nearestDist){
                    nextdest = MissionPath[i];
                    nextdest.z = Lz[L];
                    nearestDist = tmpd;
                    k = i;
                }
            }
        }

        MissionPath.erase(MissionPath.begin()+k);

        OngoingMission = true;
    }
    else{
        if (MissionPath.size() == 0){
            nextdest = mBasePos.NodePos;
            OngoingMission = false;
        }
        else{
            double nearestDist = 0;
            int k = 0; //Next Parcel Index
            for (unsigned int i = 0; i < MissionPath.size(); i++){
                if (i == 0){
                    double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                        + pow(MissionPath[i].y - cpos.y, 2)
                                        +pow(MissionPath[i].z - cpos.z, 2));
                    nextdest = MissionPath[i];
                    nearestDist = tmpd;
                    k = i;
                }
                else if (MissionPath.size() == 2){
                    double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                        + pow(MissionPath[i].y - cpos.y, 2)
                                        +pow(MissionPath[i].z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionPath[i];
                        nearestDist = tmpd;
                        k = i;
                    }
                }
                else{
                    double tmpd = sqrt(pow(MissionPath[i].x - cpos.x, 2)
                                        + pow(MissionPath[i].y - cpos.y, 2)
                                        +pow(MissionPath[i].z - cpos.z, 2));
                    if (tmpd < nearestDist){
                        nextdest = MissionPath[i];
                        nextdest.z = Lz[L];
                        nearestDist = tmpd;
                        k = i;
                    }
                }
            }
            MissionPath.erase(MissionPath.begin()+k);
        }
    }
    dest.x = nextdest.x + Xerr;
    dest.y = nextdest.y + Yerr;
    dest.z = nextdest.z;

    return dest;
}

/*Algorithm for destination selection based on:
 * Shortest path and drone capacity
 * */
/*Coord DroneNetMob::destAssignment(){

}*/
std::vector<Coord> DroneNetMob::MissionPathDefinitionFromSource(){

    std::vector <int> targets;
    int numdst;
    std::vector<Coord> seldst;
    std::vector<int> ind;

    int n = rand() % (4 - 2 + 1) + 2;
    for (int i = 0; i < n; i++){
        if (targets.empty() == true){
            int k = rand() % 6;
            targets.push_back(k);
            seldst.push_back(dst[k]);
        }
        else{
            int k;
            do{
                k = rand() % 6;
            }
            while(std::find(targets.begin(), targets.end(), k) != targets.end());
            targets.push_back(k);
            seldst.push_back(dst[k]);
        }
    }
    std::sort (seldst.begin(), seldst.end(),sortDesByShP);
    seldst.push_back(mBasePos.NodePos);

#ifdef DEBUG_DRONEMOBILITY
    std::cout <<"Mission Path: "<< getParentModule()->getFullName() <<std::endl;
    for (auto i:seldst){
        std::cout <<" TARG : (" <<i.x <<"; " <<i.y <<"; " <<i.z <<")";
    }
    std::cout <<std::endl;
#endif
    return seldst;
}


/*Multiple destination visit (2-4) that are randomly chosen amongst the destinations
 * visit them following the shortest path
 * */
std::vector<Coord> DroneNetMob::droneMissionDestinationSelection(){
    int numdst;
    std::vector<Coord> seldst;
    std::vector<int> ind;
    int n = rand() % (4 - 2 + 1) + 2;
    for (int i = 0; i < n; i++){
        int k = rand() % 6;
        ind.push_back(k);
        seldst.push_back(dst[k]);
    }
    return seldst;
}
node DroneNetMob::selectNextFlightDst(node CurDst){
    node NxtDst;
    std::vector<node> GrN = returngraphnodes();

    return NxtDst;
}
double DroneNetMob::returnHight(){
    return Lz[L];
}

void DroneNetMob::droneMissionPlan(double L, Coord bst, std::vector<Coord> dsts){
  wayback.push_back(bst);
  double z = Lz[L];// Flying height
  Coord tmpsrc = bst;
  for (auto i: dsts){
     //To layer

     double v= speedParameter->doubleValue();
     double vd = spDevParameter->doubleValue();
     int n1 = ceil (z/(v-vd));
     Coord nextp;
     nextp.x = tmpsrc.x, nextp.y = tmpsrc.y, nextp.z = 0.0;
     for (int i1 = 1; i1 <=n1; i1++){ //Take off
         Coord tp, er;
         er.x = (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);
         er.y= (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);
         er.z = (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);;
         nextp.z+=(v-vd);
         tp= nextp;
         tp.x = tp.x + er.x, tp.y = tp.y + er.y, tp.z = tp.z + er.z;
         goway.push_back(nextp);
         gowaywitherror.push_back(tp);
         goError.push_back(er);
         wayback.push_back(nextp);
         waybackwithError.push_back(tp);
         backError.push_back(er);
     }
     double d = sqrt(pow(i.x-nextp.x, 2)+ pow(i.y-nextp.y, 2) + pow(i.z-nextp.z, 2));
     int numpts = (d/(speedParameter->doubleValue() - spDevParameter->doubleValue()))/0.1; //0.1 is the move update time
     Coord v1, v2;
     v1.x= 0.0, v1.y = 0.0, v1.z = 0.0, v2.x= 0.0, v2.y = 0.0, v2.z = 0.0;
     v1.x = i.x-nextp.x, v1.y = i.y-nextp.y, v1.z= i.z-nextp.z;
     v2.x = nextp.x-nextp.x, v2.y = i.y-nextp.y, v2.z= i.z-nextp.z;
     double v1mag = sqrt(pow(v1.x,2) + pow(v1.y,2) + pow(v1.z,2));
     double v2mag = sqrt(pow(v2.x,2) + pow(v2.y,2) + pow(v2.z,2));

     double Thet = acos ((v1.x/v1mag) *(v2.x/v2mag) + (v1.y/v1mag) *(v2.y/v2mag) + (v1.z/v1mag) *(v2.z/v2mag));
     if (Thet != Thet){ // Checking nan value
         Thet = 0.0;
     }
     for (int i = 0; i < numpts; i++){ // Flying
         Coord tp, er;
         er.x = (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);
         er.y= (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);
         er.z = (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);;
         nextp.x +=v * sin(Thet), nextp.y = v*sin(Thet);
         goway.push_back(nextp);
         wayback.push_back(nextp);
         tp.x = nextp.x + er.x, tp.y = nextp.y + er.y, tp.z = nextp.z + er.z;
         gowaywitherror.push_back(tp);
         goError.push_back(er);
         waybackwithError.push_back(tp);
         backError.push_back(er);
     }
     int n2 = ceil (z/v);
     for (int i2 = n2; i2>0; i2--){ //Landing to Destination
         Coord tp, er;
         er.x = (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);
         er.y= (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);
         er.z = (rand() / (double)RAND_MAX) * (1.5 - (-1.5)) + (-1.5);;
         nextp.z=nextp.z- v;
         tp= nextp;
         tp.x = tp.x + er.x, tp.y = tp.y + er.y, tp.z = tp.z + er.z;
         goway.push_back(nextp);
         wayback.push_back(nextp);
         gowaywitherror.push_back(tp);
         goError.push_back(er);
         waybackwithError.push_back(tp);
         backError.push_back(er);
     }
     tmpsrc = i;
 }
/*  //Check Contents:
   std::cout <<"Way: (";
   for (auto i:goway){
       std::cout <<i.x <<"; "<<i.y <<"; "<<i.z <<") |";
   }
   std::cout<<endl;
   std::cout <<"Back: (";
   for (auto j:wayback){
       std::cout <<j.x <<"; "<<j.y <<"; "<<j.z <<") |";
   }*/
}
Coord DroneNetMob::EKFMotionProcessing(){
    Coord Estcoord;
    FusionEKF fEKF;
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    MeasurementPackage meas_package;
    Coord p;
    bool wayflag = true;
    if (!gowaywitherror.empty()){
        p = gowaywitherror.front();
        gowaywitherror.erase(gowaywitherror.begin());
    }
    else if (!waybackwithError.empty()){
        p = waybackwithError.front();
        waybackwithError.erase(waybackwithError.begin());
        wayflag = false;
    }
    else{
        if (strcmp(getParentModule()->getFullName(), "drone[0]") == 0){
            ofstream res1;
            res1.open("/home/iotlab/Documents/Drone-Simulations/cana/simulations/simu5gdrone/results/GTPosition.csv");
            for (auto i : gtruth){
                res1 <<i.first<<", "
                        <<i.second.x <<", "
                        <<i.second.y <<", "
                        <<i.second.z <<std::endl;
            }
            res1.close();
            ofstream res2;
            res2.open("/home/iotlab/Documents/Drone-Simulations/cana/simulations/simu5gdrone/results/RawPosition.csv");
            for (auto j : rawdata){
                res2 <<j.first<<", "
                        <<j.second.x <<", "
                        <<j.second.y <<", "
                        <<j.second.z <<std::endl;
            }
            res2.close();
            ofstream res3;
            res3.open("/home/iotlab/Documents/Drone-Simulations/cana/simulations/simu5gdrone/results/MeasPosition.csv");
            for (auto k : Measurement){
                res3 <<k.first<<", "
                        <<k.second.x <<", "
                        <<k.second.y <<", "
                        <<k.second.z <<std::endl;
            }
            res3.close();

            endSimulation();
        }
     }
 //    meas_package

     //    meas_package.raw_measurements_ << p.x, p.y, p.z;
         VectorXd rm(6);
         rm(0) = p.x;
         rm(1) = p.y;
         rm(2) = p.z;
         rm(3) = 0.0;
         rm(4) = 0.0;
         rm(5) = 0.0;
         meas_package.raw_measurements_ = rm;
         meas_package.raw_measurements_ << p.x, p.y, p.z;
         rawdata.insert(make_pair(simTime().dbl(), p));
         meas_package.timestamp_ = simTime().dbl();
         Coord gt;
         if (wayflag){
             gt = goway.front();
             goway.erase(goway.begin());
         }
         else{
             gt = wayback.front();
             wayback.erase(wayback.begin());
         }

         VectorXd gt_values(6); //make it 6 to include the velocity
         gt_values(gt.x);
         gt_values(gt.y);
         gt_values(gt.z);
         gt_values(speedParameter->doubleValue() - spDevParameter->doubleValue());
         gt_values(0.0);
         gt_values(0.0);
         gtruth.insert(make_pair(simTime().dbl(), gt));

         ground_truth.push_back(gt_values);
//         std::cout<<"************" <<std::endl;

         //Call ProcessMeasurment(meas_package) for Kalman filter
         fEKF.ProcessMeasurement(meas_package);
         //Push the current estimated x,y positon from the Kalman filter's state vector
         VectorXd estimate(6);
         double p_x = fEKF.ekf_.x_(0);
         double p_y = fEKF.ekf_.x_(1);
         double p_z = fEKF.ekf_.x_(2);
         double vx  = fEKF.ekf_.x_(3);
         double vy = fEKF.ekf_.x_(4);
         double vz = fEKF.ekf_.x_(5);
         estimate(0)= p_x;
         estimate(1)= p_y;
         estimate(2)= p_z;
         estimate(3)= vx;
         estimate(4)= vy;
         estimate(5)= vz;
         estimations.push_back(estimate);
         VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
         Estcoord.x = estimate(0);
         Estcoord.y = estimate(1);
         Estcoord.z = estimate(2);
         Measurement.insert(make_pair(simTime().dbl(), Estcoord));

    return Estcoord;

}

} // namespace inet

