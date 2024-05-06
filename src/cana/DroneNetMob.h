/*
 * DroneNetMob.h
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */

#ifndef __INET_DRONENETMOB_H
#define __INET_DRONENETMOB_H
#include <vector>
#include <algorithm>
#include <iostream>
#include "graphstruct.h"
#include <cmath>
#include "inet/environment/contract/IGround.h"
#include "inet/mobility/base/MovingMobilityBase.h"
#include "FusionEKF.h"



namespace inet {

/**
 * @brief Random mobility model for a mobile host with a mass.
 * See NED file for more info.
 *
 * @author Emin Ilker Cetinbas, Andras Varga
 */
struct parcel{
    int parcelID;
    double weight;
    int priority;
    double exp_time;
    Coord parceldest;
};
struct RemoteSensedVehicles{
    std::string vehID;
    simtime_t sensTime;
    double Dst;
    std::vector <double> vehPos;
};
struct cp2obstacle{
    double ManT;
    double cp;
};
struct CANDataCollection{
    int cid;
    int ncm;
    double emcp;
    double recordTime;
    double rightmanlcp;
    double leftmanlcp;
    double collDetDelay;
};
struct collisionRecords{
    int collNum = 0;
    double CollStrength = 0;
};
struct droneInf{
    std::string dn; //Drone Name
    Coord orp;//Base station position
    Coord dpos;//Drone current position
    Coord vel; //Drone Velocity
    double sp; //Speed
    int fLane; //Flying Lane
};

void parcelsDefinition (int nparcels);

enum parcelSelection{
    CDPF = 0,        //Closest-Deadline-Parcel-First
    CNPF = 1,       //Closest-Neighbor-Parcel-First
    EPDS = 2,      //Efficient Parcel Delivery Service distance/weight
    RSPF = 3,     //Randomly-Selected-Parcel-First
    HPF  = 4     //Heaviest Parcel First
};


//class MeasurementPackage {
//public:
//  double timestamp_;
//
//  Eigen::VectorXd raw_measurements_;
//};


class INET_API DroneNetMob : /*public MovingMobilityBase,*/ public spacegraph
{
  protected:
    // config (see NED file for explanation)
    cPar *changeIntervalParameter = nullptr;
    cPar *angleDeltaParameter = nullptr;
    cPar *rotationAxisAngleParameter = nullptr;
    cPar *speedParameter = nullptr;
    cPar *spDevParameter = nullptr;
    cPar *posError = nullptr;
    cPar *numdst = nullptr;
    cPar *ox = nullptr;
    cPar *oy = nullptr;
    cPar *oz = nullptr;

    // state
    Quaternion quaternion;
    simtime_t previousChange;
    Coord sourcePosition;
    Coord destination; //BAM
//    bool flagmovedtodst;
    double droneweightcapacity;
    double droneremainingbattery;
    int selectionMethod;
    droneInf myinfo;
    std::vector<parcel> MissionParcels;
    std::vector<Coord> MissionPath;
    double perr = 0;

    double deliveryStartTime = 0;
    double deliveryEndTime =   0;
    std::vector<std::map<double, Coord>> follTraj;
    std::vector<Coord> goway;//ground truth
    std::vector<Coord> wayback;
    std::vector<Coord> gowaywitherror; //Raw measurements
    std::vector<Coord> goError;
    std::vector<Coord> waybackwithError;
    std::vector<Coord> backError;
//    std::vector<Coord> Way;
//    std::vector<Coord> gocopy;
//    std::vector<Coord> backcopy;
    std::map<double, Coord> Measurement, rawdata, gtruth;
    bool myFlag = false;
    std::vector<Coord> myTrajectories;
//    cMessage* TrigAccident;



  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief Move the host according to the current simulation time. */
    virtual void move() override;
    void orient() override;

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition() override;

    virtual void finish() override;
    std::vector <Coord> intervalTimesToDestination (Coord CurPos, Coord TargPos);
    std::vector <Coord> interDestSet (Coord CurPos, Coord TargPos);
    std::vector <Coord> myWayToDest;
    std::vector<Coord> mycarriedmission;
    std::vector<double> mt;

  public:
    DroneNetMob();
    virtual double getMaxSpeed() const override;
    void destGen();
    int ndst = 0;
    void parcelsDefinition (int nparcels);
    std::vector<parcel> droneParcelsSelectionFromSource(int parcelSel);
    Coord missionPathNextDest(Coord curpos);
    Coord missionPathNextDest_n(Coord cp);
    Coord destAssignment();
    void managethedestcongestions();
    std::vector<Coord> droneMissionDestinationSelection();
    void shortestPathTraversal();
    node mBasePos; /*Mission base node*/
    /*Nodes to fly for the current Mission
     * This function defines the Path planning a drone will follow*/
    std::vector<node> MissionPlanedNodes;
    node curNode; /*/The node the drone is flying towards;*/
    /*return the next Node to fly towards
     * Specifically define the goal of the drone in terms of destinations*/
    node selectNextFlightDst(node curdst);
    void destinationHandle(); /*Handle self arrival to the destination based on all other arrivals*/
    Coord pointdront(Coord curPos, Coord desirPos);
    int L;
//    std::vector<Coord> WP;//Way point (ideal trajectory)
//    std::vector<Coord> createwayPoint();
    void droneMissionPlan(double L, Coord bst, std::vector<Coord> dsts);
    Coord EKFMotionProcessing();
    std::vector<Coord> myCurrentMission;
    std::vector<Coord> MissionPathDefinitionFromSource();
    double maxT2CThresh = 0;
    double minT2CThresh = 0;
    std::string n;
    double sp = 0;
    bool isCH = false;
    bool cpdone = false;
    Coord myposition(){return targetPosition;};
    bool istherecollisonrisk = false;
    bool accNode = false;
    double returnHight();
};

//class trajectoryPlanner{
//public:
//    std::vector<Coord> droneMissionPlan(double L, Coord bst, std::map<std::string,Coord> fdpos);
//};

} // namespace inet

#endif // ifndef __INET_DRONENETMOB_H

