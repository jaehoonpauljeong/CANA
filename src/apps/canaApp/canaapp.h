//
//                  Simu5G
//
// Authors: Bien Aime Mugabarigira, Jaehoon Jeong, Han-You Jeong, and Tae Oh
// Adapted from MultihopD2D.h by Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
//
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself,
// and cannot be removed from it.
//

#ifndef _LTE_CANAAPP_H_
#define _LTE_CANAAPP_H_

#include <string.h>
#include <omnetpp.h>
#include <inet/transportlayer/contract/udp/UdpSocket.h>
#include <inet/networklayer/common/L3AddressResolver.h>
#include "canaEventGenerator/canaEventGenerator.h"
#include "apps/canaApp/statistics/CANAStatistics.h"
#include "canaPacket_m.h"
#include "stack/mac/layer/LteMacBase.h"
#include "stack/phy/layer/LtePhyBase.h"
#include "../../cana/DroneNetMob.h"

class canaEventGenerator;

struct Clusterstruct{
    int CID;
    std::string cluster_Hid;
    std::map<std::string, DroneNetMob*> clusterMembMob; //Member Vehicle Id and its corresponding mobility
};
enum runtype{
    cana = 1,
    ncas = 2,
    sens = 3
};
struct lanequality{
    double ulq = 1.0; // Lane quality to upper layer
    double dlq = 1.0; // Lane quality to lower layer
    double llq = 1.0; // Lane quality to left lane
    double rlq = 1.0; // lane quality to right lane
    double elq = 1.0; // lane quality to emergent lane
};
struct laneCP{
    double ucp = 0.0; // Lane quality to upper layer
    double dcp = 0.0; // Lane quality to lower layer
    double lcp = 0.0; // Lane quality to left lane
    double rcp = 0.0; // lane quality to right lane
    double ecp = 0.0; // lane quality to emergent lane
};
class canaapp : public omnetpp::cSimpleModule
{
    static uint16_t numcanaappApps;  // counter of apps (used for assigning the ids)

protected:
    uint16_t senderAppId_;             // unique identifier of the application within the network
    uint16_t localMsgId_;              // least-significant bits for the identifier of the next message
    int msgSize_;

    double selfishProbability_;         // if = 0, the node is always collaborative
    int ttl_;                           // if < 0, do not use hops to limit the flooding
    double maxBroadcastRadius_;         // if < 0, do not use radius to limit the flooding
    omnetpp::simtime_t maxTransmissionDelay_;    // if > 0, when a new message has to be transmitted, choose an offset between 0 and maxTransmissionDelay_

    /*
     * Trickle parameters
     */
    bool trickleEnabled_;
    unsigned int k_;
    omnetpp::simtime_t I_;
    std::map<unsigned int, inet::Packet*> last_;
    std::map<unsigned int, unsigned int> counter_;
    /***************************************************/

    std::map<uint32_t,bool> relayedMsgMap_;  // indicates if a received message has been relayed before

    int localPort_;
    int destPort_;
    inet::L3Address destAddress_;
    inet::UdpSocket socket;

    omnetpp::cMessage *selfSender_;
    omnetpp::cMessage *reclustering;
    omnetpp::cMessage *accidentbegin;
    omnetpp::cMessage *senscheck;

    canaEventGenerator* eventGen_;          // reference to the canaEventGenerator
    LtePhyBase* ltePhy_;                // reference to the LtePhy
    MacNodeId lteNodeId_;               // LTE Node Id
    MacNodeId lteCellId_;               // LTE Cell Id

    // local statistics
    omnetpp::simsignal_t canaGeneratedMsg_;
    omnetpp::simsignal_t canaSentMsg_;
    omnetpp::simsignal_t canaRcvdMsg_;
    omnetpp::simsignal_t canaRcvdDupMsg_;
    omnetpp::simsignal_t canaTrickleSuppressedMsg_;


    // reference to the statistics manager
/*    CANAStatistics* stat_;*/

    virtual int numInitStages() const { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage);
    virtual void handleMessage(omnetpp::cMessage *msg);
    virtual void finish();

    void markAsReceived(uint32_t msgId);      // store the msg id in the set of received messages
    bool isAlreadyReceived(uint32_t msgId);   // returns true if the given msg has already been received
    void markAsRelayed(uint32_t msgId);       // set the corresponding entry in the set as relayed
    bool isAlreadyRelayed(uint32_t msgId);   // returns true if the given msg has already been relayed
    bool isWithinBroadcastArea(inet::Coord srcCoord, double maxRadius);

    virtual void sendPacket();
    virtual void handleRcvdPacket(omnetpp::cMessage* msg);
    virtual void handleTrickleTimer(omnetpp::cMessage* msg);
    virtual void relayPacket(omnetpp::cMessage* msg);
    /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
    virtual void populatecanamsg(canaPacket* pkt);
    uint32_t receivedccms;
    uint32_t receivedecms;
    void updateneigbours(ccm* ccmpkt);
    void flightmaneuvers(ecm* ecmpkt);
    DroneNetMob *mymobility;
    bool RisksInformed = false;

    bool isMyClusterInManeuver = false;
    double ExpTimeToCollision = 0;
    double InjTime = 0;
    double CollDetTime = 0;
    bool ecmsent = false;
    bool isccmsg = true;
    bool isdynamicObstacle = false;
    std::string getClusterHeadID (std::map<std::string,DroneNetMob*> Allvehicles);
    std::string getInterUAVsAveragedistances(std::map<std::string,DroneNetMob*> Allvehicles);
    void clustersUpdates();
    runtype currentrun;
    void informmembers();
    bool informationspanet();
    double CP2Obstacle(Coord obstPos, Coord EgovehPos, double obstSpeed, double EgovehSpeed);
    lanequality compute3dlanequality();
    double computeTimeToCollision(Coord obstPos, Coord EgovehPos, double obstSpeed, double EgovehSpeed);
    double anglebetweendrones(Coord v1, Coord v2);
    lanequality flightlaneqlty;
     std::vector<lanequality> lqstat;
     std::vector<double> lqt;
     std::vector<laneCP> avcolprob;
     std::vector<double> cpt;
     Clusterstruct returnmyCluser();
  public:
    canaapp();
    ~canaapp();
    enum canaAppMessageKinds {
        CCC_EVT,
        ECM_EVT
    };

//    virtual void handleEvent(unsigned int eventId);
};

#endif

