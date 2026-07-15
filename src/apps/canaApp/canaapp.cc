//
//                  Simu5G
//
// Authors: Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
//
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself,
// and cannot be removed from it.
//

#include "canaapp.h"
#include <cmath>
#include <fstream>

#include <inet/common/packet/chunk/ByteCountChunk.h>
#include <inet/common/TimeTag_m.h>
#include "canaTrickleTimerMsg_m.h"
#include "stack/mac/layer/LteMacBase.h"
#include "inet/common/ModuleAccess.h"  // for multicast support

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#define round(x) floor((x) + 0.5)

Define_Module(canaapp);

using namespace inet;

uint16_t canaapp::numcanaappApps = 0;

DroneNetMob* obMob;
std::vector<Clusterstruct> AllClustInfo;
bool isFirstClustVeh = false;
static int clusterValue = 1; //This value will keep increasing the number of cluster increases
double clusterFormationStart = 0;
bool AheadCHinEmMan = false;
double AheadManeuverStartTime = 0;
double AheadManDuration = 0.0;
int glManLane = 0;
int clusID = 0;
int nmemb = 0;
int nsent = 0;
int numrcvd = 0;
bool accinfspanned = false;
double obh = 0;
double obt = 0;
double accT = 0;
double elT = 0;
std::map <std::string, DroneNetMob*> allDrones;

std::vector<double> trcv; //Latency
struct ClusAss{
    int cid;
    int nmber;
    int npkt;
    int nrcd;
};
double AccidentTime = 0;
double AccInfTime = 0;
bool obstactive = false;
std::vector<ClusAss> cluscomm;
std::vector<Coord> obPos;
std::vector<double> Probs;
std::vector<lanequality> laneq;
double timetospan = 0;
int ntospan = 0;
Coord fdp;
canaapp::canaapp()
{
    senderAppId_ = numcanaappApps++;
    selfSender_ = nullptr;
    localMsgId_ = 0;
}

canaapp::~canaapp()
{
    cancelAndDelete(selfSender_);
    cancelAndDelete(reclustering);
    cancelAndDelete(accidentbegin);
    cancelAndDelete(sendTimer);


    if (trickleEnabled_)
    {
        std::map<unsigned int, inet::Packet*>::iterator it = last_.begin();
        for (; it != last_.end(); ++it)
            if (it->second != nullptr)
                delete it->second;
        last_.clear();
    }
}
void canaapp::initialize(int stage)
{
    // avoid multiple initializations
    if (stage == INITSTAGE_APPLICATION_LAYER )
    {
        EV << "canaapp initialize: stage " << stage << endl;

        destPort_ = par("destPort");
        localPort_ = par("localPort"); // Track local data plane port assignment
        destAddress_ = L3AddressResolver().resolve(par("destAddress").stringValue());

        msgSize_ = par("msgSize");
        if(B(msgSize_) < CANA_HEADER_LENGTH){
            throw cRuntimeError("canaapp::init - FATAL! Total message size cannot be less than D2D_MULTIHOP_HEADER_LENGTH");
        }

        maxBroadcastRadius_ = par("maxBroadcastRadius");
        ttl_ = par("ttl");
        maxTransmissionDelay_ = par("maxTransmissionDelay");
        selfishProbability_ = par("selfishProbability");

        trickleEnabled_ = par("trickle").boolValue();
        currentrun = rlgym;

        if (trickleEnabled_)
        {
            I_ = par("I");
            k_ = par("k");
            if (k_ <= 0)
                throw cRuntimeError("Bad value for k. It must be greater than zero");
        }

        droneUdpSocket.setOutputGate(gate("socketOut"));
        droneUdpSocket.bind(localPort_);

        int tos = par("tos");
        isdynamicObstacle = true;
        if (tos != -1)
            droneUdpSocket.setTos(tos);

        inet::IInterfaceTable *ift = inet::getModuleFromPar< inet::IInterfaceTable >(par("interfaceTableModule"), this);
        NetworkInterface *ie = ift->findInterfaceByName(par("interfaceName").stringValue());
        if (!ie)
            throw cRuntimeError("Wrong multicastInterface setting: no interface found");
        inet::MulticastGroupList mgl = ift->collectMulticastGroups();
        droneUdpSocket.joinLocalMulticastGroups(mgl);
        droneUdpSocket.setMulticastOutputInterface(ie->getInterfaceId());

        selfSender_ = new cMessage("selfSender");
        reclustering = new cMessage("Reclustering");
        accidentbegin = new cMessage("Accident");
        senscheck =  new cMessage("Sensing");

        // get references to LTE entities

        ltePhy_ = check_and_cast<LtePhyBase*>(getParentModule()->getSubmodule("cellularNic")->getSubmodule("phy"));
        LteMacBase* mac = check_and_cast<LteMacBase*>(getParentModule()->getSubmodule("cellularNic")->getSubmodule("mac"));
        lteNodeId_ = mac->getMacNodeId();
        lteCellId_ = mac->getMacCellId();

        // local statistics
        canaGeneratedMsg_ = registerSignal("canaGeneratedMsg");
        canaSentMsg_ = registerSignal("canaSentMsg");
        canaRcvdMsg_ = registerSignal("canaRcvdMsg");
        canaRcvdDupMsg_= registerSignal("canaRcvdDupMsg");
        canaTrickleSuppressedMsg_ = registerSignal("CANATrickleSuppressedMsg");

      for (cModule::SubmoduleIterator it(getParentModule()); !it.end(); ++it) {
            cModule *submodule = *it;
            if (strcmp(submodule->getName(), "mobility") == 0){
                mymobility = check_and_cast<DroneNetMob*>(submodule);
            }
        }
      if (currentrun == cana){
          /*      Cluster Formation*/
        std::string dn = getParentModule()->getFullName();
        if (dn.find( "drone") != std::string::npos){//Assign Newly injected Vehicles to a Corresponding Cluster
            InjTime = simTime().dbl();
            allDrones.insert(std::make_pair(dn,mymobility));
            if  (!isFirstClustVeh){
                fdp =  mymobility->mBasePos.NodePos;
                isFirstClustVeh = true;
                Clusterstruct tmpClust;
                clusterFormationStart = simTime().dbl();
                tmpClust.cluster_Hid = getParentModule()->getFullName();
                tmpClust.clusterMembMob.insert(std::make_pair(getParentModule()->getFullName(),mymobility));
                tmpClust.CID = clusID + 1;
                nmemb++;
                AllClustInfo.push_back(tmpClust);
            }
            else{
                Coord p = mymobility->mBasePos.NodePos;
                double Dist = sqrt(pow(p.x - fdp.x, 2) + pow(p.y - fdp.y, 2)
                        + pow(p.z - fdp.z, 2));
                if (Dist <= 300 && nmemb <=10){
                    AllClustInfo.back().clusterMembMob.insert(std::make_pair(getParentModule()->getFullName(),mymobility));
                    std::map<std::string, DroneNetMob*>  cm = AllClustInfo.back().clusterMembMob;
                    std::string CHID = getClusterHeadID(cm);
                    AllClustInfo.back().cluster_Hid = CHID;
                    nmemb++;
                }
                else if (nmemb > 10 || Dist > 300){
                    fdp =  mymobility->mBasePos.NodePos;
                    Clusterstruct tmpClust;
                    clusterFormationStart = simTime().dbl();
                    tmpClust.cluster_Hid = getParentModule()->getFullName();
                    tmpClust.clusterMembMob.insert(std::make_pair(getParentModule()->getFullName(),mymobility));
                    tmpClust.CID = clusID + 1;
                    nmemb = 1;
                    AllClustInfo.push_back(tmpClust);
                }
            }
        }
      }
      else{
          std::string dn = getParentModule()->getFullName();
          if (dn.find( "drone") != std::string::npos){//Assign Newly injected Vehicles to a Corresponding Cluster
              InjTime = simTime().dbl();
              allDrones.insert(std::make_pair(dn,mymobility));
          }
      }

      if (currentrun == sens){
          std::string dne = getParentModule()->getFullName();
          if (dne.find( "drone") != std::string::npos){
              scheduleAt(simTime() + 5.0, senscheck);
          }
      }
      else{
          simtime_t t = simTime() + 1.0;
          scheduleAt(t, selfSender_);
      }
      if (std::strcmp(getParentModule()->getFullName(), "drone1[1]") == 0/* ||
              std::strcmp(getParentModule()->getFullName(), "drone2[1]") == 0 ||
              std::strcmp(getParentModule()->getFullName(), "drone3[1]") == 0*/){//Multiple obstacles
          scheduleAfter(simTime() + 24, accidentbegin);
          if (currentrun == cana){
              scheduleAt(simTime() + 5, reclustering);
          }
      }
      if (currentrun == rlgym){
          isCloudGateway_ = par("isCloudGateway").boolValue();
          destPort_ = par("destPort");
          msgSize_ = par("msgSize");
          currentSendInterval_ = par("sendInterval").doubleValue();

          // 1. Always create the timer object first
          sendTimer = new omnetpp::cMessage("sendTimer");

          if (isCloudGateway_) {
              EV_INFO << "[CanaApp] Node marked as Cloud Gateway. Spawning native Ubuntu background socket...\n";

              int os_socket = ::socket(AF_INET, SOCK_STREAM, 0);
              if (os_socket < 0) {
                  throw omnetpp::cRuntimeError("Fatal: System failed to allocate an OS socket descriptor.");
              }

              int flags = ::fcntl(os_socket, F_GETFL, 0);
              if (flags >= 0) {
                  ::fcntl(os_socket, F_SETFL, flags | O_NONBLOCK);
              } else {
                  ::close(os_socket);
                  throw omnetpp::cRuntimeError("Fatal: Failed to retrieve socket flags via fcntl.");
              }

              struct sockaddr_in serv_addr;
              std::memset(&serv_addr, 0, sizeof(serv_addr));
              serv_addr.sin_family = AF_INET;
              serv_addr.sin_port = htons(par("connectPort").intValue());

              const char* destAddress = par("connectAddress").stringValue();
              if (::inet_pton(AF_INET, destAddress, &serv_addr.sin_addr) <= 0) {
                  ::close(os_socket);
                  throw omnetpp::cRuntimeError("Fatal: Invalid connection address layout matching: %s", destAddress);
              }

              int res = ::connect(os_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

              if (res < 0 && errno != EINPROGRESS) {
                  ::close(os_socket);
                  EV_ERROR << "[OS Socket] Direct connection failed immediately. Is your Python agent script running?\n";
                  gatewaySocketFd = -1;

                  // FIX: Even if the socket connection fails, we MUST schedule the timer
                  // so the drone can run locally without crashing the simulation network layout!
                  scheduleAt(simTime() + currentSendInterval_, sendTimer);
              } else {
                  gatewaySocketFd = os_socket;
                  EV_INFO << "[OS Socket] Non-blocking connection channel established in background.\n";

                  struct timeval tv;
                  tv.tv_sec = 0;
                  tv.tv_usec = 10000;
                  ::setsockopt(gatewaySocketFd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
                  ::setsockopt(gatewaySocketFd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof(tv));

                  std::string init_metrics = "0,0,0\n";
                  ::send(gatewaySocketFd, init_metrics.c_str(), init_metrics.length(), MSG_NOSIGNAL);

                  scheduleAt(simTime() + currentSendInterval_, sendTimer);
              }
          } else {
              scheduleAt(simTime() + currentSendInterval_, sendTimer);
          }
      }
    }
}

void canaapp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {

        if (msg == sendTimer) {
            // 1. Generate and transmit the actual underlying Simu5G network packet
            totalPacketsSent++;

            // 2. If this node is the designated Gateway, handle the RL step side-channel sync
            if (isCloudGateway_ && gatewaySocketFd != -1) {

                // Quick check: Verify the non-blocking background connection has finished connecting
                fd_set write_fds;
                FD_ZERO(&write_fds);
                FD_SET(gatewaySocketFd, &write_fds);

                struct timeval quick_check;
                quick_check.tv_sec = 0;
                quick_check.tv_usec = 0; // Immediate poll, do not block the UI thread

                // select returns > 0 if the socket is ready to transmit data
                if (::select(gatewaySocketFd + 1, nullptr, &write_fds, nullptr, &quick_check) > 0) {

                    // Format current cumulative application metrics
                    std::string metrics_str = std::to_string(totalPacketsSent) + "," +
                                                  std::to_string(totalPacketsReceived) + "," +
                                                  std::to_string(totalPacketsDropped) + "\n";

                    // Send state metrics to Python agent safely
                    ssize_t bytes_sent = ::send(gatewaySocketFd, metrics_str.c_str(), metrics_str.length(), MSG_NOSIGNAL);

                    if (bytes_sent < 0) {
                        EV_WARN << "[OS Socket] Pipe broken or connection dropped. Stopping RL synchronization.\n";
                        ::close(gatewaySocketFd);
                        gatewaySocketFd = -1;
                    }
                    else {
                        // Collect the chosen action interval back from Python (Timeout Guarded)
                        char buffer[1024] = {0};
                        int bytes_received = ::recv(gatewaySocketFd, buffer, sizeof(buffer) - 1, 0);

                        if (bytes_received > 0) {
                            std::string action_val(buffer);
                            try {
                                double next_interval = std::stod(action_val);
                                currentSendInterval_ = next_interval;
                                EV_INFO << "[RL Sync] Action successfully applied. Next interval: " << currentSendInterval_ << "s\n";
                            } catch (...) {}
                        } else {
                            EV_INFO << "[RL Sync] Python agent processing step... maintaining interval: " << currentSendInterval_ << "s\n";
                        }
                    }
                } else {
                    EV_INFO << "[RL Sync] Background TCP handshake still finalizing. Skipping this telemetry frame...\n";
                }
            }

            // 3. Dynamically reschedule the next timer event using the updated interval
            scheduleAt(simTime() + currentSendInterval_, sendTimer);
        }

        else if (!strcmp(msg->getName(), "selfSender")){
            sendPacket();
        }
        else if (!strcmp(msg->getName(), "ccm") || !strcmp(msg->getName(), "ecm")){
            if (currentrun == cana){
                if (mymobility->isCH){
                    relayPacket(msg);
                }
            }
            else{
                relayPacket(msg);
            }
        }
        else if (!strcmp(msg->getName(), "trickleTimer")){
            handleTrickleTimer(msg);
        }
        else if(strcmp(msg->getName(), "Reclustering") == 0){
             clustersUpdates();
             if (reclustering->isScheduled()){
                 cancelEvent(reclustering);
             }
//             Next recluster Time
             double rT = getmyClusterNextUpdateTime (returnmyCluster());
             scheduleAt(simTime() + rT, reclustering);
         }
        else if(strcmp(msg->getName(), "Accident") == 0){
            obstactive = true;
            std::ofstream pos;
            pos.open("/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/Positions.cvs");
            if (isdynamicObstacle){
                isccmsg = false;
                double AccidentTime = simTime().dbl();
                double AccInfTime = simTime().dbl();
//                mymobility->setNextChange(simTime() + 30);
                mymobility->istherecollisonrisk = true;
                mymobility->accNode = true;
                double H =mymobility->returnHight();
                double EDT =sqrt((2*H)/9.8);//Consigering the gravity = 9.8
                obh = H;
                obt = EDT;
                accT = simTime().dbl();
                Coord curp = mymobility->getCurrentPosition();
                pos<<curp.x <<", "
                        <<curp.y <<", "
                        <<curp.z<<std::endl;
                obPos.push_back(curp);
                for (auto k:allDrones){
                    Coord p = k.second->myposition();
                    pos<<p.x <<", "
                            <<p.y <<", "
                            <<p.z<<std::endl;

                }
            }
            else{
                isccmsg = false;
                double AccidentTime = simTime().dbl();
                double AccInfTime = simTime().dbl();
//                mymobility->setNextChange(simTime() + 30);
                mymobility->istherecollisonrisk = true;
                mymobility->accNode = true;
                Coord curp = mymobility->getCurrentPosition();
                obPos.push_back(curp);
                for (auto k:allDrones){
                    Coord p = k.second->myposition();
                   pos<<p.x <<", "
                           <<p.y <<", "
                           <<p.z<<std::endl;
                }
                pos.close();
            }
        }
        else if (strcmp(msg->getName(), "Sensing") == 0){
            double cT = simTime().dbl();

            elT = cT - accT;
            if (elT>=obt){
                obstactive = false;
            }
            if (obstactive){
                for (unsigned int i=0; i<obPos.size(); i++){
                    obPos[i].z  =obPos[i].z - (elT *9.8);
                    double cp = CP2Obstacle(obPos[i], mymobility->myposition(), 0, mymobility->sp);
                    if (cp > 0){
                        Probs.push_back(cp);
                        lanequality lq = compute3dlanequality(cp) ;
                        laneq.push_back(lq);
                    }
                }
            }
            if (senscheck->isScheduled()){
                cancelEvent(senscheck);
            }
            scheduleAt(simTime() + 0.5, senscheck);
        }
        else if ((strcmp(msg->getName(), "ccm") == 0) || (strcmp(msg->getName(), "ecm") == 0)){
            if (currentrun == cana){
                if (mymobility->isCH){
                    relayPacket(msg);
                }
            }
            else{
                relayPacket(msg);
            }
        }
        else{
            throw cRuntimeError("Unrecognized self message");
        }
    }
    else{
        if ((strcmp(msg->getName(), "ccm") == 0) || (strcmp(msg->getName(), "ecm") == 0)){
            if (currentrun == cana){
                if (mymobility->isCH){
                    handleRcvdPacket(msg);
                }
            }
            else{
                handleRcvdPacket(msg);
            }
        }
        else{
            throw cRuntimeError("Unrecognized self message");
        }
    }
}

void canaapp::sendPacket()
{
    // build the global identifier
    uint32_t msgId = ((uint32_t)senderAppId_ << 16) | localMsgId_;

    // create data corresponding to the desired multi-hop message size
    auto data = makeShared<ByteCountChunk>(B(msgSize_) - CANA_HEADER_LENGTH);

    // create new packet containing the data
    Packet* packet;

    if (isccmsg){
        packet = new inet::Packet("ccm", data);
    }
    else{
        nsent++;
        packet = new inet::Packet("ecm", data);
    }

    // add header
    auto mhop = makeShared<canaPacket>();
    mhop->setMsgid(msgId);
    mhop->setSrcId(lteNodeId_);
    mhop->setPayloadTimestamp(simTime());
    mhop->setPayloadSize(msgSize_);
    mhop->setTtl(ttl_-1);
    mhop->setHops(1);                // first hop
    mhop->setLastHopSenderId(lteNodeId_);
    if (maxBroadcastRadius_ > 0)
    {
        mhop->setSrcCoord(ltePhy_->getCoord());
        mhop->setMaxRadius(maxBroadcastRadius_);
    }
    mhop->addTag<CreationTimeTag>()->setCreationTime(simTime());

    packet->insertAtFront(mhop);

    EV << "canaapp::sendPacket - Sending msg (ID: "<< mhop->getMsgid() << " src: " << lteNodeId_ << " size: " << msgSize_ << ")" <<  endl;

//    socket.sendTo(packet, destAddress_, destPort_);
    // Updated to use the distinct droneUdpSocket handle
    droneUdpSocket.sendTo(packet, destAddress_, destPort_);

    markAsRelayed(msgId);

    emit(canaGeneratedMsg_, (long)1);
    emit(canaSentMsg_, (long)1);

  //return to org
  if (selfSender_->isScheduled()){
        cancelEvent(selfSender_);
    }
    scheduleAt(simTime() + 0.3, selfSender_);
}

void canaapp::handleRcvdPacket(cMessage* msg)
{
    EV << "canaapp::handleRcvdPacket - Received packet from lower layer" << endl;

    Packet* pPacket = check_and_cast<Packet*>(msg);

    if(strcmp(msg->getName(), "ecm") == 0){
        numrcvd++;
        double cT = simTime().dbl();

        elT = cT - accT;
        for (auto i : obPos){

            i.z  =i.z - (elT *9.8);
            double cp = CP2Obstacle(i, mymobility->myposition(), 0, mymobility->sp);
            if (cp > 0){
                Probs.push_back(cp);
                if (cp > 0){
                    lanequality lq = compute3dlanequality(cp) ;
                    laneq.push_back(lq);
                }
            }
        }

        isccmsg = false;
        if (currentrun == cana){
            mymobility->istherecollisonrisk = true;
            informmembers();
           if (mymobility->isCH == true){
               mymobility->istherecollisonrisk = true;
               isccmsg = false;
           }
        }
        else if(currentrun == ncas){
            mymobility->istherecollisonrisk = true;
            isccmsg = false;
        }
        if (!accinfspanned){
            if (informationspanet() == true){
                double curTime = simTime().dbl();
                timetospan = curTime - AccidentTime;
                ntospan = nsent;
                accinfspanned = true;
            }
        }
    }
    if (pPacket == nullptr)
    {
        throw cRuntimeError("canaapp::handleMessage - FATAL! Error when casting to inet packet");
    }

    auto mhop = pPacket->peekAtFront<canaPacket>();
    pPacket->removeControlInfo();

    uint32_t msgId = mhop->getMsgid();

    // check if this is a duplicate
    if (isAlreadyReceived(msgId))
    {
        if (trickleEnabled_)
            counter_[msgId]++;

        // do not need to relay the message again
        EV << "canaapp::handleRcvdPacket - The message has already been received, counter = " << counter_[msgId] << endl;

 /*       emit(canaRcvdDupMsg_, (long)1);
        stat_->recordDuplicateReception(msgId);*/
        delete pPacket;
    }
    else
    {
        // this is a new packet

        // mark the message as received
        markAsReceived(msgId);

        if (trickleEnabled_)
        {
            counter_[msgId] = 1;

            // store a copy of the packet
            if (last_.find(msgId) != last_.end() && last_[msgId] != nullptr)
            {
                delete last_[msgId];
                last_[msgId] = nullptr;
            }
            last_[msgId] = pPacket->dup();
        }

        // emit statistics
        simtime_t delay = simTime() - mhop->getPayloadTimestamp();
        emit(canaRcvdMsg_, (long)1);
        // check for selfish behavior of the user
        if (uniform(0.0,1.0) < selfishProbability_)
        {
            // selfish user, do not relay
            EV << "canaapp::handleRcvdPacket - The user is selfish, do not forward the message. " << endl;
            delete pPacket;
        }
        else if (mhop->getMaxRadius() > 0 && !isWithinBroadcastArea(mhop->getSrcCoord(), mhop->getMaxRadius()))
        {
            EV << "canaapp::handleRcvdPacket - The node is outside the broadcast area. Do not forward it. " << endl;
            delete pPacket;
        }
        else if (mhop->getTtl() == 0)
        {
            // TTL expired
            EV << "canaapp::handleRcvdPacket - The TTL for this message has expired. Do not forward it. " << endl;
            delete pPacket;
        }
        else
        {
            if (trickleEnabled_)
            {
                // start Trickle interval timer
                canaTrickleTimerMsg* timer = new canaTrickleTimerMsg("trickleTimer");
                timer->setMsgid(msgId);

                simtime_t t = uniform(I_/2 , I_);
                t = round(SIMTIME_DBL(t)*1000)/1000;
                EV << "canaapp::handleRcvdPacket - start Trickle interval, duration[" << t << "s]" << endl;
                scheduleAt(simTime() + t, timer);
                delete pPacket;
            }
            else
            {
                // relay the message after some random backoff time
    //
                 simtime_t offset = 0.0;
                 if (maxTransmissionDelay_ > 0)
                     offset = uniform(0,maxTransmissionDelay_);

                 offset = round(SIMTIME_DBL(offset)*1000)/1000;
                 scheduleAt(simTime() + offset, pPacket);
                 EV << "canaapp::handleRcvdPacket - will relay the message in " << offset << "s" << endl;
            }
        }
    }
    if (currentrun == cana){
        if (mymobility->isCH == true && simTime() > 10){
            isccmsg = false;
        }
    }
    else{
        if (simTime() > 10){
            isccmsg = false;
        }
    }
}

void canaapp::handleTrickleTimer(cMessage* msg)
{
    canaTrickleTimerMsg* timer = check_and_cast<canaTrickleTimerMsg*>(msg);
    unsigned int msgId = timer->getMsgid();
    if (counter_[msgId] < k_)
    {
        EV << "canaapp::handleTrickleTimer - relay the message, counter[" << counter_[msgId] << "] k[" << k_ << "]" << endl;
        relayPacket(last_[msgId]->dup());
    }
    else
    {
  /*      EV << "canaapp::handleTrickleTimer - suppressed message, counter[" << counter_[msgId] << "] k[" << k_ << "]" << endl;
        stat_->recordSuppressedMessage(msgId);
        emit(canaTrickleSuppressedMsg_, (long)1);*/
    }
    delete msg;
}

void canaapp::relayPacket(cMessage* msg)
{
    Packet* pPacket = check_and_cast<Packet*>(msg);
    auto src = pPacket->popAtFront<canaPacket>();

    // create a relay packet using the data of the source packet
    auto data = pPacket->peekData();
//    Packet* relayPacket = new inet::Packet("canaPacket", data);
    Packet* relayPacket;

   if (isccmsg){
        relayPacket = new inet::Packet("ccm", data);
    }
    else{
        relayPacket = new inet::Packet("ecm", data);
    }

    // create a new header
    auto dst = makeShared<canaPacket>();

    // increase the number of hops
    unsigned int hops = src->getHops();
    dst->setHops(++hops);

    // decrease TTL
    if (src->getTtl() > 0)
    {
        int ttl = src->getTtl();
        dst->setTtl(--ttl);
    }

    // set this node as last sender
    dst->setLastHopSenderId(lteNodeId_);

    // copy remaining header fields from original packet header
    dst->setSrcId(src->getSrcId());
    dst->setMsgid(src->getMsgid());
    dst->setPayloadSize(src->getPayloadSize());
    dst->setSrcCoord(src->getSrcCoord());
    dst->setMaxRadius(src->getMaxRadius());
    dst->setPayloadTimestamp(src->getPayloadTimestamp());

    relayPacket->insertAtFront(dst);

    EV << "canaapp::relayPacket - Relay msg " << dst->getMsgid() << " to address " << destAddress_ << endl;

//    socket.sendTo(relayPacket, destAddress_, destPort_);
    // Updated to use droneUdpSocket
    droneUdpSocket.sendTo(relayPacket, destAddress_, destPort_);

    markAsRelayed(dst->getMsgid());    // mark the message as relayed
  /*  emit(canaSentMsg_, (long)1);
    stat_->recordSentMessage(dst->getMsgid());*/

    delete pPacket;
}
void canaapp::populatecanamsg(canaPacket* pkt){
    pkt->setSrcCoord(mymobility->myposition());
    pkt->setTtl(ttl_);
    canaPacket* pt = dynamic_cast<canaPacket*>(pkt);
    if (ecm* ecmpkt = dynamic_cast<ecm*>(pt)) {
        ecmpkt->setAccCoord(mymobility->myposition());
        ecmpkt->setAccSpeed(mymobility->myposition());
        ecmpkt->setAcl(mymobility->L);

    }
    else if (ccm* ccmpkt = dynamic_cast<ccm*>(pt)) {
        ccmpkt->setSrcSpeed(mymobility->myposition());
//        receivedccms++;
//        updateneigbours(ccmpkt);
    }
    else {
//        receivedccms++;
//        updateneigbours(ccmpkt);
    }
    delete (pkt);
}

void canaapp::updateneigbours(ccm* ccmpkt){

}
void canaapp::flightmaneuvers(ecm* ecmpkt){

}
double canaapp::CP2Obstacle(Coord obstPos, Coord EgovehPos, double obstSpeed, double EgovehSpeed){
    double vcp = 0;
    double ManT;
    double t2c = computeTimeToCollision(obstPos, EgovehPos, obstSpeed, EgovehSpeed);
    if (t2c >= mymobility->maxT2CThresh){
        ManT = INFINITY;
        vcp = 0;
    }
    else if (t2c <= mymobility->minT2CThresh){
        ManT = t2c; //use it to compute the speed during collision
        vcp = 1;
    }
    else{
        ManT = t2c;
        vcp = (mymobility->maxT2CThresh - t2c)/(mymobility->maxT2CThresh-mymobility->minT2CThresh);
    }
    return vcp;
}
double canaapp::computeTimeToCollision(Coord obstPos, Coord EgovehPos, double obstSpeed, double EgovehSpeed){
    double InterDist = 0;
    double T2C = 0;
    InterDist = sqrt(pow(EgovehPos.x - obstPos.x, 2) + pow(EgovehPos.y - obstPos.y, 2)
            + pow(EgovehPos.z - obstPos.z, 2));
    T2C = fabs(InterDist/(fabs(obstSpeed - EgovehSpeed)));

    return T2C;
}
lanequality canaapp::compute3dlanequality(double cp){
    std::vector<double> cpl, cpr, cpe, cpu, cpd;
    cpe.push_back(cp);
    double lav = 0.0, rav = 0.0, eav = 0.0, uav = 0.0, dav = 0.0;
    std::map <std::string, DroneNetMob*>::iterator it;
    Coord p = mymobility->getCurrentPosition();
    Coord v = mymobility->getCurrentVelocity();
    double sp = std::sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
    lanequality lq;
    laneCP avcp;
    if (currentrun == cana){
        Clusterstruct myClus =  returnmyCluster();
        for (auto i: myClus.clusterMembMob){
            Coord tempv ;
            Coord tmpp ;
            if (i.second->accNode){
                tempv.x = 0.0, tempv.y = 0.0, tempv.z = 0.0;
            }
            else{
                tempv = i.second->getCurrentVelocity();
            }

            tmpp = i.second->getCurrentPosition();
            double tempsp = std::sqrt( tempv.x * tempv.x + tempv.y * tempv.y + tempv.z * tempv.z);
            double thetha = anglebetweendrones(v, tempv);
            double p1 = CP2Obstacle(tmpp, p, tempv.x, v.x);
            if (p1 > 0){
                cpe.push_back(p1);
            }
            double p2 = CP2Obstacle(tmpp, p, tempv.y * cos (thetha), v.y);
            if (p2 > 0){
                cpl.push_back(p2);
            }
            double p3 = CP2Obstacle(tmpp, p, tempv.y * sin(thetha), v.z);
            if (p3 > 0){
                cpr.push_back(p3);
            }
            double p4 = CP2Obstacle(tmpp, p, tempv.z, v.z);
            if (p4 > 0){
                cpu.push_back(p4);
            }
            double p5 = CP2Obstacle(tmpp, p, tempv.z *(-1), v.z);
            if (p5 > 0){
                cpd.push_back(p5);
            }
        }

        for (int i = 0; i < cpu.size(); i++){
            lq.ulq = lq.ulq * (1-cpu[i]);
            avcp.ucp += cpu[i];
        }
        if (cpu.size() > 0){
            avcp.ucp = (avcp.ucp/cpu.size());
        }
        for (int i = 0; i < cpd.size(); i++){
            lq.dlq = lq.dlq * (1-cpd[i]);
            avcp.dcp += cpd[i];
        }
        if (cpd.size() > 0){
            avcp.dcp = (avcp.dcp/cpd.size());
        }
        for (int i = 0; i < cpl.size(); i++){
            lq.llq = lq.llq * (1-cpl[i]);
            avcp.lcp += cpl[i];
        }
        if (cpl.size() > 0){
            avcp.lcp = (avcp.lcp/cpl.size());
        }
        for (int i = 0; i < cpr.size(); i++){
            lq.rlq = lq.rlq * (1-cpr[i]);
            avcp.rcp += cpr[i];
        }
        if (cpr.size() > 0){
            avcp.rcp = (avcp.rcp/cpr.size());
        }
        for (int i = 0; i < cpe.size(); i++){
            lq.elq = lq.elq * (1-cpe[i]);
            avcp.ecp += cpe[i];
        }
        if (cpe.size() > 0){
            avcp.ecp = (avcp.ecp/cpe.size());
        }
        avcolprob.push_back(avcp);
        cpt.push_back(simTime().dbl());
    }

    else if (currentrun == ncas){
        for (it = allDrones.begin(); it != allDrones.end(); it++){
            Coord tempv ;
            Coord tmpp ;
            if (it->second->accNode){
                tempv.x = 0.0, tempv.y = 0.0, tempv.z = 0.0;
            }
            else{
               tempv = it->second->getCurrentVelocity();
            }
            tmpp = it->second->getCurrentPosition();
            double tempsp = std::sqrt( tempv.x * tempv.x + tempv.y * tempv.y + tempv.z * tempv.z);
            double thetha = anglebetweendrones(v, tempv);
            double p1 = CP2Obstacle(tmpp, p, tempv.x, v.x);
            if (p1 > 0){
                cpe.push_back(p1);
            }
            double p2 = CP2Obstacle(tmpp, p, tempv.y * cos (thetha), v.y);
            if (p2 > 0){
                cpl.push_back(p2);
            }
            double p3 = CP2Obstacle(tmpp, p, tempv.y * sin(thetha), v.z);
            if (p3 > 0){
                cpr.push_back(p3);
            }
            double p4 = CP2Obstacle(tmpp, p, tempv.z, v.z);
            if (p4 > 0){
                cpu.push_back(p4);
            }
            double p5 = CP2Obstacle(tmpp, p, tempv.z *(-1), v.z);
            if (p5 > 0){
                cpd.push_back(p5);
            }
        }
        for (int i = 0; i < cpu.size(); i++){
            lq.ulq = lq.ulq * (1-cpu[i]);
            avcp.ucp += cpu[i];
        }
        if (cpu.size() > 0){
            avcp.ucp = (avcp.ucp/cpu.size());
        }
        for (int i = 0; i < cpd.size(); i++){
            lq.dlq = lq.dlq * (1-cpd[i]);
            avcp.dcp += cpd[i];
        }
        if (cpd.size() > 0){
            avcp.dcp = (avcp.dcp/cpd.size());
        }
        for (int i = 0; i < cpl.size(); i++){
            lq.llq = lq.llq * (1-cpl[i]);
            avcp.lcp += cpl[i];
        }
        if (cpl.size() > 0){
            avcp.lcp = (avcp.lcp/cpl.size());
        }
        for (int i = 0; i < cpr.size(); i++){
            lq.rlq = lq.rlq * (1-cpr[i]);
            avcp.rcp += cpr[i];
        }
        if (cpr.size() > 0){
            avcp.rcp = (avcp.rcp/cpr.size());
        }
        for (int i = 0; i < cpe.size(); i++){
            lq.elq = lq.elq * (1-cpe[i]);
            avcp.ecp += cpe[i];
        }
        if (cpe.size() > 0){
            avcp.ecp = (avcp.ecp/cpe.size());
        }
        avcolprob.push_back(avcp);
        cpt.push_back(simTime().dbl());
    }
    else{
        double pu = 0, pd = 0, pl = 0, pe = 0, pr = 0;
        Coord tempv ;
        Coord tmpp ;
        for (it = allDrones.begin(); it != allDrones.end(); it++){
            if (it->second->accNode){
                tempv.x = 0.0, tempv.y = 0.0, tempv.z = 0.0;
            }
            else{
               tempv = it->second->getCurrentVelocity();
            }
            tmpp = it->second->getCurrentPosition();
            double tempsp = std::sqrt( tempv.x * tempv.x + tempv.y * tempv.y + tempv.z * tempv.z);
            double thetha = anglebetweendrones(v, tempv);
            double prob = CP2Obstacle(tmpp, p, tempv.x, v.x);
            if (it->second->L == 0){
                if (pd < prob){
                    pd = prob;
                }
            }
            else if (it->second->L == 1){
                double p2 = CP2Obstacle(tmpp, p, tempv.y * cos (thetha), v.y);
                if (p2 > pl){
                    pl = pe;
                }
                double p3 = CP2Obstacle(tmpp, p, tempv.y * sin(thetha), v.z);
                if (p3 > pr){
                    pr = p3;
                }
                if (prob > pe){
                    pe = prob;
                }
            }
            else{
                if (pu < prob){
                    pu = prob;
                }
            }
        }
        lq.ulq = (1-pu);
        lq.dlq = (1-pd);
        lq.llq = (1-pl);
        lq.rlq = (1-pr);
        lq.elq = (1-pe);
    }
    return lq;
}
/*Return the angle between the velocity of adjacent drones
 * cos thetha = v1*v2 / magnitude (v1) *magnitude (v2)*/
double canaapp::anglebetweendrones(Coord v1, Coord v2){
    double cs = ((v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z))/
            (std::sqrt(std::pow(v1.x, 2) + std::pow(v1.y, 2) + std::pow(v1.z, 2))*
                    std::sqrt(std::pow(v2.x, 2) + std::pow(v2.y, 2) + std::pow(v2.z, 2)));

    return std::acos(cs);
}
void canaapp::markAsReceived(uint32_t msgId)
{
    std::pair<uint32_t,bool> p(msgId,false);
    relayedMsgMap_.insert(p);
}
bool canaapp::isAlreadyReceived(uint32_t msgId)
{
    if (relayedMsgMap_.find(msgId) == relayedMsgMap_.end())
        return false;
    return true;
}
void canaapp::markAsRelayed(uint32_t msgId)
{
    relayedMsgMap_[msgId] = true;
}
bool canaapp::isAlreadyRelayed(uint32_t msgId)
{
    if (relayedMsgMap_.find(msgId) == relayedMsgMap_.end()) // the message has not been received
        return false;
    if (!relayedMsgMap_[msgId])    // the message has been received but not relayed yet
        return false;

    return true;
}
bool canaapp::isWithinBroadcastArea(Coord srcCoord, double maxRadius)
{
    Coord myCoord = ltePhy_->getCoord();
    double dist = myCoord.distance(srcCoord);
    if (dist < maxRadius)
        return true;

    return false;
}
std::string canaapp::getClusterHeadID (std::map<std::string,DroneNetMob*> Allvehicles){
    std::string CHID;
    std::map<std::string,DroneNetMob*>::iterator mit;
    if(Allvehicles.size() == 0){
        error("Empty Cluster!@");
    }
    if (Allvehicles.size() == 1){
        mit = Allvehicles.begin();
        CHID = mit->first;
    }
    else if (Allvehicles.size() == 2){
        double prevY = 0;
        for (mit = Allvehicles.begin(); mit != Allvehicles.end(); mit++){
            Coord vpos = mit->second->myposition();
            if (prevY == 0){
                prevY = vpos.y;
                CHID = mit->first;
            }
            else if (prevY > vpos.y){
                CHID = mit->first;
            }
        }
    }
    else{
        CHID = getInterUAVsAveragedistances(Allvehicles);
    }
    return CHID;
}
std::string canaapp::getInterUAVsAveragedistances(std::map<std::string,DroneNetMob*> Allvehicles){
    std::map <std::string,double> AverageDistances;
    for (std::map<std::string,DroneNetMob*>::iterator mpit1 = Allvehicles.begin(); mpit1 != Allvehicles.end(); mpit1++){
        Coord curPos = mpit1->second->myposition();
        std::vector <double> distToOtherVehs;
        for (std::map<std::string,DroneNetMob*>::iterator mpit2 = Allvehicles.begin(); mpit2 != Allvehicles.end(); mpit2++){
            if (mpit2->first != mpit1->first){
                Coord NextVehPos = mpit2->second->myposition();
                double distance = sqrt(pow(NextVehPos.x - curPos.x, 2) + pow(NextVehPos.y - curPos.y, 2) + pow(NextVehPos.z - curPos.z, 2));
                distToOtherVehs.push_back(distance);
            }
        }
        double Tot = 0;
        double AvrDist = 0;
        for (unsigned int i = 0; i < distToOtherVehs.size(); i++){
            Tot = Tot + distToOtherVehs[i];
        }
        AvrDist = Tot/distToOtherVehs.size();
        AverageDistances.insert(std::make_pair(mpit1->first,AvrDist));
    }
    std::string ClustHead = "";
    double smallAvDst = 0;
    for (std::map <std::string,double>::iterator it = AverageDistances.begin(); it != AverageDistances.end(); it++){
        if (smallAvDst == 0){
            smallAvDst = it->second;
            ClustHead = it->first;
        }
        else if (it->second < smallAvDst){
            smallAvDst = it->second;
            ClustHead = it->first;
        }
    }
    return ClustHead;
}
void canaapp::clustersUpdates(){
    std::vector<Clusterstruct> NewAllClustInfo;
    std::map <std::string, DroneNetMob*>::iterator iter;
    std::map <std::string, DroneNetMob*> dcopy;
    std::vector<std::string> clusVehs;
    dcopy = allDrones;
    clusID = 0;

    for (auto i: dcopy){
        bool beginClus = false;
        Clusterstruct tmpClust;
        Coord fpos;
        if(std::find(clusVehs.begin(), clusVehs.end(), i.first) == clusVehs.end()){
            tmpClust.cluster_Hid = i.first;
            tmpClust.clusterMembMob.insert(std::make_pair(i.first,i.second));
            tmpClust.CID = clusID + 1;
            clusVehs.push_back(i.first);
            fpos = i.second->myposition();
            i.second->isCH = false;
            beginClus = true;
            NewAllClustInfo.push_back(tmpClust);
            for (auto j:dcopy){
                if (std::find(clusVehs.begin(), clusVehs.end(), j.first) == clusVehs.end()){ //Unclustered vehicle
                    if (std::find(clusVehs.begin(), clusVehs.end(), j.first) == clusVehs.end()){
                        if (j.second != NULL){
                            Coord np = j.second->myposition();
                            double d = sqrt(pow(np.x - fpos.x, 2) + pow(np.y - fpos.y, 2) + pow(np.z - fpos.z, 2));
                            if (d <= 300 && tmpClust.clusterMembMob.size() <=10){
                                j.second->isCH = false;
                                NewAllClustInfo.back().clusterMembMob.insert(std::make_pair(j.first,j.second));
                                std::map<std::string, DroneNetMob*>  cm = NewAllClustInfo.back().clusterMembMob;
                                std::string CHID = getClusterHeadID(cm);
                                NewAllClustInfo.back().cluster_Hid = CHID;
                                clusVehs.push_back(j.first);
                                nmemb++;
                            }
                            else if (tmpClust.clusterMembMob.size() >= 10 ){
                                break;
                            }
                        }
                    }
                }
            }
            if (NewAllClustInfo.back().clusterMembMob.size() == 1){
                NewAllClustInfo.back().clusterMembMob.begin()->second->isCH = true;
            }
        }

    }
    AllClustInfo.clear();
    AllClustInfo = NewAllClustInfo;

}
Clusterstruct canaapp::returnmyCluster(){
    std::string id = getParentModule()->getFullName();
    for (auto k:AllClustInfo){
       std::map<std::string, DroneNetMob*>::iterator it;
       it = k.clusterMembMob.find(id);
       if (it != k.clusterMembMob.end()){
           return k;
           break;
       }
     }
    // Fallback/default instantiation return if conditions fail
    Clusterstruct defaultCluster;
    return defaultCluster;
}
//Added in TVT major Revision
double canaapp::getmyclusteraveragespeed(Clusterstruct cl){
    std::map<std::string, DroneNetMob*>::iterator it; //Member Vehicle Id and its corresponding mobility
    DroneNetMob* CHmob;
    std::vector<double> Clsp;
    it = cl.clusterMembMob.find(cl.cluster_Hid);
    if (it !=cl.clusterMembMob.end())
        CHmob = it->second;
    for (std::map<std::string, DroneNetMob*>::iterator it2 = cl.clusterMembMob.begin();
            it2 != cl.clusterMembMob.end(); it2++){
        double S = it2->second->sp;
        Clsp.push_back(S);
    }
    double sum = std::accumulate(Clsp.begin(),Clsp.end(),0.0);

    return sum/Clsp.size();
}
//Added in TVT major Revision
double canaapp::getmyClusterNextUpdateTime(Clusterstruct cl){
    double Cr = 500.0; //V2V communication range
    double Tm = 5.0; //minimal safe update interval
    double avsp = getmyclusteraveragespeed(cl);

    return std::max(Tm,Tm/avsp);
}
void canaapp::informmembers(){
    std::string dn = getParentModule()->getFullName();
    for (auto k:AllClustInfo){
        std::string id = getParentModule()->getFullName();
         if(std::strcmp(id.c_str(), k.cluster_Hid.c_str()) == 0){
             for (auto l:k.clusterMembMob){
                 l.second->istherecollisonrisk = true;
             }
         }
     }
}
bool canaapp::informationspanet(){
    bool allset = true;
    for (auto i: allDrones){
        if (i.second->istherecollisonrisk == false){
            allset = false;
        }
    }
    return allset;
}

void canaapp::finish()
{
    // unregister from the event generator
    if(std::strcmp(getParentModule()->getFullName(), "drone1[0]") == 0){

        double T2Inf = AccInfTime - AccidentTime;
        std::ofstream res;
        std::string dst = mymobility->n;
        std::string path = "/home/iotlab/Documents/Research-Projects/CANA/Simu5G-1.2.2/simulations/NR/drone5g/results/comdata.csv";
        res.open(path);

        res <<"Num2Acc, "
                <<"T2Acc, "
                <<"NSent: ,"
                <<"T2Inf: " <<std::endl;
        res<<ntospan<<", "
                <<timetospan <<", "
                <<nsent <<", "
                <<numrcvd <<std::endl;
        res.close();
        ofstream res2;
        std::string FPath2 = "/home/iotlab/Documents/Research-Projects/CANA/Simu5G-1.2.2/simulations/NR/drone5g/results/cp.csv";

        res2.open(FPath2);
        for (int j = 0; j <Probs.size(); j++){
            res2<<Probs[j] <<std::endl;
        }
        res2.close();

        ofstream res3;
        std::string FPath3 = "/home/iotlab/Documents/Research-Projects/CANA/Simu5G-1.2.2/simulations/NR/drone5g/results/lq.csv";
        res3.open(FPath3);
        for (int i = 0; i <laneq.size(); i++){
            res3 <<laneq[i].rlq <<", "
                    <<laneq[i].elq <<", "
                    <<laneq[i].llq <<", "
                    <<laneq[i].dlq <<", "
                    <<laneq[i].ulq <<std::endl;

        }
        res3.close();

        ofstream res4;
        std::string FPath4 = "/home/iotlab/Documents/Research-Projects/CANA/Simu5G-1.2.2/simulations/NR/drone5g/results/avcp.csv";

        res4.open(FPath4);

        for (auto i: avcolprob){
            res4 <<i.ecp<<", "
                     <<i.lcp <<", "
                     <<i.rcp<<", "
                     <<i.rcp<<", "
                     <<i.dcp <<std::endl;
        }
        res4.close();
    }
//
//    close(client_fd);
//    close(server_fd);
}
void canaapp::sendPeriodicPacket() {
    auto payload = makeShared<ByteCountChunk>(B(msgSize_));
    Packet *packet = new Packet("CANAPacketData", payload);
    totalPacketsSent++;
    emit(canaSentMsg_, totalPacketsSent);
    droneUdpSocket.sendTo(packet, destAddress_, destPort_);
}
void canaapp::sendStateToGym()
{
    if (rlsocket.getState() == TcpSocket::CONNECTED) {
            std::string payloadStr = std::to_string(totalPacketsSent) + "," +
                                     std::to_string(totalPacketsReceived) + "," +
                                     std::to_string(totalPacketsDropped) + "\n";

            auto payloadChunk = makeShared<ByteCountChunk>(B(payloadStr.length()));
            Packet *gymStatePacket = new Packet("GymStateTelemetry", payloadChunk);
            rlsocket.send(gymStatePacket);
    }
}

// 2. Data Arrived
void canaapp::socketDataArrived(inet::TcpSocket *socket, inet::Packet *packet, bool urgent) {
    auto bytesChunk = packet->peekDataAsBytes();
    const std::vector<uint8_t>& rawByteVector = bytesChunk->getBytes();
    std::string compositeActionString(rawByteVector.begin(), rawByteVector.end());
    delete packet;

    std::string purifiedActionString = "";
    for (char characterElement : compositeActionString) {
        if (isdigit(characterElement) || characterElement == '.') {
            purifiedActionString += characterElement;
        }
    }

    if (!purifiedActionString.empty()) {
        double parsedIntervalDoubleValue = std::stod(purifiedActionString);
        currentSendInterval_ = parsedIntervalDoubleValue;
        EV_INFO << "Gymnasium action ingested successfully. Period updated to: " << currentSendInterval_ << "s\n";
    }
}

void canaapp::socketEstablished(TcpSocket *socket) { EV_INFO << "Control plane TCP linked to Gym.\n"; }
void canaapp::socketClosed(TcpSocket *socket) { EV_INFO << "Control plane closed.\n"; }
void canaapp::socketFailure(TcpSocket *socket, int code) { EV_INFO << "Control plane socket failure: " << code << "\n"; }
void canaapp::socketPeerClosed(TcpSocket *socket) {}
void canaapp::socketStatusArrived(TcpSocket *socket, TcpStatusInfo *status) {}
void canaapp::socketAvailable(TcpSocket *socket, TcpAvailableInfo *availableInfo) {}
void canaapp::socketDeleted(TcpSocket *socket) {}
