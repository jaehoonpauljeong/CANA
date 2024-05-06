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
//        std::cout << "canaapp initialize: stage " << stage << endl;
        EV << "canaapp initialize: stage " << stage << endl;

        localPort_ = par("localPort");
        destPort_ = par("destPort");
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
        currentrun = sens;
        if (trickleEnabled_)
        {
            I_ = par("I");
            k_ = par("k");
            if (k_ <= 0)
                throw cRuntimeError("Bad value for k. It must be greater than zero");
        }

        EV << "canaapp::initialize - binding to port: local:" << localPort_ << " , dest:" << destPort_ << endl;
        socket.setOutputGate(gate("socketOut"));
        socket.bind(localPort_);

        int tos = par("tos");
        isdynamicObstacle = true;
        if (tos != -1)
            socket.setTos(tos);

        // for multicast support
        inet::IInterfaceTable *ift = inet::getModuleFromPar< inet::IInterfaceTable >(par("interfaceTableModule"), this);
        NetworkInterface *ie = ift->findInterfaceByName(par("interfaceName").stringValue());
        if (!ie)
            throw cRuntimeError("Wrong multicastInterface setting: no interface found");
        inet::MulticastGroupList mgl = ift->collectMulticastGroups();
        socket.joinLocalMulticastGroups(mgl);
        socket.setMulticastOutputInterface(ie->getInterfaceId());
        // -------------------- //

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
    }
}

void canaapp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage())
    {
        if (!strcmp(msg->getName(), "selfSender")){
            /*std::cout <<"11111111111  " <<msg->getName() << " <--> Sent by "  <<getParentModule()->getFullName()
                    << " <--> Time = " << simTime()<< std::endl;*/
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
/*            std::cout <<"333333333333  " <<msg->getName() <<std::endl;*/
            handleTrickleTimer(msg);
        }
        else if(strcmp(msg->getName(), "Reclustering") == 0){
/*            std::cout <<"444444444444  " <<msg->getName() <<std::endl;*/
             clustersUpdates();
             if (reclustering->isScheduled()){
                 cancelEvent(reclustering);
             }
             scheduleAt(simTime() + 5, reclustering);
         }
        else if(strcmp(msg->getName(), "Accident") == 0){
            obstactive = true;
            if (isdynamicObstacle){
                isccmsg = false;
                std::ofstream pos;
                double AccidentTime = simTime().dbl();
                double AccInfTime = simTime().dbl();
                mymobility->setNextChange(simTime() + 30);
                mymobility->istherecollisonrisk = true;
                mymobility->accNode = true;
                double H =mymobility->returnHight();
                double EDT =sqrt((2*H)/9.8);//Consigering the gravity = 9.8
                obh = H;
                obt = EDT;
                std::cout <<" ---> H = " <<H << ";  EDT = " <<EDT <<std::endl;
                accT = simTime().dbl();
                pos.open("/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/Positions.cvs");
                //            obPos = mymobility->myposition();
                Coord curp = mymobility->getCurrentPosition();
                obPos.push_back(curp);
//                error("Close");
            }
            else{
    /*            std::cout <<"55555555555555  " <<msg->getName() <<std::endl;*/
                isccmsg = false;
                std::ofstream pos;
                double AccidentTime = simTime().dbl();
                double AccInfTime = simTime().dbl();
                mymobility->setNextChange(simTime() + 30);
                mymobility->istherecollisonrisk = true;
                mymobility->accNode = true;
                pos.open("/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/Positions.cvs");
    //            obPos = mymobility->myposition();
                Coord curp = mymobility->getCurrentPosition();
                obPos.push_back(curp);
                for (auto k:allDrones){
                    Coord p = k.second->myposition();
    //                Coord p = k.second->getCurrentPosition();
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
//                std::cout <<"66666666666666666  " <<msg->getName() << "obPos sz = " <<obPos.size() << std::endl;
                for (unsigned int i=0; i<obPos.size(); i++){
                    obPos[i].z  =obPos[i].z - (elT *9.8);
                    double cp = CP2Obstacle(obPos[i], mymobility->myposition(), 0, mymobility->sp);
                    if (cp > 0){
                        std::cout <<"Prob = " <<cp <<std::endl;
                        Probs.push_back(cp);
                        lanequality lq = compute3dlanequality() ;
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
/*            std::cout <<"777777777777777  " <<msg->getName() <<std::endl;*/
            if (mymobility->isCH){
//               std::cout <<"###########" <<std::endl;
                relayPacket(msg);
            }
        }
        else{
//            std::cout <<"8888888888888888888  " <<msg->getName() <<std::endl;
            std::cout <<"--- This  --- " <<msg->getName() <<std::endl;
            throw cRuntimeError("Unrecognized self message");}
    }
    else
     {
         if (!strcmp(msg->getName(), "ccm") || !strcmp(msg->getName(), "ecm")){
             handleRcvdPacket(msg);
         }
         else{
             std::cout <<"*********** " <<msg->getName() <<std::endl;
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
//        std::cout <<"@@@@@@@@@@@@  by " <<getParentModule()->getFullName() <<std::endl;
        packet = new inet::Packet("ccm", data);
    }
    else{
        nsent++;
//        std::cout <<"############  by " <<getParentModule()->getFullName() <<std::endl;
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
//    std::cout << "canaapp::sendPacket - Sending msg (ID: "<< mhop->getMsgid() << " src: " << lteNodeId_ << " size: " << msgSize_ << ")" <<  endl;

    socket.sendTo(packet, destAddress_, destPort_);

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
//    std::cout << "canaapp::handleRcvdPacket - Received packet from lower layer:: Name : " <<msg->getName() << endl;

    Packet* pPacket = check_and_cast<Packet*>(msg);

    if(strcmp(msg->getName(), "ecm") == 0){
        numrcvd++;
        double cT = simTime().dbl();

        elT = cT - accT;
        for (auto i : obPos){
            i.z  =i.z - (elT *9.8);
            double cp = CP2Obstacle(i, mymobility->myposition(), 0, mymobility->sp);
            if (cp > 0){
                std::cout <<"ECM Rcv by " << getParentModule()->getFullName() << "CP = " << cp <<std::endl;
                Probs.push_back(cp);
                if (cp > 0){
                    lanequality lq = compute3dlanequality() ;
                    std::cout << " Lql = " <<lq.llq <<" Lqr = " <<lq.rlq << "Lqe = " <<lq.elq <<" Lqu = " <<lq.ulq <<" Lqd = " <<lq.dlq <<std::endl;
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
                std::cout <<" Time To Acc = " << timetospan <<"  && Npkt = " <<ntospan <<std::endl;
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
//    std::cout <<"Receiver = " <<getParentModule()->getFullName() <<"|| PkID = " <<msgId <<std::endl;

    // check if this is a duplicate
    if (isAlreadyReceived(msgId))
    {
//        std::cout <<"aaaaaaaaaaaaaaaa" <<std::endl;
        if (trickleEnabled_)
            counter_[msgId]++;

        // do not need to relay the message again
        EV << "canaapp::handleRcvdPacket - The message has already been received, counter = " << counter_[msgId] << endl;
//        std::cout << "canaapp::handleRcvdPacket - The message has already been received, counter = " << counter_[msgId] << "msgId = " <<msgId <<endl;

 /*       emit(canaRcvdDupMsg_, (long)1);
        stat_->recordDuplicateReception(msgId);*/
        delete pPacket;
    }
    else
    {
//        std::cout <<"bbbbbbbbbbbbbbbbb" <<std::endl;
        // this is a new packet

        // mark the message as received
        markAsReceived(msgId);

        if (trickleEnabled_)
        {
//            std::cout <<"cccccccccccccccccccc" <<std::endl;
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
//            std::cout <<"eeeeeeeeeeeeeee" <<std::endl;
            // selfish user, do not relay
            EV << "canaapp::handleRcvdPacket - The user is selfish, do not forward the message. " << endl;
            delete pPacket;
        }
        else if (mhop->getMaxRadius() > 0 && !isWithinBroadcastArea(mhop->getSrcCoord(), mhop->getMaxRadius()))
        {
//            std::cout <<"ffffffffffffffffffffffffffff" <<std::endl;
            EV << "canaapp::handleRcvdPacket - The node is outside the broadcast area. Do not forward it. " << endl;
            delete pPacket;
        }
        else if (mhop->getTtl() == 0)
        {
            // TTL expired
//            std::cout <<"ggggggggggggggggggggggg" <<std::endl;
            EV << "canaapp::handleRcvdPacket - The TTL for this message has expired. Do not forward it. " << endl;
            delete pPacket;
        }
        else
        {
            if (trickleEnabled_)
            {
                // start Trickle interval timer
//                std::cout <<"hhhhhhhhhhhhhhhhhhhhhhhhhh" <<std::endl;
                canaTrickleTimerMsg* timer = new canaTrickleTimerMsg("trickleTimer");
                timer->setMsgid(msgId);

                simtime_t t = uniform(I_/2 , I_);
                t = round(SIMTIME_DBL(t)*1000)/1000;
                EV << "canaapp::handleRcvdPacket - start Trickle interval, duration[" << t << "s]" << endl;
//                std::cout  << "canaapp::handleRcvdPacket - start Trickle interval, duration[" << t << "s]" << endl;

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
    //                     std::cout << "canaapp::handleRcvdPacket - will relay the message in " << offset << "s" << endl;

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
    std::cout <<"---  canaapp::relayPacket" << msg->getName() <<std::endl;
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
//    Packet* relayPacket = new inet::Packet("canaPacket", data);

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
    std::cout << "canaapp::relayPacket - Relay msg " << dst->getMsgid() << " to address " << destAddress_ << endl;

    socket.sendTo(relayPacket, destAddress_, destPort_);

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
/*        std::cout <<"<<< Setting CCM >>>> "<<std::endl;*/
//        receivedccms++;
//        updateneigbours(ccmpkt);
    }
    else {
        error("Unkown CANA Pkt");
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
//    std::cout <<"T2C = " << t2c << " Min = "<< mymobility->minT2CThresh<<"Max = "<<mymobility->maxT2CThresh<< std::endl;
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
//    std::cout <<"!!!!!!  Prob = " <<vcp;
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
lanequality canaapp::compute3dlanequality(){
    std::vector<double> cpl, cpr, cpe, cpu, cpd;
    double lav = 0.0, rav = 0.0, eav = 0.0, uav = 0.0, dav = 0.0;
    std::map <std::string, DroneNetMob*>::iterator it;
    Coord p = mymobility->getCurrentPosition();
    Coord v = mymobility->getCurrentVelocity();
    double sp = std::sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
//    std::cout <<"Pos: ("<<p.x <<"; "<<p.y <<"; "<<p.z <<") : Vel (" <<v.x <<"; "<<v.y <<"; "<<v.z <<")";
    lanequality lq;
    laneCP avcp;
    if (currentrun == cana){
//        std::cout <<"111111111111" <<std::endl;
        Clusterstruct myClus =  returnmyCluser();
        for (auto i: myClus.clusterMembMob){
//            std::cout <<"2222222222222222" <<std::endl;
            Coord tempv ;
            Coord tmpp ;
            if (i.second->accNode){
                tempv.x = 0.0, tempv.y = 0.0, tempv.z = 0.0;
            }
            else{
                tempv = i.second->getCurrentVelocity();
            }
//            std::cout <<"33333333333333" <<std::endl;

            tmpp = i.second->getCurrentPosition();
            double tempsp = std::sqrt( tempv.x * tempv.x + tempv.y * tempv.y + tempv.z * tempv.z);
            double thetha = anglebetweendrones(v, tempv);
//            std::cout <<"Pos2: ("<<tmpp.x <<"; "<<tmpp.y <<"; "<<tmpp.z <<") : Vel2 (" <<tempv.x <<"; "<<tempv.y <<"; "<<tempv.z <<")";
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
//            std::cout <<"Pos2: ("<<tmpp.x <<"; "<<tmpp.y <<"; "<<tmpp.z <<") : Vel2 (" <<tempv.x <<"; "<<tempv.y <<"; "<<tempv.z <<")";
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
//        std::cout <<"----------------------- allDrones size = " <<allDrones.size() <<std::endl;
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
//            std::cout <<"Probability === " <<prob <<std::endl;
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
//        std::cout <<"Lqs:: u=" << lq.ulq <<"; d="<<lq.dlq <<"; l=" <<lq.rlq <<"; e="<<lq.elq<<std::endl;
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
//    std::cout <<"Already Relayed? -- msgId => "<< msgId<< "  RMSze = " <<relayedMsgMap_.size() <<std::endl;
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
//   std::cout <<"+++  Size drones = " <<dcopy.size() << "|| N Clusters = " << AllClustInfo.size () << std::endl;

   /*    for (auto k:AllClustInfo){
        std::cout <<"*** CH *** "<<k.cluster_Hid <<" Members: +++ ";
        for (auto l:k.clusterMembMob){
            std::cout <<l.first <<" : ";
        }
        std::cout <<std::endl;
    }*/

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
//                std::cout<<"7777777777777777777777" <<std::endl;
                NewAllClustInfo.back().clusterMembMob.begin()->second->isCH = true;
            }
        }

    }
    AllClustInfo.clear();
    AllClustInfo = NewAllClustInfo;


//    std::cout <<"New Clusters Size : "<<AllClustInfo.size() <<std::endl;
    /* for (auto i: AllClustInfo){
        std::cout <<"CH : " <<i.cluster_Hid <<" <++++> Members: ";
        for (auto j : i.clusterMembMob){
            std::cout <<j.first <<" : ";
        }
        std::cout<<std::endl;
    }*/
}
Clusterstruct canaapp::returnmyCluser(){
    std::string id = getParentModule()->getFullName();
//    std::cout<<"!!!!!!!!!!!!!!" <<std::endl;
    for (auto k:AllClustInfo){
       std::map<std::string, DroneNetMob*>::iterator it;
       it = k.clusterMembMob.find(id);
       if (it != k.clusterMembMob.end()){
//           std::cout<<"@@@@@@@@@@@@@@@@@@@@2" <<std::endl;
           return k;
           break;
       }
     }
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
//   eventGen_->unregisterNode(this, lteNodeId_);
//   std::cout <<"Num of sent: " <<npkt <<" N Received = " <<nrcvd <<std::endl;
    if(std::strcmp(getParentModule()->getFullName(), "drone1[0]") == 0){
//        std::cout<<" app finish!!!!!!!!!!!!!" <<std::endl;
        std::cout <<" Num Sent = " <<nsent <<" && Num Received = " << numrcvd <<std::endl;

        double T2Inf = AccInfTime - AccidentTime;
        std::ofstream res;
        std::string dst = mymobility->n;
        std::string path = "/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/comdata.csv";
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

//        std::cout <<" Mob finish !!!!   " ;
       std::cout << Probs.size() <<" && " <<laneq.size() <<std::endl;
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

        ofstream res4;
        std::string FPath4 = "/home/iotlab/Documents/DroneNavigation/Simu5G-1.2.2/simulations/NR/drone5g/results/avcp.csv";

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
}
