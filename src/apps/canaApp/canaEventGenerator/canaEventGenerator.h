/*
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

#ifndef _NR_CANAEVENTGENERATOR_H_
#define _NR_CANAEVENTGENERATOR_H_

#include <string.h>
#include <omnetpp.h>

#include "../canaapp.h"
#include "common/binder/Binder.h"
#include "stack/phy/layer/LtePhyEnb.h"

class canaapp;

class canaEventGenerator : public omnetpp::cSimpleModule
{
    omnetpp::cMessage *selfMessage_;

    uint32_t eventId_;
    Binder* binder_;

    bool singleEventSource_;

    // store references to the app modules
    std::vector<canaapp*> appVector_;

    // store LTE IDs of the nodes
    std::set<MacNodeId> lteNodeIdSet_;

    // store references to the PHY modules
    // (to speed up position retrieval)
    std::map<MacNodeId, LtePhyBase*> lteNodePhy_;

    // notify a node to start an event dissemination
    void notifyEvent();

protected:

    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;

  public:
    canaEventGenerator();
    ~canaEventGenerator();

    void computeTargetNodeSet(std::set<MacNodeId>& targetSet, MacNodeId sourceId, double maxBroadcastRadius = -1.0);
    void registerNode(canaapp* app, MacNodeId lteNodeId);
    void unregisterNode(canaapp* app, MacNodeId lteNodeId);
};

#endif

*/
