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

package simu5g.apps.canaApp;

import inet.applications.contract.IApp;

// CANAAPP
//
// This module implements an application for disseminating event's notificaitons,
// using multihop D2D transmissions. New messages are generated upon the reception 
// of external events, signaled by the EventGenerator module. The application
// sends the message to an IP multicast address, which is then mapped to D2D
// multicast transmission mode. When the application receives a message, it can
// relay it to the same IP multicast address, in order to extend the coverage
// of the D2D transmission. 
// To avoid flooding, two mechanims can be used:
// - specifying a time-to-live (TTL) for the message, which is decreased every time
//   the message is relayed;
// - specifying a target broadcast radius for the message: a node relays the message
//   only if it is located within a configurable distance from the originator.  
//
// Moreover, if Trickle suppression mechanism is enabled, the application avoids
// to relay a message when more than k duplicates have already been received. 
//
// Every message originated by a node is associated to a *target area*, i.e. the 
// set of nodes that should receive the notification. This is used for gathering
// statistics about the dissemination of the message.
//
// ref: G. Nardini, G. Stea, A. Virdis, "A fast and reliable broadcast service 
//      for LTE-Advanced exploiting multihop device-to-device transmissions", 
//      Future Internet, 2017, 9(4), 89
//
simple canaapp like IApp
{
    parameters:
        int localPort = default(3000);
        int destPort = default(3000);
        string destAddress = default("0.0.0.0");
        string interfaceName = default("cellular"); 
        int msgSize = default(40);
        double maxBroadcastRadius  @unit(m) = default(-1m);
        int ttl = default(3);        
        double maxTransmissionDelay @unit("s") = default(0.01s);
        double selfishProbability = default(0.0);  // probability that the app does not relay the received message  
           
        //# Trickle suppression mechanism parameters
        bool trickle = default(false);
        double I @unit("s") = default(0.01s);
        int k = default(3);
        
        //# Network Layer specs
        string interfaceTableModule = default(absPath(".interfaceTable"));
        
        int tos = default(-1); // if not -1, set the Type Of Service (IPv4) / Traffic Class (IPv6) field of sent packets to this value
        
        //# Statistics
        @signal[CANAGeneratedMsg];
        @statistic[CANAGeneratedMsg](title="Number of locally generated packets"; unit=""; source="CANAGeneratedMsg"; record=sum,vector);
        @signal[CANASentMsg];
        @statistic[CANASentMsg](title="Number of sent/relayed packets"; unit=""; source="CANASentMsg"; record=sum,vector);
        @signal[CANARcvdMsg];
        @statistic[CANARcvdMsg](title="Number of received packets"; unit=""; source="CANARcvdMsg"; record=sum,vector);
        @signal[CANARcvdDupMsg];
        @statistic[CANARcvdDupMsg](title="Number of received duplicate packets"; unit=""; source="CANARcvdDupMsg"; record=sum,vector);

        @signal[CANATrickleSuppressedMsg];
        @statistic[CANATrickleSuppressedMsg](title="Number of suppressed packets by the Trickle suppression mechanism"; unit=""; source="CANATrickleSuppressedMsg"; record=sum,vector);
        
        @display("i=block/source");
    gates:
        output socketOut;
        input socketIn;
}