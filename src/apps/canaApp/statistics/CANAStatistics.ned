//
//                  Simu5G
//
// Authors: Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
//
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself,
// and cannot be removed from it.

package simu5g.apps.canaApp.statistics;

//
// CANAStatistics module
//
// This module is only used to gather statistics related to events generated
// by the CANA apps. It is accessed via method calls.
//
simple CANAStatistics {
  
  parameters:

    @signal[CANAEventDeliveryRatio];
    @statistic[CANAEventDeliveryRatio](title="Avg delivery ratio within the area"; unit=""; source="CANAEventDeliveryRatio"; record=mean,sum,min,max);   
    @signal[CANAEventDelay];
    @statistic[CANAEventDelay](title="Avg delay of broadcasting within the area"; unit=""; source="CANAEventDelay"; record=mean,sum,min,max);    
    @signal[CANAEventDelay95Per];
    @statistic[CANAEventDelay95Per](title="95th percentile of the delay of broadcasting within the area"; unit=""; source="CANAEventDelay95Per"; record=mean,sum,min,max);    
    @signal[CANAEventSentMsg];
    @statistic[CANAEventSentMsg](title="Avg transmissions for broadcasting a message"; unit=""; source="CANAEventSentMsg"; record=mean,sum,min,max);  
    @signal[CANAEventTrickleSuppressedMsg];
    @statistic[CANAEventTrickleSuppressedMsg](title="Avg suppressed transmissions for broadcasting a message"; unit=""; source="CANAEventTrickleSuppressedMsg"; record=mean,sum,min,max);  
    @signal[CANAEventRcvdDupMsg];
    @statistic[CANAEventRcvdDupMsg](title="Avg num of duplicates within the area"; unit=""; source="CANAEventRcvdDupMsg"; record=stats);   
    @signal[CANAEventCompleteDeliveries];
    @statistic[CANAEventCompleteDeliveries](title="Avg complete-deliveries within the area"; unit=""; source="CANAEventCompleteDeliveries"; record=stats);   
    
    
    @display("i=block/cogwheel");
}