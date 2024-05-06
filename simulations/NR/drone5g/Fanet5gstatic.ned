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
package simu5g.simulations.NR.drone5g;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.ethernet.Eth10G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import simu5g.common.binder.Binder;
import simu5g.world.radio.LteChannelControl;
import simu5g.nodes.backgroundCell.BackgroundCell;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf;
import simu5g.nodes.NR.NRUe;
import simu5g.nodes.NR.gNodeB;
import simu5g.cana.Destination;
import simu5g.nodes.PgwStandard;
import simu5g.cana.MBase;
import simu5g.cana.fivegdrone;
//import simu5g.apps.canaApp.canaEventGenerator.canaEventGenerator;
//import simu5g.apps.canaApp.statistics.CANAStatistics;



network Fanet5gstatic 
{
    parameters:
//		int numdrones = default(10);
		int numdst;
		int dn;
        @display("i=block/network2;bgb=991,558;bgi=background/budapest");
    submodules:
        routingRecorder: RoutingTableRecorder {
            @display("p=50,75;is=s");
        }
    	configurator: Ipv4NetworkConfigurator {
            @display("p=90.79375,121.8875");
        }
   		channelControl: LteChannelControl {
           	@display("p=50,25;is=s");
       	}
        binder: Binder {
            @display("p=50,175;is=s");
        }
        carrierAggregation: CarrierAggregation {
            @display("p=50.993748,258.7;is=s");
        }
        gcs: StandardHost {
			@display("p=660,136;is=n;i=device/server");  
        }
        router: Router {
            @display("p=561,135;i=device/smallrouter");
        }
        upf: Upf {
            @display("p=462,136;is=l");
        }
        gNodeB1: gNodeB {
            @display("p=156,136;is=vl");
        }
        gNodeB2: gNodeB {
            @display("p=391,313;is=vl");
        }
		dst[numdst]: Destination {
            @display("i=misc/town;r=,,#707070;p=91,380");
        }
        bstion[4]: MBase{
        	 @display("i=dronesim/bstation");
        }
		drone1[dn]: fivegdrone {
			parameters:
			@display("p=300,300;r=,,#707070");
        }
        drone2[dn]: fivegdrone {
			parameters:
			@display("p=300,300;r=,,#707070");
        }
        drone3[dn]: fivegdrone {
			parameters:
			@display("p=300,300;r=,,#707070");
        }
        drone4[dn]: fivegdrone {
			parameters:
			@display("p=300,300;r=,,#707070");
        }
//		canaEventGenerator: canaEventGenerator {
//            @display("p=90.79375,222.63124;is=s");
//        }
//    	canaStatistics: CANAStatistics {
//            @display("p=90.79375,272.38126;is=s");
//        }
    connections:
		gcs.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        upf.pppg++ <--> Eth10G <--> gNodeB1.ppp;
        upf.pppg++ <--> Eth10G <--> gNodeB2.ppp;
        //# X2 connections
        gNodeB1.x2++ <--> Eth10G <--> gNodeB2.x2++;               
}
