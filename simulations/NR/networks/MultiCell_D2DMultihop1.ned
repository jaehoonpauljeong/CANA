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
package simu5g.simulations.NR.networks;

import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.ethernet.Eth10G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import simu5g.world.radio.ChannelControl;
import simu5g.common.binder.Binder;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.NR.gNodeB;
import simu5g.nodes.NR.NRUe;
import simu5g.nodes.Upf;
//import simu5g.nodes.Ue;
//import simu5g.nodes.eNodeB;
import simu5g.nodes.PgwStandard;
import simu5g.world.radio.LteChannelControl;
import simu5g.apps.d2dMultihop.eventGenerator.EventGenerator;
import simu5g.apps.d2dMultihop.statistics.MultihopD2DStatistics;

network MultiCell_D2DMultihop1
{
    parameters:
        int numUe1 = default(1);
        int numUe2 = default(1);
        int numUe3 = default(1);
        int numUe4 = default(1);
        int numUe5 = default(1);
        @display("i=block/network2;bgb=2000,500;");
    submodules:
        channelControl: LteChannelControl {
            @display("p=89.55,24.875;is=s");
        }
        routingRecorder: RoutingTableRecorder {
            @display("p=90.79375,70.89375;is=s");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=90.79375,121.8875");
        }
        binder: Binder {
            @display("p=90.79375,171.6375;is=s");
        }
        carrierAggregation: CarrierAggregation {
            @display("p=50.993748,258.7;is=s");
        }
        server: StandardHost {
            @display("p=500,50;is=n;i=device/server");
        }
        router: Router {
            @display("p=750,50;i=device/smallrouter");
        }
        router1: Router {
            @display("p=199,194.025;i=device/smallrouter");
        }
        router2: Router {
            @display("p=599.4875,194.025;i=device/smallrouter");
        }
        router3: Router {
            @display("p=999.975,194.025;i=device/smallrouter");
        }
        router4: Router {
            @display("p=1399.2188,194.025;i=device/smallrouter");
        }
        router5: Router {
            @display("p=1799.7062,194.025;i=device/smallrouter");
        }
        upf: Upf {
            @display("p=1000,50;is=l");
        }
        gNodeB1: gNodeB {
            @display("p=200,400;is=vl");
        }
        gNodeB2: gNodeB {
            @display("p=600,400;is=vl");
        }
        gNodeB3: gNodeB {
            @display("p=1000,400;is=vl");
        }
        gNodeB4: gNodeB {
            @display("p=1400,400;is=vl");
        }
        gNodeB5: gNodeB {
            @display("p=1800,400;is=vl");
        }
        ue1[numUe1]: NRUe {
            @display("p=300,450;is=s");
        }
        ue2[numUe2]: NRUe {
            @display("p=700,450;is=s");
        }
        ue3[numUe3]: NRUe {
            @display("p=1100,450;is=s");
        }
        ue4[numUe4]: NRUe {
            @display("p=1500,450;is=s");
        }
        ue5[numUe5]: NRUe {
            @display("p=1900,450;is=s");
        }
        eventGenerator: EventGenerator {
            @display("p=90.79375,222.63124;is=s");
        }
        d2dMultihopStatistics: MultihopD2DStatistics {
            @display("p=90.79375,272.38126;is=s");
        }
    connections:
        router1.pppg++ <--> Eth10G <--> gNodeB1.ppp;
        router2.pppg++ <--> Eth10G <--> gNodeB2.ppp;
        router3.pppg++ <--> Eth10G <--> gNodeB3.ppp;
        router4.pppg++ <--> Eth10G <--> gNodeB4.ppp;
        router5.pppg++ <--> Eth10G <--> gNodeB5.ppp;
        router1.pppg++ <--> Eth10G <--> upf.pppg++;
        router2.pppg++ <--> Eth10G <--> upf.pppg++;
        router3.pppg++ <--> Eth10G <--> upf.pppg++;
        router4.pppg++ <--> Eth10G <--> upf.pppg++;
        router5.pppg++ <--> Eth10G <--> upf.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        server.pppg++ <--> Eth10G <--> router.pppg++;
}
