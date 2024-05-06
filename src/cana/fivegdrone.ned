//
// Copyright (C) 2014 OpenSim Ltd.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

package simu5g.cana;
import simu5g.nodes.Ue;
import simu5g.stack.NRNicUe;
import simu5g.corenetwork.statsCollector.NRueStatsCollector;
import simu5g.cana.DroneNetMob;

//
//Drone Module With 5G protocol stack
//
module fivegdrone extends Ue
{
    parameters:
        @display("i=dronesim/drone");
        nicType = default("NRNicUe");
        int nrMasterId @mutable = default(0);
        int nrMacNodeId @mutable = default(0); // TODO: this is not a real parameter
        int nrMacCellId @mutable = default(0); // TODO: this is not a real parameter
        string mbase;
//		double weightCapacity = default(10);
//		double batteryCapacity = default(20);
//		int parcelSelecitionMethod = default(2); 
        //# Mobility
    	mobility.typename = default("DroneNetMob");
        
    gates:
        input nrRadioIn @directIn;     // connection to master   
        
	connections allowunconnected:

        cellularNic.nrRadioIn <-- nrRadioIn;
}
