############# CANA: Collision-Avoidance Navigation Algorithm for UAV Flying Networks with Heavy Traffic##############

Introduction on CANA
______________________________
In this project, we implement the CANA simulation. We developed a mechanism to enhance the safety of Urban Air Mobility (UAM). CANA senses the UAV flying environment, studies the collision risks, and plans the safe trajectory of a flying UAV through maneuvers in virtual lanes. CAN is simulated using OMNeT++ and extended the SIMU5GSUMO for its application and communication.

We conducted a simulation of a UAV network in the OMNeT++ simulation framework which adopted a 5G simulation of the 5G New Radio User Plane Simulation Model
(Simu5G) for INET & OMNeT++, and adapted it for use with UAVs specifically.

Related Reference documents:

- G. Nardini, D. Sabella, G. Stea, P. Thakkar, A. Virdis "Simu5G – An OMNeT++ library for end-to-end performance evaluation of 5G networks", IEEE Access (2020), DOI: 10.1109/ACCESS.2020.3028550.
- Bien Aime Mugabarigira and Jaehoon (Paul) Jeong, "Context-Aware Navigation Protocol for Safe Flying of Unmanned Aerial Vehicles", KICS-2024-Winter, Pyeongchang, Korea, January 31 to February 2, 2024.
- Context-Aware Navigation Protocol for IP-Based Vehicular Networks, URL:https://datatracker.ietf.org/doc/draft-jeong-ipwave-context-aware-navigator/,
- Basic Support for IPv6 Networks Operating over 5G Vehicle-to-Everything Communications, URL:https://datatracker.ietf.org/doc/draft-jeong-6man-ipv6-over-5g-v2x/.

Simulation Environment
________________________
To run this simulation requires:
- Install OMNeT++ 6.0.1 in Lunix environment
- inet-4.5.0-3833582230

Necessary Modifications
_________________________
Importing this modified simu5G project in your OMNeT++ workspace or extract it to run it in a command line environment.

Running CANA
_______________
- Using OMNeT++ workspace environment
    Now in OMNETpp environment, compile and run (/simu5G/simulations/NR/drone5g/omnetpp.ini).

- Using the command Line:
    in simu5G (. setenv)
    in your terminal, type: make, to compile
    and simulate by ./run
    
We hope you enjoy our CANA Simulation in OMNeT++!
