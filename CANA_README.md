############# CANA (Collision-Aware Navigation Algorithm ) ##############



Introduction on CANA
____________________

In this project,  we implement the CANA simulation. Is is a mechanism that we developed to enhance the safety of Urban Air Mobility (UAM).
CANA senses UAV flying environment, study the collision risks and plans the safe trajectory of a flying UAV through maneuvers in virtual lanes.
CAN is simulated using OMNeT++ and extended the SIMU5GSUMO for its application and communication.




Simulation Environnement
_________________________

To run this simulation requires:
- Install OMNeT++ 6.0.1 in lunix environment
- inet-4.5.0-3833582230



Necessary Modifications
_______________________
Importing this modified simu5G project in your OMNeT++ workspace or extract it to run it in command line environment.


Running CANA
___________________

Using OMNeT++ workspace environment

- Now in OMNETpp environment, compile and run (/simu5G/simulations/NR/drone5g/omnetpp.ini).

Using command Line:

- in simu5G (. setenv)
- in your terminal, type: make, to compile
- run the simulation by: ./run



Hope you enjoy our CANA Simulation in OMNeT++
