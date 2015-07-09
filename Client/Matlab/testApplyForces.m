%REMEMBER YOU MUST BE ON THE DUALARMLAB NETWORK
falcon = RobotRaconteur.Connect('tcp://localhost:2354/falconServer/Falcon');
FTService = RobotRaconteur.Connect('tcp://localhost:5300/FTSensors/FT_Service');

ft = FTService.get_ft(0);

initialWrench = ft.wrench;
initialForces = initialWrench(1:3);

startTime = tic;
while toc(startTime) < 10 %[s]
    wrench = ft.wrench;
    forces = wrench(1:3)-initialForces;
    
    %transform to falcon Coord sys by rotating about the z-axis 90 degrees
    R = [0 -1 0 ; 1  0 0 ; 0 0 1];
    falconForces = R * forces * -1; % we want the reactive force, not the applied force
    
    falcon.setForce(falconForces);
end

falcon.setForce(zeros(3,1));

    