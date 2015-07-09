function getFalconInput()
if ~exist('falcon','var')
    % Server Connection
    falcon = RobotRaconteur.Connect('tcp://localhost:2354/falconServer/Falcon');
end

falconInput = falcon.controller_input;
Input = zeros(3,1);
Input(1,1) = falconInput.positionX;
Input(2,1) = falconInput.positionY;
Input(3,1) = falconInput.positionZ;

disp(Input)

end