function handle = createJammster(varargin)
    % 
    % handle = createJammster(...)
    %
    % File to create a Jammster drawing object.  The Jammster robot 
    % consists of a Baxter robot 
    %
    %
    % Optional parameters for initialization are:
    %       'CreateFrames'      default: 'off'
    %       'Origin'            default: [eye(3) [0;0;0]; [0 0 0] 1]
    %
    % depends on the following drawing package files:
    %       createBaxter.m
    %       createCuboid.m
    %       createCylinder.m
    %       createRobot.m
    %       attachPrefix.m
    %       attachObjectToRobot.m
    %
    % returns handle to the jammster structure containing three robot
    % structures:
    %   left_arm
    %   right_arm
    %   head
    %
    % see also CREATEBAXTER, CREATEROBOT

    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];
    
    % Walk through varargin
    for i=1:2:(nargin-1)
        if strcmp(varargin{i},'CreateFrames')
            cf = varargin{i+1};
        elseif strcmp(varargin{i},'Origin')
            origin = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Default settings to optional parameters
    if ~exist('cf','var'); cf = 'off'; end
    if ~exist('origin','var'); origin = [eye(3) zed; zed' 1]; end
    
    % Transformation from origin to waist of baxter
    TObaxter = [eye(3) [-.06;0;.43]; zed' 1];
    
    % wheelchair kinematics
    wheelchair.H = [x0 y0 z0];
    wheelchair.P = [zed zed zed zed];
    wheelchair.type = [3 3 2];
    wheelchair.n = 3;
    wheelchair.origin = origin;
    
    wheelchair.link_type = [0 0 0 0];
       
    %%%%%% Define reference frame dimensions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    wheelchair.frame.scale = 0.2;
    wheelchair.frame.width = 0.01;
        
    %%%%%% Create robot objects %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    h_baxter = createBaxter('CreatePedestal','off','CreateFrames',cf, ...
                            'Origin',origin*TObaxter);
    h_wheelchair = createRobot(wheelchair,'CreateFrames',cf,'CreateGripper','off');
    
    % Augment each serial chain with wheelchair
    handle.left_arm = combineRobots(h_wheelchair, h_baxter.left_arm);
    handle.right_arm = combineRobots(h_wheelchair, h_baxter.right_arm);
    handle.head = combineRobots(h_wheelchair, h_baxter.head);
    %%%%%% Attach extra body parts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Wheelchair body
    base_props = {'FaceColor',[0;0;0.8], ...
                        'EdgeAlpha',1, ...
                        'EdgeColor',[0;0;0]};
    base_params.width = 0.7;
    base_params.length = 0.46;
    base_params.height = 0.37;
    t_base = handle.head.frame(4).t + ...
                handle.head.frame(4).R*[0.18;0;0.3];
    h_base = createCuboid(handle.head.frame(4).R, ...
                t_base,base_params,base_props{:});
    h_base.labels = attachPrefix('wheelchair_',h_base.labels);
    
    % Rear Wheels
    wheel_props = {'FaceColor',[0.45;0.45;0.45], ...
                    'EdgeAlpha',1, ...
                    'EdgeColor',[0;0;0]};
    wheel_params.radius = 0.17;
    wheel_params.height = 0.07;
    wheel_R = handle.head.frame(4).R*rot(x0,pi/2);
    leftwheel_t = handle.head.frame(4).t + ...
                        handle.head.frame(4).R*[0;0.28;0.17];
    rightwheel_t = handle.head.frame(4).t + ...
                        handle.head.frame(4).R*[0;-0.28;0.17];
    h_leftwheel = createCylinder(wheel_R,leftwheel_t, ...
                                wheel_params,wheel_props{:});
    h_rightwheel = createCylinder(wheel_R,rightwheel_t, ...
                                wheel_params,wheel_props{:});
    h_leftwheel.labels = attachPrefix('Lwheel_',h_leftwheel.labels);
    h_rightwheel.labels = attachPrefix('Rwheel_',h_rightwheel.labels);
        
    % Front Wheels (Casters)
    caster_props = {'FaceColor',[0.45;0.45;0.45], ...
                    'EdgeAlpha',1, ...
                    'EdgeColor',[0;0;0]};
    caster_params.radius = 0.09;
    caster_params.height = 0.05;
    leftcaster_t = handle.head.frame(3).t + origin(1:3,1:3)*[.52;0.2;0.09];
    rightcaster_t = handle.head.frame(3).t + origin(1:3,1:3)*[.52;-0.2;0.09];
    
    h_leftcaster = createCylinder(wheel_R,leftcaster_t, ...
                                caster_params,caster_props{:});
    h_rightcaster = createCylinder(wheel_R,rightcaster_t, ...
                                caster_params,caster_props{:});
    h_leftcaster.labels = attachPrefix('Lcaster_',h_leftcaster.labels);
    h_rightcaster.labels = attachPrefix('Rcaster_',h_rightcaster.labels);
    
    handle.head = attachObjectToRobot(h_base,3,handle.head);
    handle.head = attachObjectToRobot(h_leftwheel,3,handle.head);
    handle.head = attachObjectToRobot(h_rightwheel,3,handle.head);
    handle.head = attachObjectToRobot(h_leftcaster,3,handle.head);
    handle.head = attachObjectToRobot(h_rightcaster,3,handle.head);
    
end