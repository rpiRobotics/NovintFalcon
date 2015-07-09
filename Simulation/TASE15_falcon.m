clc;clear all;close all;
%Server connection
falcon = RobotRaconteur.Connect('tcp://localhost:2354/falconServer/Falcon');

%%Constants of the system
x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1];
vec = [z0 y0 x0 y0 x0 y0 x0];%product of exponential, left arm rotation
vectransl = [[0.00375;.25888;0.8196],[0.069;0;0],[0;0;0],[0.36435;0;-0.069], [0.37429;0;-0.01],[0;0;0],[0;0;0],[0;0;0]];%product of exponential, left arm translation
pPalm_EE=0.2346*x0;%palm position relative to the end-effector frame
gripper_max=0.06;%maximum gripper width
gripper_min=0.04;%minimum gripper width
gripper_speed=0.1;%gripper opening and closing speed
d_front=0.6;d_back=0.25;d_side=0.3;%bounding box of wheelchair for collision avoidance
IR_param=[[d_front;d_side;0;0] [d_front;d_side/3;0;0] [d_front;-d_side/3;0;0] [d_front;-d_side;0;0]];%IR sensor coordinates, facing angles and ranges for base (IR sensor only possible to detect obstacle from the front side!)
%the codes below are geometric definitions of the table, shelf, load(the little ball)
%and the target (the plate on the table)
table_dim.param.width=0.5;table_dim.param.length=0.4;table_dim.param.height=0.5;
table_dim.t=[2;-0.5;table_dim.param.height/2];
shelf.center=[3;1;0];shelf.width=0.25;shelf.length=0.8;shelf.length;shelf.height=1.8;shelf.thick=0.03;shelf.middleheights=[0.6;1];
shelf_top_dim.param.width=shelf.width-shelf.thick;shelf_top_dim.param.length=shelf.length-2*shelf.thick;shelf_top_dim.param.height=shelf.thick;
shelf_top_dim.t=shelf.center-[shelf.width/2;0;0]+[(shelf.width-shelf.thick)/2;0;shelf.height-shelf.thick/2];
shelf_bottom_dim.param=shelf_top_dim.param;
shelf_bottom_dim.t=shelf.center-[shelf.width/2;0;0]+[(shelf.width-shelf.thick)/2;0;shelf.thick/2];
shelf_left_dim.param.width=shelf.width;shelf_left_dim.param.length=shelf.thick;shelf_left_dim.param.height=shelf.height;
shelf_left_dim.t=shelf.center-[shelf.width/2;0;0]+[shelf.width/2;shelf.length/2-shelf.thick/2;shelf.height/2];
shelf_right_dim.param=shelf_left_dim.param;
shelf_right_dim.t=shelf.center-[shelf.width/2;0;0]+[shelf.width/2;-shelf.length/2+shelf.thick/2;shelf.height/2];
shelf_back_dim.param.width=shelf.thick;shelf_back_dim.param.length=shelf.length-2*shelf.thick;shelf_back_dim.param.height=shelf.height;
shelf_back_dim.t=shelf.center-[shelf.width/2;0;0]+[shelf.width-shelf.thick/2;0;shelf.height/2];
for i=1:length(shelf.middleheights)
    shelf_middle_dim(i).param=shelf_bottom_dim.param;
    shelf_middle_dim(i).t=shelf_bottom_dim.t+[0;0;shelf.middleheights(i)-shelf.thick/2];
end
load1_dim.param.radius = 0.02;load1_dim.param.height=0.1;
load1_dim.t=shelf.center-[shelf.width/2;0;0]+[0.04;0;shelf.middleheights(2)+shelf.thick/2+load1_dim.param.height/2];
target1_dim.param.radius=0.05;target1_dim.param.height=0.02;
target1_dim.t=[table_dim.t(1);table_dim.t(2);table_dim.param.height+target1_dim.param.height/2];
Target_center=[load1_dim.t target1_dim.t];

%%initial condition for the system
ql = [pi/4;0;-pi/2;0.5*pi;0;-0.249*pi;pi/2];%left arm angles
qr = zeros(7,1);qr(1)=pi;qr(2)=-pi/2;%right arm angles (right arm stretches upward so that it does not get in the way of the left-arm movement)
qb = [0;0;0];%base position and orientation (x,y,theta)
qh = 0;%head camera angle for Baxter (always 0 in this demonstration)
t=0;%initial time
qEEd_B=[1;0;0;0];%desired end-effector orientation w.r.t to the base (represented in quaternion)
%the codes below are the gripper states (whether or not it is opening,
%closing, opened or closed
gripper_left=0.06;
gripper_left_openning=0;
gripper_left_openned=1;
gripper_left_closing=0;
gripper_left_closed=0;
% ptarget=load1_center;%initial value for the target (same as load center)
Target=1;%Target=1 means the load is not grabbed yet; Target=2 means the load is in grasp
View=1;%view angle
presstime=0;%press button holding time (such that the maximum changing rate of the view is 1/0.5sec)

%figure setup
figure(1);
jamster = createJammsterNo3('CreateFrames','off');%create Jamster
axis equal;
axis([-0.5 3.5 -2 2 0 1.9]);
jamster.left_arm = updateRobot([qb;ql-[pi/4;0;0;0;0;0;0]],jamster.left_arm);%update left arm joint angle drawing information, (the joint no.1 definition is pi/4 different from the standard definition)
jamster.right_arm = updateRobot([qb;qr+[pi/4;0;0;0;0;0;0]],jamster.right_arm);%update right arm joint angle drawing information, (the joint no.1 definition is -pi/4 different from the standard definition)
jamster.left_arm.end_effector = updateParallelJawGripper(gripper_left,jamster.left_arm.end_effector);%update end-effector gripper drawing information
%the codes below update shelf, obstacle, load and target plate drawing information
shelf_top = createCuboid(eye(3), shelf_top_dim.t, shelf_top_dim.param);
shelf_bottom = createCuboid(eye(3), shelf_bottom_dim.t, shelf_bottom_dim.param);
shelf_left = createCuboid(eye(3), shelf_left_dim.t, shelf_left_dim.param);
shelf_right = createCuboid(eye(3), shelf_right_dim.t, shelf_right_dim.param);
shelf_back = createCuboid(eye(3), shelf_back_dim.t, shelf_back_dim.param);
for i=1:length(shelf.middleheights)
    shelf_middle(i) = createCuboid(eye(3), shelf_middle_dim(i).t, shelf_middle_dim(i).param);
end
table = createCuboid(eye(3), table_dim.t, table_dim.param);
delete param;
param.width=d_front+d_back;param.length=2*d_side;param.height=0.1;
boundboxBase = createCuboid(eye(3), [0;0;0]+[(d_front-d_back)/2;0;param.height/2], param);
load1 = createCylinder(eye(3),load1_dim.t,load1_dim.param,'FaceColor',[0;1;1]);
target1=createCylinder(eye(3),target1_dim.t, target1_dim.param);
view([-45 30]);
drawnow

%plot setup
F=[];
t_stack=[];
tic
while 1 %iteration
    %%Signal obtain (measurements)
    %the table and shelf are treated as obstacles. The minimum distances
    %from them to the four IR sensors placed on the front side of
    %wheelchair are calcuated
    table_points_B=q2R([cos(qb(3)/2);0;0;sin(qb(3)/2)])'*[(table_dim.t-[(table_dim.param.width)/2;(table_dim.param.length)/2;0]-[qb(1);qb(2);0])...
                                                       (table_dim.t-[(table_dim.param.width)/2;-(table_dim.param.length)/2;0]-[qb(1);qb(2);0])...
                                                       (table_dim.t-[-(table_dim.param.width)/2;-(table_dim.param.length)/2;0]-[qb(1);qb(2);0])...
                                                       (table_dim.t-[-(table_dim.param.width)/2;(table_dim.param.length)/2;0]-[qb(1);qb(2);0])];
    shelf_points_B=q2R([cos(qb(3)/2);0;0;sin(qb(3)/2)])'*[shelf.center-[shelf.width/2;shelf.length/2;0]-[qb(1);qb(2);0]...
                                                       shelf.center-[shelf.width/2;-shelf.length/2;0]-[qb(1);qb(2);0]...
                                                       shelf.center-[-shelf.width/2;-shelf.length/2;0]-[qb(1);qb(2);0]...
                                                       shelf.center-[-shelf.width/2;shelf.length/2;0]-[qb(1);qb(2);0]];
    obstacle_lines_B=[[table_points_B(1:2,1);table_points_B(1:2,2)] [table_points_B(1:2,2);table_points_B(1:2,3)] [table_points_B(1:2,4);table_points_B(1:2,4)] [table_points_B(1:2,4);table_points_B(1:2,1)]...
                      [shelf_points_B(1:2,1);shelf_points_B(1:2,2)] [shelf_points_B(1:2,2);shelf_points_B(1:2,3)] [shelf_points_B(1:2,3);shelf_points_B(1:2,4)] [shelf_points_B(1:2,4);shelf_points_B(1:2,1)]];
    dmin=10000.*ones(1,length(IR_param(1,:)));
    for i=1:length(obstacle_lines_B(1,:))
        for j=1:length(IR_param(1,:))
        dmin_temp = Dist_PointToLineseg(obstacle_lines_B(1:2,i)',obstacle_lines_B(3:4,i)',IR_param(1:2,j));
        if dmin_temp<dmin(j)-0.0001
            dmin(j)=dmin_temp;
        end
        end
    end
    %%inputs from Xbox
    falconInput = falcon.controller_input;
    Input_original(1,1)=-falconInput.positionZ;%forward-backward
                                               % minus sign because falcon
                                               % has a different coordinate
                                               % system.
    Input_original(2,1)=falconInput.positionX;%left-right
    Input_original(3,1)=falconInput.positionY;%up-down
    
    %%control
    if falconInput.center_button>0.5   %gripper open and close
        if gripper_left_openned==1
            gripper_left_openned=0;
            gripper_left_closing=1;
        elseif gripper_left_closed==1
            gripper_left_openning=1;
            gripper_left_closed=0;
        end
    end
    if falconInput.right_button>0.5 & t-presstime>0.5 %Change the view angle 
        if View==1
            View=2;
        elseif View==2
            View=3;
        else
            View=1;
        end
        presstime=t;
    end
    if falconInput.left_button>0.5 %simulation termination
        break;
    end
    
    pEE_B=pBase2Tip(ql,vec,vectransl);%end-effector position
    qEE_B=QuaternionBase2Tip(ql,vec);%end-effector orientation
    %the codes below calcualte desired end-effector angular velocity to
    %maintain orientation
    qerror=QuaternionMultiply(qEEd_B,[qEE_B(1);-qEE_B(2);-qEE_B(3);-qEE_B(4)]);
    if (qerror(1)>0)
        if real(sqrt(1-qerror(1)^2))<0.00001
            rot_vector=[1;0;0];
        else
            rot_vector=qerror(2:4)./real(sqrt(1-qerror(1)^2));
        end
        rot_angle=real(acos(qerror(1)))*2;
        omegaEE_B=0.5.*rot_angle.*rot_vector;
    else
        if real(sqrt(1-qerror(1)^2))<0.00001
            rot_vector=[1;0;0];
        else
            rot_vector=-qerror(2:4)./real(sqrt(1-qerror(1)^2));
        end
        rot_angle=real(acos(qerror(1)))*2;
        omegaEE_B=0.5.*rot_angle.*rot_vector;
    end

 
    JTl=Jacobi_omega_v(ql,vec,vectransl);   %arm Jacobian    
    %inequality constraints formulation for joint angles
    h=[];h_dot=[];sigma=[];
    for i=1:2
        if Target==i
            if norm([qb(1);qb(2)]-Target_center(1:2,i))>=1.1
                h=[h;-pi/3+ql(4)];
            else
                h=[h;ql(4)];
            end
        end
    end                
    h_dot=[h_dot;zeros(1,3) 1 zeros(1,5)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.1)];
    
    h=[h;2*pi/3-ql(4)];
    h_dot=[h_dot;zeros(1,3) -1 zeros(1,5)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    h=[h;pi/2-ql(1)];
    h_dot=[h_dot;-1 zeros(1,8)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    h=[h;ql(1)+pi/4];
    h_dot=[h_dot;1 zeros(1,8)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    h=[h;pi/3-ql(2)];
    h_dot=[h_dot;0 -1 zeros(1,7)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    h=[h;ql(2)+5*pi/12];
    h_dot=[h_dot;0 1 zeros(1,7)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    h=[h;-pi/4-ql(3)];
    h_dot=[h_dot;0 0 -1 zeros(1,6)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    h=[h;ql(3)+2*pi/3];
    h_dot=[h_dot;0 0 1 zeros(1,6)];
    sigma=[sigma;Sigma(h(end),0.9,0.15,0.15,0.01)];
    
    %inequality constraints formulation for simple self-collision avoidance
    h=[h; pEE_B(1)-0.3];
    h_dot=[h_dot;JTl(4,:) 0 0];
    sigma=[sigma;Sigma(h(end),0.9,0.1,0.15,0.01)];
    
    %obstacle avoidance IR sensors
    for j=1:length(IR_param(1,:))
        h=[h;dmin(j)];
        h_dot=[h_dot;zeros(1,8) -1];
        sigma=[sigma;Sigma(h(end),0.9,0.1,1.2,0.1)];
    end
    
    %main algorithm (notations same as in the paper)
    lambda1=1;lambda2=0.1;C=[zeros(1,7) 1 0;zeros(1,7) 0 1];
    JH=[JTl(4:6,:) [-pEE_B(2) 1;pEE_B(1) 0;0 0]];
    %conversion of original inputs to vH
    vH(1,1)=double(Input_original(1))*0.2/10000;
    vH(2,1)=-double(Input_original(2))*0.2/10000;
    vH(3,1)=double(Input_original(3))*0.12/10000;
    %equality constraints for the end-effector angular velocity
    JE=[JTl(1:3,:) zeros(3,2)];
    vE=omegaEE_B;
    if norm([vH(1) vH(2)])<0.001 %for zero inputs, no need to impose requirement 4 for wheelchair
        JHEC=null([JH;JE;C]);
        NullJE=null(JE);
        H=NullJE'*JH'*JH*NullJE+lambda1.*NullJE'*JHEC*JHEC'*NullJE+lambda2.*NullJE'*C'*C*NullJE;
        f=-(vH-JH*pinv(JE)*vE)'*JH*NullJE + lambda1.*(JHEC'*pinv(JE)*vE)'*JHEC'*NullJE + lambda2.*(C*pinv(JE)*vE)'*C*NullJE;
        [xi,fval]=quadprog(H,f,[-h_dot*NullJE;NullJE;-NullJE],[-sigma+h_dot*pinv(JE)*vE;1.*ones(9,1)-pinv(JE)*vE;1.*ones(9,1)+pinv(JE)*vE],[],[],[],[]);
        u=NullJE*xi+pinv(JE)*vE;
    else %for no-zero inputs, impose requirement 4 for wheelchair
        E=[JE;zeros(1,7) -vH(1)*(vH(2)*pEE_B(2)+vH(1)*pEE_B(1)) vH(1)*vH(2)];
        beta=[vE;0];
        JD=null([JH;E;C]);
        NullE=null(E);
        H=NullE'*JH'*JH*NullE+lambda1.*NullE'*JD*JD'*NullE+lambda2.*NullE'*C'*C*NullE;
        f=-(vH-JH*pinv(E)*beta)'*JH*NullE + lambda1.*(JD'*pinv(E)*beta)'*JD'*NullE + lambda2.*(C*pinv(E)*beta)'*C*NullE;
        [xi,fval]=quadprog(H,f,[-h_dot*NullE;NullE;-NullE],[-sigma+h_dot*pinv(E)*beta;1.*ones(9,1)-pinv(E)*beta;1.*ones(9,1)+pinv(E)*beta],[],[],[],[]);
        u=NullE*xi+pinv(E)*beta;
    end
    ql_dot=u(1:7);
    wv_dot=u(8:9);
    
    %%time update
    dT=toc-t;
    t=t+dT;

    %%System update
    ql=ql+ql_dot.*dT;%joint angle update
    qb=qb+[wv_dot(2)*cos(qb(3));wv_dot(2)*sin(qb(3));wv_dot(1)].*dT;%wheelchair position/orientation update
    ROB=q2R([cos(qb(3)/2);0;0;sin(qb(3)/2)]);%inertia to base quaternion
    RBEE=q2R(QuaternionBase2Tip(ql,vec));%base to end-effector quaternion
    %gripper state update
    if gripper_left_closing==1
        if gripper_left<=gripper_min
            gripper_left=gripper_min;
            gripper_left_closing=0;
            gripper_left_closed=1;
            pgl_EE=RBEE'*(ROB'*(load1_dim.t-[qb(1);qb(2);0])-pEE_B-q2R(qEE_B)*pPalm_EE);%gripper to load relative position, represented in end-effector frame
            if pgl_EE(1)>=0.01 & pgl_EE(1)<=0.07 & pgl_EE(2)>=-0.015 & pgl_EE(2)<=0.015 & pgl_EE(3)>=-load1_dim.param.height*2/5 & pgl_EE(3)<=load1_dim.param.height*2/5 %gripper center close enough to the load for a grasp
                jamster.left_arm = graspLoad(load1,jamster.left_arm);
                set(load1.bodies,'FaceColor',[0;0.5;1]);
                Target=2;
            end                
        else
            gripper_left=gripper_left-gripper_speed*dT;
        end
    elseif gripper_left_openning==1
        if gripper_left>=gripper_max
            gripper_left=gripper_max;
            gripper_left_openning=0;
            gripper_left_openned=1;
            if ~isempty(jamster.left_arm.load)
                plt=target1_dim.t-jamster.left_arm.load.t;%load to target relative position, represented in inertia frame
                if sqrt(plt(1)^2+plt(2)^2)<=target1_dim.param.radius/2 & abs(-plt(3)-load1_dim.param.height/2-target1_dim.param.height/2)<0.02 %gripper center close enough to the target plate to release
                    jamster.left_arm = releaseLoad(jamster.left_arm);
                    set(load1.bodies,'FaceColor',[0;1;1]);
                end               
            end
        else
            gripper_left=gripper_left+gripper_speed*dT;
        end
    end    
    
    %%update figure
    figure(1)
    jamster.left_arm = updateRobot([qb;ql-[pi/4;0;0;0;0;0;0]],jamster.left_arm);
    jamster.right_arm = updateRobot([qb;qr+[pi/4;0;0;0;0;0;0]],jamster.right_arm);
    jamster.head = updateRobot([qb;qh],jamster.head);
    jamster.left_arm.end_effector = updateParallelJawGripper(gripper_left,jamster.left_arm.end_effector);
    boundboxBase = updateRigidBody(q2R([cos(qb(3)/2);0;0;sin(qb(3)/2)]),[qb(1);qb(2);0]+q2R([cos(qb(3)/2);0;0;sin(qb(3)/2)])*[(d_front-d_back)/2;0;param.height/2],boundboxBase);
    load1 = updateRigidBody(eye(3),load1_dim.t,load1);
    drawnow
    %different views selected
    if View==1
        axis([-0.5 3.5 -2 2 0 1.9]);
        view([-45 30]);
    elseif View==2
        axis([-0.5 3.5 -2 2 0 1.9]);
        view([qb(3)*180/pi-90 90]);
    else
        if Target==1
            axis([load1_dim.t(1)-0.2 load1_dim.t(1)+0.2 load1_dim.t(2)-0.2 load1_dim.t(2)+0.2 load1_dim.t(3)-0.2 load1_dim.t(3)+0.2]);
            view([-45 30]);
        else
            axis([target1_dim.t(1)-0.2 target1_dim.t(1)+0.2 target1_dim.t(2)-0.2 target1_dim.t(2)+0.2 target1_dim.t(3)-0.2 target1_dim.t(3)+0.2]);
            view([-45 30]);
        end
    end
    opengl('software');
    F=[F getframe(gcf)];
    %data storage
    t_stack=[t_stack t];
end
% movie2avi(F,'TASE15_simulation.avi','fps',length(t_stack)/t_stack(end));