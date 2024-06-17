% Inch-worm FK & IK

robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 4);

% set parameter
L = 0.4;
l = 0.1;

%set initial theta 
theta1 = -deg2rad(0);
theta2 = -deg2rad(0);
theta3 =  -(theta1 + theta2);

%link 0
body = rigidBody('link0');
joint = rigidBodyJoint('joint0', 'revolute');
joint.HomePosition = 0;
tform0 = trvec2tform([0 0 0])*eul2tform([0 0 0]);
setFixedTransform(joint, tform0);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

%link 1
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
joint.HomePosition = 0;
tform1 = trvec2tform([0 l 0])*eul2tform([0 0 0]);
setFixedTransform(joint, tform1);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link0');

%link2
body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'revolute');
tform2 = trvec2tform([L 0 0])*eul2tform([0 0 0]);
setFixedTransform(joint, tform2);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

%link3
body = rigidBody('link3');
joint = rigidBodyJoint('joint3', 'revolute');
tform3 = trvec2tform([L 0 0])*eul2tform([0 0 0]);
setFixedTransform(joint, tform3);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');

%link4
body = rigidBody('link4');
joint = rigidBodyJoint('joint4', 'revolute');
tform4 = trvec2tform([0 -l 0])*eul2tform([0 0 0]);
setFixedTransform(joint, tform4);
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link3');

%% FK Sim
figure

%set initial theta 
theta1 = -deg2rad(45);
theta2 = -deg2rad(0);

for i=deg2rad(-0):deg2rad(30):deg2rad(100)
    for j=deg2rad(-0):deg2rad(30):deg2rad(100)
        for k=deg2rad(-100):deg2rad(10):deg2rad(110)
            theta1 = i;
            theta2 = j;
            theta3 = k;

            show(robot, [0 theta1 theta2 theta3 0]');
            view(0,90);
            hold on
            axis([-2*L-0.1 2*L+0.1 -2*L-0.1 2*L+0.1 -0.1 0.1]);
        end
    end
end

%% IK Sim
figure

%set initial theta 
theta1 = -deg2rad(0);
theta2 = -deg2rad(0);
x_i = 0.3;
x_max = 2*L;
y_max = 2*L + l;
x = x_i;

for y=0.1:-0.01:-0.1 %x move

    theta2 = -acos((x^2+y^2-2*L^2)/(2*L^2));
    alpha = atan2(y,x);
    theta1 = alpha - (asin(L*sin(theta2)/(x^2+y^2)^0.5));
    theta3 = -(theta1 + theta2);

    show(robot, [0 theta1 theta2 theta3 0]');

    if y>0
        axis([-x_max-0.2 x_max+0.2 -y_max-0.2 y_max+0.2 -0.1 0.1]);
    else
         axis([-x_max-0.2 x_max+0.2 -y_max-0.2-y y_max+0.2+y -0.1 0.1]);
    end

    
    view(-0,90);
    xline(x_max, 'r', LineWidth=3)
    yline(y_max+y, 'r', LineWidth=3)
    xline(-0.5, 'r', LineWidth=3)
    yline(-y_max+y, 'r', LineWidth=3)
    xline(x, "--", LineWidth=1.5)
    yline(y, "--", LineWidth=1.5)

    if y>0
        ground = rectangle('Position',[-0.2-x_max -0.2-y_max 0.4+2*x_max 0.2+y_max]');
    else
        ground = rectangle('Position',[-0.2-x_max -0.2-y_max 0.4+2*x_max 0.2+y_max+y]');
    end
    
    ground.FaceColor = [0 .5 .5];
    drawnow;
    pause(0.03)
end

for y=-0.1:0.01:0.1 %x move

    theta2 = -acos((x^2+y^2-2*L^2)/(2*L^2));
    alpha = atan2(y,x);
    theta1 = alpha - (asin(L*sin(theta2)/(x^2+y^2)^0.5));
    theta3 = -(theta1 + theta2);

    show(robot, [0 theta1 theta2 theta3 0]');

    if y>0
        axis([-x_max-0.2 x_max+0.2 -y_max-0.2 y_max+0.2 -0.1 0.1]);
    else
         axis([-x_max-0.2 x_max+0.2 -y_max-0.2-y y_max+0.2+y -0.1 0.1]);
    end

    
    view(-0,90);
    xline(x_max, 'r', LineWidth=3)
    yline(y_max+y, 'r', LineWidth=3)
    xline(-0.5, 'r', LineWidth=3)
    yline(-y_max+y, 'r', LineWidth=3)
    xline(x, "--", LineWidth=1.5)
    yline(y, "--", LineWidth=1.5)

    if y>0
        ground = rectangle('Position',[-0.2-x_max -0.2-y_max 0.4+2*x_max 0.2+y_max]');
    else
        ground = rectangle('Position',[-0.2-x_max -0.2-y_max 0.4+2*x_max 0.2+y_max+y]');
    end
    
    ground.FaceColor = [0 .5 .5];
    drawnow;
    pause(0.03)
end
