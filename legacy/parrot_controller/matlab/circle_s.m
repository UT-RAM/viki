% Core and topic information
core = 'http://ramflight-arena:11311';
topic = 'John/setpoint';

% Initialize node, publisher and message
node = rosmatlab.node('matlab_circle',core);
pose_pub = rosmatlab.publisher(topic, 'geometry_msgs/Pose', node);
pose = rosmatlab.message('geometry_msgs/Pose',node);
tic;

% Paramers
r = 1; % Setpoint coord.
w = 3; % wait in the beginning
z = 1.5; % Setpoint z

% Prepare plot
figure;

while true
    pose.getPosition().setX(0);
    pose.getPosition().setY(0);
    if toc() > w
        pose.getPosition().setX(cos(-toc()*(0.3))*r);
        pose.getPosition().setY(sin(-toc()*(0.3))*r);
    end
    pose.getPosition().setZ(z);
    pose.getOrientation().setW(1);
    
    plot(pose.getPosition().getX(),pose.getPosition().getY(),'rd');
    axis([-1.5 1.5 -1.5 1.5]);
    
    pause(0.1);
    pose_pub.publish(pose);
end