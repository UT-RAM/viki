% Core and topic information
core = 'http://ramflight-arena:11311';
topic = 'James/setpoint';

% Initialize node, publisher and message
node = rosmatlab.node('matlab_square',core);
pose_pub = rosmatlab.publisher(topic, 'geometry_msgs/Pose', node);
pose = rosmatlab.message('geometry_msgs/Pose',node);
tic;

% Paramers
l = 5; % Time per setpoint
d = 1; % Setpoint coord.
z = 1.5; % Setpoint z

% Prepare plot
figure;

while true
    if toc() > 0*l
        pose.getPosition().setX(-d);
        pose.getPosition().setY(-d);
    end
    if toc() > 1*l
        pose.getPosition().setX(-d);
        pose.getPosition().setY(d);
    end
    if toc() > 2*l
        pose.getPosition().setX(d);
        pose.getPosition().setY(d);
    end
    if toc() > 3*l
        pose.getPosition().setX(d);
        pose.getPosition().setY(-d);
    end
    if toc() > 4*l
        % Reset timer  to keep looping
        tic; 
    end
    pose.getPosition().setZ(z);
    pose.getOrientation().setW(1);
    
    plot(pose.getPosition().getX(),pose.getPosition().getY(),'rd');
    axis([-1.5 1.5 -1.5 1.5]);
    
    pause(0.5);
    pose_pub.publish(pose);
end