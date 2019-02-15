clear 
clc
addpath('/home/turtlebotmaster/Desktop/MATLAB/custom_msgs/matlab_gen/msggen')
addpath('/home/turtlebotmaster/Desktop/pozyx_tag_class');
%% Setup turtlebot
turtlebot=turtlebot_measurment_class(...
    'ROS_MASTER_URI','http://10.0.0.63:11311',...
    'ROS_IP','10.0.0.7',...
    'SUBSCRIBER',{'/odom'},...
    'PUBLISHER',{'/mobile_base/commands/velocity'});

%% Setup pozyx
%aID={'0x6937','0x6940','0x6935','0x6955'};
aID={'0x6940','0x6935','0x6955'};
tID='0x0000';
tag_obj=pozyx_tag(tID,aID,'/pozyx_device_range');
anchors.names={...
  %  '0x6937';...
    '0x6940';...
    '0x6935';...
    '0x6955'}; 
anchors.coords=[...
   % .04,.472;...
    6.71,0.0;...
    6.71,3.66;...
    0.00,3.66];

%% Setup compelte
disp('Start:')
while(1)
    turtlebot.receive('/odom',2,'STORE',1);
    pause(0.05);
end

%% Save
disp('saving...')
rosshutdown
save('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/trail_10')
disp('compelte')