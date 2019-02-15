clc
clear

trail_list={'1','2','3','4','5','6','7','8','9','10'};


for zz=1:length(trail_list)
    
    clearvars -except trail_list zz
    
    
    temp=strcat('trail_',trail_list{zz});
    trail=strcat(temp,'.mat');
    fprintf('Start...\n Loading %s \n',trail);
    load(strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/',trail))
    
    
    %% Pozyx related data
    % Get names of anchors (all data will be formated in order of names)
    pozyx_anchor_names=anchors.names;
    
    % Get location of anchors (x,y) in m
    pozyx_anchor_coords=anchors.coords;
    
    % Get distance measurments and time stamps from the tag_obj
    pozyx_range_measurments=tag_obj.distanceMeasure;
    pozyx_range_measurments_time_stamps=tag_obj.timeStamps;
    
    %% Turtlebot related data
    %Get saved odom data
    raw_odom=turtlebot.received__odom;
    init_time=raw_odom{1}.Header.Stamp.Sec+(raw_odom{1}.Header.Stamp.Nsec*10^-9);
    turtlebot_odom_command=zeros(2,length(raw_odom));
    turtlebot_odom_command_timestamps=zeros(1,length(raw_odom));
    turtlebot_odom_pose=zeros(2,length(raw_odom));
    
    for(ii=1:length(raw_odom))
        turtlebot_odom_command_timestamps(ii)=raw_odom{ii}.Header.Stamp.Sec+(raw_odom{ii}.Header.Stamp.Nsec*10^-9)-init_time;
        turtlebot_odom_command(:,ii)=[raw_odom{ii}.Twist.Twist.Linear.X,raw_odom{ii}.Twist.Twist.Angular.Z];
        turtlebot_odom_pose(:,ii)=[raw_odom{ii}.Pose.Pose.Position.X raw_odom{ii}.Pose.Pose.Position.Y];
    end
    
    %IMPORTANT: turtlebot_pose may not have started at (0,0). Therefore, offset
    %everything
    startX=turtlebot_odom_pose(1,1);
    startY=turtlebot_odom_pose(2,1);
    turtlebot_odom_pose(1,:)=turtlebot_odom_pose(1,:)-startX;
    turtlebot_odom_pose(2,:)=turtlebot_odom_pose(2,:)-startY;
    
    %IMPORTANT: offset the turtlebot odom by the start position (5.795, 3.666)
    turtlebot_odom_pose(1,:)=turtlebot_odom_pose(1,:)+6.044;
    turtlebot_odom_pose(2,:)=turtlebot_odom_pose(2,:)+3.666;
    trail=strcat('f',trail);
    save(strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail),'pozyx_anchor_names','pozyx_anchor_coords','pozyx_range_measurments','pozyx_range_measurments_time_stamps','turtlebot_odom_command_timestamps','turtlebot_odom_command','turtlebot_odom_pose')
    % function form
    disp('Done')
    
end