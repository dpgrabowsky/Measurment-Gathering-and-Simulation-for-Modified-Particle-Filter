clc
clear

trail_list={'1','2','3','4','5','6','7','8','9','10'};
aID={'0x6940','0x6935','0x6955'};
colors={[.75 .75 0],[1 0 0],[.3 .3 .9]};
for(zz=1:length(trail_list))
    clearvars -except trail_list zz colors aID
    %% Load and format data
    % Load the data
    
    temp=strcat('ftrail_',trail_list{zz});
    trail=strcat(temp,'.mat');
    trail=strcat('frames_pf_',trail);
    file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail);
    load(file_path)
    
    trail='';
    temp=strcat('ftrail_',trail_list{zz});
    trail=strcat(temp,'.mat');
    trail=strcat('frames_regpf_',trail);
    file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail);
    load(file_path)
    
    
    figure(1)
    clf
    plot(path_error_pf)
    hold on
    plot(path_error_regpf)
    xlabel('sample')
    ylabel('meters')
    legend(strcat('Modified PF. Mean err:  ',string(mean(path_error_pf))),strcat('Regular  PF. Mean err:  ',string(mean(path_error_regpf))))
    title('Robot Path of Modified vs Regular PF')
    hold off
    %imshow(frame1.cdata);
    %imshow(frame2.cdata);
    %     temp=cat(4,frame1.cdata,frame2.cdata);
    %     montage(temp);
    %
    
    temp=strcat('robot_error_trail_',trail_list{zz});
    trail=strcat(temp,'.png');
    trail=strcat('pf_vs_regpf_',trail);
    file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail);
    saveas(gcf,file_path)
    
    
    figure(2)
    clf
    subplot(1,2,1)
    plot(1:length(lm_error),lm_error(:,1),'-','color',colors{1})
    hold on
    plot(1:length(lm_error),lm_error(:,2),'-','color',colors{2})
    plot(1:length(lm_error),lm_error(:,3),'-','color',colors{3})
    title('Modified PF')
    xlabel('sample')
    ylabel('meters')
    hold off
    legend(strcat(aID{1},strcat('. Mean err:  ',string(mean(lm_error(:,1))))),...
        strcat(aID{2},strcat('. Mean err:  ',string(mean(lm_error(:,2))))),...
        strcat(aID{3},strcat('. Mean err:  ',string(mean(lm_error(:,3))))));
    
    
    
    subplot(1,2,2)
    plot(1:length(lm_error_regpf),lm_error_regpf(:,1),'-','color',colors{1})
    hold on
    plot(1:length(lm_error_regpf),lm_error_regpf(:,2),'-','color',colors{2})
    plot(1:length(lm_error_regpf),lm_error_regpf(:,3),'-','color',colors{3})
    title('Regular PF')
    xlabel('sample')
    ylabel('meters')
    hold off
    legend(strcat(aID{1},strcat('. Mean err:  ',string(mean(lm_error_regpf(:,1))))),...
        strcat(aID{2},strcat('. Mean err:  ',string(mean(lm_error_regpf(:,2))))),...
        strcat(aID{3},strcat('. Mean err:  ',string(mean(lm_error_regpf(:,3))))));
    
    temp=strcat('landmark_error_trail_',trail_list{zz});
    trail=strcat(temp,'.png');
    trail=strcat('pf_vs_regpf_',trail);
    file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail);
    saveas(gcf,file_path)

end








% load('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/ftrail_1.mat')
%
%
% knownPath=[5.79 3.66;...
%     5.13 3.66;...
%     4.22 3.66;...
%     3.31 3.66;...
%     3.31 3.96;...
%     2.39 3.96;...
%     1.48 3.96;...
%     0.56 3.96;...
%     0.56 3.35;...
%     1.48 3.35;...
%     2.69 3.35;...
%     2.69 2.43;...
%     2.69 1.52;...
%     2.69 0.9116;
%     ];
% pathLabels={'start';'1';'2';'3';'4';'5';'6';'7';'8';'9';'10';'11';'12';'13'};
% pathFigure=figure(1);
% clf
% 
% %% Create general plot (including just landmarks and known robot path).
% plot(knownPath(:,1),knownPath(:,2),'k-o')
% labelpoints(knownPath(:,1),knownPath(:,2),pathLabels,'SE',0.2,1)
% axis([0 6 0.5 4.5]);
% 
% 
% %% Plot the turtlebot pure odom readings 
% hold on
% plot(turtlebot_odom_pose(1,:)',turtlebot_odom_pose(2,:)','b')
% hold off
% 
% %% Plot turtlebot path from control
% control_path=zeros(length(turtlebot_odom_command),2);
% control_path(1,1)=5.795;
% control_path(1,2)=3.66;
% theta=pi;
% for(ii=1:length(turtlebot_odom_command)-1)
%             v=turtlebot_odom_command(1,ii);
%             w=turtlebot_odom_command(2,ii);
%             d_t=turtlebot_odom_command_timestamps(ii+1)-turtlebot_odom_command_timestamps(ii);
%             pose=control_path(ii,:);
%             r=v/w;
%             
%             s=sin(theta);
%             c=cos(theta);
%             s_th=sin(theta+w*d_t);
%             c_th=cos(theta+w*d_t);
%             if(w<.05)
%                 %basically robot is going straight
%                 pose(:,1)=pose(:,1)+(v*c_th)*d_t;
%                 pose(:,2)=pose(:,2)+(v*s_th)*d_t;
%                 theta=theta+(w)*d_t;
%             else
%                 %robot is turning
%                 pose(:,1)=pose(:,1)+(-r*s)+(r*s_th);
%                 pose(:,2)=pose(:,2)+(r*c)-(r*c_th);
%                 theta=theta+(w)*d_t;
%             end
%             control_path(ii+1,:)=pose;  
% end
% hold on
% plot(control_path(:,1),control_path(:,2),'g')
% hold off
