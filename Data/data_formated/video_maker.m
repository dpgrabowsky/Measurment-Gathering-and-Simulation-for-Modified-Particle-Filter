
clear
clc
clf
disp('Start')

trail='ftrail_1.mat';
file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/frames_regpf_',trail);
load(file_path)
file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/frames_pf_',trail);
load(file_path)

%load('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/frames_regpf_ftrail_2.mat');
%load('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/frames_pf_ftrail_2.mat');



%% Section




trail = erase(trail,'.mat');
writerObj = VideoWriter(strcat(trail,'.avi'));
writerObj.FrameRate = 15;
% write the frames to the video
open(writerObj);
for i=1:length(F_pf)

    % convert the image to a frame
    frame1 = F_pf(i) ;
    frame2 = F_regpf(i);
    
    %imshow(frame1.cdata);
    %imshow(frame2.cdata);
    temp=cat(4,frame1.cdata,frame2.cdata);
    montage(temp);
    text(350,0,'Modified PF')
    text(1125,0,'Regular PF')
    fig=gcf;
    frame = getframe(gcf) ;
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

disp("Done")


% % Load saved figures
% c=hgload('MyFirstFigure.fig');
% k=hgload('MySecondFigure.fig');
% % Prepare subplots
% figure
% h(1)=subplot(1,2,1);
% h(2)=subplot(1,2,2);
% % Paste figures on the subplots
% copyobj(allchild(get(c,'CurrentAxes')),h(1));
% copyobj(allchild(get(k,'CurrentAxes')),h(2));
% % Add legends
% l(1)=legend(h(1),'LegendForFirstFigure')
% l(2)=legend(h(2),'LegendForSecondFigure')
