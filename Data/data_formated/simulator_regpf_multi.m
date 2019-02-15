%This file is used to simulate the regular particle filter using the gathered data.
%It is largely uncommented and needs significant restructuring. This
%restructuring will not change the functionality.

clear
clc

%IMPORANT: This file depends on the particle_filter class (pf) be sure to
%add it to the path
addpath('/home/turtlebotmaster/Desktop/particle_filter')


%Which trails to run
trail_list={'1','2','3','4','5','6','7','8','9','10'};

%IMPORTANT: This flag sets whether the script should save figures and trail
%data or not. The simulator will run while this flag is 0, but will not
%save any data.
saveFlag=0;

for(zz=1:length(trail_list))
    %Quick clear to save memory space between trails
    clearvars -except trail_list zz saveFlag
    %% Load and format data
    
    %Absolute path to where the formated trails (ftrail_x) are stored
    absolute_path='/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/';
    %Add the trail number to the path
    temp=strcat('ftrail_',trail_list{zz});
    trail=strcat(temp,'.mat');
    
    % Load the data
    file_path=strcat(absolute_path,trail);
    load(file_path)
    
    % General algorithm: First, use the odom_command to update the position ofinitPose
    % the simulated turtlebot. Then check if odom timestamp has elapsed past
    % any range_measurment time_stamps. If so then use that range
    % measurment to update the particle filter.
    
    
    % Define vars
    numParticles=1000;
    aID={'0x6940','0x6935','0x6955'};
    colors={[.75 .75 0],[1 0 0],[.3 .3 .9]};
    %Create and init pf filter for robot
    fprintf('Initalizing  particle filter... \n');
    initPose=[6.044,3.66,pi];
    robotFilter=pf(numParticles,3+length(pozyx_anchor_names)*2);
    robotFilter.initRobot(initPose,.5)
    fprintf('Particle filter initalized\n\n');
    
    range_measurment_blacklist=cell(1,length(pozyx_anchor_names));
    most_recent_ranges=[0 0 0];
    most_recent_RP=initPose(1:2);
    path_history=[];
    path_error_regpf=[];
    lm_error_regpf=[];
    F_regpf=[];
    fprintf('Starting...')
    for(ii=2:length(turtlebot_odom_command)-1)
        %% Use odom_command to move the robot
        robotFilter.MotionModel(turtlebot_odom_command(:,ii),turtlebot_odom_command_timestamps(ii)-turtlebot_odom_command_timestamps(ii-1));
        
        %% Check for range measurements
        %Get list of all range measurments before current odom timestamp
        temp=cell(1,length(pozyx_anchor_names));
        current_time=turtlebot_odom_command_timestamps(ii);
        for(jj=1:length(pozyx_anchor_names))
            temp{jj}=pozyx_range_measurments_time_stamps{jj}<current_time;
            %convert to idxs
            temp{jj}=find(temp{jj});
            %temp{jj}=pozyx_range_measurments{jj}(temp{jj});
            
        end
        
        %See if any range measurments fall within odom timeframe.
        ranges=zeros(1,length(pozyx_anchor_names));
        if(all(~cellfun(@isempty,temp)))
            %delete all but the most recent measurments from temp
            for(jj=1:length(pozyx_anchor_names))
                temp{jj}=temp{jj}(end);
                timestamp(jj)=pozyx_range_measurments_time_stamps{jj}(temp{jj});
                %ranges(jj)=pozyx_range_measurments{jj}(temp{jj});
            end
            
            %check that the max time difference is less than .5
            if((max(timestamp)-min(timestamp))<.5)
                %black list all earlier timestamps
                for(jj=1:length(pozyx_anchor_names))
                    %If measurment is already blacklisted just set it to 0,
                    %will be noticed in later ifstatement
                    if(ismember(temp{jj},range_measurment_blacklist{jj})|| pozyx_range_measurments{jj}(temp{jj})<.50)
                        ranges(jj)=0;
                    else
                        ranges(jj)=pozyx_range_measurments{jj}(temp{jj});
                    end
                    % range_measurment_blacklist{jj}=[range_measurment_blacklist{jj} temp{jj}];
                    
                end
            end
            
        end
        
        %If any ranges were not black listed toggle the ranges_aval flag.
        ranges_aval=1;
        if(sum(ranges==0)>0)
            ranges_aval=0;
        end
        
        %% If ranges are aval then init/update landmark particles
        %Note: There is some redundancy here with the modified pf
        %simulator. Landmarkfilters variable is not utilized in this code.
        if(~exist('landmarkFilters') && ranges_aval)
            fprintf('Initalizing landmark section of pf...\n');
            for(jj=1:length(pozyx_anchor_names))
                landmarkFilters=0;
                initPose=[mean(robotFilter.particles(:,1)) mean(robotFilter.particles(:,2))];
                %initPose=pozyx_anchor_coords(jj,:)+normrnd(0,.5,[1,2]);
                robotFilter.initLandmarkFull(pozyx_anchor_coords(jj,:),jj,1,.05,'circle');
                %landmarkFilters{jj}.initLandmark(initPose(1:2),2,.05,'circle');
            end
        elseif(ranges_aval)
            
            robotPose=[mean(robotFilter.particles(:,1)) mean(robotFilter.particles(:,2))];
            robotFilter.MeasurmentModelFull(ranges(find(ranges)),find(ranges));
            
            for(jj=1:length(pozyx_anchor_names))
                range_measurment_blacklist{jj}=[range_measurment_blacklist{jj} temp{jj}];
            end
        end
        
        
        %% Optional Plot
        figure(1)
        clf
        %plot robot
        scatter(robotFilter.particles(:,1),robotFilter.particles(:,2))
        hold on
        rP=[mean(robotFilter.particles(:,1)) mean(robotFilter.particles(:,2))];
        plot(rP(1),rP(2),'mX');
        path_history=[path_history;rP];
        plot(path_history(:,1),path_history(:,2),'m--')
        %plot landmarks
        lm_temp=[];
        if(exist('landmarkFilters'))
            for(jj=1:length(pozyx_anchor_names))
                scatter(robotFilter.particles(:,jj*2+2),robotFilter.particles(:,jj*2+3),20,[colors{jj}]);
                lm=[mean(robotFilter.particles(:,jj*2+2)),mean(robotFilter.particles(:,jj*2+3))];
                lm_temp(jj)=pdist([lm;pozyx_anchor_coords(jj,:)],'euclidean');
                scatter(lm(1),lm(2),100,'kX')
                
                slightlyDiffColors=colors{jj};
                slightlyDiffColors(2)=.5;
                scatter(pozyx_anchor_coords(jj,1),pozyx_anchor_coords(jj,2),100,slightlyDiffColors,'filled')
                
                
                if(sum(ranges==0)==0)
                    most_recent_ranges=ranges;
                    most_recent_RP=rP;
                end
                
                if(ii==length(turtlebot_odom_command)-1)
                    
                    labelpoints(pozyx_anchor_coords(jj,1),pozyx_anchor_coords(jj,2),pozyx_anchor_names{jj},'N',0.2,1)
                    labelpoints(rP(1),rP(2),'robot','S',0.2,1)
                else
                    % Plot ranges
                    th = 0:pi/50:2*pi;
                    r=most_recent_ranges(jj);
                    x=most_recent_RP(1);
                    y=most_recent_RP(2);
                    xunit = r * cos(th) + x;
                    yunit = r * sin(th) + y;
                    plot(xunit, yunit,'color',slightlyDiffColors);
                end
            end
        end
        % Create general plot (including just landmarks and known robot path).
        knownPath=[6.044 3.66;...
            5.13 3.66;...
            4.22 3.66;...
            3.31 3.66;...
            3.31 3.96;...
            2.39 3.96;...
            1.48 3.96;...
            0.56 3.96;...
            0.56 3.35;...
            1.48 3.35;...
            2.69 3.35;...
            2.69 2.43;...
            2.69 1.52;...
            2.69 0.9116;
            ];
        pathLabels={'start';'1';'2';'3';'4';'5';'6';'7';'8';'9';'10';'11';'12';'13'};
        plot(knownPath(:,1),knownPath(:,2),'k-o')
        labelpoints(knownPath(:,1),knownPath(:,2),pathLabels,'SE',0.2,1)
        xlabel('meters')
        ylabel('meters')
        hold off
        
        
        axis([-1 7.5 -2.5 5.5]);
        pause(.001)
        
        
        %  F_regpf(ii-1) = getframe(gcf) ;
        drawnow
        %Save the final frame with particles
        if((ii==length(turtlebot_odom_command)-1)&&saveFlag)
            tempname = erase(trail,'.mat');
            tempname=strcat('frames_regpf_',tempname);
            tempname = strcat('final_withpar_',strcat(tempname,'.png'));
            file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',tempname);
            saveas(gcf,file_path)
        end
        
        
        path_error_regpf=[path_error_regpf; calcDistance(knownPath,rP)];
        lm_error_regpf=[lm_error_regpf;lm_temp];
    end
    
    
    figure(2)
    clf
    rP=[mean(robotFilter.particles(:,1)) mean(robotFilter.particles(:,2))];
    hold on
    scatter(rP(1),rP(2),200,'mX');
    path_history=[path_history;rP];
    plot(path_history(:,1),path_history(:,2),'m--')
    
    if(exist('landmarkFilters'))
        for(jj=1:length(pozyx_anchor_names))
            %             lm=[mean(landmarkFilters{jj}.particles(:,1)),mean(landmarkFilters{jj}.particles(:,2))];
            %             %scatter(lm(1),lm(2),200,'X','MarkerFaceColor',colors{jj})
            %             scatter(lm(1),lm(2),200,colors{jj},'X')
            %             scatter(pozyx_anchor_coords(jj,1),pozyx_anchor_coords(jj,2),100,colors{jj})
            
            lm=[mean(robotFilter.particles(:,jj*2+2)),mean(robotFilter.particles(:,jj*2+3))];
            scatter(lm(1),lm(2),200,colors{jj},'X')
            scatter(pozyx_anchor_coords(jj,1),pozyx_anchor_coords(jj,2),200,colors{jj},'filled')
            labelpoints(pozyx_anchor_coords(jj,1),pozyx_anchor_coords(jj,2),pozyx_anchor_names{jj},'N',0.2,1)
            
        end
    end
    
    knownPath=[6.044 3.66;...
        5.13 3.66;...
        4.22 3.66;...
        3.31 3.66;...
        3.31 3.96;...
        2.39 3.96;...
        1.48 3.96;...
        0.56 3.96;...
        0.56 3.35;...
        1.48 3.35;...
        2.69 3.35;...
        2.69 2.43;...
        2.69 1.52;...
        2.69 0.9116;
        ];
    pathLabels={'start';'1';'2';'3';'4';'5';'6';'7';'8';'9';'10';'11';'12';'13'};
    plot(knownPath(:,1),knownPath(:,2),'k-o')
    labelpoints(knownPath(:,1),knownPath(:,2),pathLabels,'SE',0.2,1)
    labelpoints(rP(1),rP(2),'robot','S',0.2,1)
    hold off
    axis([-1 7.5 -2.5 5.5]);
    xlabel('meters')
    ylabel('meters')
    
    
    
    %% Save video frames
    if(saveFlag)
        trail=strcat('frames_regpf_',trail);
        file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail);
        save(file_path,'F_regpf','path_error_regpf','lm_error_regpf')
        
        
        pause(5)
        trail = erase(trail,'.mat');
        trail = strcat('final_nopar_',strcat(trail,'.png'));
        file_path=strcat('/home/turtlebotmaster/Desktop/turtlebot_pozyx_measurment_gather/Data/data_formated/',trail);
        saveas(gcf,file_path)
        pause(5)
    end
end


function dist=calcDistance(path,point)
for(ii=1:length(path)-1)
    y2=path(ii+1,2);
    y1=path(ii,2);
    x2=path(ii+1,1);
    x1=path(ii,1);
    x0=point(1);
    y0=point(2);
    
    num=abs((y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1);
    denom=sqrt((y2-y1)^2+(x2-x1)^2);
    dist(ii)=num/denom;
end
dist=min(dist);
end