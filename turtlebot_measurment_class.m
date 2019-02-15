classdef turtlebot_measurment_class < dynamicprops & matlab.mixin.SetGet
    % turtlebot_measurment Start a ROS instance, connect to a master on a
    % turtlebot, then subsribe to relavent topics such as odom. Class will
    % also have functionality to recieve and parse messeges for easy use in
    % other programs
    %   Detailed explanation goes here
    %   This class is realtivly undocumented at the moment. 
    %
    % turtlebot_measurment_class methods
    %   
    
    properties
        
    end
    %
    methods
        function obj = turtlebot_measurment_class(varargin)
            %turtlebot_measurment_class Construct an instance of this class
            %   Instantiate turtlebot_measurment_class, expect inputs to be
            %   in name(string),value(xx) format.
            rosshutdown;
            %loop through all variables in varargin
            
            for ii=1:2:nargin
                name=varargin{ii};
                value=varargin{ii+1};
                switch name
                    case 'ROS_MASTER_URI'
                        setenv('ROS_MASTER_URI',value);
                    case 'ROS_IP'
                        setenv('ROS_IP',value);
                    case 'SUBSCRIBER'
                        for jj=1:length(value)
                            %First seach to see if topic exists
                            topicName=value{jj};
                            fprintf('Attempting to subsribe to ''%s''\n',topicName);
                            fprintf('    Checking topics\n');
                            topicList=rostopic('list');
                            if(find(strcmp(topicName,topicList)))
                                %If exists then create a property for a subscriber and subscribe to said topic
                                
                                propertyName=strcat('subscriber',strrep(topicName,'/','__'));
                                fprintf('    Topic found, creating subscriber named: ''%s''.\n',propertyName);
                                obj.addprop(propertyName);
                                set(obj,propertyName,rossubscriber(topicName));
                            else
                                fprintf('    WARNING: Topic does not exist, subsriber not set \n');
                            end
                        end
                    case 'PUBLISHER'
                        for jj=1:length(value)
                            %First seach to see if topic exists
                            topicName=value{jj};
                            fprintf('Attempting to publish to ''%s''\n',topicName);
                            fprintf('    Checking topics\n');
                            topicList=rostopic('list');
                            if(find(strcmp(topicName,topicList)))
                                %If exists then create a property for a subscriber and subscribe to said topic
                                
                                propertyName=strcat('publisher',strrep(topicName,'/','__'));
                                fprintf('    Topic found, creating publisher named: ''%s''.\n',propertyName);
                                obj.addprop(propertyName);
                                set(obj,propertyName,rospublisher(topicName));
                            else
                                fprintf('    WARNING: Topic does not exist, publisher not set \n');
                            end
                        end
                    otherwise
                        fprintf('WARNING: string name ''%s'' not recognized \n',name);
                end
                
                if(ii==3)
                    rosinit;
                end
                
            end
            
            
        end
        
        function send(obj,topic,msg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            topicName=strcat('publisher',strrep(topic,'/','__'));
            pub=get(obj,topicName);
            pub.send(msg);
            
        end
        
        
        function output=receive(obj,topic,varargin)
            %receive dynamic receive function for storing 
            %   Detailed explanation goes here
            topicName=strcat('subscriber',strrep(topic,'/','__'));
            sub=get(obj,topicName);
            
            
            % check if receieved msg has a time out first
            
            odd=rem(length(varargin),2);
            if(odd)
                try
                    output=receive(sub,varargin{1});
                catch
                    fprintf('    WARNING: subsriber: ''%s'' received no information and has timed out \n',topicName);
                    output=[];
                    return
                end
            else
                output=receive(sub);
            end
            
            for ii=2:2:length(varargin)
                name=varargin{ii};
                value=varargin{ii+1};
                switch name
                    case 'STORE'
                        rec=strcat('received',strrep(topic,'/','__'));
                        if(value)
                            if(isprop(obj,rec))
                                data=get(obj,rec);
                                data{end+1}=output;
                                set(obj,rec,data);
                            else
                                obj.addprop(rec);
                                data={output};
                                set(obj,rec,data);
                            end
                        end
                    otherwise
                end
            end
        end
        
        function makeOdomEasyToRead(obj)
        
        
        end
    end
end
