classdef GazeboMode < mode.ModeStrategy
    properties
        node
        Comm
        Sensfet
        sensorSubs
        whillSubs
        % whillPubs
        cmdMsg
    end
    properties (Constant)
        
    end
    methods
        function obj = GazeboMode(vehType, idx, rid, nodename, baseSens)
            disp("Setting up ROS2 configuration for Gazebo mode")
            setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
            setenv("FASTDDS_BUILTIN_TRANSPORTS","UDPv4")
            setenv("ROS_LOCALHOST_ONLY","1")
            obj.Comm = bridge.ROS2CommManager(vehType, 2, idx, nodename, rid,baseSens);
            % obj.Sensfet = bridge.SensorFetcher(2,vehType,idx,baseSens);
        end
        function setup(obj)
            % obj.sensorSubs = obj.Comm.genSensorSubs();
            % obj.whillSubs  = obj.Comm.genWhillSubs();
            obj.Comm.genSensorSubs();
            obj.Comm.genWhillSubs();
            obj.Comm.genWhillPubs();
            
        end
        function [data,Plant] = receiveData(obj)
            % [data,Plant] = obj.Sensfet.getSensorData(obj.sensorSubs,obj.whillSubs);
            [data,Plant] = obj.Comm.getSensorData();
        end
        function exeProcess(~,~), end
        function sendData(obj, cmd)
            obj.Comm.send_msgs_toWhill(cmd)
        end
        function shutdown(obj)
            Spr = utils.SpinnerStatus('Shutting down the ROS2 node. This process will take 5 seconds...');
            close(findall(groot, 'Type', 'figure'));
            rosshutdown
            pause(5)
            Spr.done('Finished.')
        end
    end
end
