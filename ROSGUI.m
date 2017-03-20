classdef ROSGUI < handle
    %ROSGUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        VERBOSE           % show console output and debug information
        world_mat         % WorldBuilder_MATLAB object
        host_os           % Host operating system: 'Windows','MacOSX','Unix'
        ip_address
        figure_map
        figure_img
        figure_error
        %figure_gui
    end
    methods (Static)
        function demo
            clear;
            clc;
            close all;
            hold off;
            global START_TIME;
            global GAZEBO_SIM;
            global GUI;
        end
    end
    
    methods
        function rosgui = ROSGUI()
            rosgui.figure_map = figure(1);
            title('MAP');
            hold on;
            set(rosgui.figure_map,'Visible','off');
            rosgui.figure_img = figure(2);
            hold on;
            title('IMAGE');
            set(rosgui.figure_img,'Visible','off');
            rosgui.figure_error = figure(3);
            hold on;
            title('ERROR');
            set(rosgui.figure_error,'Visible','off');
            rosgui.VERBOSE = true;
            if (ispc)
                rosgui.host_os='Windows';
            elseif (ismac)
                rosgui.host_os='MacOSX';
            elseif (isunix)
                rosgui.host_os='Unix';
            end
            rosgui.ip_address = rosgui.getIPAddress();
            fprintf(1,'Detected host IP address: %s\n',rosgui.ip_address);
        end
        
        function setFigure(rosgui, figString)
            switch (figString)
                case 'MAP'
                    set(0,'CurrentFigure',rosgui.figure_map);
                case 'IMAGE'
                    set(0,'CurrentFigure',rosgui.figure_img);
                case 'ERROR'
                    set(0,'CurrentFigure',rosgui.figure_error);
                otherwise
                    fprintf(1,'ROSGUI::setCurrentFigure() Could not set the figure to invalid string ''%s''\n', ...
                        figString);
            end
        end
        
        function fig_h = getFigure(rosgui, figString)
            switch (figString)
                case 'MAP'
                    fig_h = rosgui.figure_map;
                case 'IMAGE'
                    fig_h = rosgui.figure_img;
                case 'ERROR'
                    fig_h = rosgui.figure_error;
                otherwise
                    fprintf(1,'ROSGUI::getFigure() Could not get the figure to invalid string ''%s''\n', ...
                        figString);
            end
        end
        
        function ipAddrString = getIPAddress(rosgui)
            switch (rosgui.host_os)
                case 'Windows'
                    str_prefix = 'IPv4 Address.*?';
                    [~, stdout_str] = system('ipconfig');
                case 'MacOSX'
                    str_prefix = 'inet addr:';
                    [~, stdout_str] = system('ifconfig -a');
                case 'Unix'
                    str_prefix = 'inet addr:';
                    [~, stdout_str] = system('ifconfig -a');
                otherwise
                    disp('No valid value for host os was found, cannot determine IP');
            end
            regex_first_triplet = '((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)(\.)){3}';
            regex_last_digits = '(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)';
            ip_regex = strcat(str_prefix, regex_first_triplet, ...
                regex_last_digits);
            ip_regex2 = strcat(regex_first_triplet, ...
                regex_last_digits);
            ipaddress_strs = regexp(stdout_str, ip_regex, 'match');
            ipaddresses = regexp(ipaddress_strs, ip_regex2, 'match');            
            ipAddrString = '';
            for sIdx=1:length(ipaddresses)
                trimstr = strtrim(ipaddresses{sIdx}{1});
                if (~startsWith(trimstr,'127') && ...
                        ~startsWith(trimstr,'255') && ...
                        ~endsWith(trimstr,'255'))
                    ipAddrString = trimstr;
                    return;
                end
            end
        end
        
        function consolePrint(obj, stringVal)
            if (obj.VERBOSE)
                disp(stringVal)
            end
        end
        
        function ROS_DEBUG(obj, varargin)
            if (false)
                fprintf(1,strcat(varargin{1},'\n'),varargin{2:end});
            end
        end
        
        function ROS_INFO(obj, varargin)
            if (false)
                fprintf(1,strcat(varargin{1},'\n'),varargin{2:end});
            end
        end
    end
end

