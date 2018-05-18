classdef ROSGUI < handle
    %ROSGUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        VERBOSE           % show console output and debug information
        world_mat         % WorldBuilder_MATLAB object
        host_os           % Host operating system: 'Windows','MacOSX','Unix'
        ip_address
        figure_global_map
        bot_figures
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
            rosgui.figure_global_map = figure(1);
            map = strcat('MAP');
            title(map);
            hold on;
            set(rosgui.figure_global_map,'Visible','off');
            rosgui.bot_figures = containers.Map();
            rosgui.initBotFigures();
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
        
        
        function initBotFigures(rosgui, namespace)
            if (exist('namespace','var')==0)
                namespace = [];
            end
            image_fig_str = strcat(namespace,'IMAGE');
            rosgui.bot_figures(image_fig_str) = figure();
            hold on;
            title(image_fig_str);
            set(rosgui.bot_figures(image_fig_str),'Visible','off');
            
            error_fig_str = strcat(namespace,'ERROR');
            rosgui.bot_figures(error_fig_str) = figure();
            hold on;
            title(error_fig_str);
            set(rosgui.bot_figures(error_fig_str),'Visible','off');
            rosgui.VERBOSE = true;
        end
        
        function setFigure(rosgui, figString, namespace)
            if (exist('namespace','var')==0)
                namespace = [];
            end
            fig_str = strcat(namespace,figString);
            if (strcmp(figString,'MAP')==1)
                    set(0,'CurrentFigure',rosgui.figure_global_map);
            else
                try  
                    fig_h = rosgui.bot_figures(fig_str); 
                    set(0,'CurrentFigure',fig_h);
                catch
                    fprintf(1,'ROSGUI::setFigure() Could not find figure %s for namespace %s',figString, namespace); 
                end
            end            
        end
        
        function fig_h = getFigure(rosgui, figString, namespace)
            if (exist('namespace','var')==0)
                namespace = [];
            end
            fig_str = strcat(namespace,figString);
            if (strcmp(figString,'MAP')==1)
                fig_h = rosgui.figure_global_map;
            else
                try
                    fig_h = rosgui.bot_figures(fig_str);
                catch
                    fprintf(1,'ROSGUI::getFigure() Could not find figure %s for namespace %s',figString, namespace);
                end
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

