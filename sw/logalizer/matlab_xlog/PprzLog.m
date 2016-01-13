classdef PprzLog < AbstractLog
    %PprzLog    Paparazzi log file data object.
    %   
    
    properties
        %folder     from AbstractLog
        date;
        ext;
        desc;
    end
    
    methods
        function log = PprzLog()
            log@AbstractLog();
            log.folder = '';
            log.date = [ 1970, 1, 1, 0, 0, 0 ];
            log.ext  = 'data';
            log.desc = '';
        end
        
        function name = get_name(log)
            %get_name(log)  Returns the name of a log file object.
            name = log.get_filename()
        end
        
        function file = get_filename(log)
            %get_filename(log)  Returns the name of log and data files, resp.
            formatOut = 'yy_mm_dd__HH_MM_SS';
            file = [datestr( log.date, formatOut ), log.desc];
        end
        
        function ext_file = get_extfile(log, ext)
            %get_extfile    Returns file name with extension.
            file = log.get_filename();
            ext_file = sprintf( '%s.%s', file, ext );
        end
        
        function data_file = get_datafile(log)
            %get_datafile   Returns the data file name.
            data_file = log.get_extfile(log.ext);
        end
        
        function log_file = get_logfile(log)
            %get_logfile    Returns the log file name.
            log_file = log.get_extfile('log');
        end
        
        function path = get_path(log)
            %get_path   Returns the path of a log file object.
            path = log.folder;
        end
        
        function data = get_abs_file(log)
            %get_data   Returns the absolute data file path.
            path = log.get_path();
            data_file = log.get_datafile();
            
            data = sprintf( '%s\\%s', path, data_file );
        end
        
        function ex = exist_data(log)
            %exist_data     Checks whether the data file exists.
            ex = exist(log.get_datafile, 'file');
        end
    end
end

