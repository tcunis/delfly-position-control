function [lineCount, msgCount] = PprzXLogReader_0_9 ( xlog, messages )
    %PprzXLogReader_0_9     Reads paparazzi messages from x-log data file.
    % Author: Torbjoern Cunis, <t.cunis@tudelft.nl>
    % Date: 2016/01/11

    msgmap = PprzXLogMsgMap( messages );
    lineCount = 0;
    msgCount  = 0;
    
    file = xmlread( xlog.get_abs_file() );
    root = file.getDocumentElement();
    logs = root.getElementsByTagName('logdata');
    data = logs.item(0); %todo: support multiple logdata tags.
    
    entries = data.getElementsByTagName('log-entry');

    fprintf('Start parsing %d entries of %s...\n', entries.getLength, xlog.get_datafile());

    
    for index = 1:entries.getLength
        entry = entries.item(index-1);
        name = entry.getAttribute('name');
        if ( msgmap.read_entry(char(name), entry) )
            msgCount = msgCount + 1;
        end
        
        lineCount = lineCount + 1;
    end
    
    fprintf('Read %d messages from %d lines.\n', msgCount, lineCount);
    
end