classdef PprzXLogMsgMap
    %PprzXLogMsgMap     This maps paparazzi x-log message objects to the
    %                   respective message names.
    
    properties
        messages;
        names;
    end
    
    methods
        function map = PprzXLogMsgMap( messages )
            map.messages = {};
            for msg = messages
                map.messages.(msg.name) = msg;
            end
            map.names = fieldnames( map.messages );
        end
        
        function b = has( map, name )
            b = isfield(map.messages, name);
        end
        
        function msg = get( map, name )
            msg = map.messages.(name);
        end
        
        function b = read_entry( map, name, entry )
            if ( map.has(name) )
                msg = map.get(name);
                b = msg.read_entry(entry);
            else
                b = 0;
            end
        end
    end
end