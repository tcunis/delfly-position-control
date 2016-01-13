classdef PprzXLogMsg < AbstractLogMsg
    %PprzXLogMsg    Paparazzi x-log message object.
    %
    
    properties
        %name   from AbstractLogMsg
        %data   from AbstractLogMsg
        ac_id;
    end
    
    methods
        function xmsg = PprzXLogMsg( name )
            xmsg@AbstractLogMsg(name);
            %xmsg.data = {};
            xmsg.ac_id = 0;
        end
        
        function b = read_entry( xmsg, entry )
            entry_id = entry.getAttribute('ac-id');
            if ( ~xmsg.ac_id || strcmp(xmsg.ac_id, entry_id) )
                entry_data = {};
                entry_data.time = str2double(entry.getAttribute('time'));
                
                fields = entry.getElementsByTagName('field');
                for findex = 1:fields.getLength
                    field = fields.item(findex-1);
                    entry_data = xmsg.read_field(entry_data, field);
                end
                
                xmsg.add_data(entry_data);
                b = 1;
            else
                b = 0;
            end
        end
        
        function entry_data = read_field( xmsg, entry_data, field )
            field_name = field.getAttribute('name');
            raw_value = field.getAttribute('raw-value');
            values = field.getElementsByTagName('value');
            
            for vindex = 1:values.getLength
                value = values.item(vindex-1);
                entry_data = xmsg.read_value(entry_data, field_name, value, raw_value);
            end
        end
        
        function entry_data = read_value( xmsg, entry_data, field_name, value, raw_value )
            value_unit = value.getAttribute('unit');
            value_num  = str2double(value.getTextContent);
            if ( isnan(value_num) )
                value_num = str2double(raw_value);
            end 
            value_name = sprintf('%s_%s', char(field_name), char(value_unit));
            %value_name = regexprep( value_name, '[^_a-zA-Z0-9]', '' );
            value_name = matlab.lang.makeValidName(value_name);
            
            entry_data.(value_name) = value_num;
        end
        
        function add_data( xmsg, entry_data )
            %add_data   Overriding add_data@AbstractLogMsg
            if ( xmsg.row_count == 0 )
                xmsg.data = [entry_data];
                xmsg.row_count = 1;
            else
                add_data@AbstractLogMsg(xmsg, entry_data);
            end
        end
        
        function time = get_time( xmsg )
            %get_time   Overriding get_time@AbstractLogMsg
            time = [xmsg.data.time];
        end
        
        function col_data = get_column( xmsg, col_name )
            %get_column     Overriding get_column@AbstractLogMsg
            col_data = [xmsg.data.(col_name)];
        end
    end
end
