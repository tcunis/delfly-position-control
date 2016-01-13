classdef AbstractLogMsg < handle
    %AbstractLogMsg     Abstract log message object.
    %
    
    properties
        name;
        row_count;
        col0; % 1 <=> columns have titles.
        cols;
        data;
        intervall;
    end
    
    methods
        function msg = AbstractLogMsg( name )
            msg.name        = name;
            msg.row_count  = 0;
            msg.col0        = 0; %no column title per default
            msg.cols        = [];
            msg.data        = {};
            msg.intervall   = 0;
        end
        
        function col_count = set_coltitle( msg, col0 )
            %set_coltitle   Sets the titles of expected data columns.
            %               By default, first column "time" is inserted;
            %               i.e. columns are [ time, col0 ].
            %               Returns the eventual number of columns.
            msg.cols  = [ 'time', col0 ];
            col_count = length( msg.cols );
            msg.data  = msg.cols; %cell(0, col_count);
            msg.col0  = 1;
        end
        
        function set_intervall( msg, int )
            %set_intervall  Sets the time intervall of requested data.
            %               If int = 0, all data sets will be retrieved.
            msg.intervall = int;
        end
        
        function indices = get_indices( msg, columns )
            %get_indices    Returns these columns' indices named in columns.
            indices = [];
            if ( ~iscell(columns) )
                columns = {columns}
            end
            for col = columns
                indx = msg.get_index( col );
                if indx == -1;
                    continue;
                end
                %else:
                indices = [ indices, indx ];
            end
        end
        
        function indx = get_index( msg, col0 )
            %get_index      Returns the column's index of named by col0.
            indx = strmatch( col0, msg.data(1,:), 'exact' );
            if isempty( indx )
                indx = -1;
            end
        end
        
        function time = get_time( msg )
            %get_time       Retuns the time column of this message.
            time = msg.get_data( 1 );
        end
        
        function endtime = get_endtime( msg )
            time = msg.get_time;
            endtime = time(end);
        end
        
        function data = get_data( msg, indx )
            %get_data       Returns the i-th data column of this message.
            if indx > -1
                if msg.intervall == 0
                    data = cell2mat( msg.data(msg.col0+1:end, indx) );
                else
                    data = cell2mat( msg.data(msg.intervall, indx) );
                end
            else
                data = [];
            end
        end
        
        function data = get_column( msg, col0 )
            %get_column     Returns the data column named by col0.
            indx = msg.get_index( col0 );
            data = msg.get_data( indx );
        end
        
        function data = get_columns( msg, cols )
            %get_columns    Returns these data columns named in cols.
            data = msg.get_data( [msg.get_indices(cols)] );
        end
        
        function mat = get_matrix( msg, col0, toff, zoff )
            %get_matrix     Returns a timed matrix.
            mat = [ msg.get_time()-toff, msg.get_column(col0)-zoff ];
            rowsNaN = ~all(~isnan(mat(:,:)),2);
            mat(rowsNaN,:) = [];
        
            int = mat(:,1) >= 0;
            mat = mat(int,:);
        end
           
        function add_data( msg, matrix )
            %add_data       Appends the data rows in matrix.
            %               If msg.col0 is set, the number of columns of
            %               matrix has to match the length of msg.col0.
            i0 = msg.row_count+1;
            i1 = msg.row_count+size(matrix,1);
            
            msg.data(msg.col0+(i0:i1),:) = matrix; % first line has 
            msg.row_count = i1;
        end
    end
    
end