classdef PprzXLog < PprzLog
    %PprzXLog   Paparazzi x-log file object.
    %
    
    properties (Constant)
        xsl_path = fullfile(pwd,'paparazzi-logcvt.xsl');
        %xslt_exec = 0;
    end
    
    properties
        %ext    from PprzLog
    end
    
    methods
        function xlog = PprzXLog()
            xlog@PprzLog();
            xlog.ext = 'xlog';
        end
        
        function ex = transform_log(xlog)
%            if ( ~xlog.xslt_exec )
                javaaddpath(fullfile(pwd, 'saxon9he.jar'));
            
                proc = net.sf.saxon.s9api.Processor(0);
                xslt_comp = proc.newXsltCompiler();
                xsl_source = javax.xml.transform.stream.StreamSource( ...
                                java.io.File(xlog.xsl_path) );
                xslt_exec = xslt_comp.compile(xsl_source);
 %           else
 %               xslt_exec = xlog.xslt_exec;
 %           end
            
            xslt = xslt_exec.load();
            
            log_source = javax.xml.transform.stream.StreamSource( ...
                                java.io.File(xlog.get_logfile()) );
            xlog_dest = javax.xml.transform.stream.StreamDestination( ...
                                java.io.File(xlog.get_datafile()) );
            xslt.setSource(log_source);
            xslt.setDestination(xlog_dest);
            
            xslt.transform();
            
            ex = xlog.exist_data();
        end
    end
end
     