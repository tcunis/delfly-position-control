% paparazzi xlog reader usage example
%
% This is to show and describe the usage of the paparazzi xlog reader
% for MATLAB.
%
% Copyright (C) Torbjoern Cunis <t.cunis@tudelft.nl>
% 
% This file is part of paparazzi:
% 
% paparazzi is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2, or (at your option)
% any later version.
% 
% paparazzi is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with paparazzi; see the file COPYING.  If not, see
% <http://www.gnu.org/licenses/>.

%% xlog file creation
% The paparazzi xlog reader is working based on the XML log data file,
% called xlog. The xlog is created from both the log and data file written
% by paparazzi using an XSLT stylesheet.
% Up to now (v0.9), the XSL transformation of the paparazzi log file has to 
% be done manually. For that, an appropiate XSLT processor is required; we 
% recommend using Saxon 9.7 HE (opensource; http://saxon.sourceforge.net/).
% In order to perform the transformation, follow these steps:
%
%  1. Copy the stylesheet file paparazz-logcvt.xsl next to your paparazzi
%     log file;
%  2a. Navigate to the folder containing stylesheet and log file;
%  2b. Apply the stylesheet to your log file using the XSLT processor, e.g.
%       java -jar path/to/saxon9he.jar -s:YYYY_MM_DD__HH_MM_SS.log
%       -xsl:paparazzi-logcvt.xsl -o:YYYY_MM_DD__HH_MM_SS.xlog
%
% Note, that this transformation has to be done only once per log file.

%% load the xlog
% The xlog file created is load into MATLAB via an instance of the class
% PprzXLog. The following member fields can / need to be set:
%  - folder : The (absolute/relative) folder containing the xlog;
%  - date   : Date and time of the xlog file as 1x6 double
%             [year month day hour minutes seconds];
%  - desc   : An optional suffix of the xlog file name.
% The file extension .xlog is added by MATLAB automatically.
xlog = PprzXLog();
xlog.folder = '';
xlog.date   = [2015 12 15, 11 19 38];
xlog.desc   = '_SD_no_GPS';

%% specify messages
% Each desired message is load from the log by an instance of the class
% PprzXLogMsg. The name of the message is specified via the constructor.
xmsg_ins = PprzXLogMsg('INS_REF');
xmsg_gps = PprzXLogMsg('GPS_REF');

%% xlog reader call
% Apply the xlog file object as well as vector of the messages to the
% paparazzi xlog reader function PprzXLogReader_0_9(xlog, messages):
PprzXLogReader_0_9(xlog, [xmsg_ins, xmsg_gps]);

%% result
% The paparazzi xlog reader reads the message definitions as well as their
% values over time distuingished by unit. For each message read, time
% and values are stored in the struct vector PprzXLogMsg.data. Thus, the
% time column is accessible via, for example,
%       xmsg_ins.data.time
%
% The value columns are called by the field names defined in messages.xml:
%       xmsg_gps.data.tow
%
% If either a unit or alt_unit attribute is available, the respective unit
% is added to the column name:
%       xmsg_gps.data.ecef_x_cm
%       xmsg_gps.data.ecef_x0_m
% An alternative unit coefficient would have been applied to the value
% already.
% If both unit and alt_unit, as well as alt_unit_coefficient are given, two
% columns are written:
%       xmsg_gps.data.lat_1e7deg
%       xmsg_gps.data.lat_deg
%
% In order to convert a time or value column to a vector which can be
% applied to a plot function, surround the column specifier by square
% brackets:
%       plot([xmsg_gps.data.time], [xmsg_gps.data.alt_mm])

%% extend Java heap size
% MATLAB uses a Java virtual machine (JVM) in order to read the xlog. For
% large log files, the heap memory can be exceeded and MATLAB crashes 
% subsequently. In that case, increase the heap size in 
%       Preferences->MATLAB->General->Java Heap Memory