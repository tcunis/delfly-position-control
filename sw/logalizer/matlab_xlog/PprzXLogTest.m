xlog = PprzXLog();
xlog.date = [2015 12 15, 11 19 38];
xlog.desc = '_SD_no_GPS';

xmsg = PprzXLogMsg('DELFLY_STATE');
xmsg_guid = PprzXLogMsg('DELFLY_GUIDANCE');

messages = [xmsg, xmsg_guid];
PprzXLogReader_0_9(xlog, messages)