import sys, time
sys.path.insert(0,'stubserial'); sys.path.insert(0,'..')
import play_mp3 as P, serial
ser = P.open_serial(sys.argv[1], 921600); link = P.Link(ser)
st = P.handshake(link, 3, False)
print("probe: state=%s error=%s underruns=%d fill=%d" % (P.STATE_NAMES[st.state], P.ERR_NAMES[st.error], st.underruns, st.fill))
