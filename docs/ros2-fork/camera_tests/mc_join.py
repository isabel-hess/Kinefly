# Join the multicast group 239.10.10.10 on the camera's interface, so Windows tells
# the NIC to accept it, and hold the membership for N seconds.  Also counts any
# datagrams that reach this socket (the camera streams to UDP port 49152, but
# Arena's filter driver may consume them first).
#   python mc_join.py SECONDS [LOCAL_IP]
import socket
import struct
import sys
import time

seconds = float(sys.argv[1])
local_ip = sys.argv[2] if len(sys.argv) > 2 else '169.254.168.2'
group = '239.10.10.10'

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('', 49999)) # Any port; joining is what matters.
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack('4s4s', socket.inet_aton(group), socket.inet_aton(local_ip)))
s.settimeout(0.2)
print('[join] joined %s on %s for %.0f s' % (group, local_ip, seconds), flush=True)
n = 0
t_end = time.time() + seconds
while time.time() < t_end:
    try:
        s.recv(65536); n += 1
    except socket.timeout:
        pass
s.close()
print('[join] left the group; %d datagrams reached this socket' % n, flush=True)
