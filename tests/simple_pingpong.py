__author__ = 'hd'

import sys
import time
import threading
from socketcan import CanMessage, CanSocket

do_quit = False

def can_echo_thread(interface):
    echo_sock = CanSocket()
    echo_sock.open(interface)
    while not do_quit:
        msg = echo_sock.read(0.1)
        if msg is not None:
            echo_sock.send(msg)


if_master = sys.argv[1]
if_slave = sys.argv[2]
iterations = int(sys.argv[3])

echo_thread = threading.Thread(target=can_echo_thread, args=(if_slave,))
echo_thread.start()

sock = CanSocket()
sock.open(if_master)
msg = CanMessage(0x13371010, [0,0,0,0,0,0,0,0])

t_start = time.time()

for i in range(0, iterations):
    sock.send(msg)
    msg = sock.read()
    x = msg.data[0]
    msg.data[0] = (x+1) % 256

t_finish = time.time()
do_quit = True

print("\n\nsent and received %d messages in %f seconds." % (iterations, t_finish-t_start))
