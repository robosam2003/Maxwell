import can
import time
# Candlelight firmware on Linux
#bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)



# Stock cantact driver
bus = can.interface.Bus(bustype="slcan", channel="COM4", bitrate=500000)

msg = can.Message(arbitration_id=0x123,
                  data=[0, 0, 0, 0],
                  is_extended_id=False)



i=0
last_time = time.time()
# while True:
#     try:
#         i = i % 256
#         current_time = time.time()
#         msg.timestamp = current_time
#         msg.data[0] = i
#         msg.data[1] = (i+1) % 256
#         msg.data[2] = (i+2) % 256
#         msg.data[3] = (i+3) % 256
#
#         bus.send(msg)
#         print("Message : {} sent, Freq: {}".format(msg, 1/(current_time-last_time)))
#         time.sleep(1/100)
#         i+=1
#         last_time = current_time
#     except Exception as e:
#         print("Message NOT sent : {}".format(e))
#         bus.shutdown()
#         break

while True:
    # Just read messages
    try:
        message = bus.recv(timeout=1)
        if message is not None:
            print("Received message: {}".format(message))
    except Exception as e:
        print("Error receiving message: {}".format(e))
        bus.shutdown()
        break

