from socket import socket, AF_INET, SOCK_DGRAM

ADDRESS = "172.20.10.6"
PORT = 5555

s = socket(AF_INET, SOCK_DGRAM)

while True:
    msg = input("> ")
    # print(msg)
    s.sendto(msg.encode(), (ADDRESS, PORT))

s.close()

# initial angles
# was 0 -135 150 40 0 0

# work angles
# was 0 -35 50 0 0 0
# gcs 62 -63 398 -75 0 -90

'''
wcs 85 -44 411 -59 2 -79
wcs 54 -99 391 -48 5 -123
'''

