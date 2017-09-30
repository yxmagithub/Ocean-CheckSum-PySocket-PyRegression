# This script is for translating the incoming SMS format into the communication network
# if it is the CAST command, then route to the corresponding ip end point
# else if it is the SURMERGE command, then route to the simulink Pi board or INS
# else if it is the FLAG NAVIGATE INFO, then route the INS with x.x.x.210

EnvWINDOWS = False
EnvLINUX = True
import re

def dolphinSMSParser(buf):
    # Parse the buffer.
    retWd = list();
    #parser = re.compile(r"SMS\:(\d{4})(?:\,[^|]*)\|(.*)\s+");
    parser = re.compile(r"SMS\:(\d{4})");
    match = parser.search(buf);
    if match:
        print( 'Got SMS.');
        print( 'Address:', match.group(1));
        #if match.group(1) == '2009':
        if match.group(1) == '2010':
            print( 'Identifying message content...');

            match=re.search(r'\|.+',buf)
            str=match.group()
            msg_parser = re.compile(r"(\d{2})(\d{8})([CD])(\d{9})([CD])(\d{8})(\d{5})(\d{3})000")
            msg_match = msg_parser.search(str)

            cmd_parser = re.compile(r'RD:[-0-9]*|DP:[-0-9]*|TR:[-0-9]*|MS:[rdt]:[-0-9]*')
            cmd_match = cmd_parser.search(str)
            if msg_match:
                print( 'Message was recognized. Now parsing...');

                message_id = int(msg_match.group(1))
                timestamp = float(msg_match.group(2))/1000.0
                longitude = float(msg_match.group(4))/1000000.0
                if msg_match.group(3) == 'D':
                    longitude = -longitude
                latitude = float(msg_match.group(6))/1000000.0
                if msg_match.group(5) == 'D':
                    latitude = -latitude
                depth = float(msg_match.group(7))/10.0
                standard_deviation = float(msg_match.group(8))/10.0

                # print( 'Values extracted from SMS string:');
                # print( 'ID:', message_id);
                # print( 'TS:', timestamp);
                # print( 'LON:', longitude);
                # print( 'LAT:', latitude);
                # print( 'DEPTH:', depth);
                # print( 'SD:', standard_deviation);
                #retWd.append('message')
                retWd.extend(('message',message_id,timestamp,longitude,latitude,depth,standard_deviation))
                aMsg=dolphinMsgAPOSPSIMLBP(timestamp,longitude,latitude,depth,standard_deviation)
                dolphinSendUDP(aMsg,AGGIE_IMU_IPaddress,AGGIE_IMU_Port)
            elif cmd_match:
                print('command is found')
                #retWd.append('command')
                retWd.extend(('command', cmd_match.group()))
            else :
                print( 'Message was not from Becon.')
                retWd.append('none')


        # Iterate through all groups returned from match. This is just for testing purposes.
        #for index in range(1, match.lastindex+1):
            #print match.group(index)

        # Remove all data from buffer from index 0 up to the last index of the match.
        # This keeps the buffer from growing uncontrollably large as junk or otherwise
        # unparsable data enters the buffer.
        buf = buf[match.end():]
        buf = buf.strip()
    return retWd

import time

def dolphinMsgAPOSPSIMLBP(ts,lon,lat,dep,sd):
    print('assembly the downstream msg, and be ready to send to IMU')
    myMsgAPOS='PSIMLBP,'
    myMsgAPOS+=time.strftime('%H%M%S.%S', time.gmtime(ts))
    myMsgAPOS += ','
    myMsgAPOS+='Tp1,'
    myMsgAPOS += ','
    myMsgAPOS +='A,'
    myMsgAPOS +='R,' #GPS is Radian cordination system, verify by Nabil
    myMsgAPOS +=str(lon)
    myMsgAPOS += ','
    myMsgAPOS +=str(lat)
    myMsgAPOS += ','
    myMsgAPOS +=str(dep)
    myMsgAPOS +=','
    myMsgAPOS +='0.0,'
    myMsgAPOS +='0.0,'
    myMsgAPOS +='0.0,'
    myMsgAPOS +=str(sd)
    #we need the checksum
    i = 0
    checksum = 0
    while i < len(myMsgAPOS):
        checksum ^= ord(myMsgAPOS[i])
        i += 1
    print("checksum is %02X" %(checksum))
    numHex=hex(checksum)
    #myMsgAPOS ='$'+myMsgAPOS+'*'+numHex[2:4]+"\r\n"
    myMsgAPOS = '$' + myMsgAPOS + '*' + numHex[-2:] + "\r\n"
    print("The whole APOSPSI msg is %s"%(myMsgAPOS))
    return myMsgAPOS

###############IP UDP ############
import socket
if EnvWINDOWS==True:
    AGGIE_IMU_IPaddress='192.168.2.210'
    AGGIE_IMU_Port=9999
elif EnvLINUX==True:
    AGGIE_IMU_IPaddress = '192.168.2.210'
    AGGIE_IMU_Port=9999

def dolphinSendUDP(message, IMU_IPaddress, IMU_Port):
    # Create a UDP socket
    mySockUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_server_address = (IMU_IPaddress, IMU_Port)

    try:
        # Send data
        print('sending {!r}'.format(message))
        sent = mySockUDP.sendto(message.encode(encoding='utf-8'), udp_server_address)
        print('UDP sending succeed')
        # Receive response
        #print('waiting to receive')
        #data, server = mySockUDP.recvfrom(4096)
        #print('received {!r}'.format(data))
    finally:
        print('closing socket')
        mySockUDP.close()
##########end of IP UDP###########

##########The beginning of the serial comm#############
import serial
import sys

ser = serial.Serial()

ser.bytesize = 8;
ser.parity = 'N';
ser.stopbits = 1;
ser.timeout = 3.0;
ser.xonxoff = 0;
#ser.rtscts = 0
if EnvWINDOWS == True:
    ser.baudrate = 9600
    ser.port = 'COM5'
elif EnvLINUX == True:
    ser.baudrate =9600
    ser.port = '/dev/ttyS0'

try:
    ser.open()
    print('open serial port succeed!')
except Exception as e:
    sys.stderr.write('--- ERROR opening new port: {} ---\n'.format(e))
    ser.close()
    sys.exit(-1)

ser.write(b"The Serial Comm is Ready, please be Ready to say something:\r\n")
# while True:
#     #ser.write("\r\nSay something:")
#     #print("Print Say something\n")
#     rcv = ser.readline() #readlineCR(ser)
#     rcv+=b'\r\n'
#     ser.write(rcv)
############End of Serial comm###########

#############Beginning of TCP server #############
#this is socket server script socket_tcp_server.py
import time
if EnvWINDOWS:
    server_ip='192.168.2.147';
    server_port = 10000;
elif EnvLINUX:
    server_ip='192.168.2.200'
    server_port=10000;

# Create a TCP/IP socket
mySockTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = (server_ip, server_port)
print('starting up on {} port {}'.format(*server_address))
mySockTCP.bind(server_address)

# Listen for incoming connections
mySockTCP.listen(1)

srvWait=True
while srvWait:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = mySockTCP.accept()
    print('connection from', client_address)
    srvWait=False
##########end of TCP Socket Server ##############

########main loop function###################
try:
    # Receive the data in small chunks and retransmit it
    while True:
        # data = connection.recv(16)
        # print('received {!r}'.format(data))
        # if data:
        #     print('sending data back to the client')
        #     connection.sendall(data)
        # else:
        #     print('no data from', client_address)
        #     #break
        #if (ser.inWaiting() > 0):
        rcv = ser.readline()
        #rcv = ser.read(16)
        retlist=list()
        if len(rcv)!=0:
            print('serial comm received line is',format(rcv))
            retlist = dolphinSMSParser(rcv.decode('utf-8'))#decode byte array into string

        #cmdStr="RD:30"
        if retlist :
            if retlist[0]=='command':
                print('command is identified')
                connection.sendall(
                    retlist[1].encode(encoding='utf-8'))  # this method does not support unicode, change encoding to utf-8
            elif retlist[0]=='message':
                print('message is identified')
                #TODO should send to 210 INS/IMU
                #already send out UDP package inside the dolphinSMSParser()
            elif retlist[0]=='none':
                print('nothing is identified')
        else:
            print('retlist is not ready')

        time.sleep(1)

finally:
    # Clean up the connection
    mySockTCP.close()
    connection.close()
