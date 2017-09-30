# This script is for bosun role, it responsible to get the commands through socket, and assembly to Ardruino
# if it is the CAST command, then send the command of the diveplan/rudder/thruster to Ardruino

# return the integer
def calChecksum(msg):
    i = 0
    checksum = 0
    while i < len(msg):
        checksum += int('0x'+msg[i],base=16)#checksum ^= ord(msg[i]) # jonathan ask for add calculation, but checksum is XOR
        i += 1
    print("checksum is %02X" % (checksum))
    return checksum

##########The below is the serial comm#############
import sys
import serial

EnvWINDOWS = False
EnvLINUX = True

ser = serial.Serial()
ser.bytesize = 8;
ser.parity = 'N';
ser.stopbits = 1;
ser.xonxoff = 0;
#ser.rtscts = 0
if EnvWINDOWS == True:
    ser.port = 'COM5'
    ser.baudrate = 57600
    ser.timeout = 1.0;
elif EnvLINUX == True:
    ser.port = '/dev/ttyS0'
    ser.baudrate = 57600
    ser.timeout = 1.0;

try:
    ser.open()
    print("haha")
except Exception as e:
    sys.stderr.write('--- ERROR opening new port: {} ---\n'.format(e))
    ser.close()
    sys.exit(-1)

#ser.write(b"Serial Com is Ready, begin to say something:\r\n")
# while True:
#     rcv = ser.readline()
#     rcv+=b'\r\n'
#     ser.write(rcv)
############End of Serial comm###########

##############Beginning of Socket Communication#######################
# this is socket client to send out the message port,
# it is TCP connection
import socket
import sys
import re

if EnvWINDOWS:
    server_ip = '127.0.0.1';
    server_port = 10000;
elif EnvLINUX:
    server_ip = '192.168.2.200'
    server_port = 10000;

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = (server_ip, server_port)
print('begin to connect to {} port {}'.format(*server_address))
SOCKCONNRETRY=True
skerror=0
while SOCKCONNRETRY:
    try:
        skerror=sock.connect_ex(server_address)
        if skerror==0 :
            SOCKCONNRETRY = False
            print('end to connecting socket')
        else :
            #print("socket connection error is ", format(skerror))
            print("socket connection error is %d"%(skerror))
            SOCKCONNRETRY = True
    except KeyboardInterrupt:
        print( 'END socket connection retry...')
        SOCKCONNRETRY = False
        sys.exit(-100)

# Following section, it receive the socket data from Pi TCP Server 203
try:
    while True: #amount_received < amount_expected:
        data = sock.recv(128)
        #amount_received += len(data)
        print('received {!r}'.format(data))
        aparser=re.compile('(\w\w):([-0-9]*)')
        match = aparser.search(data.decode('utf-8'))#convert byte-like obj into string
        motorStr='1001'
        if match:
            if match.group(1)=='RD':
                motorStr+='A0'
                motorStr+='02'
                aNum=0
                aNum=int(match.group(2))
                if aNum > 0:
                    motorStr=motorStr+'00'+str(aNum)
                elif aNum <0:
                    motorStr=motorStr+'01'+str(abs(aNum))
                elif aNum == 0:
                    motorStr=motorStr+'00'+str(0)
                motorStr+='1003'
            elif match.group(1)=='DP':
                motorStr+='A1'
                motorStr+='02'
                aNum=0
                aNum=int(match.group(2))
                if aNum > 0:
                    motorStr=motorStr+'00'+str(aNum)
                elif aNum <0:
                    motorStr=motorStr+'01'+str(abs(aNum))
                elif aNum == 0:
                    motorStr=motorStr+'00'+str(0)
                motorStr+='1003'
            elif match.group(1)=='TR':
                motorStr+='A2'
                motorStr+='02'
                aNum=0
                aNum=int(match.group(2))
                if aNum > 0:
                    motorStr=motorStr+'00'+str(aNum)
                elif aNum <0:
                    motorStr=motorStr+'01'+str(abs(aNum))
                elif aNum == 0:
                    motorStr=motorStr+'00'+str(0)
                motorStr+='1003'
            elif match.group(1)=='MS':
                motorStr+='A3'
                motorStr+='02'
                #TODO need to revisit the regex for MS command string
                aNum=0
                aNum=int(match.group(2))
                if aNum > 0:
                    motorStr=motorStr+'00'+str(aNum)
                elif aNum <0:
                    motorStr=motorStr+'01'+str(abs(aNum))
                elif aNum == 0:
                    motorStr=motorStr+'00'+str(0)
                motorStr+='1003'
        else:
            print('none is match')
        if match:
            mychecksum = calChecksum(motorStr)
            j=hex(mychecksum)
            #motorStr = motorStr + j[2:4]
            motorStr = motorStr + j[-2:]
            print('String send to Arduino and motor is %s'%(motorStr))
            #ser.write(motorStr.encode(encoding='utf-8'))
            ser.write(motorStr.decode('hex'))
        data=''

finally:
    print('closing socket')
    sock.close()
    data=''
##############End of Socket Communication#######################
