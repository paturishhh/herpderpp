import collections, binascii, time, MySQLdb
import serial, datetime
from array import array

QUEUE_SIZE = 3;
packetQueue = collections.deque()

def parsePacket():
    "gets the packet from queue & parse commands received from node"
    print("parse parse")
    packetDetails = array('I') #stores packet details
    packet = packetQueue.popleft()
    isValid = False
    
    print(packet)
    packet = bytearray(packet) #bytearray(b'\x01\x00\x03\x0b\xfe')

    index = 0
    for b in packet: #255, 1, 0, 3, 15, 0, 0, 0, 1, 254
        if index == 9:#port data 2
            if b == 254:
                print("footer found")
                isValid = True    
        elif index == 8: #port data 1
            packetDetails.append(b)
            if (cur_count == count) == True:
                index = 9
            else:
                cur_count += 1
                index = 6
        elif index == 7: #port number 2
            packetDetails.append(b)
            index = 8
        elif index == 6: #port number 1
            packetDetails.append(b)
            index = 7
        elif index == 5: #count
            packetDetails.append(b)
            count = b
            cur_count = 0
            index = 6
        elif index == 4: #command
            packetDetails.append(b)
            if b == 15:
                index = 5
            else:
                index = 9
        elif index == 3: #api
            index = 4
        elif index == 2: #dest 
            index = 3
        elif index == 1: #source address
            packetDetails.append(b)
            index = 2
        elif index == 0:
            if b == 255:
                print("header found")
                index = 1
    if isValid == True:
        #insertToDatabase
        print("insert db")
        if packetDetails[1] == 15: 
            insertPortData(packetDetails)
        else:
            insertCommand(packetDetails)
    else:
        print("invalid packet")
    
def readSerial():
    arduino = serial.Serial()
    arduino.port = 'COM4'
    arduino.baudrate = 9600
    arduino.timeout = None
    arduino.open()

    if arduino.is_open:
        time.sleep(2) #wait
        while arduino.in_waiting > 0: #while there is data
            data = arduino.readline()[:-2] #removes /r and /n
            
            if len(packetQueue) != QUEUE_SIZE:
                packetQueue.append(data) #insert data to queue
                parsePacket() #parse one when full
                # print(data)
                # print(len(packetQueue))
            else:
                parsePacket() #parse one when full
                packetQueue.append(data)
                print('back')
                print(list(packetQueue))
        while len(packetQueue) != 0: #just empty the queue if there is no message
            parsePacket() 
        arduino.close() #close serial
    else:
        print('derp')

def insertPortData(packetDetails): #remember to close db after access    
    database = MySQLdb.connect(host="localhost", user ="root", passwd = "root", db ="thesis")
    # Create a Cursor object to execute queries.
    cur = database.cursor()
    
    # Select data from table using SQL query.
    sql = "SELECT * FROM node WHERE node_id = '%d'" % packetDetails[0]
    print(sql)
    cur.execute(sql)
     
    # print the first and second columns      
    for row in cur.fetchall():
      print (row[0], " ", row[1])
    
    database.close()
    #cur.fetchone() one row
    
def insertCommand(packetDetails):
    "save commands on database"
    print(type(packetDetails))
    nodeAddr = packetDetails[0]
    database = MySQLdb.connect(host="localhost", user ="root", passwd = "root", db ="thesis")
    cur = database.cursor()
    sql = "SELECT * FROM node WHERE node_id = '%d'" % nodeAddr

    try:
        cur.execute(sql)
        result = cur.fetchone()
        
        if cur.rowcount == 0: #new node
            addNode(nodeAddr)
        else:
            print("proceed with life")

        # sql = "INSERT INTO port value(node_id, port_value, time_stamp) VALUES ('%d', '%d', '%s')" % (nodeAddr, portValue, timeStamp)
        curr_time = time.localtime()
        # print(curr_time)
        commandCode = packetDetails[1]
        # print(commandCode)
        timeStamp = time.strftime('%Y-%m-%d %H:%M:%S', curr_time)
        # print(timeStamp)
        sql = "INSERT INTO command(node_id, command_code, time_stamp) VALUES ('%d', '%d', '%s')" % (nodeAddr, commandCode, timeStamp)
        cur.execute(sql)
        database.commit()
        database.close()
    except:
        database.rollback()
        print("error desu")

def addNode(nodeAddr):
    "add a new node when a new node is detected"
    print("@ add node")
    database = MySQLdb.connect(host="localhost", user ="root", passwd = "root", db ="thesis")
    cur = database.cursor()
    sql = "INSERT INTO node(node_address_physical, node_address_logical,node_active) VALUES ('%d', '%d', '%d')" % (nodeAddr, 0, 1)

    try:
        cur.execute(sql)
        database.commit()
    except:
        print("error")
        database.rollback()
    database.close()

def inputConfiguration(command):
    "Convert configuration to bytes for saving in the database"               
    command = bytearray(command, 'utf-8')
    print(hex(command[0]))
    print(command)
    # for segment in command:
    #     print('{0:02x}'.format(segment))
    #     print(type(segment))

# insert to database
    # send data back if there is something there
    #arduino.write
    #time.sleep

#main program
choice = 0
while choice != 3:
    print('*************************')
    # print('Command line control')
    print('Choose the corresponding number for command: ')
    print('1. Read WSAN data')
    print('2. Send WSAN config')
    print('3. Exit')
    print('*************************')
    choice = input('Enter choice: ')

    if choice == '1':
        readSerial()
    elif choice == '2':
        
        # # connectDatabase()
        # np.set_printoptions(formatter={'int': hex})
        # testPacket = [255, 0, 1, 3, 0, 255, 0, 1, 90, 254]
        # x = np.array(testPacket)
        # # x = ("0x" + ("{:0>2x}" * len(x))).format(*tuple(x))
        # x = (("{:0>2x}" * len(x))).format(*tuple(x))
        # #convert to hex string
        # packetQueue.append(x)
        
        # insertPortData(parsePacket()) # testing
        #write serial
        command = input('Enter packet to send: ')
        inputConfiguration(command)
    elif choice == '3':
        exit()

