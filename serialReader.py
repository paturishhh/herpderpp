import collections, binascii, time
import serial

QUEUE_SIZE = 2;
packetQueue = collections.deque()

def parsePacket():
	"gets the packet from queue & parse according to protocol"
	print("parse parse")
	packet = packetQueue.popleft()
	isValid = False
	index = 0
	if packet[index] == 255:
		print("header found")
		index += 1#I can't for loop coz I need the position
		sourceAddr = packet[index]
		index += 1
		destAddr = packet[index]
		index += 1
		api = packet[index]
		index += 1
		if api == 2:
			configVersion = packet[index]
		elif api == 3:
			commandType = packet[index]
		index += 1
		if api == 3 and commandType !=2 or commandType !=15:
			#expect footer next
			if packet[index] == 254:
				print("footer found")
				isValid = True
	


def readSerial():
	arduino = serial.Serial()
	arduino.port = 'COM7'
	arduino.baudrate = 9600
	arduino.timeout = None
	arduino.open()

	if arduino.is_open:
		time.sleep(2)
		while arduino.in_waiting > 0: #while there is data
			data = arduino.readline()[:-2] #removes /r and /n
			
			if len(packetQueue) != QUEUE_SIZE:
				packetQueue.append(data) #insert data to queue
				print(data)
				print(len(packetQueue))
			else:
				parsePacket() #start one when full
				packetQueue.append(data)
				print('back')
				print(list(packetQueue))
		# parsePacket() #parse is none
	else:
		print('derp')

				

# insert to database
	# send data back if there is something there
	#arduino.write
	#time.sleep

#main program

print('Command line control')
print('1. Read WSAN data')
print('2. Send WSAN config')
print('3. Exit')
choice = input('Enter your command: ')

if choice == '1':
	readSerial()
elif choice == '2':
	print('derp')
	#write serial
elif choice == '3':
	exit()

