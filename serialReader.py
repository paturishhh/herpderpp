import serial, time
arduino = serial.Serial('COM7', 9600, timeout=0.01)
# bytes([0xFF,0x00,0x01,0x03, 0x00, 0xFE])
# 'Magsend ka naaaa'.encode('utf-8')

def convertStringToByteArray(packetCommand):
	"Converts the string input into byte array read by node"
	parsedCommand = packetCommand.split(" ")
	convertedCommand = []

	for x in parsedCommand:
		# convertedCommand.append(bytes.fromhex(x))
		byteVersion = bytes.fromhex(x);
		convertedCommand.append(int.from_bytes(byteVersion, byteorder = 'little', signed = False))

	print(convertedCommand)
	# print(type(convertedCommand))
	return convertedCommand

packetCommand = input("Enter command: ")		
	

#convert input to bytes
convertedCommand = convertStringToByteArray(packetCommand)


#trying to automate sending byte per byte to arduino
for x in range(0, len(convertedCommand)):
	arduino.write([convertedCommand[x]])
# time.sleep(2)	
# arduino.write([convertedCommand[1]])
# time.sleep(2)
while True:
	time.sleep(2)
	# arduino.write(bytes([0xFF,0x00,0x01,0x03, 0x00, 0xFE])
	data = arduino.readline()[:-2] #removes /r and /n
	if data:
		print (data)
	else:
		# print('no data')
		packetCommand = input("Enter command: ")		
		convertedCommand = convertStringToByteArray(packetCommand)
		for x in range(0, len(convertedCommand)):
			arduino.write([convertedCommand[x]])