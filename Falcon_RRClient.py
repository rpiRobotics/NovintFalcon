# Falcon Client 
import RobotRaconteur as RR
import time

RRN = RR.RobotRaconteurNode.s

#Create a LocalTransport and register it
t1 = RR.LocalTransport()
RRN.RegisterTransport(t1)

#Create a TcpTransport and register it
t2 = RR.TcpTransport()
RRN.RegisterTransport(t2)

#Connect to the service
obj = RRN.ConnectService('tcp://localhost:2354/falconServer/Falcon')

# pos = obj.getPosition()
# print pos
raw_input("About to start Button check")
start = time.time()
now = start
while (now-start) < 5: #seconds
	print obj.getButtonStatus()
	now = time.time()

# force = float(raw_input("Enter A Force to Apply [N]\n"))
# while True:
	# obj.applyForce(force)
	# force = float(raw_input("Enter A Force to Apply [N]\n"))
	# if force == 717.0:
		# break

# Shutdown!
RRN.Shutdown()
