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
print "Connecting to Falcon..."
falcon = RRN.ConnectService('tcp://localhost:2354/falconServer/Falcon')

# Get the raw data from the falcon
raw_input("Press Enter to get all controller input")
falconInput = falcon.controller_input
for val in falconInput:
	print val

# Check the position for 5 seconds
raw_input("Press Enter to start Position Check")
start = time.time()
now = start
while(now-start) < 5:
	print falcon.position
	now = time.time()

# Check the button status for 5 seconds
raw_input("Press Enter to start Button check")
start = time.time()
now = start
while (now-start) < 5: #seconds
	print falcon.button_status
	now = time.time()

# Apply a force
# Set as [X,Y,Z] forces
force = float(raw_input("Enter A Force to Apply [N]\n"))
falcon.setForce([0.0, 0.0, force])

raw_input("Press Enter to Stop")
falcon.setForce([0.0,0.0,0.0])
	

# Shutdown!
RRN.Shutdown()
