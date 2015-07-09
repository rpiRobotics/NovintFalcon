// Comment the following line to get a console window on startup
//#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include "falcon_impl.h"

using namespace RobotRaconteur;



int main(int argc, char *argv[])
{
	//Initialize the Falcon obj
	RR_SHARED_PTR<Falcon_impl> Falcon = RR_MAKE_SHARED<Falcon_impl>();
	
	//Create local transport
	RR_SHARED_PTR<LocalTransport> t1 = RR_MAKE_SHARED<LocalTransport>();
	t1->StartServerAsNodeName("falconServer");
	RobotRaconteurNode::s()->RegisterTransport(t1);

	//Initialize the TCP transport and start listening on port 2354
	RR_SHARED_PTR<TcpTransport> t = RR_MAKE_SHARED<TcpTransport>();
	t->StartServer(2354);

	//Enable auto-discovery announcement
	t->EnableNodeAnnounce(IPNodeDiscoveryFlags_LINK_LOCAL | IPNodeDiscoveryFlags_SITE_LOCAL | IPNodeDiscoveryFlags_NODE_LOCAL);

	//Register the TCP transport
	RobotRaconteurNode::s()->RegisterTransport(t);

	//Register the Falcon interface type so that the node can understand the service definition
	RobotRaconteurNode::s()->RegisterServiceType(RR_MAKE_SHARED<falcon_serviceFactory>());

	//Register the Falcon object as a service so that it can be connected to
	RobotRaconteurNode::s()->RegisterService("Falcon","falcon_service",Falcon);

	//Stay open until shut down
	cout << "Falcon server started. Connect with URL\n"
		"\t'tcp://localhost:2354/falconServer/Falcon' \n"
		"available functions: \n\tsetForce(double force[3])\n"
		"available propeties: \n\tcontroller_input\n\tposition\n\tbutton_status\n\n"
		<< endl;
	
	cout << "Press enter to quit" << endl;
	getchar();

	//Shutdown the node. This must be called at program exit
	RobotRaconteurNode::s() ->Shutdown();

	return 0;
}
