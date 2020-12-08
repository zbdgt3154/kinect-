#include "SportDetection.h"
// I am tanglx!
int main(int argc, char* argv[])
{
	SportDetection application;
	application.ProcessCommand(argc,argv);
	
	if (application.Init())
	{
		std::cout << "Start Data Collect!" << std::endl;
		application.Run();
	}
	else
	{
		std::cout << "No device!" << std::endl;
	}


	return 0;
}