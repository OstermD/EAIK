#include <iostream>
#include "IK_system_tests_6R.h"
#include "IK_system_tests_5R.h"
#include "IK_system_tests_3R.h"

int main(int argc, char *argv[])
{
    bool allPass = run_5R_Tests();
    //allPass &= run_3R_Tests();
    //allPass &= run_5R_Tests();

	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass)
	{
		std::cout<< "                     ALL PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     SOME FAILED                     "<<std::endl;
	}

	std::cout<< "==================================================="<<std::endl;
}