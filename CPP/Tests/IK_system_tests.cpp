#include <iostream>
#include "IK_system_tests_6R.h"
#include "IK_system_tests_5R.h"
#include "IK_system_tests_4R.h"
#include "IK_system_tests_3R.h"
#include "IK_system_tests_2R.h"
#include "IK_system_tests_1R.h"

int main(int argc, char *argv[])
{
	std::cout<< std::endl<<"====================== 6R: ======================"<<std::endl;
    bool allPass6 = run_6R_Tests();
	std::cout<< std::endl<<"====================== 5R: ======================"<<std::endl;
    bool allPass5 = run_5R_Tests();
	std::cout<< std::endl<<"====================== 5R: ======================"<<std::endl;
    bool allPass4 = run_4R_Tests();
	std::cout<< std::endl<<"====================== 3R: ======================"<<std::endl;
    bool allPass3 = run_3R_Tests();
	std::cout<< std::endl<<"====================== 2R: ======================"<<std::endl;
    bool allPass2 = run_2R_Tests();
	std::cout<< std::endl<<"====================== 1R: ======================"<<std::endl;
    bool allPass1 = run_1R_Tests();
	
	std::cout<< std::endl<<"====================== RESULT: ======================"<<std::endl;

	if(allPass6 && allPass5 && allPass4 && allPass3 && allPass2 && allPass1 )
	{
		std::cout<< "                     ALL PASSING                     "<<std::endl;
	}
	else
	{
		std::cout<< "                     SOME FAILED                     "<<std::endl;
	}

	std::cout<< "====================================================="<<std::endl;
}