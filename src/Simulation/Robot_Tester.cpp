// Robot_Tester.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file




#include "pch.h"
#include <iostream>
#pragma comment (lib,"winmm")
#include "Robot_Tester.h"

void DisplayHelp()
{
	printf(
		"cls\n"
		"Start   Like hitting enable button from driver station\n"
		"Stop    for disable\n"
		"Game <0 auton 1 teleop 2 test>\n"
		"ShowControls <1=show>\n "
		"Help (displays this)\n"
		"\nType \"Quit\" at anytime to exit this application\n"
	);
}

/************************************************************************
* FUNCTION: cls(HANDLE hConsole)                                        *
*                                                                       *
* PURPOSE: clear the screen by filling it with blanks, then home cursor *
*                                                                       *
* INPUT: the console buffer to clear                                    *
*                                                                       *
* RETURNS: none                                                         *
*************************************************************************/
void cls(HANDLE hConsole = NULL)
{
	if (!hConsole)
		hConsole = ::GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coordScreen = { 0, 0 }; /* here's where we'll home the cursor */
	BOOL bSuccess;
	DWORD cCharsWritten;
	CONSOLE_SCREEN_BUFFER_INFO csbi; /* to get buffer info */
	DWORD dwConSize; /* number of character cells in the current buffer */
	/* get the number of character cells in the current buffer */
	bSuccess = GetConsoleScreenBufferInfo(hConsole, &csbi);
	//PERR(bSuccess, "GetConsoleScreenBufferInfo");   
	dwConSize = csbi.dwSize.X * csbi.dwSize.Y;   /* fill the entire screen with blanks */
	bSuccess = FillConsoleOutputCharacter(hConsole, (TCHAR) ' ',
		dwConSize, coordScreen, &cCharsWritten);
	//PERR(bSuccess, "FillConsoleOutputCharacter");   /* get the current text attribute */   
	bSuccess = GetConsoleScreenBufferInfo(hConsole, &csbi);
	//PERR(bSuccess, "ConsoleScreenBufferInfo");   /* now set the buffer's attributes accordingly */   
	bSuccess = FillConsoleOutputAttribute(hConsole, csbi.wAttributes, dwConSize, coordScreen, &cCharsWritten);
	//PERR(bSuccess, "FillConsoleOutputAttribute");   /* put the cursor at (0, 0) */   
	bSuccess = SetConsoleCursorPosition(hConsole, coordScreen);
	//PERR(bSuccess, "SetConsoleCursorPosition");   return; 
}

std::string DefaultCommandPrompt()
{
	return ">";
}

static std::function<std::string(void)> s_CommandPrompt = DefaultCommandPrompt;

void SetCommandPromptCallback(std::function<std::string(void)> callback)
{
	s_CommandPrompt = callback;
}

void CommandLineInterface()
{
	RobotTester _robot_tester;
	_robot_tester.RobotTester_create();
	_robot_tester.RobotTester_init();

	using namespace std;
	cout << endl;
	cout << "Ready." << endl;

	char input_line[128];
	while (cout << s_CommandPrompt(), cin.getline(input_line, 128))
	{
		char		command[32];
		char		str_1[MAX_PATH];
		char		str_2[MAX_PATH];
		char		str_3[MAX_PATH];
		char		str_4[MAX_PATH];

		command[0] = 0;
		str_1[0] = 0;
		str_2[0] = 0;
		str_3[0] = 0;
		str_4[0] = 0;

		if (sscanf(input_line, "%s %s %s %s %s", command, str_1, str_2, str_3, str_4) >= 1)
		{
			if (!_strnicmp(input_line, "test", 4))
			{
				_robot_tester.Test(atoi(str_1));
			}
			else if (!_strnicmp(input_line, "start", 5))
				_robot_tester.StartStreaming();
			else if (!_strnicmp(input_line, "stop", 5))
				_robot_tester.StopStreaming();
			else if (!_strnicmp(input_line, "game", 4))
				_robot_tester.SetGameMode(atoi(str_1));
			else if (!_strnicmp(input_line, "showc", 5))
				_robot_tester.ShowControls(atoi(str_1)!=0);
			else if (!_strnicmp(input_line, "cls", 3))
				cls();
			else if (!_strnicmp(input_line, "Help", 4))
				DisplayHelp();
			else if (!_strnicmp(input_line, "Quit", 4))
				break;
			else
				cout << "huh? - try \"help\"" << endl;
		}
	}
}



int main()
{
	//These commands help Sleep(1) to actually sleep for 1ms
	{ // Three magic lines of code
		TIMECAPS TimeCaps_;
		timeGetDevCaps(&TimeCaps_, sizeof(TimeCaps_));
		timeBeginPeriod(TimeCaps_.wPeriodMin);
	}

	DisplayHelp();
	CommandLineInterface();
}
