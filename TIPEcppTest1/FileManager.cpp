#include "FileManager.h"
#include <iostream>
#include <fstream>

vector<vector<string>> FileManager::read_file(string filename, bool EnableCout)
{
	std::ifstream file(filename, std::ifstream::binary);
	if (file.is_open())
	{
		vector<vector<string>> lineList;
		int wordNum = -1;
		char c = ',';
		while (file.good())
		{
			string myLine;
			std::getline(file, myLine);
			if (myLine == "\0")
			{
				//\0 is the null char, marking the end-of-file
				break;
			}
			//Cutting the line into words
			std::string delim = ", ";
			vector<string> wordList;
			auto start = 0U;
			auto end = myLine.find(delim);
			while (end != std::string::npos)
			{
				wordList.push_back(myLine.substr(start, end - start));
				start = end + delim.length();
				end = myLine.find(delim, start);
			}
			wordList.push_back(myLine.substr(start, end));
			//Init number of words
			if (wordNum == -1)
			{
				wordNum = wordList.size();
			}
			else if (wordNum != wordList.size())
			{
				cerr << "ERROR: File word count per line is not consistent!" << endl;
			}

			if (EnableCout) { cout << myLine << endl; }

			lineList.push_back(wordList);
		}
		return lineList;
		file.close();
	}
	else
	{
		cerr << "ERROR: Unable to read file" << endl;
		return vector<vector<string>> {};
	}
}

float** FileManager::read_standardized_csv_trucks(vector<vector<string>> lines, bool useTime, bool EnableCout)
{
	//We define our dynamic matrix here
	int row_colCount = int(sqrt(lines.size() - 1));
	float** Distance = new float* [row_colCount];
	for (int i = 0; i < row_colCount; ++i)
		Distance[i] = new float[row_colCount];

	//We ignore the first line
	for (int p = 1; p < lines.size(); p++)
	{
		int i = std::stoi(lines[p][0]);
		int j = std::stoi(lines[p][1]);
		Distance[i][j] = std::stof(lines[p][(useTime ? 2 : 3)]);
		if (EnableCout)
		{
			cout << "i: " << i << " j: " << j << " dist: " << Distance[i][j] << endl;
		}
	}
	return Distance;
}

float* FileManager::read_standardized_csv_drones(vector<vector<string>> lines, bool useTime, bool EnableCout)
{
	//We define our dynamic matrix here
	int row_colCount = int(sqrt(lines.size() - 1));
	float* Distance = new float[row_colCount];

	//We ignore the first line
	for (int p = 1; p < lines.size(); p++)
	{
		int i = std::stoi(lines[p][0]);
		Distance[i] = std::stof(lines[p][(useTime ? 6 : 7)]);
		if (EnableCout)
		{
			cout << "i: " << i << " dist: " << Distance[i] << endl;
		}
	}
	return Distance;
}