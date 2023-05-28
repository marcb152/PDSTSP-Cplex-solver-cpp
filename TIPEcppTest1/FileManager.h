#pragma once
#include <vector>
#include <string>
#include <set>

using namespace std;

static class FileManager
{
public:
	/// <summary>
	/// This function reads a csv file and outputs a list of lines containing a list of comma separated words
	/// It tells if the number of words is consistent or not
	/// </summary>
	/// <param name="filename">The path to find the csv file</param>
	/// <param name="EnableCout">Enable debug output to console</param>
	/// <returns>A list of lines containing a list of words</returns>
	static vector<vector<string>> read_file(string filename, bool EnableCout = false);

	/// <summary>
	/// This is a parser, parses the wanted csv file to retrieve the wanted matrices
	/// </summary>
	/// <param name="lines">The csv files, already organized</param>
	/// <param name="EnableCout">Enable debug output to console</param>
	/// <returns>A matrix containing the data of the csv</returns>
	static float** read_standardized_csv_trucks(vector<vector<string>> lines, bool useTime, bool EnableCout = false);

	/// <summary>
	/// This is a parser, parses the wanted csv file to retrieve the wanted list
	/// </summary>
	/// <param name="lines">The csv files, already organized</param>
	/// <param name="EnableCout">Enable debug output to console</param>
	/// <returns>A list containing the data of the csv</returns>
	static float* read_standardized_csv_drones(vector<vector<string>> lines, bool useTime, bool EnableCout = false);

	/// <summary>
	/// This is a parser, parses the wanted csv file to retrieve the wanted list
	/// </summary>
	/// <param name="lines">The csv files, already organized</param>
	/// <param name="EnableCout">Enable debug output to console</param>
	/// <returns>A list containing the data of the csv</returns>
	static std::set<int> get_drone_clients_csv(vector<vector<string>> lines, bool EnableCout = false);
};
