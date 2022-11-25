#pragma once
#include <vector>

using namespace std;

static class utilities
{
public:
	/// <summary>
	/// Function used to generate all the subsets of a given set of numbers
	/// </summary>
	/// <param name="arg">The set to generate subsets from</param>
	/// <returns>The array of subsets</returns>
	static vector<vector<int>> subset(vector<int> arg);

	/// <summary>
	/// Utility function used to debug to console a list of subsets
	/// </summary>
	/// <param name="subsets">The list of subsets to print</param>
	static void print_subsets(vector<vector<int>> subsets);
};
