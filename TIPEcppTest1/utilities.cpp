#include "utilities.h"
#include <iostream>
#include <fstream>

vector<vector<int>> utilities::subset(vector<int> arg)
{
	//TODO: Find another way to generate the list of subsets, see the command line in CPLEX Studio
	//Condition d'arrêt
	if (arg.size() == 0)
	{
		vector<vector<int>> empty = { {} };
		return empty;
	}

	//We create a copy
	vector<int> newArg(arg);
	//We remove the last element
	newArg.pop_back();

	vector<vector<int>> Pn = subset(newArg);

	vector<vector<int>> truc;
	for (int x = 0; x < Pn.size(); x++)
	{
		vector<int> temp{ arg.back() };
		vector<int> combined;
		combined.reserve(temp.size() + Pn[x].size()); // preallocate memory
		combined.insert(combined.end(), temp.begin(), temp.end());
		combined.insert(combined.end(), Pn[x].begin(), Pn[x].end());
		truc.push_back(combined);
	}
	vector<vector<int>> allCombined;
	allCombined.reserve(truc.size() + Pn.size()); // preallocate memory
	allCombined.insert(allCombined.end(), Pn.begin(), Pn.end());
	allCombined.insert(allCombined.end(), truc.begin(), truc.end());

	return allCombined;
}

void utilities::print_subsets(vector<vector<int>> subsets)
{
	//Debug
	cout << "Subsets number: " << subsets.size() << endl;
	//This code prints all subsets to console in Python format
	cout << "Subsets: [";
	for (vector<int> r : subsets)
	{
		cout << "[";
		for (int t : r)
		{
			cout << t;
			if (t != r.back())
			{
				cout << ", ";
			}
		}
		cout << "]";
		if (r != subsets.back())
		{
			cout << ", ";
		}
	}
	cout << "]" << endl;
}