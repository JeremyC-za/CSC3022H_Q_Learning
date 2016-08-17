#pragma once
#include "cdisccontroller.h"
#include "CParams.h"
#include "CDiscCollisionObject.h"
#include <cmath>

typedef unsigned int uint;
class CQLearningController :
	public CDiscController
{
private:
	uint _grid_size_x;
	uint _grid_size_y;
public:
	CQLearningController(HWND hwndMain);
	virtual void InitializeLearningAlgorithm(void);
	double R(uint x, uint y, uint sweeper_no);
	virtual bool Update(void);
	virtual ~CQLearningController(void);
	std::vector<std::vector<std::vector<double>>> Qtables;		//Qtable: [sweeper [x [y]]]
	std::vector<std::vector<int>> knownMap;						//knownMap: [x[y]]
	int mineCost = 100, superMineCost = -100, rockCost = -50;
};

