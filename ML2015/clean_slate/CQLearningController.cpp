/**
         (                                      
   (     )\ )                                   
 ( )\   (()/(   (    ) (        (        (  (   
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(  
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\  
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_) 
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |  
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |  
                                        |___/   

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"


CQLearningController::CQLearningController(HWND hwndMain):
	CDiscController(hwndMain),
	_grid_size_x(CParams::WindowWidth / CParams::iGridCellDim + 1),
	_grid_size_y(CParams::WindowHeight / CParams::iGridCellDim + 1)
{
}
/**
 The update method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm(void)
{
	std::cout << "Initializing learning algorithm..." << std::endl;

	knownMap.resize(_grid_size_x);
	for (int i = 0; i < _grid_size_x; i++){
		knownMap[i].resize(_grid_size_y);
	}

	for (int i = 0; i < m_vecObjects.size(); i++){
		SVector2D<int> coordinates = m_vecObjects[i]->getPosition();
		if (m_vecObjects[i]->getType() == CCollisionObject::Mine){
			knownMap[coordinates.x / 10][coordinates.y / 10] = mineCost;		// so they know when they've reached a mine
		}
	}

	Qtables.resize(m_vecSweepers.size());
	std::cout << "Filling QTables for each sweeper..." << std::endl;
	for (int i = 0; i < m_vecSweepers.size(); i++){		// for each sweeper
		Qtables[i].resize(_grid_size_x);
		for (int j = 0; j < _grid_size_x; j++){			// for each x value
			Qtables[i][j].resize(_grid_size_y);
			for (int k = 0; k < _grid_size_y; k++){		// for each y value
				if (knownMap[j][k] == mineCost || knownMap[j][k] == superMineCost || knownMap[j][k] == rockCost){  // they'll never equal superminecost and rockcost, but for some reason when I remove those || statements, it messes up the number of mines gathered, saying 65 out of 40 mines picked up etc...
					Qtables[i][j][k] = knownMap[j][k]; // basically sets the mines cost
				}
				else{
					Qtables[i][j][k] = 0;
				}
			}
		}
	}


	std::cout << "Learning algorithm completed..." << std::endl;
}
/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no){
	return Qtables[sweeper_no][x][y];
}
/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
int iterations = 0;
bool CQLearningController::Update(void)
{
	if (::iterations != m_iIterations){
		std::cout << std::endl << "Starting iteration " << iterations << "..." << std::endl;
		std::cout << "Previous deaths: " << m_vecDeaths[::iterations] << std::endl;
		std::cout << "Previous max mines gathered: " << m_vecMostMinesGathered[::iterations] << std::endl;
		std::cout << "Previous average mines gathered: " << m_vecAvMinesGathered[::iterations] << std::endl;
		std::cout << "Total mines gathered: " << (int)(m_vecAvMinesGathered[::iterations] * m_NumSweepers) << " out of " << m_NumMines << std::endl;
		::iterations = m_iIterations;
	}

	//m_vecSweepers is the array of minesweepers
	//everything you need will be m_[something] ;)
	uint cDead = std::count_if(m_vecSweepers.begin(),
							   m_vecSweepers.end(),
						       [](CDiscMinesweeper * s)->bool{
								return s->isDead();
							   });
	if (cDead == CParams::iNumSweepers){
		std::cout << "All sweepers are dead: skipping to next iteration..." << std::endl;
		m_iTicks = CParams::iNumTicks;
	}

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		/**
		Q-learning algorithm according to:
		Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
		*/
		//1:::Observe the current state:
		SVector2D<int> pos = m_vecSweepers[sw]->Position();
		int xCord = pos.x / 10;
		int yCord = pos.y / 10;
		double currentState = Qtables[sw][xCord][yCord];
		double down, up, left, right;

		if (xCord == 40){
			right = Qtables[sw][0][yCord];
		}
		else{
			right = Qtables[sw][xCord + 1][yCord];
		}

		if (xCord == 0){
			left = Qtables[sw][40][yCord];
		}
		else{
			left = Qtables[sw][xCord - 1][yCord];
		}

		if (yCord == 40){
			up = Qtables[sw][xCord][0];
		}
		else{
			up = Qtables[sw][xCord][yCord + 1];
		}

		if (yCord == 0){
			down = Qtables[sw][xCord][40];
		}
		else{
			down = Qtables[sw][xCord][yCord - 1];
		}

		//2:::Select action with highest historic return:

		double maxLR = (left, right);
		double maxUD = (down, up);
		double max = max(maxLR, maxUD);

		if (left == right && up == down && left == up){	// if all equal, can move back to original position
			int temp = RandInt(0, 3);
			if (temp == 0){
				m_vecSweepers[sw]->setRotation(WEST);
			}
			else if (temp == 1){
				m_vecSweepers[sw]->setRotation(EAST);
			}
			else if (temp == 2){
				m_vecSweepers[sw]->setRotation(NORTH);
			}
			else if (temp == 3){
				m_vecSweepers[sw]->setRotation(SOUTH);
			}
			
		}
		else if (maxLR == maxUD){	// if two equal
			int temp = RandInt(0, 1);
			if (temp == 0){
				if (max == left) m_vecSweepers[sw]->setRotation(WEST);
				else if (max == right) m_vecSweepers[sw]->setRotation(EAST);
				else if (max == up) m_vecSweepers[sw]->setRotation(NORTH);
				else if (max == down) m_vecSweepers[sw]->setRotation(SOUTH);
			}
			else if (temp == 1){
				if (max == down) m_vecSweepers[sw]->setRotation(SOUTH);
				else if (max == up) m_vecSweepers[sw]->setRotation(NORTH);
				else if (max == right) m_vecSweepers[sw]->setRotation(EAST);
				else if (max == left) m_vecSweepers[sw]->setRotation(WEST);
			}
		}
		else{
			if (max == left) m_vecSweepers[sw]->setRotation(WEST);
			else if (max == right) m_vecSweepers[sw]->setRotation(EAST);
			else if (max == up) m_vecSweepers[sw]->setRotation(NORTH);
			else if (max == down) m_vecSweepers[sw]->setRotation(SOUTH);
		}
		

		//now call the parents update, so all the sweepers fulfill their chosen action
	}
	
	CDiscController::Update(); //call the parent's class update. Do not delete this.

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){

		double learningRate = 0.1;
		double discountFactor = 0.5;

		if (m_vecSweepers[sw]->isDead()){
			SVector2D<int> Prevpos = m_vecSweepers[sw]->PrevPosition();
			int prevXCord = Prevpos.x / 10;
			int prevYCord = Prevpos.y / 10;
			SVector2D<int> pos = m_vecSweepers[sw]->Position();
			int xCord = pos.x / 10;
			int yCord = pos.y / 10;
			Qtables[sw][xCord][yCord] = -100;
			Qtables[sw][prevXCord][prevYCord] = Qtables[sw][prevXCord][prevYCord] + learningRate * (R(prevXCord, prevYCord, sw) + (discountFactor * Qtables[sw][xCord][yCord]) - abs(Qtables[sw][prevXCord][prevYCord]));
			continue;
		}
		//compute your indexes.. it may also be necessary to keep track of the previous state
		SVector2D<int> Prevpos = m_vecSweepers[sw]->PrevPosition();
		int prevXCord = Prevpos.x / 10;
		int prevYCord = Prevpos.y / 10;
		SVector2D<int> pos = m_vecSweepers[sw]->Position();
		int xCord = pos.x / 10;
		int yCord = pos.y / 10;


		//3:::Observe new state:

		double max = Qtables[sw][xCord][yCord];

		//4:::Update _Q_s_a accordingly:

		Qtables[sw][prevXCord][prevYCord] = Qtables[sw][prevXCord][prevYCord] + learningRate * (R(prevXCord, prevYCord, sw) + (discountFactor * max) - abs(Qtables[sw][prevXCord][prevYCord]));
	}
	return true;
}

CQLearningController::~CQLearningController(void)
{
	// using vectors, no need to deallocate memory	
}
