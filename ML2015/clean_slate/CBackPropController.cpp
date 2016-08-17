/*
                                                                           
   (               )                                        )              
 ( )\     )     ( /(       (                  (  (     ) ( /((             
 )((_) ( /(  (  )\())`  )  )(   (  `  )   (   )\))( ( /( )\())\  (   (     
((_)_  )(_)) )\((_)\ /(/( (()\  )\ /(/(   )\ ((_))\ )(_)|_))((_) )\  )\ )  
 | _ )((_)_ ((_) |(_|(_)_\ ((_)((_|(_)_\ ((_) (()(_|(_)_| |_ (_)((_)_(_/(  
 | _ \/ _` / _|| / /| '_ \) '_/ _ \ '_ \) _ \/ _` |/ _` |  _|| / _ \ ' \)) 
 |___/\__,_\__||_\_\| .__/|_| \___/ .__/\___/\__, |\__,_|\__||_\___/_||_|  
                    |_|           |_|        |___/                         

                                            
			   (                )        (  (           
			   )\            ( /((       )\ )\  (  (    
			 (((_)  (   (    )\())(   ( ((_|(_)))\ )(   
			 )\___  )\  )\ )(_))(()\  )\ _  _ /((_|()\  
			((/ __|((_)_(_/(| |_ ((_)((_) || (_))  ((_) 
			 | (__/ _ \ ' \))  _| '_/ _ \ || / -_)| '_| 
			  \___\___/_||_| \__|_| \___/_||_\___||_|   
                                            
 */

#include "CBackPropController.h"


CBackPropController::CBackPropController(HWND hwndMain):
	CContController(hwndMain)
{

}

void CBackPropController::InitializeLearningAlgorithm(void)
{
	CContController::InitializeLearningAlgorithm(); //call the parent's learning algorithm initialization
	
	//read training data from file (this is pretty basic text file reading, but at least the files can be inspected and modified if necessary)
	std::vector<std::vector<double> > inp;
	std::vector<std::vector<double> > out;
	uint no_training_samples;
	uint dist_effect_cutoff;
	uint no_inputs;
	uint no_hidden;
	uint no_out;
	double learning_rate;
	double mse_cutoff;
	ifstream f(CParams::sTrainingFilename.c_str());
	assert(f.is_open());

		f >> no_training_samples;
		f >> no_inputs;
		f >> no_hidden;
		f >> no_out;
		f >> learning_rate;
		f >> mse_cutoff;
		inp.resize(no_training_samples);
		out.resize(no_training_samples);

		for (uint32_t i = 0; i < no_training_samples; ++i){
			printf("Reading file ... %f%%\n",i / float(no_training_samples)*100.0);
			std::vector<double> temp1;
			for (uint32_t inp_s = 0; inp_s < no_inputs; ++inp_s){
				double in;
				f >> in;
				temp1.push_back(in);
			}
			inp[i] = temp1;
			temp1.clear();

			std::vector<double> temp2;
			for (uint32_t out_s = 0; out_s < no_out; ++out_s){
				double out;
				f >> out;
				temp2.push_back(out);
			}
			out[i] = temp2;
			temp2.clear();
		}
		f.close();
	//init the neural net and train it
		_neuralnet = new CNeuralNet(no_inputs,no_hidden,no_out,learning_rate,mse_cutoff);
		_neuralnet->initWeights();
		_neuralnet->train(inp, out, no_training_samples);
	//release the memory we alloced
		//for (uint32_t i = 0; i < no_training_samples; ++i){
		//	delete[] inp[i];
		//	delete[] out[i];
		//}
		//delete[] inp;
		//delete[] out;


}

/**
Returns the dot product between the sweeper's look vector and the vector from the sweeper to the object
*/
inline double dot_between_vlook_and_vObject(const CContMinesweeper &s,const CContCollisionObject &o){
	SVector2D<double> vLook = s.getLookAt();
	SVector2D<double> pt = o.getPosition();
		//get the vector to the point from the sweepers current position:
		SVector2D<double> vObj(SVector2D<double>(pt.x,pt.y) - s.Position());
		Vec2DNormalize<double>(vObj);
		//remember (MAM1000 / CSC3020) the dot product between two normalized vectors returns
		//1 if the two vectors point in the same direction
		//0 if the two vectors are perpendicular
		//-1 if the two vectors are pointing in opposite directions
		return Vec2DDot<double>(vLook,vObj);
}

bool CBackPropController::Update(void)
{
	CContController::Update(); //call the parent's class update. Do not delete this.
	for (auto s = m_vecSweepers.begin(); s != m_vecSweepers.end(); ++s){
		//compute the dot between the look vector and vector to the closest mine:
		double dot_mine = dot_between_vlook_and_vObject(**s,*m_vecObjects[(*s)->getClosestMine()]);
		double dot_rock = dot_between_vlook_and_vObject(**s,*m_vecObjects[(*s)->getClosestRock()]);
		double dot_supermine = dot_between_vlook_and_vObject(**s,*m_vecObjects[(*s)->getClosestSupermine()]);
		double dist_rock = Vec2DLength(m_vecObjects[(*s)->getClosestRock()]->getPosition() - (*s)->Position());
		double dist_supermine = Vec2DLength(m_vecObjects[(*s)->getClosestSupermine()]->getPosition() - (*s)->Position());
		//cheat a bit here... passing the distance into the neural net as well increases the search space dramatrically... :
		std::vector<double> dots = { dot_mine, (dist_rock < 50 || dist_supermine < 50) ? ((dist_rock < dist_supermine) ? dot_rock : dot_supermine) : -1};           // HERE HERE HERE HERE: double[2] dots
		if (_neuralnet->classify(dots) == 0){ // turn towards the mine                                                                                                                      (const double*)&
			SPoint pt(m_vecObjects[(*s)->getClosestMine()]->getPosition().x,
					  m_vecObjects[(*s)->getClosestMine()]->getPosition().y); 
			(*s)->turn(pt,1);
		} else {//turn away from a rock or supermine
			if (dist_rock < dist_supermine){
				SPoint pt(m_vecObjects[(*s)->getClosestRock()]->getPosition().x,
					  m_vecObjects[(*s)->getClosestRock()]->getPosition().y); 
				(*s)->turn(pt,1,false);
			} else {
				SPoint pt(m_vecObjects[(*s)->getClosestSupermine()]->getPosition().x,
					  m_vecObjects[(*s)->getClosestSupermine()]->getPosition().y); 
				(*s)->turn(pt,1,false);
			}
		}
	}

	return true; //method returns true if successful. Do not delete this.
}

CBackPropController::~CBackPropController(void)
{
	delete _neuralnet;
}
