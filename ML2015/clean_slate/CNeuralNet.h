/*
 * CNeuralNet.h
 *
 *  Created on: 26 Dec 2013
 *      Author: benjamin
 */

#ifndef CNEURALNET_H_
#define CNEURALNET_H_
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <cstring>
#include <stdio.h>
#include <stdint.h>

typedef unsigned int uint;

class CNeuralNet {
protected:
	void feedForward(std::vector<double> inputs); //you may modify this to do std::vector<double> if you want
	void propagateErrorBackward(std::vector<double> desiredOutput); //you may modify this to do std::vector<double> if you want
	double meanSquaredError(std::vector<double> desiredOutput); //you may modify this to do std::vector<double> if you want
public:

	uint inputLayerSize;
	uint hiddenLayerSize;
	uint outputLayerSize;
	double lRate;
	double mse_cutoff;

	std::vector<double> inputLayer;
	std::vector<double> hiddenLayer;
	std::vector<double> hiddenError;
	std::vector<double> outputError;
	std::vector<double> outputLayer;
	std::vector<std::vector<double> > hiddenLayerWeights;
	std::vector<std::vector<double> > outputLayerWeights;

	CNeuralNet(uint inputLayerSize, uint hiddenLayerSize, uint outputLayerSize, double lRate, double mse_cutoff);
	void initWeights();
	void train(std::vector<std::vector<double> > inputs, std::vector<std::vector<double> > outputs, uint trainingSetSize); //you may modify this to do std::vector<std::vector<double> > or do boost multiarray or something else if you want
	uint classify(std::vector<double> input); //you may modify this to do std::vector<double> if you want
	double getOutput(uint index) const;
	virtual ~CNeuralNet();
};

#endif /* CNEURALNET_H_ */
