#pragma once

#include "../Framework.h"

#include "../Util/ModelConfig.h"

namespace GGL {

	typedef std::function<RLGC::ObsBuilder*()> MakeObsFn;
	typedef std::function<RLGC::ActionParser*()> MakeActFn;

	typedef std::function<std::vector<int>(const RLGC::Player&, const RLGC::GameState&)> MapActionsFn;

	struct TransferLearnConfig {
		// This function should return the old obs builder (can be the same as the new)
		MakeObsFn makeOldObsFn;

		// This function should return the old action parser (can be the same as the new)
		MakeActFn makeOldActFn;

		// If the action spaces of the old and new action parser don't match, you will need to implement
		//	this mapping function to map the old action indices to the new action indices.
		// The size of the returned vector<int> should be equal to the number of actions in the new action parser.
		// Each element of the vector<int> is the corresponding action index in the old action parser.
		// This mapping can be dynamic and change depending on the state
		MapActionsFn mapActsFn = NULL;

		PartialModelConfig oldPolicyConfig;
		PartialModelConfig oldSharedHeadConfig; // If the old model didn't have shared layers, don't set this
		std::filesystem::path oldModelsPath; // Path to the directory with the old policy model(s) in it.

		// NOTE: All of the learning parameters below mostly depend on the complexity of your old policy's gameplay
		//	For refined/skilled bots, you may want a larger batch size and a much smaller learning rate
		//	The settings below are for a very basic ballchasing bot

		float lr = 3e-4;

		// NOTE: There are no minibatches
		int batchSize = 50'000;
		int epochs = 5;

		// Whether or not to use KL-Div (Kullback-Leibler divergence) as loss
		//	Otherwise, (a-b).abs().mean() is used
		bool useKLDiv = false;

		// Scale of the loss (prevents optimizers from dying since the natural loss is very low)
		float lossScale = 500.f;

		// Exponent for the transfer learn KL div loss
		float lossExponent = 1.f;
	};
}