#pragma once
#include "Report.h"
#include <pybind11/pybind11.h>
#include <RLGymCPP/Gamestates/GameState.h>
#include <RLGymCPP/BasicTypes/Action.h>
#include <GigaLearnCPP/Util/Timer.h>

namespace GGL {
	struct RG_IMEXPORT RenderSender {
		pybind11::module pyMod;

		float timeScale;
		double adaptiveRenderDelay = -1;
		Timer renderTimer = {};

		RenderSender(float timeScale);

		RG_NO_COPY(RenderSender);

		void Send(const RLGC::GameState& state);

		~RenderSender();
	};
}