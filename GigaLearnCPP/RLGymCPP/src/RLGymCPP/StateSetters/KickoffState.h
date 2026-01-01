#pragma once
#include "StateSetter.h"

namespace RLGC {
	class KickoffState : public StateSetter {
	public:
		void ResetArena(Arena* arena) {
			arena->ResetToRandomKickoff();
		}
	};
}