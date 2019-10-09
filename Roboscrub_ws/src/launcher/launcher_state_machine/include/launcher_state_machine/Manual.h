//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#pragma once

#include "ScrubberStates.h"
#include "Mission.h"

namespace rock::scrubber::launcher {
	class Manual : public Mission{
	public:
		explicit Manual(std::shared_ptr<ScrubberStates>& states);

		void start() override;

		void pause() override;

		void resume() override;

		void cancel() override;

	private:
		std::shared_ptr<ScrubberStates> states_;
	};
}

