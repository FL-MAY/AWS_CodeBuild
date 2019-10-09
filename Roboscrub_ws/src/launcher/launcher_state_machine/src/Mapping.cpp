//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "launcher_state_machine/Mapping.h"

namespace rock::scrubber::launcher {

	Mapping::Mapping(std::shared_ptr<ScrubberStates>& states) {
		states_ = states;
		states_->shiftState(States::MAPPING);
	}

	void Mapping::start() {
		//TODO: Unlock wheel
	}

	void Mapping::pause() {
		//TODO: Lock wheel
		states_->shiftState(States::PAUSE);
	}

	void Mapping::resume() {
		//TODO: Unlock wheel
		states_->shiftState(States::MAPPING);
	}

	void Mapping::cancel() {
		done_    = true;
		success_ = true;
	}
}