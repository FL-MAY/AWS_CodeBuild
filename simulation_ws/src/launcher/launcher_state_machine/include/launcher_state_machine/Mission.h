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

namespace rock::scrubber::launcher {
	class Mission {
	public:
		virtual void start() = 0;

		virtual void pause() = 0;

		virtual void resume() = 0;

		virtual void cancel() { cancelled_ = true; };

		virtual bool isDone() const { return done_; };

		virtual bool isSuccess() const { return success_; };

		virtual ~Mission() = default;

	protected:
		Mission():cancelled_(false), done_(false), success_(false){};
		bool cancelled_;
		bool done_;
		bool success_;
	};
}

