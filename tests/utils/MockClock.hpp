// Created by dinies on 11/10/2018.

#pragma once
#include <gmock/gmock.h>
#include "../../src/utils/ClockInterface.hpp"

namespace dyn_modeling {

  class MockClock: public ClockInterface {

    public:
      MOCK_METHOD0(getDeltaT, double());
      MOCK_METHOD0(getCurrTime, double());
      MOCK_METHOD1(setCurrTime, void(const double));
      MOCK_METHOD0(tick, void());

      virtual ~MockClock() {};
  };
}
