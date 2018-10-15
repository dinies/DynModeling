#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "MockClock.hpp"

using ::testing::AtLeast;
namespace dyn_modeling {

  //TODO delete this, used to understand templates and interfaces( stil gives an error)
// template < class T>  class ClockUtilizer {
// private:
//     T m_clock = NULL;
//   public:
//   ClockUtilizer(T t_clock):
//   m_clock(t_clock)
//  {};
//
//     ClockUtilizer()= default;
//
//   inline void useIt(){ m_clock.getCurrTime();};
//
// };
//  ClockUtilizer<MockClock> utilizer(clock);
//   utilizer.useIt();
 

  TEST( ClockTest, withMock){
    MockClock clock;
    EXPECT_CALL( clock, getCurrTime())
      .Times(AtLeast(1));

    clock.getCurrTime();
 }
}



int main( int argc, char** argv) {
  ::testing::InitGoogleMock( &argc,argv);
  return RUN_ALL_TESTS();
}

