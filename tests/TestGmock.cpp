#include <gmock/gmock.h>

using ::testing::StrictMock;

struct interface {
  virtual ~interface() = default;
  virtual int get() const = 0;
  virtual void foo(int) = 0;
  virtual void bar(int, const std::string&) = 0;
};

int main() {
  StrictMock<interface> mock;
  EXPECT_CALL(mock, foo(42));
  interface& i = mock;
  i.foo(42);
}
