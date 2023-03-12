#include <cstdio>
#include "ivp_control/ivp_control.h"

Control::Control() {
  test = false;
  std::cout << "Hello" << test << std::endl;
}

Control::~Control() {
  std::cout << "Destructor" << std::endl;
}
