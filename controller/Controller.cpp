#include "Kinematics.hpp"
#include "MotionEngine.hpp"

#include <fstream>
#include <iostream>

class Controller : public cnoid::SimpleController
{
public:
  virtual bool initialize(cnoid::SimpleControllerIO * io) override
  {
    motion_engine_ = std::make_shared<MotionEngine>();

    motion_engine_->initialize(io);

    return true;
  }
  virtual bool control() override
  {
    motion_engine_->control();

    return true;
  }

private:
  std::shared_ptr<MotionEngine> motion_engine_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Controller)
