#include "hooks.h"

Hooks *Hooks::globalRef = nullptr;

void Hooks::init()
{
  set<bool>("isOn", true);
  hookTank.close();
};

void Hooks::periodic()
{
  if (RobotState::getStateOf("toggling_descore"))
    holding = true;
  else
  {
    if (holding)
    {
      toggled = !toggled;
      holding = false;
    }
  }
  if (toggled)
  {
    hookTank.open();
  }
  else
  {
    hookTank.close();
  }
};

void Hooks::updateTelemetry()
{
  return;
}

void Hooks::stop()
{
  hookTank.close();
}