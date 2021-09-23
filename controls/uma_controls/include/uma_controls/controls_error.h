// Copyright 2019 UM::Autonomy
#ifndef UMA_CONTROLS_CONTROLS_ERROR_H
#define UMA_CONTROLS_CONTROLS_ERROR_H

#include <string>
#include "errors/errors.h"

class ControlsException : public uma::UMAException
{
public:
    explicit ControlsException(const std::string &msg):
        UMAException(msg)
    {}
};

#endif  // UMA_CONTROLS_CONTROLS_ERROR_H
