// Copyright 2019 UM::Autonomy

#ifndef UMA_CONTROLS_NMEA_0183_H
#define UMA_CONTROLS_NMEA_0183_H

#include <utility>
#include <vector>
#include <iterator>
#include <unordered_map>
#include <string>
#include "utils/string_utils.h"
#include <algorithm>
#include "uma_controls/controls_error.h"

class NMEAMessage
{
public:
    explicit NMEAMessage(const std::string &nmea_format) :
    nmea_format(nmea_format)
    {
        // Remove everything after the asterisk
        std::string str;
        str = nmea_format.substr(0, std::find(nmea_format.begin(), nmea_format.end(), '*') - nmea_format.begin());

        // Split by comma separator
        std::vector<std::string> message_parts = uma::split(str, ',');

        // If not empty message
        if (message_parts.size() > 0)
        {
            command = message_parts[0];

            for (const std::string &s : message_parts)
            {
                // If there's an '=' it is a 'parameter' if not, a 'flag'
                std::vector<std::string> split = uma::split(s, '=', true);
                if (split.size() == 1)
                {
                    flags.push_back(split[0]);
                }
                else if (split.size() == 2)
                {
                    parameters[split[0]] = split[1];
                }
                else if (split.size() > 2)
                {
                    throw ControlsException("Invalid NMEA parameter (multiple equals signs)");
                }
            }
        }
        else
        {
            throw ControlsException("Empty NMEA message");
        }
    }

    std::string get_command()
    {
        return command;
    }

    std::string get_flag(int index)
    {
        if (index < flags.size())
        {
          return flags[index];
        }
        throw ControlsException("Flag does not exist");
    }

    bool flag_present(const std::string &name)
    {
        return std::find(flags.begin(), flags.end(), name) != flags.end();
    }

    bool parameter_present(const std::string &name)
    {
        return(parameters.count(name) > 0);
    }

    template <typename  InputIterator>
    bool parameters_present(const InputIterator &begin, const InputIterator &end)
    {
        InputIterator &iter = begin;
        while (iter != end)
        {
            if (!parameter_present(*iter))
                return false;
            ++iter;
        }

        return true;
    }

    std::string get_parameter(const std::string &name)
    {
        if (!parameters.count(name))
            throw ControlsException("Parameter does not exist");

        return parameters[name];
    }

    int num_parameters()
    {
        return parameters.size();
    }

    int num_flags()
    {
        return flags.size();
    }

protected:
    std::string nmea_format;
    std::string command;
    std::vector<std::string> flags;
    std::unordered_map<std::string, std::string> parameters;
};

#endif  // UMA_CONTROLS_NMEA_0183_H
