/**
 * @file      console.hpp
 * @brief     Experiment "State Machine" for controlling flow
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-08-11
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef CONSOLE_HPP_
#define CONSOLE_HPP_

#include <iostream>

// Console printing MACROS
#define ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m";    // RED
#define WARN(x) std::cerr << "\033[22;33;1m" << x << "\033[0m";   // YELLOW
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";   // WHITE
#define DEBUG(x) std::cerr << "\033[22;34;1m" << x << "\033[0m";  // BLUE
#define CLEAR() std::cerr << "\x1B[2J\x1B[H";

#endif /* CONSOLE_HPP_ */
