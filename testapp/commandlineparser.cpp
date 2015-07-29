/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz            *
 *                                                                             *
 *  All rights reserved.                                                       *
 *                                                                             *
 *  Redistribution and use in source and binary forms, with or without         *
 *  modification, are permitted provided that the following conditions are met:*
 *   - Redistributions of source code must retain the above copyright          *
 *     notice, this list of conditions and the following disclaimer.           *
 *   - Redistributions in binary form must reproduce the above copyright       *
 *     notice, this list of conditions and the following disclaimer in the     *
 *     documentation and/or other materials provided with the distribution.    *
 *   - Neither the name of the PG510 nor the names of its contributors may be  *
 *     used to endorse or promote products derived from this software without  *
 *     specific prior written permission.                                      *
 *                                                                             *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR      *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER *
 *  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,   *
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,        *
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR         *
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF     *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING       *
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS         *
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE                *
 *******************************************************************************/

#include "commandlineparser.h"

#include <cstring>
#include <iostream>

CommandLineParser::CommandLineParser() {
    addSwitch("help", "h", "Show help");
}

CommandLineParser::~CommandLineParser() {
}

/*!
 * \return TRUE normally or FALSE if the application is meant to quit (e.g.
 * because the user typed --help)
 */
bool CommandLineParser::parseCommandLinesArgs(int argc, char** argv) {
    if (argc < 1) {
        std::cerr << "ERROR: first argument does not exist" << std::endl;
        return false; // application is meant to quit now
    }

    mApplication = argv[0];

    for (std::vector<bool>::iterator it = mOptionUsed.begin(); it != mOptionUsed.end(); ++it) {
        *it = false;
    }

    if (mOptionShortNames.size() != mOptionNames.size() ||
            mOptionUsed.size() != mOptionNames.size() ||
            mOptionHelp.size() != mOptionNames.size() ||
            mOptionHelpStringParameter.size() != mOptionNames.size() ||
            mOptionType.size() != mOptionNames.size()) {
        std::cerr << "parseCommandLinesArgs(): internal error" << std::endl;
        return false;
    }

    mOptionStringValue.resize(mOptionNames.size());

    for (int i = 1; i < argc; i++) {
        char* arg = argv[i];
        if (strlen(arg) < 1) {
            std::cerr << "ERROR: oops: strlen(arg) < 1" << std::endl;
            continue;
        }
        if (arg[0] != '-') {
            std::cerr << "parse error: expected \"-\" or \"--\" have: " << arg << std::endl;
            return false;
        }
        if (strncmp("-psn_", arg, 5) == 0) {
            // MAC OSX always provides such an arg - we just ignore it
            continue;
        }
        int optionIndex = findOptionIndex(arg);
        if (optionIndex < 0) {
            printUsage();
            std::cerr << std::endl;
            std::cerr << "unknown option \"" << arg << "\"" << std::endl;
            return false;
        }

        mOptionUsed[optionIndex] = true;

        if (mOptionType[optionIndex] == OPTION_TYPE_SWITCH) {
            // nothing to do
        } else if (mOptionType[optionIndex] == OPTION_TYPE_STRING) {
            if (i + 1 >= argc) {
                printUsage();
                std::cerr << std::endl;
                std::cerr << "parse error: expected string after " << arg << std::endl;
                return false;
            }
            std::string value = argv[i + 1];
            if (value.empty() || value[0] == '-') {
                printUsage();
                std::cerr << std::endl;
                std::cerr << "parse error: expected string after " << arg << std::endl;
                return false;
            }
            mOptionStringValue[optionIndex] = value;
            i++; // next iteration skips the string
        } else {
            std::cerr << "parseCommandLinesArgs(): internal error (unknown option type)" << std::endl;
        }
    }


    if (isOptionUsed("help")) {
        printUsage();
        return false;
    }

    return true;
}


/*!
 * Checks if \p arg is the argument described by \p name or \p shortName (both
 * are allowed to be "").
 *
 * \p name and \p shortName are meant to be the \em name of the argument only,
 * not the whole argument, i.e. "help" instead of "--help" and "h" instead of
 * "-h".
 */
bool CommandLineParser::isOption(const char* arg, const std::string& name, const std::string& shortName) const {
    if (!arg) {
        return false;
    }
    if (!name.empty()) {
        if (strlen(arg) > 2) {
            if (arg[0] == '-' && arg[1] == '-') {
                const char* argName = arg + 2;
                if (name.size() == strlen(argName)) {
                    if (strncmp(argName, name.c_str(), name.size()) == 0) {
                        return true;
                    }
                }
            }
        }
    }

    if (!shortName.empty()) {
        if (strlen(arg) > 1) {
            if (arg[0] == '-') {
                const char* argName = arg + 1;
                if (shortName.size() == strlen(argName)) {
                    if (strncmp(argName, shortName.c_str(), shortName.size()) == 0) {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

void CommandLineParser::printUsage() {
    std::cout << "Usage: " << mApplication << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    for (unsigned int i = 0; i < mOptionNames.size(); i++) {
        std::string s = "  ";
        if (!mOptionShortNames[i].empty()) {
            s += "-";
            s += mOptionShortNames[i];
            if (!mOptionHelpStringParameter[i].empty()) {
                s += " ";
                s += mOptionHelpStringParameter[i];
            }
            s += ", ";
        }
        s += "--";
        s += mOptionNames[i];
        if (!mOptionHelpStringParameter[i].empty()) {
            s += " ";
            s += mOptionHelpStringParameter[i];
        }
        while (s.size() < 30) {
            s += " ";
        }
        s += "  ";
        s += mOptionHelp[i];
        std::cout << s << std::endl;
    }
}

void CommandLineParser::addSwitch(const std::string& name, const std::string& shortName, const std::string& helpText) {
    mOptionNames.push_back(name);
    mOptionShortNames.push_back(shortName);
    mOptionUsed.push_back(false);
    mOptionType.push_back(OPTION_TYPE_SWITCH);
    mOptionHelp.push_back(helpText);
    mOptionHelpStringParameter.push_back("");
}

void CommandLineParser::addStringOption(const std::string& name, const std::string& shortName, const std::string& parameterName, const std::string& helpText) {
    mOptionNames.push_back(name);
    mOptionShortNames.push_back(shortName);
    mOptionUsed.push_back(false);
    mOptionType.push_back(OPTION_TYPE_STRING);
    mOptionHelp.push_back(helpText);
    mOptionHelpStringParameter.push_back(parameterName);

    // parameters:
    // -> "--file <file>   Load <file> on startup"  with "file" (without the --)
    //    as name, with "<file>" as parameterName, and "Load <file> on startup"
    //    as helpText
}

/*!
 * \param name The (long) name of an option, without "-" and "--". Example:
 * "help". None of "--help", "-h" or "h" ("h" is the short name of help) will
 * work, only the long name without "--" will (to make the calling code more
 * readable).
 */
bool CommandLineParser::isOptionUsed(const std::string& name) const {
    int i = 0;
    for (std::vector<std::string>::const_iterator it = mOptionNames.begin(); it != mOptionNames.end(); ++it) {
        if (name == (*it)) {
            return mOptionUsed[i];
        }
        i++;
    }

    std::cerr << "isOptionUsed(): ERROR: option " << name << " was not registered in this class" << std::endl;

    return false;
}

/*!
 * \param arg The whole option as used on command line, i.e. including "-" or
 * "--" (i.e. note the name only).
 */
int CommandLineParser::findOptionIndex(const char* arg) const {
    unsigned int optionCount = mOptionNames.size();
    for (unsigned int i = 0; i < optionCount; i++) {
        if (isOption(arg, mOptionNames[i], mOptionShortNames[i])) {
            return i;
        }
    }
    return -1;
}

std::string CommandLineParser::getStringValue(const std::string& name) const {
    if (name.empty()) {
        return "";
    }
    int i = 0;
    for (std::vector<std::string>::const_iterator it = mOptionNames.begin(); it != mOptionNames.end(); ++it) {
        if (name == (*it)) {
            if (mOptionType[i] != OPTION_TYPE_STRING) {
                std::cerr << "ERROR: option \"" << name << "\" is not a string option" << std::endl;
                return "";
            }
            return mOptionStringValue[i];
        }
        i++;
    }

    std::cerr << "getStringValue(): ERROR: option \"" << name << "\" was not registered in this class" << std::endl;

    return false;
}

/*
 * vim: et sw=4 ts=4
 */
