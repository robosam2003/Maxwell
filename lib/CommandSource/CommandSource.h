//
// Created by SamScott on 16/02/2026.
//

#ifndef MAXWELL_COMMANDSOURCE_H
#define MAXWELL_COMMANDSOURCE_H


class CommandSource {
public:
    virtual float read();
    float command_gain = 1.0; // Default gain of 1.0, can be adjusted by the user
};


#endif //MAXWELL_COMMANDSOURCE_H