Kill\_Handling
==============

Kill handling software

## Usage
C++ listener
'''
#include <kill_handling/listener.h>
kill_handling::KillListener kill_listener_(/*on_kill_cb*/, /*on_unkill_cb*/);

...

// Get reason for kill 
std::vector<std::string> res = kill_listener_.get_kills();
'''