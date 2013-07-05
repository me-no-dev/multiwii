===============================================================
For Users:
===============================================================

Documentation for releases should be in the official wiki at 
http://www.multiwii.com/wiki
For development versions you are on your own - good luck.

For discussion and questions the official forum is at
http://www.multiwii.com/forum

===============================================================
For Developers:
===============================================================

how to participate :
-------------------------
You have this great addition to the software which you feel must become integrated into MultiWii -
Great. We love contributions. For a successful contribution, the following has worked in the past:
For your implementation, please observe the notes on features and code format.
In the forum start a new thread with title '[patch] new gizmo" for your new contribution, describe what it does and include a unified diff (or 'diff -u' output) so we can easily see and follow what changes you made. Most often one of the registered active developers will pick up on the topic and incorporate it. Chances are, it will not happen - then you can always contact a developer who is 'close to your subject' and _kindly_ urge him to take a look. As a last resort everyone is free to contact Alex.

The repository :
-------------------------
MultiWii and MultiWiiConf
   -> is Alex working area, do not modified files here

Multiwii_shared and MultiWiiConf_shared can be modified by every contributors
   -> is a shared area for every commiter

Features :
-------------------------
New features should use readily available hardware so others can benefit. This would increase the chance for the code to be accepted for the main branch, 

Extending or enhancing an existing feature is preferred over yet another implementation; please check the existing code before coding.

Code and code format :
-------------------------

Some simple rules:
   - whatever development tools you use, code must compile in Processing rsp. Arduino IDE.
   - do not commit something that does not compile / check with various config settings
   - try to respect the tabulations for readability - use 2 spaces
   - style is K&R, 
   - cpp directives get indented like regular code
   - everything with underscore or all upercase is const or a define, it is ok to use upper case for acronym or short word
   - avoid float numbers
   - try to isolate your code if possible via #define statements
   - no new files
   - no C++ class
   - only stdint variables (no int or no long, int16_t or int32_t instead)
   - short code and fast code :)

