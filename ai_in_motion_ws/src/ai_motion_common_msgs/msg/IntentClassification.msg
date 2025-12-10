# Intent Classification Message
# Represents the classification of user intent from voice commands

string command
string intent
float32 confidence
builtin_interfaces/Time timestamp

# Common intent types:
# - navigation: Moving to locations
# - manipulation: Interacting with objects
# - perception: Detecting objects/people
# - report: Providing status information
# - wait: Pausing execution
# - unknown: Unable to classify intent