##
## Severity level constants
##
# byte FATAL=1  #fatal/critical level
# byte ERROR=10 #error level
# byte WARN=20  #warning level
# byte DEBUG=30 #debug level
# byte INFO=100 #general level
##
## Fields
##
byte level
string name #name of node that failed (autoset)
string msg #error message 
string[] topics #topic names that the node publishes
