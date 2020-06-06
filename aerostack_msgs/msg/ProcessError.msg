# Information about error ocurred in a process

Header header         # Message header
ErrorType error_type  # Error type enum
string ns             # Namespace ('ns' used instead of 'namespace' because it is a reserved word)
string hostname       # Name of the process 
string process_name   # Host of the process
string function       # Error function
string description    # Error description
int32 reference_code  # Error id number
