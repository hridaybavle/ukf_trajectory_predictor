# Request of a behavior activation/deactivation

BehaviorCommandPriority behavior_command   # Requested behavior
int8 request_type                  # Type of the request
bool success                       # True if the operation was successful 
string failure_cause               # Code of the behavior event

int8 DEACTIVATE=0
int8 ACTIVATE=1
int8 ACTIVATION_FINISHED=2


